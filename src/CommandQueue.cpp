/*********************************************************************************
 *  Copyright (c) 2010-2011, Elliott Cooper-Balis
 *                             Paul Rosenfeld
 *                             Bruce Jacob
 *                             University of Maryland
 *                             dramninjas [at] gmail [dot] com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright notice,
 *        this list of conditions and the following disclaimer in the documentation
 *        and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

#include <assert.h>

#include "AddressMapping.h"
#include "CommandQueue.h"
#include "MemoryController.h"

using namespace DRAMSim;

CommandQueue::CommandQueue(vector<vector<BankState>>& states, ostream& simLog)
    : dramsimLog(simLog),
      bankStates(states),
      nextBank(0),
      nextRank(0),
      nextBankPRE(0),
      nextRankPRE(0),
      refreshRank(0),
      refreshBank(0),
      refreshWaiting(false),
      sendAct(true)
{
    // set here to avoid compile errors
    currentClockCycle = 0;

    // set system parameters
    num_ranks_ = getConfigParam(UINT, "NUM_RANKS");
    num_banks_ = getConfigParam(UINT, "NUM_BANKS");
    cmd_queue_depth_ = getConfigParam(UINT, "CMD_QUEUE_DEPTH");
    xaw_ = getConfigParam(UINT, "XAW");
    total_row_accesses_ = getConfigParam(UINT, "TOTAL_ROW_ACCESSES");
    schedulingPolicy_ = PIMConfiguration::getSchedulingPolicy();
    queuingStructure_ = PIMConfiguration::getQueueingStructure();

    // use numBankQueus below to create queue structure
    size_t numBankQueues;
    if (queuingStructure_ == PerRank)
    {
        numBankQueues = 1;
    }
    else if (queuingStructure_ == PerRankPerBank)
    {
        numBankQueues = num_banks_;
    }
    else
    {
        ERROR("== Error - Unknown queuing structure");
        exit(0);
    }

    // vector of counters used to ensure rows don't stay open too long
    rowAccessCounters = vector<vector<unsigned>>(num_ranks_, vector<unsigned>(num_banks_, 0));

    // create queue based on the structure we want
    BusPacket1D actualQueue;
    BusPacket2D perBankQueue = BusPacket2D();
    queues = BusPacket3D();
    for (size_t rank = 0; rank < num_ranks_; rank++)
    {
        // this loop will run only once for per-rank and NUM_BANKS times for
        // per-rank-per-bank
        for (size_t bank = 0; bank < numBankQueues; bank++)
        {
            actualQueue = BusPacket1D();
            perBankQueue.push_back(actualQueue);
        }
        queues.push_back(perBankQueue);
    }

    // X-bank activation window
    //    this will count the number of activations within a given window
    //    (decrementing counter)
    //
    // countdown vector will have decrementing counters starting at tXAW
    //  when the 0th element reaches 0, remove it
    tXAWCountdown.reserve(num_ranks_);
    for (size_t i = 0; i < num_ranks_; i++)
    {
        tXAWCountdown.push_back(vector<unsigned>());
    }
}

CommandQueue::~CommandQueue()
{
    // ERROR("COMMAND QUEUE destructor");
    size_t bankMax = num_ranks_;
    if (queuingStructure_ == PerRank)
    {
        bankMax = 1;
    }
    for (size_t r = 0; r < num_ranks_; r++)
    {
        for (size_t b = 0; b < bankMax; b++)
        {
            for (size_t i = 0; i < queues[r][b].size(); i++)
            {
                delete (queues[r][b][i]);
            }
            queues[r][b].clear();
        }
    }
}

// Adds a command to appropriate queue
void CommandQueue::enqueue(BusPacket* newBusPacket)
{
    unsigned rank = newBusPacket->rank;
    unsigned bank = newBusPacket->bank;
    if (queuingStructure_ == PerRank)
    {
        queues[rank][0].push_back(newBusPacket);
        if (queues[rank][0].size() > cmd_queue_depth_)
        {
            ERROR("== Error - Enqueued more than allowed in command queue");
            ERROR(
                "                        Need to call .hasRoomFor(int numberToEnqueue, "
                "unsigned rank, unsigned bank) first");
            exit(0);
        }
    }
    else if (queuingStructure_ == PerRankPerBank)
    {
        queues[rank][bank].push_back(newBusPacket);
        if (queues[rank][bank].size() > cmd_queue_depth_)
        {
            ERROR("== Error - Enqueued more than allowed in command queue");
            ERROR(
                "                        Need to call .hasRoomFor(int numberToEnqueue, "
                "unsigned rank, unsigned bank) first");
            exit(0);
        }
    }
    else
    {
        ERROR("== Error - Unknown queuing structure");
        exit(0);
    }
}

bool CommandQueue::process_refresh(BusPacket** busPacket)
{
    if (refreshWaiting)
    {
        bool sendREF = true;
        for (size_t b = 0; b < num_banks_; b++)
        {
            if (bankStates[refreshRank][b].currentBankState == RowActive)
            {
                sendREF = false;
                *busPacket =
                    new BusPacket(PRECHARGE, 0, 0, bankStates[refreshRank][b].openRowAddress,
                                  refreshRank, b, nullptr, dramsimLog);
                if (isIssuable(*busPacket))
                {
                    return true;
                }
                else
                {
                    delete *busPacket;
                }
            }
        }
        if (sendREF)
        {
            *busPacket = new BusPacket(REF, 0, 0, 0, refreshRank, 0, nullptr, dramsimLog);
            if (isIssuable(*busPacket))
            {
                refreshWaiting = false;
                return true;
            }
            else
            {
                delete *busPacket;
            }
        }
    }
    return false;
}

bool CommandQueue::process_command(BusPacket** busPacket)
{
    unsigned startingRank = nextRank;
    unsigned startingBank = nextBank;
    // if(refreshWaiting)
    //         return false;
    do
    {
        vector<BusPacket*>& queue = getCommandQueue(nextRank, nextBank);
        for (size_t i = 0; i < queue.size(); i++)
        {
            BusPacket* packet = queue[i];

            if (isIssuable(packet))
            {
                if (i != 0 && queue[i]->tag.find("BAR", 0) != std::string::npos)
                {
                    break;
                }
                else
                {
                    bool depend = false;
                    for (size_t j = 0; j < i; j++)
                    {
                        if (queue[i]->bank == queue[j]->bank && queue[i]->row == queue[j]->row &&
                            queue[i]->column == queue[j]->column)
                        {
                            depend = true;
                            break;
                        }
                        if (queue[j]->tag.find("BAR", 0) != std::string::npos)
                        {
                            depend = true;
                            break;
                        }
                    }
                    if (!depend)
                    {
                        *busPacket = packet;
                        queue.erase(queue.begin() + i);
                        return true;
                    }
                }
            }
        }

        for (size_t i = 0; i < queue.size(); i++)
        {
            if (i != 0 && queue[i]->tag.find("BAR", 0) != std::string::npos)
            {
                break;
            }

            BusPacket* packet = queue[i];
            if (bankStates[packet->rank][packet->bank].currentBankState == Idle)
            {
                *busPacket =
                    new BusPacket(ACTIVATE, packet->physicalAddress, packet->column, packet->row,
                                  packet->rank, packet->bank, nullptr, dramsimLog, packet->tag);
                if (isIssuable(*busPacket))
                {
                    return true;
                }
                else
                {
                    delete *busPacket;
                }
            }
        }

        if (queuingStructure_ == PerRank)
            nextRank = (nextRank + 1) % num_ranks_;
        else
            nextRankAndBank(nextRank, nextBank);
    } while (!(startingRank == nextRank && startingBank == nextBank));

    return false;
}

bool CommandQueue::process_precharge(BusPacket** busPacket)
{
    unsigned startingRank = nextRankPRE;
    unsigned startingBank = nextBankPRE;

    do
    {
        bool found = false;
        vector<BusPacket*>& queue = getCommandQueue(nextRankPRE, nextBankPRE);
        for (size_t i = 0; i < queue.size(); i++)
        {
            BusPacket* packet = queue[i];
            if (nextRankPRE == packet->rank && nextBankPRE == packet->bank &&
                bankStates[packet->rank][packet->bank].currentBankState == RowActive &&
                packet->row == bankStates[packet->rank][packet->bank].openRowAddress)
                found = true;
            if (packet->tag.find("BAR", 0) != std::string::npos)
                break;
        }
        if (!found)
        {
            *busPacket =
                new BusPacket(PRECHARGE, 0, 0, bankStates[nextRankPRE][nextBankPRE].openRowAddress,
                              nextRankPRE, nextBankPRE, nullptr, dramsimLog);
            if (isIssuable(*busPacket))
                return true;
            else
                delete *busPacket;
        }
        nextRankAndBank(nextRankPRE, nextBankPRE);
    } while (!(startingRank == nextRankPRE && startingBank == nextBankPRE));

    return false;
}

bool CommandQueue::pop(BusPacket** busPacket)
{
    if (queuingStructure_ == PerRankPerBank)
    {
        ERROR("== Error - queuingStructure_ PerRankPerBank is not allowed");
        exit(0);
    }
    for (size_t i = 0; i < num_ranks_; i++)
    {
        // decrement all the counters we have going
        for (size_t j = 0; j < tXAWCountdown[i].size(); j++) tXAWCountdown[i][j]--;
        // the head will always be the smallest counter, so check if it has reached 0
        if (tXAWCountdown[i].size() > 0 && tXAWCountdown[i][0] == 0)
            tXAWCountdown[i].erase(tXAWCountdown[i].begin());
    }

    if (process_refresh(busPacket))
        return true;
    else if (process_command(busPacket))
        return true;
    else if (process_precharge(busPacket))
        return true;
    else
        return false;
    return false;
}

// check if a rank/bank queue has room for a certain number of bus packets
bool CommandQueue::hasRoomFor(unsigned numberToEnqueue, unsigned rank, unsigned bank)
{
    vector<BusPacket*>& queue = getCommandQueue(rank, bank);
    return ((cmd_queue_depth_ - queue.size()) >= numberToEnqueue);
}

// prints the contents of the command queue
void CommandQueue::print()
{
    if (queuingStructure_ == PerRank)
    {
        PRINT(endl << "== Printing Per Rank Queue");
        for (size_t i = 0; i < num_ranks_; i++)
        {
            PRINT(" = Rank " << i << "  size : " << queues[i][0].size());
            for (size_t j = 0; j < queues[i][0].size(); j++)
            {
                PRINTN("    " << j << "]");
                queues[i][0][j]->print();
            }
        }
    }
    else if (queuingStructure_ == PerRankPerBank)
    {
        PRINT("\n== Printing Per Rank, Per Bank Queue");

        for (size_t i = 0; i < num_ranks_; i++)
        {
            PRINT(" = Rank " << i);
            for (size_t j = 0; j < num_banks_; j++)
            {
                PRINT("    Bank " << j << "   size : " << queues[i][j].size());

                for (size_t k = 0; k < queues[i][j].size(); k++)
                {
                    PRINTN("       " << k << "]");
                    queues[i][j][k]->print();
                }
            }
        }
    }
}

/**
 * return a reference to the queue for a given rank, bank. Since we
 * don't always have a per bank queuing structure, sometimes the bank
 * argument is ignored (and the 0th index is returned
 */
vector<BusPacket*>& CommandQueue::getCommandQueue(unsigned rank, unsigned bank)
{
    if (queuingStructure_ == PerRankPerBank)
        return queues[rank][bank];
    else if (queuingStructure_ == PerRank)
        return queues[rank][0];
    else
    {
        ERROR("Unknown queue structure");
        abort();
    }
}

// checks if busPacket is allowed to be issued
bool CommandQueue::isIssuable(BusPacket* busPacket)
{
    switch (busPacket->busPacketType)
    {
        case REF:
        case RFCSB:
            return true;
            break;
        case ACTIVATE:

            if ((*ranks)[busPacket->rank]->mode_ != dramMode::SB && busPacket->bank >= 2)
            {
                return false;
            }

            if ((bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle ||
                 bankStates[busPacket->rank][busPacket->bank].currentBankState == Refreshing) &&
                currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextActivate &&
                tXAWCountdown[busPacket->rank].size() < xaw_)
            {
                return true;
            }
            else
            {
                return false;
            }
            break;

        case WRITE:
            if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
                currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextWrite &&
                busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress &&
                rowAccessCounters[busPacket->rank][busPacket->bank] < total_row_accesses_)
            {
                return true;
            }
            else
            {
                return false;
            }
            break;
        case READ:
            if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
                currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextRead &&
                busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress &&
                rowAccessCounters[busPacket->rank][busPacket->bank] < total_row_accesses_)
            {
                return true;
            }
            else
            {
                return false;
            }
            break;
        case PRECHARGE:
            if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
                currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextPrecharge)
            {
                return true;
            }
            else
            {
                return false;
            }
            break;

        default:
            ERROR("== Error - Trying to issue a crazy bus packet type : ");
            busPacket->print();
            exit(0);
    }
    return false;
}

// figures out if a rank's queue is empty
bool CommandQueue::isEmpty(unsigned rank)
{
    if (queuingStructure_ == PerRank)
    {
        return queues[rank][0].empty();
    }
    else if (queuingStructure_ == PerRankPerBank)
    {
        for (size_t i = 0; i < num_banks_; i++)
        {
            if (!queues[rank][i].empty())
                return false;
        }
        return true;
    }
    else
    {
        DEBUG("Invalid Queueing Stucture");
        abort();
    }
}

// tells the command queue that a particular rank is in need of a refresh
void CommandQueue::needRefresh(unsigned rank)
{
    refreshWaiting = true;
    refreshRank = rank;
}

void CommandQueue::nextRankAndBank(unsigned& rank, unsigned& bank)
{
    if (schedulingPolicy_ == RankThenBankRoundRobin)
    {
        rank++;
        if (rank == num_ranks_)
        {
            rank = 0;
            bank++;
            if (bank == num_banks_)
            {
                bank = 0;
            }
        }
    }
    // bank-then-rank round robin
    else if (schedulingPolicy_ == BankThenRankRoundRobin)
    {
        bank++;
        if (bank == num_banks_)
        {
            bank = 0;
            rank++;
            if (rank == num_ranks_)
            {
                rank = 0;
            }
        }
    }
    else
    {
        ERROR("== Error - Unknown scheduling policy");
        exit(0);
    }
}

void CommandQueue::update()
{
    // do nothing since pop() is effectively update(),
    // needed for SimulatorObject
    // TODO: make CommandQueue not a SimulatorObject
}
