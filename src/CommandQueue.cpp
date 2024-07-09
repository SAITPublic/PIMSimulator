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
      bankStates_sub(emptyBankStatesSub),
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
    ranks = nullptr;
    // set system parameters
    num_ranks_ = getConfigParam(UINT, "NUM_RANKS");
    num_banks_ = getConfigParam(UINT, "NUM_BANKS");
    num_subarrays_ = 1;

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
        //how about make subqueues?
    }
    else
    {
        ERROR("== Error - Unknown queuing structure");
        exit(0);
    }

    // vector of counters used to ensure rows don't stay open too long
    rowAccessCounters = vector<vector<unsigned>>(num_ranks_, vector<unsigned>(num_banks_, 0));
    commandCounters.reserve(cmd_queue_depth_);
    processedCommands.reserve(cmd_queue_depth_);
    commandCounters.clear();
    processedCommands.clear();

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

CommandQueue::CommandQueue(vector<vector<BankState>>& states, ostream& simLog, bool is_salp)
    : dramsimLog(simLog),
      bankStates(emptyBankStates),
      bankStates_sub(states),
      nextBank(0),
      nextRank(0),
      nextSub(0),
      nextBankPRE(0),
      nextRankPRE(0),
      nextSubPRE(0),
      refreshRank(0),
      refreshBank(0),
      refreshSub(0),
      refreshWaiting(false),
      sendAct(true)
{
    // set here to avoid compile errors
    currentClockCycle = 0;
    ranks = nullptr;
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
        numBankQueues = 1; //we use this logic bugt...
        num_subarrays_ = 1;
    }
    else if (queuingStructure_ == PerRankPerBank)
    {
        numBankQueues = num_banks_;
        //how about make subqueues?
        num_subarrays_ = 1;
    }
    else if (queuingStructure_ == PerRankPerBankPerSubarray)
    {
        numBankQueues = num_banks_;
        num_subarrays_ = 4;
    }
    else
    {
        ERROR("== Error - Unknown queuing structure");
        exit(0);
    }

    // vector of counters used to ensure rows don't stay open too long
    rowAccessCounters_sub = vector<vector<vector<unsigned>>>(num_ranks_, vector<vector<unsigned>>(num_banks_, vector<unsigned>(4, 0)));
    commandCounters.reserve(cmd_queue_depth_);
    processedCommands.reserve(cmd_queue_depth_);
    commandCounters.clear();
    processedCommands.clear();
    // create queue based on the structure we want
    BusPacket1D actualQueue_sub, actualQueue;
    BusPacket2D perSubQueue = BusPacket2D();
    BusPacket2D perbankqueue = BusPacket2D();
    BusPacket3D queues_bank = BusPacket3D();
    queues = BusPacket3D();
    queues_sub = BusPacket4D();
    for (size_t rank = 0; rank < num_ranks_; rank++)
    {
        // this loop will run only once for per-rank and NUM_BANKS times for
        // per-rank-per-bank
        for (size_t bank = 0; bank < numBankQueues; bank++)
        {
            for(size_t sub = 0; sub < num_subarrays_; sub++)
            {
                actualQueue_sub = BusPacket1D();
                perSubQueue.push_back(actualQueue_sub);
            }
            actualQueue = BusPacket1D();
            perbankqueue.push_back(actualQueue);
            queues_bank.push_back(perSubQueue);
        }
        queues.push_back(perbankqueue);
        queues_sub.push_back(queues_bank);
    
    }
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
    size_t subarrayMax = 4;
    if (queuingStructure_ == PerRank)
    {
        bankMax = 1;
        subarrayMax = 1;
    }
    for (size_t r = 0; r < num_ranks_; r++)
    {
        for (size_t b = 0; b < bankMax; b++)
        {
            for (size_t i = 0; i < queues[r][b].size(); i++)
            {
                queues[r][b][i] = nullptr;
                delete (queues[r][b][i]);
                
            }    
            for (size_t s = 0; s< subarrayMax; s++)
            {
                for(size_t j = 0; j < queues_sub[r][b][s].size(); j++)
                {
                    queues_sub[r][b][s][j] = nullptr;
                    delete (queues_sub[r][b][s][j]); //per rank logic...
                }
                queues_sub[r][b][s].clear();
            }        
            queues[r][b].clear();
            queues_sub[r][b].clear();
        }
    }
}

// Adds a command to appropriate queue
void CommandQueue::enqueue(BusPacket* newBusPacket)
{
    //cout<<"[commandqueue] enqueue: cycle is "<<currentClockCycle<<" and rank is "<<newBusPacket->rank<<" and bank is "<<newBusPacket->bank<<
    //" and queue size is "<<queues[0][0].size()<<endl;
    unsigned rank = newBusPacket->rank;
    unsigned bank = newBusPacket->bank;
    if (queuingStructure_ == PerRank)
    {
        if(newBusPacket!=nullptr)
        {
            if(queues[rank][0].capacity() == queues[rank][0].size())
            {
                queues[rank][0].reserve(queues[rank][0].size() + 64);
            }
            queues[rank][0].push_back(newBusPacket);
            //if(newBusPacket->chan!=0) cout<<"[commandqueue] enqueue: cycle is "<<currentClockCycle<<" and chan is "<<newBusPacket->chan<<endl;
            //commandCounters.push_back(0);
            //processedCommands.push_back(false);
            //cout<<"cycle is "<<currentClockCycle<<" and rank is "<<rank<<" and bank is "<<bank<<" and row is "<<newBusPacket->row<<endl;
        }
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

void CommandQueue::enqueue_sub(BusPacket* newBusPacket)
{
    //cout<<"[commandqueue] enqueue_sub: cycle is "<<currentClockCycle<<" and rank is "<<newBusPacket->rank<<" and bank is "<<newBusPacket->bank<<" and size is "<<queues_sub[0][0][0].size()<<endl;
    unsigned rank = newBusPacket->rank;
    unsigned bank = newBusPacket->bank;
    unsigned subarray = AddrMapping::findsubarray(newBusPacket->row);
    if(queuingStructure_ == PerRank)
    {
        //if(newBusPacket!=nullptr)
        //{
            /*if(queues_sub[rank][0][0].capacity() == queues_sub[rank][0][0].size())
            {
                queues_sub[rank][0][0].reserve(queues_sub[rank][0][0].size() + 10);
            }*/
            queues_sub[rank][0][0].push_back(newBusPacket);
            //commandCounters.push_back(0);
            //processedCommands.push_back(false);
        //}
    }
    else if(queuingStructure_ == PerRankPerBank)
    {
        queues_sub[rank][bank][0].push_back(newBusPacket);
    }
    else if (queuingStructure_ == PerRankPerBankPerSubarray)
    {
        queues_sub[rank][bank][subarray].push_back(newBusPacket);
    }
    else
    {
        ERROR("== Error - Unknown queuing structure");
        exit(0);
    }
}

bool CommandQueue::process_refresh(BusPacket** busPacket) //it's buspacket for command, not transaction
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
                    //*busPacket = nullptr;
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
                //*busPacket = nullptr;
            }
        }
    }
    return false;
}
//7800마다 오류가 생기는건 아마도 refresh의 문제일 가능성이 높아보이긴 함
bool CommandQueue::process_refresh_sub(BusPacket** busPacket)
{
    if (refreshWaiting)
    {
        bool sendREF = true;
        for (int b = 0; b < 16; b++)
        {
            for(size_t s = 0; s < 4; s++)
            {
                if(bankStates_sub[refreshRank][b*4+s].currentBankState == RowActive)
                {
                    sendREF = false;
                    *busPacket = new BusPacket(PRECHARGE, 0, 0, bankStates_sub[refreshRank][b*4+s].openRowAddress,
                                               refreshRank, b, nullptr, dramsimLog);
                    if(isIssuable_sub(*busPacket))
                    {
                        //cout<<"[commandqueue] process_refresh_sub: cycle is "<<currentClockCycle<<" and rank is "<<refreshRank<<" and bank is "<<refreshBank<<" and sub is "<<refreshSub<<" and type is "<<(*busPacket)->busPacketType<<endl;
                        return true;
                    }
                    else
                    {
                        //cout<<"[commandqueue] process_refresh_sub: cycle is "<<currentClockCycle<<" and rank is "<<refreshRank<<" and bank is "<<refreshBank<<" and sub is "<<refreshSub<<" and wait is "<<refreshWaiting<<endl;
                        delete *busPacket;
                        //*busPacket = nullptr;
                    }
                }
            }
        }
        //cout<<"[commandqueue] process_refresh_sub: cycle is "<<currentClockCycle<<" and rank is "<<refreshRank<<" and bank is "<<refreshBank<<" and sub is "<<refreshSub<<" and type is "<<(*busPacket)->busPacketType<<" and sendref is "<<sendREF<<endl;
        if (sendREF)
        {
            *busPacket = new BusPacket(REF, 0, 0, 0, refreshRank, 0, nullptr, dramsimLog);
            if(isIssuable_sub(*busPacket))
            {
                //cout<<"[commandqueue] process_refresh_sub: cycle is "<<currentClockCycle<<" and rank is "<<refreshRank<<" and bank is "<<refreshBank<<" and sub is "<<refreshSub<<" and type is "<<(*busPacket)->busPacketType<<endl;
                refreshWaiting = false;
                return true;
            }
            else
            {
                //cout<<"[commandqueue] process_refresh_sub: cycle is "<<currentClockCycle<<" and rank is "<<refreshRank<<" and bank is "<<refreshBank<<" and sub is "<<refreshSub<<" and type is "<<(*busPacket)->busPacketType<<endl;
                delete *busPacket;
                //*busPacket = nullptr;
            }
        }
    }
    else    return false;
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
            if(queue[i]!=nullptr)
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
                            if(queue[j]!=nullptr)
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
                        }
                        if (!depend)
                        {
                            *busPacket = queue[i];
                            queues[0][0].erase(queues[0][0].begin() + i);
                            return true;
                        }
                    }
                }
            }
            else
            {
                //queue.erase(queue.begin() + i);
            }
        }

        for (size_t i = 0; i < queue.size(); i++)
        {
            if(queue[i]!=nullptr)  
            {
                if (i != 0 && queue[i]->tag.find("BAR", 0) != std::string::npos)
                {
                    break;
                }

                BusPacket* packet = queue[i];
                //if(packet->rank <= num_ranks_ && packet->bank <= num_banks_)
                //{
                    if (bankStates[packet->rank][packet->bank].currentBankState == Idle) //activate command!
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
                            //*busPacket=nullptr;
                            delete *busPacket;
                        }
                    }
                //}
            }
            else
            {
                //queue.erase(queue.begin() + i);
            }
        }

        if (queuingStructure_ == PerRank)
            nextRank = (nextRank + 1) % num_ranks_;
        else if(queuingStructure_ == PerRankPerBank)
            nextRankAndBank(nextRank, nextBank);
    } while (!(startingRank == nextRank && startingBank == nextBank));

    return false;
}

bool CommandQueue::process_command_sub(BusPacket** busPacket)
{
    unsigned startingRank = nextRank;
    unsigned startingBank = nextBank;
    unsigned startingSub = nextSub;
    int sub = 0;
    // if(refreshWaiting)
    //         return false;
    do
    {
        vector<BusPacket*>& queue = getCommandQueue(nextRank, nextBank, nextSub);
        //cout<<"[commandQueue] process_command_sub: cycle is "<<currentClockCycle<<" and rank is "<<nextRank<<" and bank is "<<nextBank<<" and sub is "<<nextSub<<" and chan is "<<(*ranks)[0]->getChanId()<<" and size is "<<queue.size()<<endl;
        //if(&queue != nullptr)
        //{
            for (size_t i = 0; i < queue.size(); i++)
            {
                BusPacket* packet = queue[i];
                //if(packet->row!=NULL)
                //{
                    //cout<<"[commandqueue] process_command_sub: cycle is "<<currentClockCycle<<" and chan is "<<(*ranks)[0]->getChanId()<<" and row is "<<packet->physicalAddress<<endl;
                    sub = (packet->row<0x2000)?0:(packet->row<0x4000)?1:(packet->row<0x6000)?2:3;
                    if (isIssuable_sub(packet))
                    {
                        if (i != 0 && queue[i]->tag.find("BAR", 0) != std::string::npos)
                        {
                            //cout<<"[commandqueue] error: cycle is "<<currentClockCycle<<" and rank is "<<nextRank<<" and bank is "<<queue[i]->bank<<" and row is "<<queue[i]->row<<endl;
                            break;
                        }
                        else
                        {
                            bool depend = false;
                            for (size_t j = 0; j < i; j++)
                            {
                                if (queue[j]->tag.find("BAR", 0) != std::string::npos)
                                {
                                    depend = true;
                                    break;
                                }
                                if (queue[i]->bank == queue[j]->bank && queue[i]->row == queue[j]->row &&
                                    queue[i]->column == queue[j]->column)
                                {
                                    //cout<<"[commandqueue] error: cycle is "<<currentClockCycle<<" and rank is "<<nextRank<<" and bank is "<<queue[i]->bank<<" and row is "<<queue[i]->row<<endl;
                                    //processedCommands[j] = true;
                                    depend = true;
                                    break;
                                }
                            }
                            if (!depend)
                            {
                                *busPacket = packet;
                                //cout<<"[commandqueue] read or write success: cycle is "<<currentClockCycle<<" and rank is 0 "<<" and bank is "<<queue[i]->bank<< " and row is "
                                //<<queue[i]->row<<" and col is "<<queue[i]->column<<endl<<" and type is "<<queue[i]->busPacketType<<" and openrow is "<<bankStates_sub[0][queue[i]->bank][sub].openRowAddress<<endl;
                                queue.erase(queue.begin() + i);
                                //processedCommands.erase(processedCommands.begin() + i);
                                //commandCounters.erase(commandCounters.begin() + i);
                                return true;
                            }
                        }
                    }
                //}
                else
                {
                    //queue.erase(queue.begin() + i);
                }
            }
        
            for (size_t i = 0; i < queue.size(); i++)
            {
                if (i != 0 && queue[i]->tag.find("BAR", 0) != std::string::npos)
                {
                    break;
                }
                BusPacket* packet = queue[i];
                //if(queue[i]->physicalAddress!=NULL)
                //{
                    sub = (packet->row < 0x2000)?0:(packet->row < 0x4000)?1:(packet->row < 0x6000)?2:3;
                    if (bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState == Idle)
                    {
                        //cout<<"[commandqueue] process_command_sub: cycle is "<<currentClockCycle<<" and rank is "<<packet->rank<<" and bank is "<<packet->bank<<" and row is "<<packet->row<<endl;
                        //cout<<" and data is "<<packet->data<<" and tag is "<<packet->tag<<endl;
                        *busPacket =
                            new BusPacket(ACTIVATE, packet->physicalAddress, packet->column, packet->row,
                                        0, packet->bank, nullptr, dramsimLog, "activate");
                        if (isIssuable_sub(*busPacket))
                        {
                            return true;
                        }
                        else
                        {
                            delete *busPacket;
                        }
                    }
                //}
                //else
                //{
                    //queue.erase(queue.begin() + i);
                //}
            }
        //}
        if (queuingStructure_ == PerRank)
            nextRank = (nextRank + 1) % num_ranks_; //we got only one rank, which means...
        else if(queuingStructure_ == PerRankPerBank)
            nextRankAndBank(nextRank, nextBank);
        else if(queuingStructure_ == PerRankPerBankPerSubarray)
            nextRankAndBankandSubarray(nextRank, nextBank, nextSub);
    } while (!(startingRank == nextRank && startingBank == nextBank && startingSub == nextSub));
    return false;
}
bool CommandQueue::process_precharge(BusPacket** busPacket)
{
    unsigned startingRank = nextRankPRE;
    unsigned startingBank = nextBankPRE;
    //for this logic nextbankpre is constrained to 0..
    do
    {
        bool found = false;
        vector<BusPacket*>& queue = getCommandQueue(nextRankPRE, nextBankPRE);
        for (auto it = queues[0][0].begin(); it != queues[0][0].end(); it++)
        {
            BusPacket* packet = *it;
            auto index = it - queues[0][0].begin();
            if(packet != nullptr)
            {
                if(nextRankPRE == packet->rank && nextBankPRE == packet->bank &&
                    bankStates[packet->rank][packet->bank].currentBankState == RowActive &&
                    packet->row == bankStates[packet->rank][packet->bank].openRowAddress)
                {
                    found = true;
                    break;
                }
                if (packet->tag.find("BAR", 0) != std::string::npos)
                    break; 
            }
            else
            {
                //*it = nullptr;
                //queues[0][0].erase(queues[0][0].begin() + index);
            }
        }
        if (!found)
        {
            *busPacket =
                new BusPacket(PRECHARGE, 0, 0, bankStates[nextRankPRE][nextBankPRE].openRowAddress,
                              nextRankPRE, nextBankPRE, nullptr, dramsimLog);
            if (isIssuable(*busPacket))
            {
                return true;
            }
            else
                delete *busPacket;
                *busPacket = nullptr;
        }
        nextRankAndBank(nextRankPRE, nextBankPRE);
    } while (!(startingRank == nextRankPRE && startingBank == nextBankPRE));

    return false;
}

bool CommandQueue::process_precharge_sub(BusPacket** busPacket)
{
    unsigned startingRank = nextRankPRE;
    unsigned startingBank = nextBankPRE;
    unsigned startingSub = nextSubPRE;  
    //if(currentClockCycle > 610) cout<<"[commandqueue] process_precharge_sub: cycle is "<<currentClockCycle<<" and bank is "<<nextBankPRE<<" and sub is "<<nextSubPRE<<endl;
    do{
        bool found = false;
        vector<BusPacket*>& queue_sub = getCommandQueue(nextRankPRE, nextBankPRE, nextSubPRE);
        //if(&queue_sub != nullptr)
        //{
            for (size_t j = 0; j < queue_sub.size(); j++)
            {
                BusPacket* packet = queue_sub[j];
                unsigned sub = (packet->row < 0x2000)?0:(packet->row < 0x4000)?1:(packet->row < 0x6000)?2:3;
                if (nextRankPRE == packet->rank && nextBankPRE == packet->bank && nextSubPRE == sub &&
                    bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState == RowActive &&
                    packet->row == bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress)
                {
                    found = true;
                    //cout<<"[commandqueue] cannot precharge: cycle is "<<currentClockCycle<<" and bank is "<<nextBankPRE<<" and sub is "<<nextSubPRE<<" and index is "<<j<<endl;
                    break;
                }
                if(packet->tag.find("BAR", 0) != std::string::npos)
                    break;
            }
        //}
        if (!found)
        {
            *busPacket =
                new BusPacket(PRECHARGE, 0, 0, bankStates_sub[nextRankPRE][nextBankPRE*4+nextSubPRE].openRowAddress,
                              nextRankPRE, nextBankPRE, nullptr, dramsimLog);
            unsigned sub = (bankStates_sub[nextRankPRE][nextBankPRE*4+nextSubPRE].openRowAddress < 0x2000)?0:
                           (bankStates_sub[nextRankPRE][nextBankPRE*4+nextSubPRE].openRowAddress < 0x4000)?1:
                           (bankStates_sub[nextRankPRE][nextBankPRE*4+nextSubPRE].openRowAddress < 0x6000)?2:3;
            if (isIssuable_sub(*busPacket) && sub == nextSubPRE)
            {
                return true;
            }
            else
            {
                delete *busPacket;
                //*busPacket = nullptr;
            }
        }
        nextRankAndBankandSubarray(nextRankPRE, nextBankPRE, nextSubPRE);
    }while (!(startingRank == nextRankPRE && startingBank == nextBankPRE && startingSub == nextSubPRE));

    return false;
}
bool CommandQueue::pop(BusPacket** busPacket)
{
    if (queuingStructure_ == PerRankPerBank)
    {
        ERROR("== Error - queuingStructure_ PerRankPerBank and queueingStructure_ PerRankPerBankPerSubarray is not allowed");
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
    {
        return true;
    }
    else if (process_command(busPacket))
    {
        return true;
    }
    else if (process_precharge(busPacket))
    {
        return true;
    }
    else
    {
        //*(busPacket) = new BusPacket(RFCSB, 0, 0, 0, 0, 0, nullptr, dramsimLog);
        //*busPacket = nullptr;
        return false;
    }
}

bool CommandQueue::pop_sub(BusPacket** busPacket)
{
    //if((*busPacket)!=nullptr)   cout<<"[commandqueue] pop_sub: cycle is "<<currentClockCycle<<" and rank is "<<(*busPacket)->rank<<" and bank is "<<(*busPacket)->bank<<" and type is "<<(*busPacket)->busPacketType<<endl;
    if(queuingStructure_ == PerRankPerBankPerSubarray || queuingStructure_ == PerRankPerBank)
    {
        ERROR("== Error - queuingStructure_ PerRank and queueingStructure_ PerRankPerBankPerSubarray is not allowed");
        exit(0);
    }
    for(size_t i = 0; i < num_ranks_; i++)
    {
        for(size_t j = 0; j < tXAWCountdown[i].size(); j++) tXAWCountdown[i][j]--;
        if(tXAWCountdown[i].size() > 0 && tXAWCountdown[i][0] == 0)
            tXAWCountdown[i].erase(tXAWCountdown[i].begin());
    }
    //in first time, poppedBusPacket is null!
    if (process_refresh_sub(busPacket))
    {
        //if((*busPacket)!=nullptr)   cout<<"[commandqueue] pop_sub_refresh: cycle is "<<currentClockCycle<<" and rank is "<<(*busPacket)->rank<<" and bank is "<<(*busPacket)->bank<<" and type is "<<(*busPacket)->busPacketType<<endl;
        return true;
    }
    else if (process_command_sub(busPacket))
    {
        //if((*busPacket)!=nullptr)   cout<<"[commandqueue] pop_sub_command: cycle is "<<currentClockCycle<<" and rank is "<<(*busPacket)->rank<<" and bank is "<<(*busPacket)->bank<<" and type is "<<(*busPacket)->busPacketType<<endl;
        return true;
    }
    else if (process_precharge_sub(busPacket))
    {
        //if((*busPacket)!=nullptr)   cout<<"[commandqueue] pop_sub_precharge: cycle is "<<currentClockCycle<<" and rank is "<<(*busPacket)->rank<<" and bank is "<<(*busPacket)->bank<<" and type is "<<(*busPacket)->busPacketType<<endl;
        return true;
    }
    else
    {
        //if((*busPacket)!=nullptr)   cout<<"[commandqueue] pop_sub_else: cycle is "<<currentClockCycle<<" and rank is "<<(*busPacket)->rank<<" and bank is "<<(*busPacket)->bank<<" and type is "<<(*busPacket)->busPacketType<<endl;
        //*(busPacket) = new BusPacket(RFCSB, 0, 0, 0, 0, 0, nullptr, dramsimLog);
        //*busPacket = nullptr;
        return false;
    }
}
void CommandQueue::process_queue()
{
    if (queuingStructure_ == PerRankPerBank)
    {
        ERROR("== Error - queuingStructure_ PerRankPerBank and queueingStructure_ PerRankPerBankPerSubarray is not allowed");
        exit(0);
    }
    else
    {
        for(int i = 0; i < queues[0][0].size(); i++)//if(cmd_queue_depth_ <= queues_sub[0][0][0].size())
        {
            commandCounters[i]++;
        }
        if(queues[0][0].size() >= cmd_queue_depth_)
        {
            for(int i = 0; i < queues[0][0].size(); i++)
            {
                if(commandCounters[i] >= 250)
                {
                    queues[0][0].erase(queues[0][0].begin() + i);
                    commandCounters.erase(commandCounters.begin() + i);
                }
            }
        }
    }
}
void CommandQueue::process_queue_sub()
{
    if (queuingStructure_ == PerRankPerBankPerSubarray || queuingStructure_ == PerRankPerBank)
    {
        ERROR("== Error - queuingStructure_ PerRank and queueingStructure_ PerRankPerBankPerSubarray is not allowed");
        exit(0);
    }
    else
    {
        for(int i = 0; i < queues_sub[0][0][0].size(); i++)//if(cmd_queue_depth_ <= queues_sub[0][0][0].size())
        {
            //commandCounters[i]++;
            //cout<<i<<endl;
        }
        if(queues_sub[0][0][0].size() >= cmd_queue_depth_)
        {
            for(int i = 0; i < queues_sub[0][0][0].size(); i++)
            {
                if(commandCounters[i] >= 300 && processedCommands[i] == true)
                {
                    queues_sub[0][0][0].erase(queues_sub[0][0][0].begin() + i);
                    //commandCounters.erase(commandCounters.begin() + i);
                }
            }
        }
    }
}
// check if a rank/bank queue has room for a certain number of bus packets
bool CommandQueue::hasRoomFor(unsigned numberToEnqueue, unsigned rank, unsigned bank)
{
    vector<BusPacket*>& queue = getCommandQueue(rank, bank);
    return ((cmd_queue_depth_ - queue.size()) >= numberToEnqueue);
}

bool CommandQueue::hasRoomFor(unsigned numberToEnqueue, unsigned rank, unsigned bank, unsigned sub)
{
    vector<BusPacket*>& queue = getCommandQueue(rank, bank, sub);
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

vector<BusPacket*>& CommandQueue::getCommandQueue(unsigned rank, unsigned bank, unsigned sub)
{
    if(queuingStructure_ == PerRankPerBankPerSubarray)
        return queues_sub[rank][bank][sub];
    else if(queuingStructure_ == PerRankPerBank)
        return queues_sub[rank][bank][0];
    else if(queuingStructure_ == PerRank)
    {                                                   
        //cout<<"[commandqueue] error: cycle is "<<currentClockCycle<<" and rank is "<<rank<<" and bank is "<<bank<<" and sub is "<<sub<<" and size is "<<queues_sub[rank][0][0].size()<<endl;
        return queues_sub[rank][0][0];
    }
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
            if(bankStates[busPacket->rank].size() == num_banks_)
            {
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
            cout<<"[commandqueue] error: cycle is "<<currentClockCycle<<" and type is "<<busPacket->busPacketType<<endl;
            ERROR("== Error - Trying to issue a crazy bus packet type : ");
            busPacket->print();
            exit(0);
    }
    return false;
}
bool CommandQueue::isIssuable_sub(BusPacket* packet)
{
    int sub = (packet->row<0X2000)?0:(packet->row<0x4000)?1:(packet->row<0x6000)?2:3;
    switch (packet->busPacketType)
    {
        case REF:
        case RFCSB:
            return true;
            break;
        case ACTIVATE:
            if ((*ranks)[packet->rank]->mode_ != dramMode::SB && packet->bank >= 2)
            {
                return false;
            }
            if ((bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState == Idle ||
                bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState == Refreshing) &&
                currentClockCycle >= bankStates_sub[packet->rank][4*packet->bank + sub].nextActivate &&
                tXAWCountdown[packet->rank].size() < xaw_)
            {
                //cout<<"[commandqueue]: activate issue and clock is"<<currentClockCycle<<" and bank is "<<packet->bank<<" and sub is "<<sub<<
                //" and state is " <<bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState<<" and nextact is "<<bankStates_sub[packet->rank][4*packet->bank + sub].nextActivate
                //<<" and openrow is "<<bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress<<" and type is "<<packet->busPacketType<<endl;
                return true;
            }
            else
            {
                return false;
            }
            break;
        case WRITE:
            if (bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState == RowActive &&
                currentClockCycle >= bankStates_sub[packet->rank][4*packet->bank + sub].nextWrite &&
                packet->row == bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress &&
                rowAccessCounters_sub[packet->rank][packet->bank][sub] < total_row_accesses_)
            {
                //cout<<"[commandqueue]: write issue and clock is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and sub is "<<sub<<
                //" and state is " <<bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState<<" and nextpre is "<<bankStates_sub[packet->rank][4*packet->bank + sub].nextPrecharge
                //<<" and openrow is "<<bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress<<" and type is "<<packet->busPacketType<<endl;
                return true;
            }
            else
            {
                return false;
            }
            break;
        case READ:
            if (bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState == RowActive &&
                currentClockCycle >= bankStates_sub[packet->rank][4*packet->bank + sub].nextRead &&
                packet->row == bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress &&
                rowAccessCounters_sub[packet->rank][packet->bank][sub] < total_row_accesses_)
            {
                //cout<<"[commandqueue]: read issue and clock is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and sub is "<<sub<<
                // " and state is " <<bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState<<" and nextact is "<<bankStates_sub[packet->rank][4*packet->bank + sub].nextActivate
                //<<" and openrow is "<<bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress<<" and type is "<<packet->busPacketType<<endl;
                return true;
            }
            else
            {
                return false;
            }
            break;
        case PRECHARGE:
            //if(currentClockCycle > 615)  cout<<"[commandqueue]: precharge issue and clock is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and sub is "<<sub<<
            //" and state is " <<bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState<<" and nextact is "<<bankStates_sub[packet->rank][4*packet->bank + sub].nextActivate
            //<<" and openrow is "<<bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress<<" and type is "<<packet->busPacketType<<endl;
            if (bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState == RowActive &&
                currentClockCycle >= bankStates_sub[packet->rank][4*packet->bank + sub].nextPrecharge)
            {
                /*for(size_t j = 0; j < queues_sub[0][0][0].size(); j++)
                {
                    BusPacket* index = queues_sub[0][0][0][j];
                    if(index->rank == packet->rank && index->bank == packet->bank && index->rank == packet->rank &&
                        processedCommands[j] == true &&(index->busPacketType == READ || index->busPacketType == WRITE))
                    {
                        queues_sub[0][0][0].erase(queues_sub[0][0][0].begin() + j);
                        commandCounters.erase(commandCounters.begin() + j);
                        processedCommands.erase(processedCommands.begin() + j);
                    }
                }*/
                //cout<<"[commandqueue]: precharge issue and clock is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and sub is "<<sub<<
                //" and state is " <<bankStates_sub[packet->rank][4*packet->bank + sub].currentBankState<<" and nextact is "<<bankStates_sub[packet->rank][4*packet->bank + sub].nextActivate
                //<<" and openrow is "<<bankStates_sub[packet->rank][4*packet->bank + sub].openRowAddress<<" and type is "<<packet->busPacketType<<endl;
                return true;
            }
            else
            {
                return false;
            }
            break;
        default:
            ERROR("== Error - Trying to issue a crazy bus packet type : ");
            packet->print();
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
bool CommandQueue::isEmpty_sub(unsigned rank)
{
    if(queuingStructure_ == PerRank)
    {
        return queues_sub[rank][0][0].empty();
    }
    else if(queuingStructure_ == PerRankPerBank)
    {
        for(size_t i = 0; i < num_banks_; i++)
        {
            if(!queues_sub[rank][i][0].empty())
                return false;
        }
        return true;
    }
    else if(queuingStructure_== PerRankPerBankPerSubarray)
    {
        for(size_t i = 0; i < num_banks_; i++)
        {
            for(size_t j = 0; j < num_subarrays_; j++)
            {
                if(!queues_sub[rank][i][j].empty())
                    return false;
            }
        }
        return true;
    }
    else
    {
        DEBUG("Invalid Queueing Structure");
        abort();
    }
}

// tells the command queue that a particular rank is in need of a refresh
void CommandQueue::needRefresh(unsigned rank)
{
    cout<<"[commandqueue] needRefresh: cycle is "<<currentClockCycle<<" and rank is "<<rank<<endl;
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

void CommandQueue::nextRankAndBankandSubarray(unsigned& rank, unsigned& bank, unsigned& sub)
{
    if (schedulingPolicy_ == RankThenBankThenSubarrayRoundRobin)
    {
        sub++;
        if(sub == 4)
        {
            sub = 0;
            bank++;
            if(bank == num_banks_)
            {
                bank = 0;
                rank++;
                if(rank == num_ranks_)
                {
                    rank = 0;
                }
            }
        }
    }
}
void CommandQueue::update()
{
    // do nothing since pop() is effectively update(),
    // needed for SimulatorObject
    // TODO: make CommandQueue not a SimulatorObject
}
