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

#include <iostream>

#include "AddressMapping.h"
#include "MemoryController.h"
#include "MemorySystem.h"

#define SEQUENTIAL(rank, bank) (rank * config.NUM_BANKS) + bank
#define SEQUENTIAL_SUB(rank, bank, sub) (rank * config.NUM_BANKS * 4) + (bank * 4) + sub

using namespace DRAMSim;
MemoryController::MemoryController(MemorySystem* parent, CSVWriter& csvOut_, ostream& simLog,
                                   Configuration& configuration)
    : dramsimLog(simLog),
      config(configuration),
      bankStates(getConfigParam(UINT, "NUM_RANKS"),
                 vector<BankState>(getConfigParam(UINT, "NUM_BANKS"), dramsimLog)),
      bankStates_SUB(getConfigParam(UINT, "NUM_RANKS"), 
                 vector<BankState>(getConfigParam(UINT, "NUM_BANKS")*4, dramsimLog)),
      outgoingCmdPacket(NULL),
      outgoingDataPacket(NULL),
      dataCyclesLeft(0),
      cmdCyclesLeft(0),
      commandQueue(bankStates, simLog),
      commandQueue_SUB(bankStates_SUB, simLog, true),
      poppedBusPacket(NULL),
      csvOut(csvOut_),
      totalTransactions(0),
      totalRefreshes(0),
      refreshRank(0),
      refreshBank(0),
      totalReads(0),
      totalWrites(0)
{
    // get handle on parent
    parentMemorySystem = parent;

    currentClockCycle = 0;

    // reserve memory for vectors
    transactionQueue.reserve(config.TRANS_QUEUE_DEPTH);
    powerDown = vector<bool>(config.NUM_RANKS, false);

    grandTotalBankAccesses = totalReadsPerBank = totalWritesPerBank = totalActivatesPerBank =
        vector<uint64_t>(config.NUM_RANKS * config.NUM_BANKS, 0);
    totalReadsPerRank = totalWritesPerRank = totalActivatesPerRank =
        vector<uint64_t>(config.NUM_RANKS, 0);
        writeDataCountdown.reserve(config.NUM_RANKS);
    writeDataToSend.reserve(config.NUM_RANKS);
    refreshCountdown.reserve(config.NUM_RANKS);
    refreshCountdownBank.reserve(config.NUM_BANKS);

    // Power related packets
    backgroundEnergy = burstEnergy = actpreEnergy = vector<uint64_t>(config.NUM_RANKS, 0);
    refreshEnergy = aluPIMEnergy = vector<uint64_t>(config.NUM_RANKS, 0); //logic of pimrank...
    readPIMEnergy = vector<uint64_t>(config.NUM_BANKS, 0);
    totalBandwidth = 0.0;

    totalEpochLatency = vector<uint64_t>(config.NUM_RANKS * config.NUM_BANKS * 4, 0);

    // staggers when each rank is due for a refresh
    for (size_t i = 0; i < config.NUM_RANKS; i++)
        refreshCountdown.push_back((int)((config.tREFI / config.tCK) / config.NUM_RANKS) * (i + 1));
    for (size_t i = 0; i < config.NUM_BANKS; i++)
        refreshCountdownBank.push_back((int)((config.tREFISB / config.tCK)) * (i + 1));

    memoryContStats = new MemoryControllerStats(
        parentMemorySystem, csvOut, dramsimLog, config, totalTransactions, grandTotalBankAccesses,
        totalReadsPerRank, totalWritesPerRank, totalReadsPerBank, totalWritesPerBank,
        totalActivatesPerRank, totalActivatesPerBank, totalRefreshes, backgroundEnergy, burstEnergy,
        actpreEnergy, refreshEnergy, aluPIMEnergy, refreshEnergy, pendingReadTransactions, false);
}
    
MemoryController::MemoryController(MemorySystem* parent, CSVWriter& csvOut_, ostream& simLog,
                                   Configuration& configuration, bool is_salp)
    : dramsimLog(simLog),
      config(configuration),
      bankStates(getConfigParam(UINT, "NUM_RANKS"),
                vector<BankState>(getConfigParam(UINT, "NUM_BANKS"), dramsimLog)),
      bankStates_SUB(getConfigParam(UINT, "NUM_RANKS"), 
                vector<BankState>(getConfigParam(UINT, "NUM_BANKS")*4, dramsimLog)),
      outgoingCmdPacket(nullptr),
      outgoingDataPacket(nullptr),
      dataCyclesLeft(0),
      cmdCyclesLeft(0),
      commandQueue(bankStates, simLog),
      commandQueue_SUB(bankStates_SUB, simLog, true),
      poppedBusPacket(nullptr),
      csvOut(csvOut_),
      totalTransactions(0),
      totalRefreshes(0),
      refreshRank(0),
      refreshBank(0),
      totalReads(0),
      totalWrites(0),
      is_salp_(is_salp)
{
    // get handle on parent
    parentMemorySystem = parent;
    currentClockCycle = 0;

    // reserve memory for vectors
    transactionQueue.reserve(config.TRANS_QUEUE_DEPTH);
    powerDown = vector<bool>(config.NUM_RANKS, false);

    grandTotalBankAccesses = totalReadsPerBank = totalWritesPerBank = totalActivatesPerBank =
        vector<uint64_t>(config.NUM_RANKS * config.NUM_BANKS, 0);
    totalReadsPerRank = totalWritesPerRank = totalActivatesPerRank =
        vector<uint64_t>(config.NUM_RANKS, 0);

    writeDataCountdown.reserve(config.NUM_RANKS);
    writeDataToSend.reserve(config.NUM_RANKS);
    refreshCountdown.reserve(config.NUM_RANKS);
    refreshCountdownBank.reserve(config.NUM_BANKS);

    // Power related packets
    backgroundEnergy = burstEnergy = actpreEnergy = vector<uint64_t>(config.NUM_RANKS, 0);
    refreshEnergy = aluPIMEnergy = vector<uint64_t>(config.NUM_RANKS, 0); //logic of pimrank...
    readPIMEnergy = vector<uint64_t>(config.NUM_BANKS, 0);
    totalBandwidth = 0.0;

    totalEpochLatency = vector<uint64_t>(config.NUM_RANKS * config.NUM_BANKS * 4, 0);

    // staggers when each rank is due for a refresh
    for (size_t i = 0; i < config.NUM_RANKS; i++)
        refreshCountdown.push_back((int)((config.tREFI / config.tCK) / config.NUM_RANKS) * (i + 1));
    for (size_t i = 0; i < config.NUM_BANKS; i++)
        refreshCountdownBank.push_back((int)((config.tREFISB / config.tCK)) * (i + 1));
    pendingReadTransactions.reserve(32);
    pendingReadTransactions.clear();
    returnTransaction.reserve(32); 
    returnTransaction.clear();
    memoryContStats = new MemoryControllerStats(
        parentMemorySystem, csvOut, dramsimLog, config, totalTransactions, grandTotalBankAccesses,
        totalReadsPerRank, totalWritesPerRank, totalReadsPerBank, totalWritesPerBank,
        totalActivatesPerRank, totalActivatesPerBank, totalRefreshes, backgroundEnergy, burstEnergy,
        actpreEnergy, refreshEnergy, aluPIMEnergy, refreshEnergy, pendingReadTransactions, is_salp_);
}

//do we need subarray controller?

// get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(BusPacket* bpacket)
{
    if (bpacket->busPacketType != DATA)
    {
        ERROR("== Error - Memory Controller received a non-DATA bus packet from rank");
        bpacket->print();
        exit(0);
    }

    if (DEBUG_BUS)
    {
        PRINTN(" -- MC Receiving From Data Bus : ");
        bpacket->print();
    }

    // add to return read data queue
    if(bpacket->busPacketType == DATA)
        returnTransaction.push_back(
            new Transaction(RETURN_DATA, bpacket->physicalAddress, bpacket->data));

    // this delete statement saves a mindboggling amount of memory
    delete (bpacket);
}

// sends read data back to the CPU
void MemoryController::returnReadData(const Transaction* trans)
{
    if (parentMemorySystem->ReturnReadData != NULL)
        (*parentMemorySystem->ReturnReadData)(parentMemorySystem->systemID, trans->address,
                                              currentClockCycle);
    parentMemorySystem->numOnTheFlyTransactions--;
}

bool MemoryController::addBarrier()
{
    if (transactionQueue.size())
    {
        transactionQueue.back()->tag += "BAR";
        return true;
    }
    return false;
}

// gives the memory controller a handle on the rank objects
void MemoryController::attachRanks(vector<Rank*>* ranks)
{
    this->ranks = ranks;
    if(!is_salp_)    commandQueue.ranks = ranks;
    else commandQueue_SUB.ranks = ranks;
}

void MemoryController::setBankStatesRW(size_t ra, size_t ba, uint64_t RdCycle, uint64_t WrCycle)
{
    bankStates[ra][ba].nextRead = max(bankStates[ra][ba].nextRead, currentClockCycle + RdCycle);
    bankStates[ra][ba].nextWrite = max(bankStates[ra][ba].nextWrite, currentClockCycle + WrCycle);
}

void MemoryController::setBankStatesRW(size_t ra, size_t ba, size_t sa, uint64_t RdCycle, uint64_t WrCycle)
{
    bankStates_SUB[ra][ba*4+sa].nextRead = max(bankStates_SUB[ra][ba*4+sa].nextRead, currentClockCycle+RdCycle);
    bankStates_SUB[ra][ba*4+sa].nextWrite = max(bankStates_SUB[ra][ba*4+sa].nextWrite, currentClockCycle + WrCycle);
}

void MemoryController::setBankStates(size_t rank, size_t bank, CurrentBankState currentBankState,
                                     BusPacketType lastCommand, uint64_t stateChangeCountdown,
                                     uint64_t nextActivate)
{
    bankStates[rank][bank].currentBankState = currentBankState;
    bankStates[rank][bank].lastCommand = lastCommand;
    if (stateChangeCountdown != 0)
        bankStates[rank][bank].stateChangeCountdown = stateChangeCountdown;
    bankStates[rank][bank].nextActivate = nextActivate;
}

void MemoryController::setBankStates(size_t rank, size_t bank, size_t sub, CurrentBankState currentBankState,
                                     BusPacketType lastCommand, uint64_t stateChangeCountdown,
                                     uint64_t nextActivate)
{
    bankStates_SUB[rank][bank*4+sub].currentBankState = currentBankState;
    bankStates_SUB[rank][bank*4+sub].lastCommand = lastCommand;
    if (stateChangeCountdown != 0)
        bankStates_SUB[rank][bank*4+sub].stateChangeCountdown = stateChangeCountdown;
    bankStates_SUB[rank][bank*4+sub].nextActivate = nextActivate;
}

void MemoryController::updateCommandQueue(BusPacket* poppedBusPacket)
{
    if (poppedBusPacket!=nullptr && poppedBusPacket->busPacketType == WRITE)
    {   
        if(writeDataToSend.capacity() == writeDataToSend.size())
        {
            writeDataToSend.reserve(writeDataToSend.size() + 32);
            writeDataCountdown.reserve(writeDataCountdown.size() + 32);
        }
        writeDataToSend.push_back(new BusPacket(
            DATA, poppedBusPacket->physicalAddress, poppedBusPacket->column, poppedBusPacket->row,
            poppedBusPacket->rank, poppedBusPacket->bank, poppedBusPacket->data, dramsimLog));
        writeDataCountdown.push_back(config.WL);
    }
    
    // update each bank's state based on the command that was just popped
    // out of the command queue for readability's sake
    //if(poppedBusPacket == nullptr) return;
    unsigned rank = poppedBusPacket->rank;
    unsigned bank = poppedBusPacket->bank;
    unsigned sub = (poppedBusPacket->row < 0x2000)? 0 : (poppedBusPacket->row < 0x4000)? 1 : (poppedBusPacket->row < 0x6000)? 2 : 3;
    auto am = config.addrMapping;
    switch (poppedBusPacket->busPacketType)
    {
        case READ:
            if(!is_salp_)
            {
                bankStates[rank][bank].nextPrecharge = max(currentClockCycle + config.READ_TO_PRE_DELAY,
                                                       bankStates[rank][bank].nextPrecharge);
                bankStates[rank][bank].lastCommand = READ;
            }
            else{          
                bankStates_SUB[rank][4*bank + sub].lastCommand = READ;
                bankStates_SUB[rank][4*bank + sub].nextPrecharge = max(currentClockCycle + config.READ_TO_PRE_DELAY,
                                                    bankStates_SUB[rank][4*bank + sub].nextPrecharge); 
            }
            for (size_t i = 0; i < config.NUM_RANKS; i++)
            {
                for (size_t j = 0; j < config.NUM_BANKS; j++)
                {
                    if (i != poppedBusPacket->rank)
                    {
                        if (bankStates[i][j].currentBankState == RowActive)
                        {
                            if(!is_salp_)    setBankStatesRW(i, j, config.BL / 2 + config.tRTRS,
                                            config.READ_TO_WRITE_DELAY); //MAYBE ROWMISS FUNCTION
                            else{
                                for(int s = 0; s<4; s++)
                                {
                                    setBankStatesRW(i, j, s, config.BL / 2 + config.tRTRS,
                                                    config.READ_TO_WRITE_DELAY);
                                }
                            }
                        }
                    }
                    else
                    {
                        uint64_t RdCycle =
                            max((am.isSameBankgroup(j, bank) ? config.tCCDL : config.tCCDS),
                                config.BL / 2);
                        if(!is_salp_)    setBankStatesRW(i, j, RdCycle, config.READ_TO_WRITE_DELAY); 
                        else{
                            if(j == bank)
                            {
                                for(int s = 0; s<4; s++){
                                    setBankStatesRW(i, j, s, config.tCCDL, config.tRTW);
                                    //bankStates_SUB[i][j][s].nextPrecharge = max(currentClockCycle + config.READ_TO_PRE_DELAY,
                                    //                bankStates_SUB[i][j][s].nextPrecharge); 
                                }
                            }
                            else{
                                for(int s = 0; s<4; s++)    setBankStatesRW(i, j, s, RdCycle, config.READ_TO_WRITE_DELAY);
                            }
                        }
                    }
                }
            }
            totalReads++;
            break;

        case WRITE:
            if(!is_salp_)
            {
                bankStates[rank][bank].nextPrecharge =
                max(currentClockCycle + config.WRITE_TO_PRE_DELAY,
                    bankStates[rank][bank].nextPrecharge);
                bankStates[rank][bank].lastCommand = WRITE;
            }
            else{
                bankStates_SUB[rank][4*bank + sub].nextPrecharge = 
                    max(currentClockCycle + config.tWTP, 
                        bankStates_SUB[rank][4*bank + sub].nextPrecharge);
                bankStates_SUB[rank][4*bank + sub].lastCommand = WRITE;
            }
            for (size_t i = 0; i < config.NUM_RANKS; i++)
            {
                for (size_t j = 0; j < config.NUM_BANKS; j++)
                {
                    if (i != poppedBusPacket->rank)
                    {
                        if(!is_salp_)
                        {
                            if (bankStates[i][j].currentBankState == RowActive) //different rank est
                            {
                                setBankStatesRW(i, j, config.WRITE_TO_READ_DELAY_R,
                                                config.BL / 2 + config.tRTRS); 
                            }
                        }
                        else
                        {
                            for(int s = 0; s<4; s++)    
                            {
                                if(bankStates_SUB[i][j*4+s].currentBankState == RowActive)
                                    setBankStatesRW(i, j, s, config.WRITE_TO_READ_DELAY_R,
                                                                    config.BL / 2 + config.tRTRS);
                            }
                        }
                    }
                    else
                    {
                        if(!is_salp_)
                        {
                            uint64_t WrCycle =
                                max((am.isSameBankgroup(j, bank) ? config.tCCDL : config.tCCDS),
                                    config.BL / 2); //burst
                            setBankStatesRW(i, j, config.WRITE_TO_READ_DELAY_B_LONG, WrCycle);
                        }
                        else
                        {
                            for(int s = 0; s < 4; s++)
                            {
                                uint64_t WrCycle =
                                    max((am.isSameBankgroup(j, bank) ? config.tCCDL : config.tCCDS),
                                        config.BL / 2); //burst
                                uint64_t RCycle = max((am.isSameBankgroup(j, bank) ? config.WRITE_TO_READ_DELAY_B_LONG : config.WRITE_TO_READ_DELAY_B_SHORT),
                                    config.BL / 2); //burst
                                if(j == bank)
                                {
                                    setBankStatesRW(i, j, s, config.tWTR, WrCycle);
                                    //bankStates_SUB[i][j][s].nextPrecharge = max(currentClockCycle + config.READ_TO_PRE_DELAY,
                                    //                bankStates_SUB[i][j][s].nextPrecharge); 
                                }
                                else    setBankStatesRW(i, j, s, RCycle, WrCycle);
                            }
                        }
                    }
                }
            }
            totalWrites++;
            //cout<<"[MC] updatecommand for write and type is "<<poppedBusPacket->busPacketType<<" and clock is "<<currentClockCycle<<" and openrow is "<<
            //bankStates_SUB[poppedBusPacket->rank][poppedBusPacket->bank][AddrMapping::findsubarray(poppedBusPacket->row)].openRowAddress<<
            //" and bank is "<<poppedBusPacket->bank<<" and sub is "<<AddrMapping::findsubarray(poppedBusPacket->row)<<" and nextpre is "<<
            //bankStates_SUB[poppedBusPacket->rank][poppedBusPacket->bank][AddrMapping::findsubarray(poppedBusPacket->row)].nextPrecharge<<endl;
            break;

        case ACTIVATE:
            if(!is_salp_)
            {
                setBankStates(rank, bank, RowActive, ACTIVATE, 0,
                          max(currentClockCycle + config.tRC, bankStates[rank][bank].nextActivate));
                bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
                bankStates[rank][bank].nextPrecharge =
                    max(currentClockCycle + config.tRAS, bankStates[rank][bank].nextPrecharge);
                setBankStatesRW(rank, bank, (config.tRCDRD - config.AL), (config.tRCDWR - config.AL));
            }
            else{
                //bool cond = (poppedBusPacket->row!=bankStates_SUB[rank][4*bank + sub].openRowAddress);
                setBankStates(rank, bank, sub, RowActive, ACTIVATE, 0, 
                          max(currentClockCycle+config.tRC, bankStates_SUB[rank][4*bank + sub].nextActivate));
                bankStates_SUB[rank][4*bank + sub].openRowAddress = poppedBusPacket->row;
                bankStates_SUB[rank][4*bank + sub].nextPrecharge = 
                    max(currentClockCycle + config.tRAS, bankStates_SUB[rank][4*bank + sub].nextPrecharge);
                setBankStatesRW(rank, bank, sub, (config.tRCDRD - config.AL), (config.tRCDWR - config.AL));
            }
            for (size_t i = 0; i < config.NUM_BANKS; i++)
            {
                if(!is_salp_)
                {
                    if (i != poppedBusPacket->bank)
                    {
                        bankStates[rank][i].nextActivate =
                            max(currentClockCycle +
                                    (am.isSameBankgroup(i, bank) ? config.tRRDL : config.tRRDS),
                                bankStates[rank][i].nextActivate);
                    } //latency is ready but not salp logic indeed
                }
                else{
                    for(int s = 0; s <4; s++)
                    {
                        if(i != poppedBusPacket->bank || s != sub)
                        {
                            //bankStates_SUB[rank][i][s].currentBankState = Idle;
                            bankStates_SUB[rank][i*4+s].nextActivate = max(currentClockCycle + (am.isSameBankgroup(i, bank) ? config.tRRDL : config.tRRDS),
                                                                    bankStates_SUB[rank][i*4+s].nextActivate);
                        }
                    }
                }
            }
            //cout<<"[MC] updatecommand and type is "<<poppedBusPacket->busPacketType<<" and clock is "<<currentClockCycle<<" and openrow is "<<
            //bankStates_SUB[poppedBusPacket->rank][poppedBusPacket->bank][AddrMapping::findsubarray(poppedBusPacket->row)].openRowAddress<<
            //" and bank is "<<poppedBusPacket->bank<<" and sub is "<<AddrMapping::findsubarray(poppedBusPacket->row)<<endl;
            break;
        case PRECHARGE:
            if(!is_salp_)
            {
                setBankStates(rank, bank, Precharging, PRECHARGE, config.tRP,
                          max(currentClockCycle + config.tRP, bankStates[rank][bank].nextActivate));
            }
            else{
                //for(int s = 0; s < 4; s++)  //bankStates_SUB[rank][bank][s].nextPrecharge = max(currentClockCycle + config.tRP, bankStates_SUB[rank][bank][s].nextActivate);
                    setBankStates(rank, bank, sub, Precharging, PRECHARGE, config.tRP,
                          max(currentClockCycle + config.tRP, bankStates_SUB[rank][4*bank + sub].nextActivate));
            }
            break;

        case REF:
            //cout<<"[MC] updatecommand and type is "<<poppedBusPacket->busPacketType<<" and clock is "<<currentClockCycle<<" and bank is "<<bank<<" and sub is "<<sub
            //<<" and state is "<<bankStates_SUB[rank][4*bank + sub].currentBankState<<endl;
            for (size_t i = 0; i < config.NUM_BANKS; i++)
            {
                if(!is_salp_)
                {
                    setBankStates(rank, i, Refreshing, REF, config.tRFC,
                              currentClockCycle + config.tRFC);
                }  
                else{
                    for(size_t s = 0; s < 4; s++)
                        setBankStates(rank, i, s, Refreshing, REF, config.tRFC,
                                    currentClockCycle + config.tRFC);
                } 
            }

            break;
        /*case DATA:
            break;
        case SUBSEL:
            break;*/
        default:
            ERROR("== Error - Popped a command we shouldn't have of type : "
                  << poppedBusPacket->busPacketType <<" at cycle "<<currentClockCycle
                  << " and rank is "<<rank<<" and bank is "<<bank);
            exit(0);
    }

    // issue on bus and print debug
    if (DEBUG_BUS)
    {
        PRINTN(" -- MC Issuing On Command Bus : ");
        poppedBusPacket->print();
    }

    // check for collision on bus
    if (outgoingCmdPacket != NULL)
    {
        ERROR("== Error - Command Bus Collision");
        exit(-1);
    }
    outgoingCmdPacket = poppedBusPacket;
    cmdCyclesLeft = config.tCMD;
}

void MemoryController::updateTransactionQueue()
{
    //if(transactionQueue.size() < 10)    cout<<"clock is "<<currentClockCycle<<" and Transaction Queue Size: "<<transactionQueue.size()<<endl;
    auto am = config.addrMapping;
    for (size_t i = 0; i < transactionQueue.size(); i++)
    {
        // pop off top transaction from queue assuming simple scheduling at the moment
        // will eventually add policies here
        Transaction* transaction = transactionQueue[i];
        // map address to rank,bank,row,col
        unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow,
            newTransactionColumn;

        // pass these in as references so they get set by the addressMapping function
        am.addressMapping(transaction->address, newTransactionChan, newTransactionRank,
                          newTransactionBank, newTransactionRow, newTransactionColumn);
        //if(transaction->tag!="" && currentClockCycle == 77091)    cout<<"[MC] tag is "<<transaction->tag<<" and cycle is "<<currentClockCycle<<endl;
        if ((!is_salp_) && (commandQueue.hasRoomFor(1, newTransactionRank, newTransactionBank)) || 
        (is_salp_) && (commandQueue_SUB.hasRoomFor(1, newTransactionRank, newTransactionBank, AddrMapping::findsubarray(newTransactionRow))))
        {
            if (DEBUG_ADDR_MAP)
            {
                PRINTN("== New Transaction - Mapping Address [0x" << hex << transaction->address
                                                                  << dec << "]");
                if (transaction->transactionType == DATA_READ)
                    PRINT(" (Read)");
                else
                    PRINT(" (Write)");
                PRINT("  Rank : " << newTransactionRank);
                PRINT("  Bank : " << newTransactionBank);
                PRINT("  Row  : " << newTransactionRow);
                PRINT("  Col  : " << newTransactionColumn);
            }

            // now that we know there is room in the command queue, we can remove from
            // the transaction queue
            transactionQueue.erase(transactionQueue.begin() + i);
            // create read or write command and enqueue it 
            if((transaction->transactionType==DATA_READ || transaction->transactionType==DATA_WRITE))
            {          
                BusPacket* command;
                BusPacketType bpType = transaction->getBusPacketType();
                //string tg = "";
                //cout<<"[MC] transaction type is "<<transaction->transactionType<<" and bpType is "<<bpType<<" and data is "<<transaction->data<<endl;
                command = new BusPacket(bpType, transaction->address, newTransactionColumn,
                                        newTransactionRow, newTransactionRank, newTransactionBank,
                                        transaction->data, dramsimLog);//, tg);
                //if(!transaction->tag.empty() && transaction != nullptr)
                //{
                    /*if(currentClockCycle > 67700 && (*ranks)[0]->mode_ == dramMode::SB)  
                    {
                        cout<<"[MC] currentcycle is "<<currentClockCycle<<" and addr is "<<transaction->address<<endl;
                    }*/
                command->tag = transaction->tag;
                //}
                if(!is_salp_)
                {   
                    commandQueue.enqueue(command); 
                } 
                else
                {
                    commandQueue_SUB.enqueue_sub(command);
                }
                // If we have a read, save the transaction so when the data comes back
                // in a bus packet, we can staple it back into a transaction and return it
                if (transaction->transactionType == DATA_READ && transaction!= nullptr)
                {
                    if(pendingReadTransactions.capacity() == pendingReadTransactions.size())
                    {
                        pendingReadTransactions.reserve(pendingReadTransactions.size() + 32);
                    }
                    pendingReadTransactions.push_back(transaction);  
                }
                else
                {
                    //transaction = nullptr;    
                    //delete transaction;
                }
                /* only allow one transaction to be scheduled per cycle -- this
                * should
                * be a reasonable assumption considering how much logic would be
                * required to schedule multiple entries per cycle (parallel data
                * lines, switching logic, decision logic)
                */
                break;
            }
            
            else
            {
                transaction = nullptr;
                delete transaction;
                break;
            }
        }
        else // no room, do nothing this cycle
        {
            // PRINT( "== Warning - No room in command queue" << endl;
            //transaction = nullptr;
            //delete transaction;
        }
    }
}
//need print for subarray logic
void MemoryController::printDebugOnUpate()
{
    if (DEBUG_TRANS_Q)
    {
        PRINT("== Printing transaction queue");
        for (size_t i = 0; i < transactionQueue.size(); i++)
            PRINTN("  " << i << "] " << *transactionQueue[i]);
    }

    if (DEBUG_BANKSTATE)
    {
        PRINT("== Printing bank states (According to MC)");
        for (size_t i = 0; i < config.NUM_RANKS; i++)
        {
            for (size_t j = 0; j < config.NUM_BANKS; j++)
            {
                if (bankStates[i][j].currentBankState == RowActive)
                    PRINTN("[" << bankStates[i][j].openRowAddress << "] ");
                else if (bankStates[i][j].currentBankState == Idle)
                    PRINTN("[idle] ");
                else if (bankStates[i][j].currentBankState == Precharging)
                    PRINTN("[pre] ");
                else if (bankStates[i][j].currentBankState == Refreshing)
                    PRINTN("[ref] ");
                else if (bankStates[i][j].currentBankState == PowerDown)
                    PRINTN("[lowp] ");
            }
            PRINT(""); 
        }
    }

    if (DEBUG_CMD_Q)
    {
        if(!is_salp_)    commandQueue.print();
        else    commandQueue_SUB.print();
    }
}

void MemoryController::updateBankState()
{
    for (int i = 0; i < config.NUM_RANKS; i++)
    {
        for (int j = 0; j < config.NUM_BANKS; j++)
        {
            if(!is_salp_)
            {
                if (bankStates[i][j].stateChangeCountdown > 0)
                {
                    // decrement counters
                    bankStates[i][j].stateChangeCountdown--;

                    // if counter has reached 0, change state
                    if (bankStates[i][j].stateChangeCountdown == 0)
                    {
                        switch (bankStates[i][j].lastCommand)
                        {
                            case REF:
                            case RFCSB:
                            case PRECHARGE:
                                bankStates[i][j].currentBankState = Idle;
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
            else
            {
                for(int s = 0; s<4; s++)
                {
                    if (bankStates_SUB[i][j*4+s].stateChangeCountdown > 0)
                    {
                        // decrement counters
                        bankStates_SUB[i][j*4+s].stateChangeCountdown--;

                        // if counter has reached 0, change state
                        if (bankStates_SUB[i][j*4+s].stateChangeCountdown == 0)
                        {
                            switch (bankStates_SUB[i][j*4+s].lastCommand)
                            {
                                case REF:
                                case RFCSB:
                                case PRECHARGE:
                                    bankStates_SUB[i][j*4+s].currentBankState = Idle;
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
            }
        }
    }
}

void MemoryController::updateRefresh()
{
    //cout<<"[MC] update refresh and clock is "<<currentClockCycle<<" and refreshcountdown is "<<powerDown[0]<<endl;
    if (refreshCountdown[refreshRank] == 0)
    {
        if(!is_salp_)   commandQueue.needRefresh(refreshRank);
        else    commandQueue_SUB.needRefresh(refreshRank);
        (*ranks)[refreshRank]->refreshWaiting = true;
        // PRINT("REF request rank" << refreshRank << " @" << currentClockCycle);
        refreshCountdown[refreshRank] = config.tREFI / config.tCK;
        refreshRank++;
        if (refreshRank == config.NUM_RANKS)
            refreshRank = 0;
    }
    // if a rank is powered down, make sure we power it up in time for a refresh
    else if (refreshCountdown[refreshRank] <= config.tXP)
    {
        //if(powerDown[refreshRank])  (*ranks)[refreshRank]->refreshWaiting = true;
    }
}

void MemoryController::update()
{
    //if((*ranks)[0]->getChanId() == 1)   cout<<"[MC] update and clock is "<<currentClockCycle<<" and state is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<endl;
    updateBankState();
    //if((*ranks)[0]->getChanId() == 1)   cout<<"[MC] update and clock is "<<currentClockCycle<<" and state is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<endl;
    // check for outgoing command packets and handle countdowns
    if (outgoingCmdPacket != NULL)
    {
        cmdCyclesLeft--;
        if (cmdCyclesLeft == 0)  // packet is ready to be received by rank
        {
            //cout<<"[MC] cmdCyclesLeft is 0 and clock is "<<currentClockCycle<<" and bank is "<<outgoingCmdPacket->bank<<" and row is "<<outgoingCmdPacket->row<<endl;
            //(*ranks)[outgoingCmdPacket->rank]->receiveFromBus(outgoingCmdPacket);
            outgoingCmdPacket = NULL;
        }
    }
    //if((*ranks)[0]->getChanId() == 1)   cout<<"[MC] update and clock is "<<currentClockCycle<<" and state is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<endl;
    if (outgoingDataPacket != NULL)
    {
        dataCyclesLeft--;
        if (dataCyclesLeft == 0)
        {
            // inform upper levels that a write is done
            if (parentMemorySystem->WriteDataDone != NULL)
            {
                (*parentMemorySystem->WriteDataDone)(parentMemorySystem->systemID,
                                                     outgoingDataPacket->physicalAddress,
                                                     currentClockCycle);
            }
            parentMemorySystem->numOnTheFlyTransactions--;
            (*ranks)[outgoingDataPacket->rank]->receiveFromBus(outgoingDataPacket);
            outgoingDataPacket = NULL;
        }
    }
    // if any outstanding write data needs to be sent
    // and the appropriate amount of time has passed (WL)
    // then send data on bus
    //
    // write data held in fifo vector along with countdowns
    //if((*ranks)[0]->getChanId() == 1)   cout<<"[MC] update and clock is "<<currentClockCycle<<" and state is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<endl;
    if (writeDataCountdown.size() > 0)
    {
        for (size_t i = 0; i < writeDataCountdown.size(); i++) writeDataCountdown[i]--;

        if (writeDataCountdown[0] == 0)
        {
            // send to bus and print debug stuff
            if (DEBUG_BUS)
            {
                PRINTN(" -- MC Issuing On Data Bus    : ");
                writeDataToSend[0]->print();
            }

            // queue up the packet to be sent
            if (outgoingDataPacket != NULL)
            {
                ERROR("== Error - Data Bus Collision");
                exit(-1);
            }

            outgoingDataPacket = writeDataToSend[0];
            dataCyclesLeft = config.BL / 2;   //which is the bitline 

            totalTransactions++;

            writeDataCountdown.erase(writeDataCountdown.begin());
            writeDataToSend.erase(writeDataToSend.begin());
            //cout<<"[MC] writeDataCountdown is 0 and clock is "<<currentClockCycle<<" and bank is "<<outgoingDataPacket->bank<<" and row is "
            //<<outgoingDataPacket->row<<endl;
        }
    }
    // if its time for a refresh issue a refresh
    // else pop from command queue if it's not empty
    updateRefresh();
    // pass a pointer to a poppedBusPacket
    // function returns true if there is something valid in poppedBusPacket
    if(!is_salp_)
    {
        if (commandQueue.pop(&poppedBusPacket))
        {
            updateCommandQueue(poppedBusPacket);
        }
    }
    else{
        if(commandQueue_SUB.pop_sub(&poppedBusPacket))
        {
            //cout<<"[MC] update and clock is "<<currentClockCycle<<" and type is "<<poppedBusPacket->busPacketType<<" and bank is "<<poppedBusPacket->bank<<" and sub is "<<AddrMapping::findsubarray(poppedBusPacket->row)<<endl;
            updateCommandQueue(poppedBusPacket);
        }
    }
    updateTransactionQueue();
    if (returnTransaction.size() > 0)
    {
        if (DEBUG_BUS)
            PRINTN(" -- MC Issuing to CPU bus : " << *returnTransaction[0]);
        totalTransactions++;

        bool foundMatch = false;
        // find the pending read transaction to calculate latency
        for (size_t i = 0; i < pendingReadTransactions.size(); i++)
        {
            if(pendingReadTransactions[i] != nullptr && returnTransaction[0] != nullptr)
            {
                if (pendingReadTransactions[i]->address == returnTransaction[0]->address)
                {
                    unsigned chan, rank, bank, row, col, sub;
                    config.addrMapping.addressMapping(returnTransaction[0]->address, chan, rank, bank,
                                                    row, col);
                    sub = (row < 0x2000)? 0 : (row < 0x4000)? 1 : (row < 0x6000)? 2 : 3;
                    //if(!is_salp_)   memoryContStats->insertHistogram(currentClockCycle - pendingReadTransactions[i]->timeAdded, rank, bank);
                    //else    memoryContStats->insertHistogram(currentClockCycle - pendingReadTransactions[i]->timeAdded, rank, bank, sub);
                    // FIXME. Is it correct?
                    // memcpy(pendingReadTransactions[i]->data,
                    // returnTransaction[0]->data, config.BL * (JEDEC_DATA_BUS_BITS / 8));
                    returnReadData(pendingReadTransactions[i]);
                    pendingReadTransactions[i] = nullptr;
                    delete pendingReadTransactions[i];
                    pendingReadTransactions.erase(pendingReadTransactions.begin() + i);
                    foundMatch = true;
                    break;
                }
            }
            else
            {
                //delete pendingReadTransactions[i];
                //pendingReadTransactions.erase(pendingReadTransactions.begin() + i);
                //foundMatch = true;
                //break;
            }
        }
        if (!foundMatch)
        {
            ERROR("Can't find a matching transaction for 0x" << hex << returnTransaction[0]->address
                                                             << dec);
            abort();
        }
        delete returnTransaction[0];
        returnTransaction.erase(returnTransaction.begin());
    }
    // decrement refresh counters
    for (size_t i = 0; i < config.NUM_RANKS; i++) refreshCountdown[i]--;
    for (size_t i = 0; i < config.NUM_BANKS; i++) refreshCountdownBank[i]--;

    // print debug
    printDebugOnUpate();
    if(is_salp_) commandQueue_SUB.step();
    else    commandQueue.step();
}

bool MemoryController::WillAcceptTransaction()
{
    return transactionQueue.size() < getConfigParam(UINT, "TRANS_QUEUE_DEPTH");
}

// allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction* trans)
{
    auto am = config.addrMapping;
    unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow,
            newTransactionColumn;

        // pass these in as references so they get set by the addressMapping function
    am.addressMapping(trans->address, newTransactionChan, newTransactionRank,
        newTransactionBank, newTransactionRow, newTransactionColumn);
    if (WillAcceptTransaction())
    {
        parentMemorySystem->numOnTheFlyTransactions++;
        trans->timeAdded = currentClockCycle;
        transactionQueue.push_back(trans);
        //cout<<" [MC] addTransaction and clock is "<<currentClockCycle<<" and row is "<<trans->address<<endl;
        return true;
    }
    else
    {
        return false;
    }
}

void MemoryController::resetStats()
{
    memoryContStats->resetStats();
}

// prints statistics at the end of an epoch or simulation
void MemoryController::printStats(bool finalStats)
{
    memoryContStats->printStats(finalStats, parentMemorySystem->systemID, currentClockCycle);
}

MemoryController::~MemoryController()
{
    // ERROR("MEMORY CONTROLLER DESTRUCTOR");
    // abort();
    for (size_t i = 0; i < pendingReadTransactions.size(); i++) delete pendingReadTransactions[i];
    for (size_t i = 0; i < returnTransaction.size(); i++) delete returnTransaction[i];
    delete memoryContStats;
}

// inserts a latency into the latency histogram
void MemoryControllerStats::insertHistogram(unsigned latencyValue, unsigned rank, unsigned bank)
{
    totalEpochLatency[SEQUENTIAL(rank, bank)] += latencyValue;
    // poor man's way to bin things.
    unsigned histogram_bin_size = getConfigParam(UINT, "HISTOGRAM_BIN_SIZE");
    latencies[(latencyValue / histogram_bin_size) * histogram_bin_size]+=1;
}

void MemoryControllerStats::insertHistogram(unsigned latencyValue, unsigned rank, unsigned bank, unsigned sub)
{
    totalEpochLatency[SEQUENTIAL_SUB(rank, bank, sub)] += latencyValue;
    // poor man's way to bin things.
    unsigned histogram_bin_size = getConfigParam(UINT, "HISTOGRAM_BIN_SIZE");
    auto key = (latencyValue / histogram_bin_size) * histogram_bin_size;
    latencies[key]++;
}
void MemoryControllerStats::printStats(bool finalStats, unsigned myChannel,
                                       uint64_t currentClockCycle)
{
    // if we are not at the end of the epoch, make sure to adjust for the actual number of
    // cycles elapsed
    uint64_t cyclesElapsed = (currentClockCycle % config.EPOCH_LENGTH == 0)
                                 ? config.EPOCH_LENGTH
                                 : currentClockCycle % config.EPOCH_LENGTH;
    unsigned bytesPerTransaction = (getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") * config.BL) / 8;
    uint64_t totalBytesTransferred = totalTransactions * bytesPerTransaction;
    double secondsThisEpoch = (double)cyclesElapsed * config.tCK * 1E-9;

    // only per rank
    vector<double> backgroundPower = vector<double>(config.NUM_RANKS, 0.0);
    vector<double> burstPower = vector<double>(config.NUM_RANKS, 0.0);
    vector<double> refreshPower = vector<double>(config.NUM_RANKS, 0.0);
    vector<double> actprePower = vector<double>(config.NUM_RANKS, 0.0);
    vector<double> averagePower = vector<double>(config.NUM_RANKS, 0.0);
    vector<double> aluPIMPower = vector<double>(config.NUM_RANKS, 0.0);

    // per bank variables
    vector<double> averageLatency = vector<double>(config.NUM_RANKS * config.NUM_BANKS*4, 0.0);
    vector<double> bandwidth = vector<double>(config.NUM_RANKS * config.NUM_BANKS*4, 0.0);

    for (size_t i = 0; i < config.NUM_RANKS; i++)
    {
        for (size_t j = 0; j < config.NUM_BANKS; j++)
        {
            if(is_salp_)
            {
                for(size_t s = 0; s < 4; s++)
                {
                    bandwidth[SEQUENTIAL_SUB(i, j, s)] = (((double)(totalReadsPerBank[SEQUENTIAL_SUB(i, j, s)] +
                                                            totalWritesPerBank[SEQUENTIAL_SUB(i, j, s)]) *
                                                    (double)bytesPerTransaction) /
                                                    (1024.0 * 1024.0 * 1024.0)) /
                                                    secondsThisEpoch;
                    averageLatency[SEQUENTIAL_SUB(i, j, s)] = ((float)totalEpochLatency[SEQUENTIAL_SUB(i, j, s)] /
                                                        (float)(totalReadsPerBank[SEQUENTIAL_SUB(i, j, s)])) *
                                                        config.tCK;
                    totalBandwidth += bandwidth[SEQUENTIAL_SUB(i, j, s)];
                    totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL_SUB(i, j, s)];
                    totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL_SUB(i, j, s)];
                    totalActivatesPerRank[i] += totalActivatesPerBank[SEQUENTIAL_SUB(i, j, s)];
                
                }
            }
            else{
                bandwidth[SEQUENTIAL(i, j)] = (((double)(totalReadsPerBank[SEQUENTIAL(i, j)] +
                                                        totalWritesPerBank[SEQUENTIAL(i, j)]) *
                                                (double)bytesPerTransaction) /
                                            (1024.0 * 1024.0 * 1024.0)) /
                                            secondsThisEpoch;
                averageLatency[SEQUENTIAL(i, j)] = ((float)totalEpochLatency[SEQUENTIAL(i, j)] /
                                                    (float)(totalReadsPerBank[SEQUENTIAL(i, j)])) *
                                                config.tCK;
                totalBandwidth += bandwidth[SEQUENTIAL(i, j)];
                totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL(i, j)];
                totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i, j)];
                totalActivatesPerRank[i] += totalActivatesPerBank[SEQUENTIAL(i, j)];
            }
        }
    }
    LOG_OUTPUT ? dramsimLog.setf(ios::fixed, ios::floatfield)
               : cout.setf(ios::fixed, ios::floatfield);

    PRINTC(PRINT_CHAN_STAT, " =======================================================");
    PRINTC(PRINT_CHAN_STAT, " ============== Printing Statistics [id:"
                                << parentMemorySystem->systemID << "]==============");
    PRINTNC(PRINT_CHAN_STAT, "   Total Return Transactions : " << totalTransactions);
    PRINTC(PRINT_CHAN_STAT, " (" << totalBytesTransferred << " bytes) aggregate average bandwidth "
                                 << totalBandwidth << "GB/s");
    PRINTC(PRINT_CHAN_STAT, "   Total Refreshes : " << totalRefreshes);

    double totalAggregateBandwidth = 0.0;
    for (size_t r = 0; r < config.NUM_RANKS; r++)
    {
        PRINTC(PRINT_CHAN_STAT, "      -Rank   " << r << " : ");
        PRINTNC(PRINT_CHAN_STAT, "        -Reads  : " << totalReadsPerRank[r]);
        PRINTC(PRINT_CHAN_STAT, " (" << totalReadsPerRank[r] * bytesPerTransaction << " bytes)");
        PRINTNC(PRINT_CHAN_STAT, "        -Writes : " << totalWritesPerRank[r]);
        PRINTC(PRINT_CHAN_STAT, " (" << totalWritesPerRank[r] * bytesPerTransaction << " bytes)");
        PRINTC(PRINT_CHAN_STAT, "        -Activates  : " << totalActivatesPerRank[r]);
        // PRINT( " ("<<totalReadsPerRank[r] * bytesPerTransaction<<" bytes)");

        for (size_t j = 0; j < config.NUM_BANKS; j++)
        {
            PRINTC(PRINT_CHAN_STAT, "        -Bandwidth / Latency  (Bank "
                                        << j << "): " << bandwidth[SEQUENTIAL(r, j)] << " GB/s\t\t"
                                        << averageLatency[SEQUENTIAL(r, j)] << " ns");
        }

        burstPower[r] = ((double)burstEnergy[r] / (double)(cyclesElapsed)) / 1000.0;  // (pw)
        actprePower[r] = ((double)actpreEnergy[r] / (double)(cyclesElapsed)) / 1000.0;
        aluPIMPower[r] = ((double)aluPIMEnergy[r] / (double)cyclesElapsed) / 1000.0;
        averagePower[r] = burstPower[r] + actprePower[r] + aluPIMPower[r];

        if ((*parentMemorySystem->ReportPower) != NULL)
            (*parentMemorySystem->ReportPower)(backgroundPower[r], burstPower[r], refreshPower[r],
                                               actprePower[r]);

        PRINTC(PRINT_CHAN_STAT, " == Power Data for Rank        " << r);
        PRINTC(PRINT_CHAN_STAT, "   Average Power (watts)     : " << averagePower[r]);
        PRINTC(PRINT_CHAN_STAT, "     -Act/Pre    (watts)     : " << actprePower[r]);
        PRINTC(PRINT_CHAN_STAT, "     -Burst      (watts)     : " << burstPower[r]);
        PRINTC(PRINT_CHAN_STAT, "     -AluPIM     (watts)     : " << aluPIMPower[r]);

        if (VIS_FILE_OUTPUT)
        {
            // write the vis file output
            csvOut << CSVWriter::IndexedName("ACT_PRE_Power", myChannel, r) << actprePower[r];
            csvOut << CSVWriter::IndexedName("Burst_Power", myChannel, r) << burstPower[r];
            double totalRankBandwidth = 0.0;
            for (size_t b = 0; b < config.NUM_BANKS; b++)
            {
                csvOut << CSVWriter::IndexedName("Bandwidth", myChannel, r, b)
                       << bandwidth[SEQUENTIAL(r, b)];
                totalRankBandwidth += bandwidth[SEQUENTIAL(r, b)];
                totalAggregateBandwidth += bandwidth[SEQUENTIAL(r, b)];
                csvOut << CSVWriter::IndexedName("Average_Latency", myChannel, r, b)
                       << averageLatency[SEQUENTIAL(r, b)];
            }
            csvOut << CSVWriter::IndexedName("Rank_Aggregate_Bandwidth", myChannel, r)
                   << totalRankBandwidth;
            csvOut << CSVWriter::IndexedName("Rank_Average_Bandwidth", myChannel, r)
                   << totalRankBandwidth / config.NUM_RANKS;
        }
    }

    if (VIS_FILE_OUTPUT)
    {
        csvOut << CSVWriter::IndexedName("Aggregate_Bandwidth", myChannel)
               << totalAggregateBandwidth;
        csvOut << CSVWriter::IndexedName("Average_Bandwidth", myChannel)
               << totalAggregateBandwidth / (config.NUM_RANKS * config.NUM_BANKS);
    }

    // only print the latency histogram at the end of the simulation since it
    // clogs the output too much to print every epoch
    if (finalStats)
    {
        PRINTC(PRINT_CHAN_STAT, " ---  Latency list (" << latencies.size() << ")");
        PRINTC(PRINT_CHAN_STAT, "       [lat] : #");
        if (VIS_FILE_OUTPUT)
            csvOut.getOutputStream() << "!!HISTOGRAM_DATA" << endl;

        map<unsigned, unsigned>::iterator it;
        for (it = latencies.begin(); it != latencies.end(); it++)
        {
            auto histogram_bin_size = getConfigParam(UINT, "HISTOGRAM_BIN_SIZE");
            PRINTC(PRINT_CHAN_STAT, "       [" << it->first << "-"
                                               << it->first + (histogram_bin_size - 1)
                                               << "] : " << it->second);
            if (VIS_FILE_OUTPUT)
                csvOut.getOutputStream() << it->first << "=" << it->second << endl;
        }
        if (currentClockCycle > 0 && currentClockCycle % config.EPOCH_LENGTH == 0)
        {
            PRINTC(PRINT_CHAN_STAT, " --- Grand Total Bank usage list");
            for (size_t i = 0; i < config.NUM_RANKS; i++)
            {
                PRINTC(PRINT_CHAN_STAT, "Rank " << i << ":");
                for (size_t j = 0; j < config.NUM_BANKS; j++)
                {
                    if(!is_salp_)    PRINTC(PRINT_CHAN_STAT,
                           "  b" << j << ": " << grandTotalBankAccesses[SEQUENTIAL(i, j)]);
                    else  
                    {
                        for(size_t s = 0; s < 4; s++)   
                        {
                            PRINTC(PRINT_CHAN_STAT,
                           "  b" << j << ": " << grandTotalBankAccesses[SEQUENTIAL_SUB(i, j, s)]);
                        }
                    }
                }
            }
        }
    }

    PRINTC(PRINT_CHAN_STAT, endl << " == Pending Transactions : " << pendingReadTransactions.size()
                                 << " (" << currentClockCycle << ")==");

    if (LOG_OUTPUT)
        dramsimLog.flush();

    // do in the  mprintstate
    // resetStats();
}

void MemoryControllerStats::resetStats()
{
    totalRefreshes = 0;
    totalBandwidth = 0;
    for (size_t i = 0; i < config.NUM_RANKS; i++)
    {
        for (size_t j = 0; j < config.NUM_BANKS; j++)
        {
            // XXX: this means the bank list won't be printed for partial epochs
            if(!is_salp_)
            {
                grandTotalBankAccesses[SEQUENTIAL(i, j)] +=
                    (totalReadsPerBank[SEQUENTIAL(i, j)] + totalWritesPerBank[SEQUENTIAL(i, j)]);
                totalReadsPerBank[SEQUENTIAL(i, j)] = 0;
                totalWritesPerBank[SEQUENTIAL(i, j)] = 0;
                totalActivatesPerBank[SEQUENTIAL(i, j)] = 0;
                totalEpochLatency[SEQUENTIAL(i, j)] = 0;
            }
            else
            {
                for(size_t s = 0; s < 4; s++)
                {
                    /*grandTotalBankAccesses[SEQUENTIAL_SUB(i, j, s)] +=
                        (totalReadsPerBank[SEQUENTIAL_SUB(i, j, s)] + totalWritesPerBank[SEQUENTIAL_SUB(i, j, s)]);
                    totalReadsPerBank[SEQUENTIAL_SUB(i, j, s)] = 0;
                    totalWritesPerBank[SEQUENTIAL_SUB(i, j, s)] = 0;
                    totalActivatesPerBank[SEQUENTIAL_SUB(i, j, s)] = 0;
                    //totalEpochLatency[SEQUENTIAL_SUB(i, j, s)] = 0;*/
                }
            
            }
        }
        burstEnergy[i] = 0;
        actpreEnergy[i] = 0;
        refreshEnergy[i] = 0;
        backgroundEnergy[i] = 0;
        aluPIMEnergy[i] = 0;
        totalReadsPerRank[i] = 0;
        totalWritesPerRank[i] = 0;
        totalActivatesPerRank[i] = 0;
    }
}
