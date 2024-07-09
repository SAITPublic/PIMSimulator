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
#include "Rank.h"

using namespace std;
using namespace DRAMSim;

Rank::Rank(ostream& simLog, Configuration& configuration)
    : chanId(-1),
      rankId(-1),
      dramsimLog(simLog),
      isPowerDown(false),
      refreshWaiting(false),
      readReturnCountdown(0),
      banks(getConfigParam(UINT, "NUM_BANKS"), Bank(simLog)),
      bankStates(getConfigParam(UINT, "NUM_BANKS"), BankState(simLog)),
      config(configuration),
      outgoingDataPacket(NULL),
      dataCyclesLeft(0),
      mode_(dramMode::SB)
{
    memoryController = NULL;
    currentClockCycle = 0;
    abmr1Even_ = abmr1Odd_ = abmr2Even_ = abmr2Odd_ = sbmr1_ = sbmr2_ = false;

    pimRank = new PIMRank(dramsimLog, config, false);
    pimRank->attachRank(this);
}

Rank::Rank(ostream& simLog, Configuration& configuration, bool is_salp)
    : chanId(-1),
      rankId(-1),
      dramsimLog(simLog),
      isPowerDown(false),
      refreshWaiting(false),
      readReturnCountdown(0),
      banks(getConfigParam(UINT, "NUM_BANKS"), Bank(simLog)),
      bankStates(getConfigParam(UINT, "NUM_BANKS"), BankState(simLog)),
      banks_sub(getConfigParam(UINT, "NUM_BANKS")*4, Bank(simLog)),
      bankStates_SUB(getConfigParam(UINT, "NUM_BANKS")*4, BankState(simLog)),
      config(configuration),
      outgoingDataPacket(NULL),
      dataCyclesLeft(0),
      mode_(dramMode::SB),
      is_salp_(is_salp)
{
    memoryController = NULL;
    currentClockCycle = 0;
    abmr1Even_ = abmr1Odd_ = abmr2Even_ = abmr2Odd_ = sbmr1_ = sbmr2_ = false;

    readReturnCountdown.clear();
    readReturnCountdown.reserve(32);
    //banks_sub = vector<vector<Bank>>(16*4);
    //bankStates_SUB = vector<vector<BankState>>(16, vector<BankState>(4, dramsimLog));
    pimRank = new PIMRank(dramsimLog, config, is_salp_);
    pimRank->attachRank(this);
}

void Rank::setChanId(int id)
{
    this->chanId = id;
}

void Rank::setRankId(int id)
{
    this->rankId = id;
}

int Rank::getChanId() const
{
    return this->chanId;
}

int Rank::getRankId() const
{
    return this->rankId;
}

// attachMemoryController() must be called before any other Rank functions are called
void Rank::attachMemoryController(MemoryController* mc)
{
    this->memoryController = mc;
}

Rank::~Rank()
{
    for (size_t i = 0; i < readReturnPacket.size(); i++) delete readReturnPacket[i];

    readReturnPacket.clear();
    delete outgoingDataPacket;
}

void Rank::receiveFromBus(BusPacket* packet) //outgoingcmdpacket -->comes from poppedbuspacket
{
    //cout<<"[receiveFromBus] packettype"<<packet->busPacketType<<" and currentcycle is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and row is "<<packet->row<<endl;
    if (DEBUG_BUS)
    {
        PRINTN(" -- R" << getChanId() << " Receiving On Bus    : ");
        packet->print();
    }
    if (VERIFICATION_OUTPUT)
    {
        packet->print(currentClockCycle, false);
    }
    if (!(packet->row & 1 << 12))
    {
        check(packet);
        updateState(packet);
    }
    execute(packet);
}

void Rank::checkBank(BusPacketType type, int bank, int row)
{
    //if(bank==4) cout<<"[rank]:check and mode_ is sb and cycle is "<<currentClockCycle<<" and bank is " <<bank<<" and curstate is "<<bankStates[bank].currentBankState<<endl;
    switch (type)
    {
        case READ:
            if (bankStates[bank].currentBankState != RowActive ||
                currentClockCycle < bankStates[bank].nextRead ||
                row != bankStates[bank].openRowAddress)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a READ when not allowed @ "
                                       << currentClockCycle << " and nextRead: " << bankStates[bank].nextRead);
                exit(-1);
            }
            break;
        case WRITE:
            if (bankStates[bank].currentBankState != RowActive ||
                currentClockCycle < bankStates[bank].nextWrite ||
                row != bankStates[bank].openRowAddress)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a WRITE when not allowed @ "
                                       << currentClockCycle);
                bankStates[bank].print();
                exit(-1);
            }
            break;

        case ACTIVATE:
            if (bankStates[bank].currentBankState != Idle ||
                currentClockCycle < bankStates[bank].nextActivate)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a ACT when not allowed @ "
                                       << currentClockCycle);
                bankStates[bank].print();
                exit(-1);
            }
            break;

        case PRECHARGE:
            if (bankStates[bank].currentBankState != RowActive ||
                currentClockCycle < bankStates[bank].nextPrecharge)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a PRE when not allowed @ "
                                       << currentClockCycle << " and nextPrecharge: " << bankStates[bank].nextPrecharge
                                       << " and state: " << bankStates[bank].currentBankState);
                bankStates[bank].print();
                exit(-1);
            }
            break;
        case DATA:
            break;
        case SUBSEL:
            break;
        default:
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
            exit(-1);
            break;
    }
}

void Rank::checkBank(BusPacketType type, int bank, int sub, int row)
{
    //if(bank==4 && sub==3)   cout<<"[rank]:check and cycle is "<<currentClockCycle<<" and chan is "<<chanId<<" and bank is " <<bank<<" and sub is "<<sub<<" and curstate is "<<bankStates_SUB[4*bank + sub].currentBankState<<endl;
    switch (type)
    {
        case READ:
            if (bankStates_SUB[4*bank + sub].currentBankState != RowActive ||
                currentClockCycle < bankStates_SUB[4*bank + sub].nextRead ||
                row != bankStates_SUB[4*bank + sub].openRowAddress)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a READ when not allowed @ "
                                       << currentClockCycle<< " and nextRead: "<<bankStates_SUB[4*bank + sub].nextRead
                                       <<" and state: "<<bankStates_SUB[4*bank + sub].currentBankState
                                       <<" and openRowAddress: "<<bankStates_SUB[4*bank + sub].openRowAddress
                                       <<" and row: "<<row << " and sub is "<<sub);
                exit(-1);
            }
            break;
        case WRITE:
            if (bankStates_SUB[4*bank + sub].currentBankState != RowActive ||
                currentClockCycle < bankStates_SUB[4*bank + sub].nextWrite ||
                row != bankStates_SUB[4*bank + sub].openRowAddress)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a WRITE when not allowed @ "
                                       << currentClockCycle << " and nextWrite: "<<bankStates_SUB[4*bank + sub].nextWrite
                                       <<" and state: "<<bankStates_SUB[4*bank + sub].currentBankState
                                       <<" and openRowAddress: "<<bankStates_SUB[4*bank + sub].openRowAddress
                                       <<" and row: "<<row);
                bankStates_SUB[4*bank + sub].print();
                exit(-1);
            }
            break;

        case ACTIVATE:
            if (bankStates_SUB[4*bank + sub].currentBankState != Idle ||
                currentClockCycle < bankStates_SUB[4*bank + sub].nextActivate)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a ACT when not allowed @ "
                                       << currentClockCycle << " nextActivate: " << bankStates_SUB[4*bank + sub].nextActivate
                                       << " and state: " << bankStates_SUB[4*bank + sub].currentBankState);
                bankStates_SUB[4*bank + sub].print();
                exit(-1);
            }
            break;

        case PRECHARGE:
            if (bankStates_SUB[4*bank + sub].currentBankState != RowActive ||
                currentClockCycle < bankStates_SUB[4*bank + sub].nextPrecharge)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a PRE when not allowed @ "
                                       << currentClockCycle << " nextPrecharge: " << bankStates_SUB[4*bank + sub].nextPrecharge);
                exit(-1);
            }
            break;
        case DATA:
            break;
        case SUBSEL:
            break;
        default:
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
            exit(-1);
            break;
    }
}
void Rank::check(BusPacket* packet)
{
    if (packet->busPacketType == REF) //referesh mode
    {
        for (size_t i = 0; i < config.NUM_BANKS; i++)
        {
            if (bankStates[i].currentBankState != Idle)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId()
                                       << " received a REF when not allowed");
                exit(-1);
            } //in refresh mode: every bankstate should be in idle.
        }
    }
    else if (mode_ == dramMode::SB)
    {
        if(!is_salp_)    checkBank(packet->busPacketType, packet->bank, packet->row);
        else    checkBank(packet->busPacketType, packet->bank, controlsubarray(packet), packet->row);
    }
    else
    {
        cout<<"[rank]:check and mode_ is hab_pim and cycle is "<<currentClockCycle<<endl;
        if(is_salp_)
        {
            for (int bank = 0; bank < config.NUM_BANKS; bank++)
            //for (int bank = 0; bank < 1; bank +=4)
            {
                int sub = (packet->row < 0x2000)?0:(packet->row < 0x4000)?1:(packet->row < 0x6000)?2:3;
                checkBank(packet->busPacketType, bank, sub, packet->row);
            }
        }
        else
        {
            for (int bank = (packet->bank % 2); bank < config.NUM_BANKS; bank += 2)
                checkBank(packet->busPacketType, bank, packet->row);
        }
    }
}

void Rank::updateState(BusPacket* packet)
{
    auto addrMapping = config.addrMapping;
    int targetsub = (packet->row < 0x2000)?0:(packet->row < 0x4000)?1:(packet->row < 0x6000)?2:3;
    if (packet->busPacketType == REF)
    {
        refreshWaiting = false;
        for (size_t i = 0; i < config.NUM_BANKS; i++)
        {
            if(is_salp_)
            {
                for(size_t j = 0; j < 4; j++)
                {
                    bankStates_SUB[i*4 + j].nextActivate = currentClockCycle + config.tRFC;
                }
            }
            else       bankStates[i].nextActivate = currentClockCycle + config.tRFC;
        }
    }
    else if (mode_ == dramMode::SB)
    {
        for (int bank = 0; bank < 16; bank++)
        {
            if(is_salp_)
            {
                for(int sub = 0; sub < 4; sub++)
                {
                    updateBank(packet->busPacketType, bank, packet->row, sub, bank==packet->bank,
                            addrMapping.isSameBankgroup(bank, packet->bank), targetsub == sub);
                }
            }
            else updateBank(packet->busPacketType, bank, packet->row, bank == packet->bank, addrMapping.isSameBankgroup(bank, packet->bank));
        }
    }
    else //drawmode all pim or something
    {
        for (int bank = 0; bank < 16; bank++)
        {
            if(is_salp_)
            {
                for(int sub = 0; sub < 4; sub++)
                {
                    updateBank(packet->busPacketType, bank, packet->row, sub, true,
                            true, targetsub == sub);
                }
            }
            else    updateBank(packet->busPacketType, bank, packet->row, (bank % 2) == packet->bank, true);
        }
    }
}

int Rank::controlsubarray(BusPacket* packet)
{
    int row = packet->row;
    if(row < 0x2000) return 0;
    else if(row < 0x4000) return 1;
    else if(row < 0x6000) return 2;
    else return 3;
}
//need tRA, tWA which means next activate, next select logic...
void Rank::updateBank(BusPacketType type, int bank, int row, int sub, bool targetBank, bool targetBankgroup, bool targetSubarray) //just use target subarray to do 
{
    switch (type)
    {
        case READ:
            if (targetBank){
                if(targetSubarray)  
                    bankStates_SUB[4*bank + sub].nextPrecharge = max(bankStates_SUB[4*bank + sub].nextPrecharge,
                                                     currentClockCycle + config.READ_TO_PRE_DELAY); //precharge keeps later....
                bankStates_SUB[4*bank + sub].nextWrite = max(bankStates_SUB[4*bank + sub].nextWrite, currentClockCycle + config.tRTW);
                bankStates_SUB[4*bank + sub].nextRead =
                    max(bankStates_SUB[4*bank + sub].nextRead,
                        currentClockCycle + max(config.tCCDL, config.BL / 2));
            }
            if (targetBankgroup)
            {
                bankStates_SUB[4*bank + sub].nextRead =
                    max(bankStates_SUB[4*bank + sub].nextRead,
                        currentClockCycle + max(config.tCCDL, config.BL / 2));
                if(!targetBank) bankStates_SUB[4*bank + sub].nextWrite =
                    max(bankStates_SUB[4*bank + sub].nextWrite, currentClockCycle + config.READ_TO_WRITE_DELAY);
            }
            else //not targetgroup mode...
            {
                bankStates_SUB[4*bank + sub].nextRead =
                    max(bankStates_SUB[4*bank + sub].nextRead,
                        currentClockCycle + max(config.tCCDS, config.BL / 2));
                bankStates_SUB[4*bank + sub].nextWrite =
                    max(bankStates_SUB[4*bank + sub].nextWrite, currentClockCycle + config.READ_TO_WRITE_DELAY);
            }
            //if(bank==4 && sub == 3) cout<<"[updateBank] and cycle is "<<currentClockCycle<<" and chan is "<<chanId<<" and bank is "<<bank<<" and sub is "<<sub<<" and row is "<<row<<" and targetBank is "<<targetBank<<" and targetBankgroup is "<<targetBankgroup<<" and targetSubarray is "<<targetSubarray<<
            //" and state is "<<bankStates_SUB[4*bank + sub].currentBankState<<endl;
            break;
        case WRITE:
            // update state table
            if (targetBank){
                //int tWTP = config.tCWL + config.BL + config.tWR;
                bankStates_SUB[4*bank + sub].nextRead = max(bankStates_SUB[4*bank + sub].nextRead, currentClockCycle + config.tWTR);
                if(targetSubarray)
                    bankStates_SUB[4*bank + sub].nextPrecharge = max(bankStates_SUB[4*bank + sub].nextPrecharge,
                                                     currentClockCycle + config.tWTP);
            }
            if(targetBankgroup)
            {
                if(!targetBank)
                    bankStates_SUB[4*bank + sub].nextRead = max(bankStates_SUB[4*bank + sub].nextRead, currentClockCycle + config.WRITE_TO_READ_DELAY_B_LONG);
                bankStates_SUB[4*bank + sub].nextWrite =
                    max(bankStates_SUB[4*bank + sub].nextWrite,
                        currentClockCycle + max(config.BL / 2, config.tCCDL));
            }
            else
            {
                bankStates_SUB[4*bank + sub].nextRead =
                    max(bankStates_SUB[4*bank + sub].nextRead,
                        currentClockCycle + config.WRITE_TO_READ_DELAY_B_SHORT);
                bankStates_SUB[4*bank + sub].nextWrite =
                    max(bankStates_SUB[4*bank + sub].nextWrite,
                        currentClockCycle + max(config.BL / 2, config.tCCDS));
            }
            //if(bank==4 && sub == 3) cout<<"[updateBank] and cycle is "<<currentClockCycle<<" and chan is "<<chanId<<" and bank is "<<bank<<" and sub is "<<sub<<" and row is "<<row<<" and targetBank is "<<targetBank<<" and targetBankgroup is "<<targetBankgroup<<" and targetSubarray is "<<targetSubarray<<
            //" and state is "<<bankStates_SUB[4*bank + sub].currentBankState<<endl;
            break;
        case ACTIVATE:
            if (targetBank)
            {
                if(targetSubarray){
                    bankStates_SUB[4*bank + sub].currentBankState = RowActive;
                    bankStates_SUB[4*bank + sub].nextActivate = max(bankStates_SUB[4*bank + sub].nextActivate, currentClockCycle + config.tRC);
                    bankStates_SUB[4*bank + sub].openRowAddress = row;
                    bankStates_SUB[4*bank + sub].nextRead = max(bankStates_SUB[4*bank + sub].nextRead, currentClockCycle + (config.tRCDRD - config.AL));
                    bankStates_SUB[4*bank + sub].nextWrite =max(bankStates_SUB[4*bank + sub].nextWrite, currentClockCycle + (config.tRCDWR - config.AL));
                    bankStates_SUB[4*bank + sub].nextPrecharge = max(bankStates_SUB[4*bank + sub].nextPrecharge, currentClockCycle + config.tRAS);
                }
                else
                {
                    bankStates_SUB[4*bank + sub].nextActivate =
                        max(bankStates_SUB[4*bank + sub].nextActivate, currentClockCycle + config.tRRDL);
                }
            }
            else
            {
                bankStates_SUB[4*bank + sub].nextActivate =
                    (targetBankgroup)
                        ? max(bankStates_SUB[4*bank + sub].nextActivate, currentClockCycle + config.tRRDL)
                        : max(bankStates_SUB[4*bank + sub].nextActivate, currentClockCycle + config.tRRDS);
            }
            //cout<<"[ACTIVATE] "<<"currentClockCYCLE "<<currentClockCycle<<" and "<<" bank: "<<bank<<" sub: "<<sub<<" nextActivate: "<<bankStates_SUB[4*bank + sub].nextActivate<<endl;
            //if(bank==4 && sub == 3) cout<<"[updateBank] and cycle is "<<currentClockCycle<<" and chan is "<<chanId<<" and bank is "<<bank<<" and sub is "<<sub<<" and row is "<<row<<" and targetBank is "<<targetBank<<" and targetBankgroup is "<<targetBankgroup<<" and targetSubarray is "<<targetSubarray<<
            //" and state is "<<bankStates_SUB[4*bank + sub].currentBankState<<endl;
            break;

        case PRECHARGE:
            if (targetSubarray)// || targetBank)
            {
                bankStates_SUB[4*bank + sub].currentBankState = Idle;
                bankStates_SUB[4*bank + sub].nextActivate =
                    max(bankStates_SUB[4*bank + sub].nextActivate, currentClockCycle + config.tRP);
            }
            //if(bank==4 && sub == 3) cout<<"[updateBank] and cycle is "<<currentClockCycle<<" and chan is "<<chanId<<" and bank is "<<bank<<" and sub is "<<sub<<" and row is "<<row<<" and targetBank is "<<targetBank<<" and targetBankgroup is "<<targetBankgroup<<" and targetSubarray is "<<targetSubarray<<
            //" and state is "<<bankStates_SUB[4*bank + sub].currentBankState<<endl;
            break;

        case DATA:
            break;
        case SUBSEL:
            break;
        default:
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
            exit(0);
            break;
    }
}
void Rank::updateBank(BusPacketType type, int bank, int row, bool targetBank, bool targetBankgroup)
{
    switch (type)
    {
        case READ:
            if (targetBank)
                bankStates[bank].nextPrecharge = max(bankStates[bank].nextPrecharge,
                                                     currentClockCycle + config.READ_TO_PRE_DELAY);

            if (targetBankgroup)
            {
                bankStates[bank].nextRead =
                    max(bankStates[bank].nextRead,
                        currentClockCycle + max(config.tCCDL, config.BL / 2));
            }
            else
            {
                bankStates[bank].nextRead =
                    max(bankStates[bank].nextRead,
                        currentClockCycle + max(config.tCCDS, config.BL / 2));
            }
            bankStates[bank].nextWrite =
                max(bankStates[bank].nextWrite, currentClockCycle + config.READ_TO_WRITE_DELAY);

            break;
        case WRITE:
            // update state table
            if (targetBank)
                bankStates[bank].nextPrecharge = max(bankStates[bank].nextPrecharge,
                                                     currentClockCycle + config.WRITE_TO_PRE_DELAY);
            if (targetBankgroup)
            {
                bankStates[bank].nextRead =
                    max(bankStates[bank].nextRead,
                        currentClockCycle + config.WRITE_TO_READ_DELAY_B_LONG);
                bankStates[bank].nextWrite =
                    max(bankStates[bank].nextWrite,
                        currentClockCycle + max(config.BL / 2, config.tCCDL));
            }
            else
            {
                bankStates[bank].nextRead =
                    max(bankStates[bank].nextRead,
                        currentClockCycle + config.WRITE_TO_READ_DELAY_B_SHORT);
                bankStates[bank].nextWrite =
                    max(bankStates[bank].nextWrite,
                        currentClockCycle + max(config.BL / 2, config.tCCDS));
            }
            break;
        case ACTIVATE:
            if (targetBank)
            {
                bankStates[bank].currentBankState = RowActive;
                bankStates[bank].nextActivate = currentClockCycle + config.tRC;
                bankStates[bank].openRowAddress = row;
                bankStates[bank].nextWrite = currentClockCycle + (config.tRCDWR - config.AL);
                bankStates[bank].nextRead = currentClockCycle + (config.tRCDRD - config.AL);
                bankStates[bank].nextPrecharge = currentClockCycle + config.tRAS;
            }
            else
            {
                bankStates[bank].nextActivate =
                    (targetBankgroup)
                        ? max(bankStates[bank].nextActivate, currentClockCycle + config.tRRDL)
                        : max(bankStates[bank].nextActivate, currentClockCycle + config.tRRDS);
            }
            break;
        case PRECHARGE:
            if (targetBank)
            {
                bankStates[bank].currentBankState = Idle;
                bankStates[bank].nextActivate =
                    max(bankStates[bank].nextActivate, currentClockCycle + config.tRP);
            }
            break;

        case DATA:
            break;
        case SUBSEL:
            break;
        default:
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
            exit(0);
            break;
    }
}
void Rank::readSb(BusPacket* packet)
{
    int sub = (packet->row<0x2000)?0:(packet->row<0x4000)?1:(packet->row<0x6000)?2:3;
    if (DEBUG_CMD_TRACE)
    {
        if (packet->row == 0x3fff)
        {
            if(!is_salp_)
            {
                if (0x08 <= packet->column && packet->column <= 0x0f)
                {
                    PRINT(OUTLOG_GRF_A("READ_GRF_A"));
                }
                else if (0x18 <= packet->column && packet->column <= 0x1f)
                {
                    PRINT(OUTLOG_GRF_B("READ_GRF_B"));
                }
            }
            else
            {
                if (0x08 <= packet->column && packet->column <= 0x11)
                {
                    PRINT(OUTLOG_GRF_A("READ_GRF_A"));
                }
                else if(packet->column == 0x12)
                {
                    PRINT(OUTLOG_B_GRF_B("READ_GRF_B"));
                }
            }
        }
        else if (packet->row & (1 << 12))
        {
            PRINTC(GRAY, OUTLOG_ALL("READ"));
        }
        else
        {
            PRINT(OUTLOG_ALL("READ"));
        }
    }

#ifndef NO_STORAGE
    if (packet->row == 0x3fff)
    {
        if(!is_salp_)
        {
            if (0x08 <= packet->column && packet->column <= 0x0f)
            *(packet->data) = pimRank->pimBlocks[packet->bank / 2].grfA[packet->column - 0x8];
            else if (0x18 <= packet->column && packet->column <= 0x1f)
                *(packet->data) = pimRank->pimBlocks[packet->bank / 2].grfB[packet->column - 0x18];
            else
                banks[packet->bank].read(packet);
        }
        else
        {
            if (0x08 <= packet->column && packet->column <= 0x11)
                *(packet->data) = pimRank->sblocks[packet->bank].grf[packet->column - 0x8];
            else if (packet->column == 0x12)
                *(packet->data) = pimRank->sblocks[packet->bank].blf;
            else
                banks_sub[packet->bank*4+sub].read(packet);
        }
    }
    else
    {
        if(is_salp_)
        {
            //if(banks_sub[packet->bank].size() == 4)
            banks_sub[packet->bank*4+sub].read(packet);
        }
        else
        {
            //cout<<"[RANK] readsb and currentClockCycle is "<<currentClockCycle<<" and chan is "<<packet->chan<<" and bank is "<<packet->bank<<" and row is "<<packet->row<<" and col is "<<packet->column<<endl;
            banks[packet->bank].read(packet);
        }
    }
#endif
}

void Rank::writeSb(BusPacket* packet)
{
    if (DEBUG_CMD_TRACE)
    {
        if (packet->row == 0x3fff || packet->row & (1 << 12))
        {
            PRINTC(GRAY, OUTLOG_ALL("WRITE"));
        }
        else
        {
            PRINT(OUTLOG_ALL("WRITE"));
        }
    }

#ifndef NO_STORAGE
    if (!(packet->row == 0x3fff) && !(packet->row & (1 << 12)))
    {
        if(!is_salp_){
            if(packet!=NULL && packet->data!=NULL)
            {
                banks[packet->bank].write(packet);
            }
        }
        else
        {
            //cout<<"[rank]:write and cycle is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and row is "<<packet->row
            //<<" and col is" <<packet->column<<endl;
            int sub = (packet->row < 0x2000)?0:(packet->row < 0x4000)?1:(packet->row < 0x6000)?2:3;
            //if(banks_sub[packet->bank].size() == 4)
            //{
            banks_sub[packet->bank*4+sub].write(packet);
            //}   
            //else{}
        }
    }
#endif
}

void Rank::execute(BusPacket* packet)
{
    //if(mode_ == dramMode::HAB_PIM)   cout<<"[rank]:execute and cycle is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and row is "<<packet->row
    //<<" and col is" <<packet->column<<" and type is "<<packet->busPacketType<<endl;
    //if(mode_ == dramMode::HAB_PIM)  cout<<"[rank]: execute for hab_pim and cycle is "<<currentClockCycle<<" and cond is "<<pimRank->isToggleCond(packet)<<endl;
    switch (packet->busPacketType)
    {
        case READ:
            if (mode_ == dramMode::SB)
            {
                //cout<<"[rank]:execute for read cycle is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and row is "<<packet->row
                //<<" and col is" <<packet->column<<endl;
                readSb(packet);  //packets come and subarray acting possible...
            }
            else if (mode_ == dramMode::HAB_PIM && pimRank->isToggleCond(packet)) //hab_pim mode
            {
                //cout<<"clock is "<<currentClockCycle<<" and row is "<<packet->row<<endl;
                pimRank->doPIM(packet);
            }
            /*else if (mode_ == dramMode::HAB_PIM && is_salp_)
            {
                //cout<<"[rank]:execute for read cycle is "<<currentClockCycle<<" and bank is "<<packet->bank<<" and row is "<<packet->row
                //<<" and col is" <<packet->column<<endl;
                //packet = packet;
                pimRank->doPIM(packet);
            }*/
            else
                pimRank->readHab(packet);
            packet->busPacketType = DATA;
            if(readReturnPacket.capacity() == readReturnPacket.size())  readReturnPacket.reserve(readReturnPacket.size() + 32);
            readReturnPacket.push_back(packet);
            if(readReturnCountdown.capacity() == readReturnCountdown.size())
                readReturnCountdown.reserve(readReturnCountdown.size() + 32);
            readReturnCountdown.push_back(config.RL); 
            //delete(packet); 
            break;
        case WRITE:
            if (mode_ == dramMode::SB)
                writeSb(packet);
            else if (mode_ == dramMode::HAB_PIM && pimRank->isToggleCond(packet))
                pimRank->doPIM(packet);
            /*else if(mode_ == dramMode::HAB_PIM && is_salp_)
                pimRank->doPIM(packet);*/
            else
                pimRank->writeHab(packet);
            //delete (packet);
            break;
        case ACTIVATE:
            if (DEBUG_CMD_TRACE)
            {
                PRINTC(getModeColor(), OUTLOG_ALL("ACTIVATE") << " tag : " << packet->tag);
            }
            if (mode_ == dramMode::SB && packet->row == 0x17ff && packet->column == 0x1f) //avoid selecting these ones
            { //need mode change for sb_pim
                abmr1Even_ = (packet->bank == 0) ? true : abmr1Even_; //nothing to bother....
                abmr1Odd_ = (packet->bank == 1) ? true : abmr1Odd_;
                abmr2Even_ = (packet->bank == 8) ? true : abmr2Even_;
                abmr2Odd_ = (packet->bank == 9) ? true : abmr2Odd_;

                if ((config.NUM_BANKS <= 2 && abmr1Even_ && abmr1Odd_) ||
                    (config.NUM_BANKS > 2 && abmr1Even_ && abmr1Odd_ && abmr2Even_ && abmr2Odd_))
                {
                    //cout<<"[rank]: execute and mode is hab and cycle is "<<currentClockCycle<<endl;
                    abmr1Even_ = abmr1Odd_ = abmr2Even_ = abmr2Odd_ = false;
                    mode_ = dramMode::HAB;
                    if (DEBUG_CMD_TRACE)
                    {
                        PRINTC(RED, OUTLOG_CH_RA("HAB") << " tag : " << packet->tag);
                    }
                }
            }
            delete (packet);
            break;
        case PRECHARGE:
            if (DEBUG_CMD_TRACE)
            {
                if (mode_ == dramMode::SB || packet->bank < 2)
                {
                    PRINTC(getModeColor(), OUTLOG_PRECHARGE("PRECHARGE"));
                }
                else
                {
                    PRINTC(GRAY, OUTLOG_PRECHARGE("PRECHARGE"));
                }
            }

            if (mode_ == dramMode::HAB && packet->row == 0x1fff)
            {
                sbmr1_ = (packet->bank == 0) ? true : sbmr1_; //toggle condition?
                sbmr2_ = (packet->bank == 1) ? true : sbmr2_;

                if (sbmr1_ && sbmr2_)
                {
                    //cout<<"[rank]: execute and mode is sb and cycle is "<<currentClockCycle<<endl;
                    sbmr1_ = sbmr2_ = false;
                    mode_ = dramMode::SB;
                    if (DEBUG_CMD_TRACE)
                    {
                        PRINTC(RED, OUTLOG_CH_RA("SB mode"));
                    }
                }
            }

            delete (packet);
            break;

        case REF:
            refreshWaiting = false;
            if (DEBUG_CMD_TRACE)
            {
                PRINT(OUTLOG_CH_RA("REF"));
            }
            delete (packet);
            break;

        case DATA:
            delete (packet);
            break;

        default:
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank and packettype is "<<packet->busPacketType<<" and clockcycle is "<<currentClockCycle);
            exit(0);
            break;
    }
}

void Rank::update()
{
    // An outgoing packet is one that is currently sending on the bus
    // do the book keeping for the packet's time left on the bus
    /*cout<<"[rank]:rank is updated and cycle is "<<currentClockCycle<< " and bank0 size is "<<banks_sub[0].size()<<" and 1 size is "<<banks_sub[1].size()<<" and 2 size is "<<banks_sub[2].size()<<" and 3 size is "<<banks_sub[3].size()
    <<" and 4 size is "<<banks_sub[4].size()<<" and 5 size is "<<banks_sub[5].size()<<" and 6 size is "<<banks_sub[6].size()<<" and 7 size is "<<banks_sub[7].size()<<
    " and 8 size is "<<banks_sub[8].size()<<" and 9 size is "<<banks_sub[9].size()<<" and 10 size is "<<banks_sub[10].size()<<" and 11 size is "<<banks_sub[11].size()<<
    " and 12 size is "<<banks_sub[12].size()<<" and 13 size is "<<banks_sub[13].size()<<" and 14 size is "<<banks_sub[14].size()<<" and 15 size is "<<banks_sub[15].size()<<endl;*/
    if (outgoingDataPacket != NULL)
    {
        dataCyclesLeft--;
        if (dataCyclesLeft == 0 && outgoingDataPacket->busPacketType == DATA)
        {
            // if the packet is done on the bus, call receiveFromBus and free up
            // the bus
            memoryController->receiveFromBus(outgoingDataPacket);
            outgoingDataPacket = NULL;
        }
    }

    // decrement the counter for all packets waiting to be sent back
    //auto it = readReturnCountdown.begin();
    if (readReturnCountdown.size() > 0){
        for (int i = 0; i < readReturnCountdown.size(); i++) {
            if(readReturnCountdown[i]>0){
                /*if(readReturnCountdown.size() > 10000000000)
                    cout<<"[READ] "<<"readReturnCountdown["<<i<<"]: "<<readReturnCountdown[i]<<endl;*/
                readReturnCountdown[i]--;
            }
        }
    }
    if (readReturnCountdown.size() > 0 /*&& readReturnCountdown.size() < 100*/ && readReturnCountdown[0] == 0)
    {
        // RL time has passed since the read was issued; this packet is
        // ready to go out on the bus
        outgoingDataPacket = readReturnPacket[0];
        dataCyclesLeft = config.BL / 2;

        // remove the packet from the ranks
        readReturnPacket.erase(readReturnPacket.begin());
        readReturnCountdown.erase(readReturnCountdown.begin());

        if (DEBUG_BUS)
        {
            PRINTN(" -- R" << getChanId() << " Issuing On Data Bus : ");
            outgoingDataPacket->print();
            PRINT("");
        }
    }
    pimRank->update();
    pimRank->step();
    //if(chanId ==1)  cout<<"[rank]:update and cycle is "<<currentClockCycle<<" and chan is "<<chanId<<" and currenstate is "<<bankStates_SUB[4*4+3].currentBankState<<endl;
}

// power down the rank
void Rank::powerDown()
{
    // perform checks
    for (size_t i = 0; i < config.NUM_BANKS; i++)
    {
        if(!is_salp_)
        {
            if (bankStates[i].currentBankState != Idle) //how?
            {
                ERROR("== Error - Trying to power down rank " << getChanId()
                                                            << " while not all banks are idle");
                exit(0);
            }
            bankStates[i].nextPowerUp = currentClockCycle + config.tCKE;
            bankStates[i].currentBankState = PowerDown;
        }
        else
        {
            for(size_t s = 0; s < 4; s++)
            {
                if(bankStates_SUB[4*i+s].currentBankState != Idle)
                {
                    ERROR("== Error - Trying to power down rank " << getChanId()
                                                                  << " while not all banks are idle");
                    exit(0);
                }
                bankStates_SUB[4*i+s].nextPowerUp = currentClockCycle + config.tCKE;
                bankStates_SUB[4*i+s].currentBankState = PowerDown;
            }
        }
    }

    isPowerDown = true;
}

// power up the rank
void Rank::powerUp()
{
    if (!isPowerDown)
    {
        ERROR("== Error - Trying to power up rank " << getChanId()
                                                    << " while it is not already powered down");
        exit(0);
    }

    isPowerDown = false;

    for (size_t i = 0; i < config.NUM_BANKS; i++)
    {
        if(!is_salp_)
        {
            if (bankStates[i].nextPowerUp > currentClockCycle)
            {
                ERROR("== Error - Trying to power up rank " << getChanId()
                                                            << " before we're allowed to");
                ERROR(bankStates[i].nextPowerUp << "    " << currentClockCycle);
                exit(0);
            }
            bankStates[i].nextActivate = currentClockCycle + config.tXP;
            bankStates[i].currentBankState = Idle;
        }
        else
        {
            for(size_t s = 0; s < 4; s++)
            {
                if(bankStates_SUB[4*i+s].nextPowerUp > currentClockCycle)
                {
                    ERROR("== Error - Trying to power up rank" << getChanId()
                                                               << " before we're allowed to");
                    ERROR(bankStates_SUB[4*i+s].nextPowerUp << "   "<<currentClockCycle);
                    exit(0);
                }
                bankStates_SUB[4*i+s].nextActivate = currentClockCycle + config.tXP;
                bankStates_SUB[4*i+s].currentBankState = Idle;
            }
        }
    }
}
