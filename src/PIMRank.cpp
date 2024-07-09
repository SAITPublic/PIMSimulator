/***************************************************************************************************
 * Copyright (C) 2021 Samsung Electronics Co. LTD
 *
 * This software is a property of Samsung Electronics.
 * No part of this software, either material or conceptual may be copied or distributed,
 * transmitted, transcribed, stored in a retrieval system, or translated into any human
 * or computer language in any form by any means,electronic, mechanical, manual or otherwise,
 * or disclosed to third parties without the express written permission of Samsung Electronics.
 * (Use of the Software is restricted to non-commercial, personal or academic, research purpose
 * only)
 **************************************************************************************************/

#include <bitset>
#include <iostream>

#include "AddressMapping.h"
#include "PIMCmd.h"
#include "PIMRank.h"

using namespace std;
using namespace DRAMSim;

PIMRank::PIMRank(ostream& simLog, Configuration& configuration, bool is_salp)
    : chanId(-1),
      rankId(-1),
      dramsimLog(simLog),
      pimPC_(0),
      lastJumpIdx_(-1),
      numJumpToBeTaken_(-1),
      lastRepeatIdx_(-1),
      numRepeatToBeDone_(-1),
      useAllGrf_(true),
      crfExit_(false),
      config(configuration),
      pimBlocks(getConfigParam(UINT, "NUM_PIM_BLOCKS"),
                PIMBlock(PIMConfiguration::getPIMPrecision())),
      sblocks(getConfigParam(UINT, "NUM_S_BLOCKS"),
                SBlock(PIMConfiguration::getPIMPrecision())),
      is_salp_(is_salp)
{
    currentClockCycle = 0;
    rank = nullptr;
}

void PIMRank::attachRank(Rank* r)
{
    this->rank = r;
    /*cout<<"[pimrank] attach rank and bank0 size is "<<rank->banks_sub[0].size()<<" and bank1 size is "<<rank->banks_sub[1].size()<<" and bank2 size is "<<rank->banks_sub[2].size()<<" and bank3 size is "<<rank->banks_sub[3].size()<<
    " and bank4 size is "<<rank->banks_sub[4].size()<<" and bank5 size is "<<rank->banks_sub[5].size()<<" and bank6 size is "<<rank->banks_sub[6].size()<<" and bank7 size is "<<rank->banks_sub[7].size()<<
    " and bank8 size is "<<rank->banks_sub[8].size()<<" and bank9 size is "<<rank->banks_sub[9].size()<<" and bank10 size is "<<rank->banks_sub[10].size()<<" and bank11 size is "<<rank->banks_sub[11].size()<<
    " and bank12 size is "<<rank->banks_sub[12].size()<<" and bank13 size is "<<rank->banks_sub[13].size()<<" and bank14 size is "<<rank->banks_sub[14].size()<<" and bank15 size is "<<rank->banks_sub[15].size()<<endl;*/
}

void PIMRank::setChanId(int id)
{
    this->chanId = id;
}

void PIMRank::setRankId(int id)
{
    this->rankId = id;
}

int PIMRank::getChanId() const
{
    return this->chanId;
}

int PIMRank::getRankId() const
{
    return this->rankId;
}

void PIMRank::update() {}


//think about controlpim -->how to control each pim block operate solely? not together?
void PIMRank::controlPIM(BusPacket* packet) //make salp-control pim
{
    if(!is_salp_)
    {
        uint8_t grf_a_zeroize = packet->data->u8Data_[20];
        if (grf_a_zeroize)
        {
            if (DEBUG_CMD_TRACE)
            {
                PRINTC(RED, OUTLOG_CH_RA("GRF_A_ZEROIZE"));
            }
            BurstType burst_zero;
            for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
            {
                for (int i = 0; i < 8; i++) pimBlocks[pb].grfA[i] = burst_zero;
            }
        }
        uint8_t grf_b_zeroize = packet->data->u8Data_[21];
        if (grf_b_zeroize)
        {
            if (DEBUG_CMD_TRACE)
            {
                PRINTC(RED, OUTLOG_CH_RA("GRF_B_ZEROIZE"));
            }
            BurstType burst_zero;
            for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
            {
                for (int i = 0; i < 8; i++) pimBlocks[pb].grfB[i] = burst_zero; //set idx..
            }
        }
    }
    else{
        uint8_t grf_zeroize = packet->data->u8Data_[20];
        if(grf_zeroize)
        {
            if (DEBUG_CMD_TRACE)
            {
                PRINTC(RED, OUTLOG_CH_RA("GRF_ZEROIZE"));
            }
            BurstType burst_zero;
            for (int sb = 0; sb < config.NUM_S_BLOCKS; sb++)
            {
                for (int i = 0; i < 4; i++) sblocks[sb].grf[i] = burst_zero;
                sblocks[sb].blf = burst_zero;
            }
        }
    }
    pimOpMode_ = packet->data->u8Data_[0] & 1;
    //pimOpMode_single_ = packet->data->u8Data_[0] & 2;
    toggleEvenBank_ = !(packet->data->u8Data_[16] & 1);
    toggleOddBank_ = !(packet->data->u8Data_[16] & 2);
    toggleRa12h_ = (packet->data->u8Data_[16] & 4);
    useAllGrf_ = packet->data->u8Data_[10] & 1; //how to set?
    //cout<<"[pimrank] control pim and clock is "<<currentClockCycle<<" and pimOpMode is "<<pimOpMode_<<" and toggleEvenBank is "<<toggleEvenBank_<<" and toggleOddBank is "<<toggleOddBank_<<" and toggleRa12h is "<<toggleRa12h_<<" and useAllGrf is "<<useAllGrf_<<endl;
    if (pimOpMode_) //we can use various pimopmode for doing.....
    {
        //cout<<"[pimrank] pimOpMode is activated and clock is "<<currentClockCycle<<endl;
        rank->mode_ = dramMode::HAB_PIM;
        pimPC_ = 0;
        lastJumpIdx_ = numJumpToBeTaken_ = lastRepeatIdx_ = numRepeatToBeDone_ = -1;
        crfExit_ = false;
        PRINTC(RED, OUTLOG_CH_RA("HAB_PIM"));
    }
    else
    {
        rank->mode_ = dramMode::HAB;
        PRINTC(RED, OUTLOG_CH_RA("HAB mode"));
    }
}

bool PIMRank::isToggleCond(BusPacket* packet)
{
    if (pimOpMode_ && !crfExit_)
    {
        if (toggleRa12h_)
        {
            if (toggleEvenBank_ && ((packet->bank & 1) == 0))
                return true;
            else if (toggleOddBank_ && ((packet->bank & 1) == 1))
                return true;
            return false;
        }
        else if (!toggleRa12h_ && !(packet->row & (1 << 12)))
        {
            if (toggleEvenBank_ && ((packet->bank & 1) == 0))
                return true;
            else if (toggleOddBank_ && ((packet->bank & 1) == 1))
                return true;
            return false;
        }
        return false;
    }
    else
    {
        return false;
    }
}


//to control pim block for solely executing --> use pch and bank to do it..
void PIMRank::readHab(BusPacket* packet) 
{
    if (packet->row & (1 << 12))  // ignored
    {
        PRINTC(GRAY, OUTLOG_ALL("READ"));
    }
    else
    {
        PRINTC(GRAY, OUTLOG_ALL("BANK_TO_PIM"));
#ifndef NO_STORAGE
        int grf_id;
        int grf_id_sub;
        if (useAllGrf_) //THINK ABOUT THIS... WE WOULD USE HTIS?
        {
            grf_id = packet->column & 0xf;
            grf_id_sub = packet->column & 0x4;
            if(!is_salp_)
            {
                for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
                {
                    {
                        rank->banks[pb * 2 + packet->bank].read(packet);
                        if (grf_id < 8)
                            pimBlocks[pb].grfA[grf_id] = *(packet->data);
                        else
                            pimBlocks[pb].grfB[grf_id - 8] = *(packet->data);
                    }
                }
            }
            else{
                for(int sb = 0; sb  < config.NUM_S_BLOCKS; sb++)
                {
                    rank->banks_sub[4*sb+grf_id_sub].read(packet);
                    if(grf_id_sub < 3)
                        sblocks[sb].grf[grf_id_sub] = *(packet->data);
                    else
                        sblocks[sb].blf = *(packet->data);
                }
            }
        }
        else
        {
            grf_id = getGrfIdx(packet->column);
            grf_id_sub = getGrfIdxsalp(packet->column);
            if(!is_salp_)
            {
                for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
                {
                    rank->banks[pb * 2 + packet->bank].read(packet);
                    pimBlocks[pb].grfB[grf_id] = *(packet->data);
                }
            }
            else
            {
                for (int sb = 0; sb  <config.NUM_S_BLOCKS; sb++)
                {
                    int sub = (packet->row < 0x2000) ? 0 : (packet->row < 0x4000) ? 1 : (packet->row < 0x6000) ? 2 : 3; 
                    rank->banks_sub[sb*4+sub].read(packet);
                    sblocks[sb].grf[grf_id_sub] = *(packet->data);
                }
            }
        }
#endif
    }
}


void PIMRank::writeHab(BusPacket* packet)
{
    //cout<<"[pimrank] control pim and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<" and data is " <<packet->data<<endl;
    if (packet->row == 0x3fff)  // WRIO to PIM Broadcasting
    {
        if (packet->column == 0x00)
        {
            controlPIM(packet); //same for 
        }
        if ((0x08 <= packet->column && packet->column <= 0x0f) ||
            (0x18 <= packet->column && packet->column <= 0x1f)) //GRFA = 0X08~0X0F / GRFB = 0X18~0X1F
        {
            if (DEBUG_CMD_TRACE)
            {
                if (packet->column - 8 < 8)
                    PRINTC(GREEN, OUTLOG_B_GRF_A("BWRITE_GRF_A"));
                else
                    PRINTC(GREEN, OUTLOG_B_GRF_B("BWRITE_GRF_B"));
            }
#ifndef NO_STORAGE
            if(!is_salp_)
            {    
                for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++) 
                {
                    if (packet->column - 8 < 8)
                    {
                        pimBlocks[pb].grfA[packet->column - 0x8] = *(packet->data);
                    }
                    else
                        pimBlocks[pb].grfB[packet->column - 0x18] = *(packet->data);
                }
            }
            else
            {
                for(int sb = 0; sb<config.NUM_S_BLOCKS; sb++)
                {
                    if(packet!=nullptr)
                    {
                        //cout<<"[pimrank] write grf and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<" and data is "<<packet->data<<endl;
                        if(packet->column - 12 == 0)   sblocks[sb].blf = *(packet->data);
                        else if(packet->column >= 0x8 && packet->column <= 0x11)
                        {
                            //cout<<"[pimrank] write grf and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<" and data is "<<packet->data<<endl;
                            sblocks[sb].grf[packet->column - 0x8] = *(packet->data);
                        }
                    }
                }
            }
#endif
        }
        else if (0x04 <= packet->column && packet->column <= 0x07)
        {
            if (DEBUG_CMD_TRACE)
                PRINTC(GREEN, OUTLOG_B_CRF("BWRITE_CRF"));
            crf.bst[packet->column - 0x04] = *(packet->data); //same for salpim and 
        }
        else if (packet->column == 0x1)
        {
            if (DEBUG_CMD_TRACE)
                PRINTC(GREEN, OUTLOG_CH_RA("BWRITE_SRF"));
            for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++) pimBlocks[pb].srf = *(packet->data);
        }
    }
    else if (packet->row & 1 << 12)
    {
        PRINTC(GRAY, OUTLOG_ALL("WRITE"));
    }
    else  // PIM (only GRF) to Bank Move
    {
        PRINTC(GREEN, OUTLOG_ALL("PIM_TO_BANK"));

#ifndef NO_STORAGE
        int grf_id = packet->column & 0xf;
        int grf_id_sub = packet->column & 0x4;
        if (useAllGrf_)
        {
            for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
            {
                if (grf_id < 8)
                    *(packet->data) = pimBlocks[pb].grfA[grf_id];
                else
                    *(packet->data) = pimBlocks[pb].grfB[grf_id - 8];
                //*(packet->data) = sblocks[pb].grf[grf_id_sub];
                rank->banks[pb * 2 + packet->bank].write(packet);
                //rank->banks_sub[pb][grf_id].write(packet);
            }
        }
        else
        {
            if(!is_salp_)
            {   
                for (int pb = 0; pb < config.NUM_PIM_BLOCKS; pb++)
                {
                    if (packet->bank == 0)
                    {
                        *(packet->data) = pimBlocks[pb].grfA[grf_id];
                        rank->banks[pb * 2].write(packet);  // basically read from bank;
                    }
                    else if (packet->bank == 1)
                    {
                        *(packet->data) = pimBlocks[pb].grfB[grf_id];
                        rank->banks[pb * 2 + 1].write(packet);  // basically read from bank.
                    }
                }
            }
            else
            {
                int sub = (packet->row < 0x2000) ? 0 : (packet->row < 0x4000) ? 1 : (packet->row < 0x6000) ? 2 : 3;
                for (int sb = 0; sb < config.NUM_S_BLOCKS; sb++)
                {
                    if(grf_id_sub < 4)   *(packet->data) = sblocks[sb].grf[grf_id_sub];
                    else    *(packet->data) = sblocks[sb].blf;
                    rank->banks_sub[sb*4+sub].write(packet);
                }
            }
        }
#endif
    }
}

//for subarray logic we need other read/write opds (such as s_reg/c_reg)
void PIMRank::readOpd(int pb, BurstType& bst, PIMOpdType type, BusPacket* packet, int idx,
                      bool is_auto, bool is_mac)
{
    idx = (is_salp_)?getGrfIdxsalp(idx):getGrfIdx(idx);
    unsigned sub = (packet->row<0x2000)?0:(packet->row<0x4000)?1:(packet->row<0x6000)?2:3;
    switch (type)
    {
        case PIMOpdType::A_OUT:
            bst = pimBlocks[pb].aOut;
            return;
        case PIMOpdType::M_OUT:
            bst = pimBlocks[pb].mOut;
            return;
        case PIMOpdType::BANK:
            if(is_salp_)
            {
                //cout<<"row is "<<packet->row<<" and sub is "<<sub<<endl;
                //cout<<"[pimrank] read bank and clock is "<<currentClockCycle<<" and pb is "<<pb<<" and idx is "<<idx<<" and sub is "<<sub<<" and ch is "<<
                //rank->getChanId()<<" and size is "<<rank->banks_sub.size()<<endl;
                //if (rank->banks_sub[pb].size() ==4)
                //{
                rank->banks_sub[pb*4+sub].read(packet);
                //if(packet->data != nullptr)
                bst = *(packet->data);
                //}
            }
            else{}
            return;
        case PIMOpdType::GRF_A:
            if (is_auto)
                bst = pimBlocks[pb].grfA[(is_mac) ? getGrfIdxHigh(packet->row, packet->column)
                                                  : getGrfIdx(packet->column)];
            else
                bst = pimBlocks[pb].grfA[idx];
            return;
        case PIMOpdType::GRF_B: //why 
            bst = pimBlocks[pb].grfB[(is_auto) ? getGrfIdx(packet->column) : idx];
            return;
        case PIMOpdType::GRF: //no auto mode indeed
            //cout<<"[pimrank] read grf and clock is "<<currentClockCycle<<" and idx is "<<idx<<" and pb is "<<pb<<endl;
            bst = sblocks[pb].grf[idx];
        case PIMOpdType::BLF:
        {
            //cout<<"[pimrank] read blf and clock is "<<currentClockCycle<<endl;
            bst.set(sblocks[pb].blf.fp16Data_[idx]);
        }
        case PIMOpdType::SRF_M:
            bst.set(pimBlocks[pb].srf.fp16Data_[idx]);
            return;
        case PIMOpdType::SRF_A:
            bst.set(pimBlocks[pb].srf.fp16Data_[idx + 8]);
            return;
        case PIMOpdType::EVEN_BANK:
            if(!is_salp_)
            {
                if (packet->bank % 2 != 0)
                    PRINT("Warning, CRF bank coding and bank id from packet are inconsistent");
                //cout<<"[pimrank] read even bank and clock is "<<currentClockCycle<<" and pb is "<<pb<<endl;
                rank->banks[pb * 2].read(packet);  // basically read from bank.
                bst = *(packet->data);
            }
            return;
        case PIMOpdType::ODD_BANK:
            if(!is_salp_)
            {
                if (packet->bank % 2 == 0)
                    PRINT("Warning, CRF bank coding and bank id from packet are inconsistent");
                //cout<<"[pimrank] read odd bank and clock is "<<currentClockCycle<<" and pb is "<<pb<<endl;
                rank->banks[pb * 2 + 1].read(packet);  // basically read from bank.
                bst = *(packet->data);
            }
            return;
    }
    return;
}

void PIMRank::writeOpd(int pb, BurstType& bst, PIMOpdType type, BusPacket* packet, int idx,
                       bool is_auto, bool is_mac) //pb-->pb_id
{
    idx = (is_salp_)?getGrfIdxsalp(idx):getGrfIdx(idx);
    int sub = (packet->row<0x2000)?0:(packet->row<0x4000)?1:(packet->row<0x6000)?2:3;
    switch (type)
    {
        case PIMOpdType::A_OUT:
            pimBlocks[pb].aOut = bst; //which means a_out
            return;
        case PIMOpdType::M_OUT:
            pimBlocks[pb].mOut = bst;
            return;
        case PIMOpdType::BANK:
            //cout<<"[pimrank] write bank and clock is "<<currentClockCycle<<" and pb is "<<pb<<" and idx is "<<idx<<" and sub is "<<sub<<" and banks_sub size is "<<
            //rank->banks_sub[pb].size()<<endl;
            if(is_salp_)
            {
                *(packet->data) = bst;
                //if(rank->banks_sub[pb].size() == 4)
                rank->banks_sub[pb*4+sub].write(packet);
            }
            return;
        case PIMOpdType::GRF_A:
            if(!is_salp_)
            {
                if (is_auto)
                    pimBlocks[pb].grfA[(is_mac) ? getGrfIdxHigh(packet->row, packet->column)
                                                : getGrfIdx(packet->column)] = bst;
                else
                    pimBlocks[pb].grfA[idx] = bst;
            }
            return;
        case PIMOpdType::GRF_B:
            if(!is_salp_)
            {
                if (is_auto)
                    pimBlocks[pb].grfB[getGrfIdx(packet->column)] = bst;
                else
                    pimBlocks[pb].grfB[idx] = bst;
            }
            return;
        case PIMOpdType::GRF:
            //cout<<"[pimrank] write grf and clock is "<<currentClockCycle<<" and idx is "<<idx<<" and pb is "<<pb<<endl;
            if(is_salp_)
            {
                if (is_auto)
                    sblocks[pb].grf[(is_mac) ? getGrfIdxHigh(packet->row, packet->column)
                                            : getGrfIdxsalp(packet->column)] = bst;
                else
                    sblocks[pb].grf[idx] = bst;
            }
            return;
        case PIMOpdType::BLF:
            sblocks[pb].blf = bst;
            return;
        case PIMOpdType::SRF_M:
            pimBlocks[pb].srf = bst;
            return;
        case PIMOpdType::SRF_A:
            pimBlocks[pb].srf = bst;
            return;
        case PIMOpdType::EVEN_BANK:
            if(!is_salp_)
            {
                if (packet->bank % 2 != 0)
                {
                    PRINT("CRF bank coding and bank id from packet are inconsistent");
                    cout<<"crf bank coding and bank id from packet are inconsistent"<<endl;
                }
                //cout<<"[pimrank] read even bank and clock is "<<currentClockCycle<<" and pb is "<<pb<<endl;
                *(packet->data) = bst;
                rank->banks[pb * 2].write(packet);  // basically read from bank.
            }
            return;
        case PIMOpdType::ODD_BANK:
            if(!is_salp_)
            {
                if (packet->bank % 2 == 0)
                {
                    PRINT("CRF bank coding and bank id from packet are inconsistent"); //bank data? how to connect crf to bank idx?
                    exit(-1);
                }
                //cout<<"[pimrank] read even bank and clock is "<<currentClockCycle<<" and pb is "<<pb<<endl;
                *(packet->data) = bst;
                rank->banks[pb * 2 + 1].write(packet);  // basically read from bank.
            }
            return;
    }
    return;
}

void PIMRank::doPIM(BusPacket* packet)
{
    PIMCmd cCmd;
    //cout<<"[pimrank] do pim and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<endl;
    //packet->row = packet->row & ((1 << 16) - 1); //which is 0x7fff
    //cout<<"[pimrank] do pim and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<" and cmd type is " <<cCmd.type_<<endl;
    do
    {
        cCmd.fromInt(crf.data[pimPC_]);
        if (DEBUG_CMD_TRACE)
        {
            PRINTC(CYAN, string((packet->busPacketType == READ) ? "READ ch" : "WRITE ch")
                             << getChanId() << " ra" << getRankId() << " bg"
                             << config.addrMapping.bankgroupId(packet->bank) << " b" << packet->bank
                             << " r" << packet->row << " c" << packet->column << "|| [" << pimPC_
                             << "] " << cCmd.toStr() << " @ " << currentClockCycle);
        }

        if (cCmd.type_ == PIMCmdType::EXIT)
        {
            crfExit_ = true;
            break;
        }
        else if (cCmd.type_ == PIMCmdType::JUMP)
        {
            if (lastJumpIdx_ != pimPC_)
            {
                if (cCmd.loopCounter_ > 0)
                {
                    lastJumpIdx_ = pimPC_;
                    numJumpToBeTaken_ = cCmd.loopCounter_;
                }
            }
            if (numJumpToBeTaken_ > 0)
            {
                pimPC_ -= cCmd.loopOffset_;
                numJumpToBeTaken_--;
            }
        }
        else
        {
            if (cCmd.type_ == PIMCmdType::FILL || cCmd.isAuto_)
            {
                if (lastRepeatIdx_ != pimPC_)
                {
                    lastRepeatIdx_ = pimPC_;
                    numRepeatToBeDone_ = 8 - 1; //not always doing 8 loops.. tricky one should exists..
                }

                if (numRepeatToBeDone_ > 0)
                {
                    pimPC_ -= 1;
                    numRepeatToBeDone_--;
                }
                else
                    lastRepeatIdx_ = -1;
            }
            else if(cCmd.type_ == PIMCmdType::MOV)
            {
                if (lastRepeatIdx_!=pimPC_)
                {
                    lastRepeatIdx_ = pimPC_;
                    numRepeatToBeDone_ = cCmd.loopCounter_;
                }

                if (numRepeatToBeDone_ > 0)
                {
                    pimPC_ -= 1;
                    numRepeatToBeDone_--;
                }
                else
                    lastRepeatIdx_ = -1;
            }
            else if (cCmd.type_ == PIMCmdType::NOP)
            {
                if (lastRepeatIdx_ != pimPC_)
                {
                    lastRepeatIdx_ = pimPC_;
                    numRepeatToBeDone_ = cCmd.loopCounter_;
                }

                if (numRepeatToBeDone_ > 0)
                {
                    pimPC_ -= 1;
                    numRepeatToBeDone_--;
                }
                else
                    lastRepeatIdx_ = -1;
            }
            if(!is_salp_){
                for (int pimblock_id = 0; pimblock_id < config.NUM_PIM_BLOCKS; pimblock_id++)
                {
                    if (DEBUG_PIM_BLOCK && pimblock_id == 0)
                    {
                        PRINT(pimBlocks[pimblock_id].print());
                        PRINT("[BANK_R]" << packet->data->binToStr());
                        PRINT("[CMD]" << bitset<32>(cCmd.toInt()) << "(" << cCmd.toStr() << ")");
                    }

                    doPIMBlock(packet, cCmd, pimblock_id);

                    if (DEBUG_PIM_BLOCK && pimblock_id == 0)
                    {
                        PRINT(pimBlocks[pimblock_id].print());
                        PRINT("----------");
                    }
                }
            }
            else
            {
                for (int sblock_id = 0; sblock_id < config.NUM_S_BLOCKS; sblock_id++)
                {
                    //cout<<"[pimrank] do pim and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<" and id is "
                    // <<sblock_id<<" and block num is "<<config.NUM_S_BLOCKS<<endl;
                    if (DEBUG_PIM_BLOCK && sblock_id == 0)
                    {
                        PRINT(sblocks[sblock_id].print());
                        PRINT("[BANK_R]" << packet->data->binToStr());
                        PRINT("[CMD]" << bitset<32>(cCmd.toInt()) << "(" << cCmd.toStr() << ")");
                    }
                    //cout<<"[pimrank] do pim and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<" and id is "
                    //<<sblock_id<<" and block num is "<<config.NUM_S_BLOCKS<<endl;
                    doPIMBlock(packet, cCmd, sblock_id);

                    if (DEBUG_PIM_BLOCK && sblock_id == 0)
                    {
                        PRINT(sblocks[sblock_id].print());
                        PRINT("----------");
                    }
                }
            }
        }
        pimPC_++;
        // EXIT check
        PIMCmd next_cmd;
        next_cmd.fromInt(crf.data[pimPC_]);
        if (next_cmd.type_ == PIMCmdType::EXIT)
            crfExit_ = true;
    } while (cCmd.type_ == PIMCmdType::JUMP);
}

void PIMRank::doPIMBlock(BusPacket* packet, PIMCmd cCmd, int pimblock_id) //how to avoid all pim mode
{
    if (cCmd.type_ == PIMCmdType::FILL || cCmd.type_ == PIMCmdType::MOV) //how about use move term
    {
        BurstType bst;
        bool is_auto = (cCmd.type_ == PIMCmdType::FILL) ? true : false;  
        //cout<<"[pimrank] do pim block and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<" and id is"<<
        //pimblock_id<<endl; 
        readOpd(pimblock_id, bst, cCmd.src0_, packet, cCmd.src0Idx_, is_auto, false); //16x logic...
        if (cCmd.isRelu_)
        {
            for (int i = 0; i < 16; i++)
                bst.u16Data_[i] = (bst.u16Data_[i] & (1 << 15)) ? 0 : bst.u16Data_[i];
        }
        writeOpd(pimblock_id, bst, cCmd.dst_, packet, cCmd.dstIdx_, is_auto, false); //16x logic...
    }
    else if (cCmd.type_ == PIMCmdType::ADD || cCmd.type_ == PIMCmdType::MUL || cCmd.type_ == PIMCmdType::MAX)
    {
        BurstType dstBst;
        BurstType src0Bst;
        BurstType src1Bst;
        //cout<<"[pimrank] do pim block and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<endl;
        readOpd(pimblock_id, src0Bst, cCmd.src0_, packet, cCmd.src0Idx_, cCmd.isAuto_, false);
        readOpd(pimblock_id, src1Bst, cCmd.src1_, packet, cCmd.src1Idx_, cCmd.isAuto_, false);

        if (cCmd.type_ == PIMCmdType::ADD)
        {
            // dstBst = src0Bst + src1Bst;
            if(!is_salp_)   pimBlocks[pimblock_id].add(dstBst, src0Bst, src1Bst);
            else    sblocks[pimblock_id].add(dstBst, src0Bst, src1Bst);
        }
        else if (cCmd.type_ == PIMCmdType::MUL)
        {
            // dstBst = src0Bst * src1Bst;
            if(!is_salp_)   pimBlocks[pimblock_id].mul(dstBst, src0Bst, src1Bst);
            else    sblocks[pimblock_id].mul(dstBst, src0Bst, src1Bst);
        }
        else if (cCmd.type_ == PIMCmdType::MAX)
        {
            // dstBst = max(src0Bst, src1Bst);
            if(!is_salp_)   pimBlocks[pimblock_id].burstmax(dstBst, src0Bst, src1Bst);
            else    sblocks[pimblock_id].burstmax(dstBst, src0Bst, src1Bst);
        }
        writeOpd(pimblock_id, dstBst, cCmd.dst_, packet, cCmd.dstIdx_, cCmd.isAuto_, false);
    }
    else if (cCmd.type_ == PIMCmdType::MAC || cCmd.type_ == PIMCmdType::MAD)
    {
        BurstType dstBst;
        BurstType src0Bst;
        BurstType src1Bst;
        bool is_mac = (cCmd.type_ == PIMCmdType::MAC) ? true : false;
        //cout<<"[pimrank] do pim block and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<endl;
        readOpd(pimblock_id, src0Bst, cCmd.src0_, packet, cCmd.src0Idx_, cCmd.isAuto_, is_mac); //same
        readOpd(pimblock_id, src1Bst, cCmd.src1_, packet, cCmd.src1Idx_, cCmd.isAuto_, is_mac); //bank or something
        if (is_mac)
        {
            readOpd(pimblock_id, dstBst, cCmd.dst_, packet, cCmd.dstIdx_, cCmd.isAuto_, is_mac);
            // dstBst = src0Bst * src1Bst + dstBst;
            if(!is_salp_)    pimBlocks[pimblock_id].mac(dstBst, src0Bst, src1Bst);
            else    sblocks[pimblock_id].mac(dstBst, src0Bst, src1Bst);
        }
        else
        {
            BurstType src2Bst;
            readOpd(pimblock_id, src2Bst, cCmd.src2_, packet, cCmd.src1Idx_, cCmd.isAuto_, is_mac);
            // dstBst = src0Bst * src1Bst + src2Bst;
            pimBlocks[pimblock_id].mad(dstBst, src0Bst, src1Bst, src2Bst);
            //nothing to do in sblock beacuse it did not have such function...
        }
        writeOpd(pimblock_id, dstBst, cCmd.dst_, packet, cCmd.dstIdx_, cCmd.isAuto_, is_mac);
    }
    else if (cCmd.type_ == PIMCmdType::NOP && packet->busPacketType == WRITE)
    {
        int grf_id = getGrfIdx(packet->column);
        int grf_id_sub = getGrfIdxsalp(packet->column);
        //cout<<"[pimrank] do pim block and clock is "<<currentClockCycle<<" and row is "<<packet->row<<" and col is "<<packet->column<<endl;
        if(!is_salp_){    
            if (packet->bank == 0)
            {
                *(packet->data) = pimBlocks[pimblock_id].grfA[grf_id];
                rank->banks[pimblock_id * 2].write(packet);  
            }
            else if (packet->bank == 1)
            {
                *(packet->data) = pimBlocks[pimblock_id].grfB[grf_id];
                rank->banks[pimblock_id * 2 + 1].write(packet);
            }
        }
        else
        {
            *(packet->data) = sblocks[pimblock_id].grf[grf_id_sub];
            //if(rank->banks_sub[pimblock_id].size() == 4)
            rank->banks_sub[pimblock_id*4+grf_id_sub].write(packet); 
        }
    }
}
//we need some store logic, but this risc-v form has only load logic(fill) --> need to prove
