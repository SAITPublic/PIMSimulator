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

    pimRank = new PIMRank(dramsimLog, config);
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

void Rank::receiveFromBus(BusPacket* packet)
{
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
    switch (type)
    {
        case READ:
            if (bankStates[bank].currentBankState != RowActive ||
                currentClockCycle < bankStates[bank].nextRead ||
                row != bankStates[bank].openRowAddress)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId() << " ba" << bank
                                       << " received a READ when not allowed @ "
                                       << currentClockCycle);
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
                                       << currentClockCycle);
                exit(-1);
            }
            break;
        case DATA:
            break;
        default:
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
            exit(-1);
            break;
    }
}

void Rank::check(BusPacket* packet)
{
    if (packet->busPacketType == REF)
    {
        for (size_t i = 0; i < config.NUM_BANKS; i++)
        {
            if (bankStates[i].currentBankState != Idle)
            {
                ERROR("== Error - ch " << getChanId() << " ra" << getRankId()
                                       << " received a REF when not allowed");
                exit(-1);
            }
        }
    }
    else if (mode_ == dramMode::SB)
    {
        checkBank(packet->busPacketType, packet->bank, packet->row);
    }
    else
    {
        for (int bank = (packet->bank % 2); bank < config.NUM_BANKS; bank += 2)
            checkBank(packet->busPacketType, bank, packet->row);
    }
}

void Rank::updateState(BusPacket* packet)
{
    auto addrMapping = config.addrMapping;
    if (packet->busPacketType == REF)
    {
        refreshWaiting = false;
        for (size_t i = 0; i < config.NUM_BANKS; i++)
        {
            bankStates[i].nextActivate = currentClockCycle + config.tRFC;
        }
    }
    else if (mode_ == dramMode::SB)
    {
        for (int bank = 0; bank < config.NUM_BANKS; bank++)
        {
            updateBank(packet->busPacketType, bank, packet->row, bank == packet->bank,
                       addrMapping.isSameBankgroup(bank, packet->bank));
        }
    }
    else
    {
        for (int bank = 0; bank < config.NUM_BANKS; bank++)
        {
            updateBank(packet->busPacketType, bank, packet->row, (bank % 2) == packet->bank, true);
        }
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

        default:
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
            exit(0);
            break;
    }
}

void Rank::readSb(BusPacket* packet)
{
    if (DEBUG_CMD_TRACE)
    {
        if (packet->row == 0x3fff)
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
        if (0x08 <= packet->column && packet->column <= 0x0f)
            *(packet->data) = pimRank->pimBlocks[packet->bank / 2].grfA[packet->column - 0x8];
        else if (0x18 <= packet->column && packet->column <= 0x1f)
            *(packet->data) = pimRank->pimBlocks[packet->bank / 2].grfB[packet->column - 0x18];
        else
            banks[packet->bank].read(packet);
    }
    else
        banks[packet->bank].read(packet);
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
        banks[packet->bank].write(packet);
#endif
}

void Rank::execute(BusPacket* packet)
{
    switch (packet->busPacketType)
    {
        case READ:
            if (mode_ == dramMode::SB)
                readSb(packet);
            else if (mode_ == dramMode::HAB_PIM && pimRank->isToggleCond(packet))
                pimRank->doPIM(packet);
            else
                pimRank->readHab(packet);
            packet->busPacketType = DATA;
            readReturnPacket.push_back(packet);
            readReturnCountdown.push_back(config.RL);
            break;
        case WRITE:
            if (mode_ == dramMode::SB)
                writeSb(packet);
            else if (mode_ == dramMode::HAB_PIM && pimRank->isToggleCond(packet))
                pimRank->doPIM(packet);
            else
                pimRank->writeHab(packet);
            delete (packet);
            break;
        case ACTIVATE:
            if (DEBUG_CMD_TRACE)
            {
                PRINTC(getModeColor(), OUTLOG_ALL("ACTIVATE") << " tag : " << packet->tag);
            }
            if (mode_ == dramMode::SB && packet->row == 0x17ff && packet->column == 0x1f)
            {
                abmr1Even_ = (packet->bank == 0) ? true : abmr1Even_;
                abmr1Odd_ = (packet->bank == 1) ? true : abmr1Odd_;
                abmr2Even_ = (packet->bank == 8) ? true : abmr2Even_;
                abmr2Odd_ = (packet->bank == 9) ? true : abmr2Odd_;

                if ((config.NUM_BANKS <= 2 && abmr1Even_ && abmr1Odd_) ||
                    (config.NUM_BANKS > 2 && abmr1Even_ && abmr1Odd_ && abmr2Even_ && abmr2Odd_))
                {
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
                sbmr1_ = (packet->bank == 0) ? true : sbmr1_;
                sbmr2_ = (packet->bank == 1) ? true : sbmr2_;

                if (sbmr1_ && sbmr2_)
                {
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
            ERROR("== Error - Unknown BusPacketType trying to be sent to Bank");
            exit(0);
            break;
    }
}

void Rank::update()
{
    // An outgoing packet is one that is currently sending on the bus
    // do the book keeping for the packet's time left on the bus
    if (outgoingDataPacket != NULL)
    {
        dataCyclesLeft--;
        if (dataCyclesLeft == 0)
        {
            // if the packet is done on the bus, call receiveFromBus and free up
            // the bus
            memoryController->receiveFromBus(outgoingDataPacket);
            outgoingDataPacket = NULL;
        }
    }

    // decrement the counter for all packets waiting to be sent back
    for (size_t i = 0; i < readReturnCountdown.size(); i++) readReturnCountdown[i]--;

    if (readReturnCountdown.size() > 0 && readReturnCountdown[0] == 0)
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
}

// power down the rank
void Rank::powerDown()
{
    // perform checks
    for (size_t i = 0; i < config.NUM_BANKS; i++)
    {
        if (bankStates[i].currentBankState != Idle)
        {
            ERROR("== Error - Trying to power down rank " << getChanId()
                                                          << " while not all banks are idle");
            exit(0);
        }
        bankStates[i].nextPowerUp = currentClockCycle + config.tCKE;
        bankStates[i].currentBankState = PowerDown;
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
}
