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

#ifndef RANK_H
#define RANK_H

#include <vector>

#include "AddressMapping.h"
#include "Bank.h"
#include "BankState.h"
#include "BusPacket.h"
#include "Configuration.h"
#include "PIMRank.h"
#include "SimulatorObject.h"
//#include "Subarray.h"

using namespace std;
using namespace DRAMSim;

namespace DRAMSim
{
class MemoryController;  // forward declaration
class PIMRank;           // forward declaration
class Rank : public SimulatorObject
{
  private:
    int chanId;
    int rankId; //there is no more than 1 rank...
    ostream& dramsimLog;
    bool isPowerDown;
    Configuration& config;
    bool is_salp_;

  public:
    // functions
    Rank(ostream& simLog, Configuration& configuration);
    Rank(ostream& simLog, Configuration& configuration, bool is_salp);
    virtual ~Rank();

    void receiveFromBus(BusPacket* packet);
    void check(BusPacket* packet);
    void updateState(BusPacket* packet);
    void execute(BusPacket* packet);

    void checkBank(BusPacketType type, int bank, int row); 
    void checkBank(BusPacketType type, int bank, int sub, int row);
    void updateBank(BusPacketType type, int bank, int row, bool targetBank, bool targetBankgroup); //how about use this function to regulate subarray model
    void updateBank(BusPacketType type, int bank, int sub, int row, bool targetBank, bool targetBankgroup, bool targetSubarray);
    void attachMemoryController(MemoryController* mc);
    int getChanId() const;
    void setChanId(int id);
    int getRankId() const;
    void setRankId(int id);
    void update();
    void powerUp();
    void powerDown();
    int controlsubarray(BusPacket* packet);
    
    void readSb(BusPacket* packet);
    void writeSb(BusPacket* packet);

    // fields
    MemoryController* memoryController;
    BusPacket* outgoingDataPacket;
    PIMRank* pimRank;
    unsigned dataCyclesLeft;
    bool refreshWaiting;

    // these are vectors so that each element is per-bank
    vector<BusPacket*> readReturnPacket;
    vector<unsigned> readReturnCountdown;

    //vector<vector<Bank>> banks_sub; //which to modify...
    vector<Bank> banks;
    vector<Bank> banks_sub;
    vector<BankState> bankStates;
    //vector<vector<BankState>> bankStates_SUB;    
    vector<BankState> bankStates_SUB;

    dramMode mode_;
    bool abmr1Even_, abmr1Odd_, abmr2Even_, abmr2Odd_, sbmr1_, sbmr2_; //single bank/all bank what is mr?

    const char* getModeColor()
    {
        switch (mode_)
        {
            case dramMode::SB:
                return END;
            case dramMode::HAB:
                return GREEN;
            case dramMode::HAB_PIM:
                return CYAN;
        }
        return GRAY;
    }
};
}  // namespace DRAMSim
#endif
