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

#ifndef CMDQUEUE_H
#define CMDQUEUE_H

#include <vector>

#include "BankState.h"
#include "BusPacket.h"
#include "SimulatorObject.h"
#include "SystemConfiguration.h"
#include "Transaction.h"

using namespace std;

namespace DRAMSim
{
class Rank;
class CommandQueue : public SimulatorObject
{
    CommandQueue();
    ostream& dramsimLog;

  public:
    // typedefs
    typedef vector<BusPacket*> BusPacket1D;
    typedef vector<BusPacket1D> BusPacket2D;
    typedef vector<BusPacket2D> BusPacket3D;
    typedef vector<BusPacket3D> BusPacket4D;

    // functions
    CommandQueue(vector<vector<BankState>>& states, ostream& dramsimLog);
    CommandQueue(vector<vector<BankState>>& states, ostream& dramsimLog, bool is_salp);
    virtual ~CommandQueue();

    void enqueue(BusPacket* newBusPacket);
    void enqueue_sub(BusPacket* newBusPacket);
    void process_queue();
    void process_queue_sub();
    bool pop(BusPacket **busPacket);
    bool pop_sub(BusPacket** busPacket);

    // TODO: rename this...
    bool process_refresh(BusPacket** busPacket);
    bool process_refresh_sub(BusPacket** busPacket);
    bool process_command(BusPacket** busPacket);
    bool process_command_sub(BusPacket** busPacket);  
    bool process_precharge(BusPacket** busPacket);
    bool process_precharge_sub(BusPacket** busPacket);

    bool hasRoomFor(unsigned numberToEnqueue, unsigned rank, unsigned bank);
    bool hasRoomFor(unsigned numberToEnqueue, unsigned rank, unsigned bank, unsigned subarray);
    bool isIssuable(BusPacket* busPacket);
    bool isIssuable_sub(BusPacket* busPacket);
    bool isEmpty(unsigned rank);
    bool isEmpty_sub(unsigned rank);
    void needRefresh(unsigned rank);

    void print();
    void update();  // SimulatorObject requirement
    vector<BusPacket*>& getCommandQueue(unsigned rank, unsigned bank);
    vector<BusPacket*>& getCommandQueue(unsigned rank, unsigned bank, unsigned sub);

    // fields
    BusPacket3D queues;  // 3D array of BusPacket pointers
    BusPacket4D queues_sub;
    vector<vector<BankState>>& bankStates;
    vector<vector<BankState>>& bankStates_sub;
    vector<Rank*>* ranks;

    vector<vector<BankState>> emptyBankStates;
    vector<vector<BankState>> emptyBankStatesSub;

  private:
    void nextRankAndBank(unsigned& rank, unsigned& bank);
    void nextRankAndBankandSubarray(unsigned& rank, unsigned& bank, unsigned& sub);
    // fields

    unsigned nextBank;
    unsigned nextRank;
    unsigned nextSub; //clearly need cause.. in 

    unsigned nextBankPRE;
    unsigned nextRankPRE;
    unsigned nextSubPRE;

    unsigned refreshRank;
    unsigned refreshBank;
    unsigned refreshSub;

    bool refreshWaiting;
    
    vector<unsigned> commandCounters;
    vector<bool> processedCommands;
    vector<vector<unsigned>> tXAWCountdown;
    vector<vector<unsigned>> rowAccessCounters;
    vector<vector<vector<unsigned>>> rowAccessCounters_sub;

    bool sendAct;
    bool is_salp_;

    // preloaded system configuration parameters
    unsigned num_ranks_;
    unsigned num_banks_;
    unsigned num_subarrays_;
    unsigned cmd_queue_depth_;
    unsigned xaw_;
    unsigned total_row_accesses_;
    SchedulingPolicy schedulingPolicy_;
    QueuingStructure queuingStructure_;
};

}  // namespace DRAMSim
#endif
