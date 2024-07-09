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

#include <unistd.h>

#include "MemorySystem.h"

using namespace std;

ofstream cmd_verify_out;  // used in Rank.cpp and MemoryController.cpp if
                          // VERIFICATION_OUTPUT is set
namespace DRAMSim
{
powerCallBack_t MemorySystem::ReportPower = NULL;

MemorySystem::MemorySystem(unsigned id, unsigned int megsOfMemory, CSVWriter& csvOut_,
                           ostream& simLog, Configuration& configuration, bool is_salp)
    : dramsimLog(simLog),
      ReturnReadData(NULL),
      WriteDataDone(NULL),
      systemID(id),
      csvOut(csvOut_),
      numOnTheFlyTransactions(0),
      config(configuration),
      is_salp_(is_salp)
{
    currentClockCycle = 0;
    //maybe i need some boolean salp logic to figure whether use subarray level or not
    DEBUG("===== MemorySystem " << systemID << " =====");

    // calculate the total storage based on the devices the user selected and the number of

    // calculate number of devices
    /************************
      This code has always been problematic even though it's pretty simple. I'll try to explain it
      for my own sanity.
      There are two main variables here that we could let the user choose:
      NUM_RANKS or TOTAL_STORAGE.  Since the density and width of the part is
      fixed by the device ini file, the only variable that is really
      controllable is the number of ranks. Users care more about choosing the
      total amount of storage, but with a fixed device they might choose a total
      storage that isn't possible. In that sense it's not as good to allow them
      to choose TOTAL_STORAGE (because any NUM_RANKS value >1 will be valid).
      However, users don't care (or know) about ranks, they care about total
      storage, so maybe it's better to let them choose and just throw an error
      if they choose something invalid.
      A bit of background:
      Each column contains DEVICE_WIDTH bits. A row contains NUM_COLS columns.
      Each bank contains NUM_ROWS rows. Therefore, the total storage per DRAM device is:
            PER_DEVICE_STORAGE = NUM_ROWS*NUM_COLS*DEVICE_WIDTH*NUM_BANKS (in bits)
     A rank *must* have a 64 bit output bus (JEDEC standard), so each rank must have:
            NUM_DEVICES_PER_RANK = 64/DEVICE_WIDTH
            (note: if you have multiple channels ganged together, the bus width is
            effectively NUM_CHANS * 64/DEVICE_WIDTH)

    If we multiply these two numbers to get the storage per rank (in bits), we get:
            PER_RANK_STORAGE = PER_DEVICE_STORAGE*NUM_DEVICES_PER_RANK =
    NUM_ROWS*NUM_COLS*NUM_BANKS*64 Finally, to get TOTAL_STORAGE, we need to multiply by NUM_RANKS
            TOTAL_STORAGE = PER_RANK_STORAGE*NUM_RANKS (total storage in bits)
    So one could compute this in reverse -- compute NUM_DEVICES,
    PER_DEVICE_STORAGE, and PER_RANK_STORAGE first since all these parameters
    are set by the device ini. Then, TOTAL_STORAGE/PER_RANK_STORAGE = NUM_RANKS
    The only way this could run into problems is if TOTAL_STORAGE < PER_RANK_STORAGE,
    which could happen for very dense parts.
    *********************/

    // number of bytes per rank
    // uint64_t megsOfStoragePerRank = ((((long long)NUM_ROWS * (NUM_COLS * DEVICE_WIDTH) *
    // NUM_BANKS) *
    //                                  ((long long)JEDEC_DATA_BUS_BITS / DEVICE_WIDTH)) / 8) >> 20;

    uint64_t bytePerRank = static_cast<uint64_t>(
        (getConfigParam(UINT64, "NUM_ROWS") *
         (getConfigParam(UINT, "NUM_COLS") * getConfigParam(UINT, "DEVICE_WIDTH")) *
         getConfigParam(UINT, "NUM_BANKS") *
         (getConfigParam(UINT64, "JEDEC_DATA_BUS_BITS") / getConfigParam(UINT, "DEVICE_WIDTH"))) /
        8);
    uint64_t megsOfStoragePerRank = bytePerRank >> 20;

    num_ranks_ = 1;//(megsOfMemory / Byte2MB(bytePerRank));

    // If this is set, effectively override the number of ranks
    if (megsOfMemory != 0)
    {
        num_ranks_ = (num_ranks_ > 0) ? num_ranks_ : 1;
        if (num_ranks_ == 0)
        {
            PRINT("WARNING: Cannot create memory system with "
                  << megsOfMemory << "MB, defaulting to minimum size of " << megsOfStoragePerRank
                  << "MB");
        }
        setDevConfigParam(UINT, "NUM_RANKS", num_ranks_);
        // FIXME, need to synchronize between ConfigurationDB and Configuration
        config.NUM_RANKS = num_ranks_;
    }

    unsigned num_devices =
        (getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") / getConfigParam(UINT, "DEVICE_WIDTH"));
    setDevConfigParam(UINT, "NUM_DEVICES", num_devices);
    setDevConfigParam(UINT64, "TOTAL_STORAGE", num_ranks_ * Byte2MB(bytePerRank));

    PRINT("CH. " << systemID << " TOTAL_STORAGE : " << getConfigParam(UINT64, "TOTAL_STORAGE")
                 << "MB | " << getConfigParam(UINT, "NUM_RANKS") << " Ranks | "
                 << getConfigParam(UINT, "NUM_DEVICES") << " Devices per rank");

    memoryController = new MemoryController(this, csvOut, dramsimLog, config, is_salp_);

    // TODO: change to other vector constructor?
    ranks = new vector<Rank*>();    

    for (size_t i = 0; i < num_ranks_; i++) //only one rank indeed
    {
        Rank* r = new Rank(dramsimLog, config, is_salp_);
        r->setChanId(systemID);
        r->setRankId(i);
        r->attachMemoryController(memoryController);
        r->pimRank->setChanId(systemID);
        r->pimRank->setRankId(i);
        ranks->push_back(r);
    }

    memoryController->attachRanks(ranks);
}

MemorySystem::~MemorySystem()
{
    /* the MemorySystem should exist for all time, nothing should be destroying
     * it
     */
    //    ERROR("MEMORY SYSTEM DESTRUCTOR with ID "<<systemID);
    //    abort();

    delete (memoryController);

    for (size_t i = 0; i < num_ranks_; i++)
    {
        delete (*ranks)[i];
    }
    ranks->clear();
    delete (ranks);

    if (VERIFICATION_OUTPUT)
    {
        cmd_verify_out.flush();
        cmd_verify_out.close();
    }
}

bool MemorySystem::addTransaction(bool isWrite, uint64_t addr, BurstType* data)
{
    TransactionType type = isWrite ? DATA_WRITE : DATA_READ;
    Transaction* trans = new Transaction(type, addr, data);
    return addTransaction(trans);
}

bool MemorySystem::addTransaction(bool isWrite, uint64_t addr, const std::string& str,
                                  BurstType* data)
{
    TransactionType type = isWrite ? DATA_WRITE : DATA_READ;
    Transaction* trans = new Transaction(type, addr, str, data);
    return addTransaction(trans);
}

bool MemorySystem::addBarrier()
{
    if (memoryController->WillAcceptTransaction())
    {
        return memoryController->addBarrier();
    }
    else
    {
        if (pendingTransactions.size())
        {
            pendingTransactions.back()->tag += "BAR";
            return true;
        }
        else
        {
            return false;
        }
    }
}

bool MemorySystem::addTransaction(Transaction* trans)
{
    if (memoryController->WillAcceptTransaction())
    {
        return memoryController->addTransaction(trans);
    }
    else
    {
        pendingTransactions.push_back(trans);
        return true;
    }
}

// prints statistics
void MemorySystem::printStats(bool finalStats)
{
    memoryController->printStats(finalStats);
}

// update the memory systems state
void MemorySystem::update()
{
    if(currentClockCycle % 100 == 0 && systemID == 0)
        cout<<"MemorySystem::update() and clock is "<<currentClockCycle<<" and trsize is "<<pendingTransactions.size()<<endl;
    // PRINT(" ----------------- Memory System Update ------------------");
    // updates the state of each of the objects
    // NOTE - do not change order
    //if(systemID==1) cout<<"MemorySystem::update() and clock is "<<currentClockCycle<<" and curstate is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<" and openrow is "<<(*ranks)[0]->bankStates_SUB[4*4+3].openRowAddress<<endl;
    /*cout<<" [MemorySystem::start] and systemID is"<<systemID<<" and clock is "<<currentClockCycle<<" and banks 0 size is "<<(*ranks)[0]->banks_sub[0].size()<<" and bank 1 size is "<<(*ranks)[0]->banks_sub[1].size()<<" and bank 2 size is "<<(*ranks)[0]->banks_sub[2].size()<<
    " and bank 3 size is "<<(*ranks)[0]->banks_sub[3].size()<<" and bank 4 size is "<<(*ranks)[0]->banks_sub[4].size()<<" and bank 5 size is "<<(*ranks)[0]->banks_sub[5].size()<<" and bank 6 size is "<<(*ranks)[0]->banks_sub[6].size()<<" and bank 7 size is "<<(*ranks)[0]->banks_sub[7].size()<<
    " and bank 8 size is "<<(*ranks)[0]->banks_sub[8].size()<<" and bank 9 size is "<<(*ranks)[0]->banks_sub[9].size()<<" and bank 10 size is "<<(*ranks)[0]->banks_sub[10].size()<<" and bank 11 size is "<<(*ranks)[0]->banks_sub[11].size()<<" and bank 12 size is "<<(*ranks)[0]->banks_sub[12].size()<<
    " and bank 13 size is "<<(*ranks)[0]->banks_sub[13].size()<<" and bank 14 size is "<<(*ranks)[0]->banks_sub[14].size()<<" and bank 15 size is "<<(*ranks)[0]->banks_sub[15].size()<<endl;*/
    for (size_t i = 0; i < 1; i++)
    {
        (*ranks)[i]->update();
    }
    //if(systemID==1) cout<<"MemorySystem::update() and clock is "<<currentClockCycle<<" and curstate is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<" and openrow is "<<(*ranks)[0]->bankStates_SUB[4*4+3].openRowAddress<<endl;
    /*cout<<" [MemorySystem::rank::update] and systemID is"<<systemID<<" and clock is "<<currentClockCycle<<" and banks 0 size is "<<(*ranks)[0]->banks_sub[0].size()<<" and bank 1 size is "<<(*ranks)[0]->banks_sub[1].size()<<" and bank 2 size is "<<(*ranks)[0]->banks_sub[2].size()<<
    " and bank 3 size is "<<(*ranks)[0]->banks_sub[3].size()<<" and bank 4 size is "<<(*ranks)[0]->banks_sub[4].size()<<" and bank 5 size is "<<(*ranks)[0]->banks_sub[5].size()<<" and bank 6 size is "<<(*ranks)[0]->banks_sub[6].size()<<" and bank 7 size is "<<(*ranks)[0]->banks_sub[7].size()<<
    " and bank 8 size is "<<(*ranks)[0]->banks_sub[8].size()<<" and bank 9 size is "<<(*ranks)[0]->banks_sub[9].size()<<" and bank 10 size is "<<(*ranks)[0]->banks_sub[10].size()<<" and bank 11 size is "<<(*ranks)[0]->banks_sub[11].size()<<" and bank 12 size is "<<(*ranks)[0]->banks_sub[12].size()<<
    " and bank 13 size is "<<(*ranks)[0]->banks_sub[13].size()<<" and bank 14 size is "<<(*ranks)[0]->banks_sub[14].size()<<" and bank 15 size is "<<(*ranks)[0]->banks_sub[15].size()<<endl;*/
    //update rank first and doing memeorycontroller update....
    // pendingTransactions will only have stuff in it if MARSS is adding stuff
    if (pendingTransactions.size() > 0 && memoryController->WillAcceptTransaction())
    {
        memoryController->addTransaction(pendingTransactions.front());
        pendingTransactions.pop_front();
        //if(pendingTransactions.size() < 100)   cout<<"pendingTransactions.size() = "<<pendingTransactions.size() << " and currentclockcycle is "<<currentClockCycle<<endl;
    }
    memoryController->update();
    //if(systemID==1) cout<<"MemorySystem::update() and clock is "<<currentClockCycle<<" and curstate is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<" and openrow is "<<(*ranks)[0]->bankStates_SUB[4*4+3].openRowAddress<<endl;
    /*cout<<" [MemorySystem::memorycontroller::update] and clock is "<<currentClockCycle<<" and banks 0 size is "<<(*ranks)[0]->banks_sub[0].size()<<" and bank 1 size is "<<(*ranks)[0]->banks_sub[1].size()<<" and bank 2 size is "<<(*ranks)[0]->banks_sub[2].size()<<
    " and bank 3 size is "<<(*ranks)[0]->banks_sub[3].size()<<" and bank 4 size is "<<(*ranks)[0]->banks_sub[4].size()<<" and bank 5 size is "<<(*ranks)[0]->banks_sub[5].size()<<" and bank 6 size is "<<(*ranks)[0]->banks_sub[6].size()<<" and bank 7 size is "<<(*ranks)[0]->banks_sub[7].size()<<
    " and bank 8 size is "<<(*ranks)[0]->banks_sub[8].size()<<" and bank 9 size is "<<(*ranks)[0]->banks_sub[9].size()<<" and bank 10 size is "<<(*ranks)[0]->banks_sub[10].size()<<" and bank 11 size is "<<(*ranks)[0]->banks_sub[11].size()<<" and bank 12 size is "<<(*ranks)[0]->banks_sub[12].size()<<
    " and bank 13 size is "<<(*ranks)[0]->banks_sub[13].size()<<" and bank 14 size is "<<(*ranks)[0]->banks_sub[14].size()<<" and bank 15 size is "<<(*ranks)[0]->banks_sub[15].size()<<endl;*/
    // simply increments the currentClockCycle field for each object
    for (size_t i = 0; i < num_ranks_; i++)
    {
        (*ranks)[i]->step();
    }
    memoryController->step();
    this->step();
    //if(systemID==1) cout<<"MemorySystem::update() and clock is "<<currentClockCycle<<" and curstate is "<<(*ranks)[0]->bankStates_SUB[4*4+3].currentBankState<<" and openrow is "<<(*ranks)[0]->bankStates_SUB[4*4+3].openRowAddress<<endl;
    /*cout<<" [MemorySystem::step] and clock is "<<currentClockCycle<<" and banks 0 size is "<<(*ranks)[0]->banks_sub[0].size()<<" and bank 1 size is "<<(*ranks)[0]->banks_sub[1].size()<<" and bank 2 size is "<<(*ranks)[0]->banks_sub[2].size()<<
    " and bank 3 size is "<<(*ranks)[0]->banks_sub[3].size()<<" and bank 4 size is "<<(*ranks)[0]->banks_sub[4].size()<<" and bank 5 size is "<<(*ranks)[0]->banks_sub[5].size()<<" and bank 6 size is "<<(*ranks)[0]->banks_sub[6].size()<<" and bank 7 size is "<<(*ranks)[0]->banks_sub[7].size()<<
    " and bank 8 size is "<<(*ranks)[0]->banks_sub[8].size()<<" and bank 9 size is "<<(*ranks)[0]->banks_sub[9].size()<<" and bank 10 size is "<<(*ranks)[0]->banks_sub[10].size()<<" and bank 11 size is "<<(*ranks)[0]->banks_sub[11].size()<<" and bank 12 size is "<<(*ranks)[0]->banks_sub[12].size()<<
    " and bank 13 size is "<<(*ranks)[0]->banks_sub[13].size()<<" and bank 14 size is "<<(*ranks)[0]->banks_sub[14].size()<<" and bank 15 size is "<<(*ranks)[0]->banks_sub[15].size()<<endl;*/
    // PRINT("\n"); // two new lines
}

void MemorySystem::RegisterCallbacks(Callback_t* readCB, Callback_t* writeCB,
                                     void (*reportPower)(double bgpower, double burstpower,
                                                         double refreshpower, double actprepower))
{
    ReturnReadData = readCB;
    WriteDataDone = writeCB;
    ReportPower = reportPower;
}

bool MemorySystem::WillAcceptTransaction(uint64_t addr)
{
    return memoryController->WillAcceptTransaction();
}

bool MemorySystem::WillAcceptTransaction()
{
    return memoryController->WillAcceptTransaction();
}

}  // namespace DRAMSim

// This function can be used by autoconf AC_CHECK_LIB since
// apparently it can't detect C++ functions.
// Basically just an entry in the symbol table
extern "C" { void libdramsim_is_present(void) { ; } }
