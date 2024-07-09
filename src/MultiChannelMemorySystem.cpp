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

#include <errno.h>
#include <stdlib.h>  // getenv()
#include <bitset>
#include <iomanip>
#include <locale>
#include <sstream>  // stringstream
// for directory operations
#include <sys/stat.h>
#include <sys/types.h>

#include "AddressMapping.h"
#include "MultiChannelMemorySystem.h"
#include "ParameterReader.h"
#include "SystemConfiguration.h"
#include "Utils.h"

template <class T>
std::string FormatWithCommas(T value)
{
    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss << std::fixed << value;
    return ss.str();
}

using namespace DRAMSim;

MultiChannelMemorySystem::MultiChannelMemorySystem(const string& deviceIniFilename_,
                                                   const string& systemIniFilename_,
                                                   const string& pwd_, const string& traceFilename_,
                                                   unsigned megsOfMemory_, string* visFilename_,
                                                   bool is_salp)
    : megsOfMemory(megsOfMemory_),
      deviceIniFilename(deviceIniFilename_),
      systemIniFilename(systemIniFilename_),
      traceFilename(traceFilename_),
      pwd(pwd_),
      visFilename(visFilename_),
      clockDomainCrosser(new ClockDomain::Callback<MultiChannelMemorySystem, void>(
          this, &MultiChannelMemorySystem::actual_update)),
      csvOut(new CSVWriter(visDataOut)),
      is_salp_(is_salp)
{
    currentClockCycle = 0;
    if (visFilename)
        printf("CC VISFILENAME=%s\n", visFilename->c_str());

    if (!isPowerOfTwo(megsOfMemory))
    {
        ERROR("Please specify a power of 2 memory size");
        abort();
    }

    ConfigurationDB& configDB = ConfigurationDB::getDB();
    configDB.initialize();
    configDB.updatefromFile(deviceIniFilename);
    configDB.updatefromFile(systemIniFilename);

    addrMapping = new AddrMapping();
    configuration = new Configuration(*addrMapping);
    numFence = new unsigned[configuration->NUM_CHANS]();

    for (size_t i = 0; i < configuration->NUM_CHANS; i++)
    {
        MemorySystem* channel = new MemorySystem(i, megsOfMemory / 64,
                                                 (*csvOut), dramsimLog, *configuration, is_salp_);
        //cout<<"channel "<<i<<" created"<<" and bank size is "<<channel->ranks->front()->banks_sub.size()<<endl;       
        channels.push_back(channel);
    }
}

/* Initialize the ClockDomainCrosser to use the CPU speed
   If cpuClkFreqHz == 0, then assume a 1:1 ratio (like for TraceBasedSim)
 */
void MultiChannelMemorySystem::setCPUClockSpeed(uint64_t cpuClkFreqHz)
{
    uint64_t dramsimClkFreqHz = (uint64_t)(1.0 / (configuration->tCK * 1e-9));
    clockDomainCrosser.clock1 = dramsimClkFreqHz;
    clockDomainCrosser.clock2 = (cpuClkFreqHz == 0) ? dramsimClkFreqHz : cpuClkFreqHz;
}

bool fileExists(string& path)
{
    struct stat stat_buf;
    if (stat(path.c_str(), &stat_buf) != 0)
    {
        if (errno == ENOENT)
        {
            return false;
        }
        ERROR("Warning: some other kind of error happened with stat(), should probably check that");
    }
    return true;
}

string FilenameWithNumberSuffix(const string& filename, const string& extension,
                                unsigned maxNumber = 100)
{
    string currentFilename = filename + extension;
    if (!fileExists(currentFilename))
    {
        return currentFilename;
    }

    // otherwise, add the suffixes and test them out until we find one that works
    stringstream tmpNum;
    tmpNum << "." << 1;
    for (unsigned i = 1; i < maxNumber; i++)
    {
        currentFilename = filename + tmpNum.str() + extension;
        if (fileExists(currentFilename))
        {
            currentFilename = filename;
            tmpNum.seekp(0);
            tmpNum << "." << i;
        }
        else
        {
            return currentFilename;
        }
    }
    // if we can't find one, just give up and return whatever is the current filename
    ERROR("Warning: Couldn't find a suitable suffix for " << filename);
    return currentFilename;
}
/**
 * This function creates up to 3 output files:
 *     - The .log file if LOG_OUTPUT is set
 *     - the .vis file where csv data for each epoch will go
 *     - the .tmp file if verification output is enabled
 * The results directory is setup to be in PWD/TRACEFILENAME.[SIM_DESC]/DRAM_PARTNAME/PARAMS.vis
 * The environment variable SIM_DESC is also appended to output files/directories
 *
 * TODO: verification info needs to be generated per channel so it has to be
 * moved back to MemorySystem
 **/
void MultiChannelMemorySystem::InitOutputFiles(string traceFilename)
{
    size_t lastSlash;
    size_t deviceIniFilenameLength = deviceIniFilename.length();
    string sim_description_str;
    string deviceName;

    char* sim_description = getenv("SIM_DESC");
    if (sim_description)
    {
        sim_description_str = string(sim_description);
    }

    // create a properly named verification output file if need be and open iti
    // as the stream 'cmd_verify_out'
    if (VERIFICATION_OUTPUT)
    {
        string basefilename = deviceIniFilename.substr(deviceIniFilename.find_last_of("/") + 1);
        string verify_filename = "sim_out_" + basefilename;
        if (sim_description != NULL)
        {
            verify_filename += "." + sim_description_str;
        }
        verify_filename += ".tmp";
        cmd_verify_out.open(verify_filename.c_str());
        if (!cmd_verify_out)
        {
            ERROR("Cannot open " << verify_filename);
            abort();
        }
    }
    // This sets up the vis file output along with the creating the result directory
    // structure if it doesn't exist
    if (VIS_FILE_OUTPUT)
    {
        stringstream out, tmpNum;
        string path;
        string filename;

        if (!visFilename)
        {
            path = "results/";
            // chop off the .ini if it's there
            if (deviceIniFilename.substr(deviceIniFilenameLength - 4) == ".ini")
            {
                deviceName = deviceIniFilename.substr(0, deviceIniFilenameLength - 4);
                deviceIniFilenameLength -= 4;
            }

            // chop off everything past the last / (i.e. leave filename only)
            if ((lastSlash = deviceName.find_last_of("/")) != string::npos)
            {
                deviceName =
                    deviceName.substr(lastSlash + 1, deviceIniFilenameLength - lastSlash - 1);
            }

            string rest;
            // working backwards, chop off the next piece of the directory
            if ((lastSlash = traceFilename.find_last_of("/")) != string::npos)
            {
                traceFilename =
                    traceFilename.substr(lastSlash + 1, traceFilename.length() - lastSlash - 1);
            }
            if (sim_description != NULL)
            {
                traceFilename += "." + sim_description_str;
            }

            if (pwd.length() > 0)
            {
                path = pwd + "/" + path;
            }

            // create the directories if they don't exist
            mkdirIfNotExist(path);
            path = path + traceFilename + "/";
            mkdirIfNotExist(path);
            path = path + deviceName + "/";
            mkdirIfNotExist(path);

            // finally, figure out the filename
            string sched = "BtR";
            string queue = "pRank";
            if (configuration->SCHEDULING_POLICY == RankThenBankRoundRobin)
            {
                sched = "RtB";
            }
            if (configuration->QUEUING_STRUCTURE == PerRankPerBank)
            {
                queue = "pRankpBank";
            }

            /* I really don't see how "the C++ way" is better than snprintf()  */
            // out << (TOTAL_STORAGE >> 10) << "GB." << NUM_CHANS << "Ch." << NUM_RANKS << "R."
            //     << ADDRESS_MAPPING_SCHEME
            out << (getConfigParam(UINT64, "TOTAL_STORAGE") >> 10) << "GB."
                << getConfigParam(UINT, "NUM_CHANS") << "Ch." << getConfigParam(UINT, "NUM_RANKS")
                << "R." << getConfigParam(UINT, "ADDRESS_MAPPING_SCHEME") << "."
                << getConfigParam(UINT, "ROW_BUFFER_POLICY") << "."
                << getConfigParam(UINT, "TRANS_QUEUE_DEPTH") << "TQ."
                << getConfigParam(UINT, "CMD_QUEUE_DEPTH") << "CQ." << sched << "." << queue;
        }
        else  // visFilename given
        {
            out << *visFilename;
        }
        if (sim_description)
        {
            out << "." << sim_description;
        }

        // filename so far, without extension, see if it exists already
        filename = out.str();

        filename = FilenameWithNumberSuffix(filename, ".vis");
        path.append(filename);
        cerr << "writing vis file to " << path << endl;

        visDataOut.open(path.c_str());
        if (!visDataOut)
        {
            ERROR("Cannot open '" << path << "'");
            exit(-1);
        }
        // write out the ini config values for the visualizer tool
        ConfigurationDB& configDB = ConfigurationDB::getDB();
        configDB.dump(visDataOut);
    }
    else
    {
        // cerr << "vis file output disabled\n";
    }
    if (LOG_OUTPUT)
    {
        // string dramsimLogFilename("dramsim");
        string dramsimLogFilename(SIM_TRACE_FILE);

        if (sim_description != NULL)
        {
            dramsimLogFilename += "." + sim_description_str;
        }

        //    dramsimLogFilename = FilenameWithNumberSuffix(dramsimLogFilename, ".log");

        dramsimLog.open(dramsimLogFilename.c_str(), ios_base::out | ios_base::trunc);

        if (!dramsimLog)
        {
            ERROR("Cannot open " << dramsimLogFilename);
            //    exit(-1);
        }
    }
}

void MultiChannelMemorySystem::mkdirIfNotExist(string path)
{
    struct stat stat_buf;
    // check if the directory exists
    if (stat(path.c_str(), &stat_buf) != 0)  // nonzero return value on error, check errno
    {
        if (errno == ENOENT)
        {
            // DEBUG("\t directory doesn't exist, trying to create ...");
            // set permissions dwxr-xr-x on the results directories
            mode_t mode = (S_IXOTH | S_IXGRP | S_IXUSR | S_IROTH | S_IRGRP | S_IRUSR | S_IWUSR);
            if (mkdir(path.c_str(), mode) != 0)
            {
                perror("Error Has occurred while trying to make directory: ");
                cerr << path << endl;
                abort();
            }
        }
        else
        {
            perror("Something else when wrong: ");
            abort();
        }
    }
    else  // directory already exists
    {
        if (!S_ISDIR(stat_buf.st_mode))
        {
            ERROR(path << "is not a directory");
            abort();
        }
    }
}

MultiChannelMemorySystem::~MultiChannelMemorySystem()
{
    // delete clockDomainCrosser;
    delete[] numFence;
    delete addrMapping;

    for (size_t i = 0; i < configuration->NUM_CHANS; i++)
    {
        delete channels[i];
    }
    channels.clear();

    delete configuration;
    delete csvOut;
    // flush our streams and close them up
    if (LOG_OUTPUT)
    {
        dramsimLog.flush();
        dramsimLog.close();
    }

    if (VIS_FILE_OUTPUT)
    {
        visDataOut.flush();
        visDataOut.close();
    }
}

void MultiChannelMemorySystem::update()
{
    clockDomainCrosser.update();
}

void MultiChannelMemorySystem::actual_update()
{
    /*if (currentClockCycle == 0)
    {
        InitOutputFiles(traceFilename);
        DEBUG("DRAMSim2 Clock Frequency =" << clockDomainCrosser.clock1
                                           << "Hz, CPU Clock Frequency="
                                           << clockDomainCrosser.clock2 << "Hz");
    }*/

    /*if (currentClockCycle > 0 && currentClockCycle % configuration->EPOCH_LENGTH == 0)
    {
        (*csvOut) << "ms" << currentClockCycle * configuration->tCK * 1E-6;
        for (size_t i = 0; i < configuration->NUM_CHANS; i++)
        {
            channels[i]->printStats(false);
        }
        csvOut->finalize();
    }*/

    for (size_t i = 0; i < configuration->NUM_CHANS; i++)
    {
        channels[i]->update();
    }

    currentClockCycle++;
}

unsigned MultiChannelMemorySystem::findChannelNumber(uint64_t addr)
{
    // Single channel case is a trivial shortcut case
    if (configuration->NUM_CHANS == 1)
    {
        return 0;
    }

    /*if (!isPowerOfTwo(configuration->NUM_CHANS))
    {
        ERROR("We can only support power of two # of channels.\n"
              << "I don't know what Intel was thinking, but trying to address map half"
                 " a bit is a neat trick that we're not sure how to do");
        abort();
    }*/

    // only chan is used from this set
    unsigned channelNumber, rank, bank, row, col;
    addrMapping->addressMapping(addr, channelNumber, rank, bank, row, col);
    if (channelNumber >= configuration->NUM_CHANS)
    {
        ERROR("Got channel index " << channelNumber << " but only " << configuration->NUM_CHANS
                                   << " exist");
        abort();
    }
    return channelNumber;
}

ostream& MultiChannelMemorySystem::getLogFile()
{
    return dramsimLog;
}

bool MultiChannelMemorySystem::addTransaction(Transaction* trans)
{
    unsigned channelNumber = findChannelNumber(trans->address);
    return channels[channelNumber]->addTransaction(trans);
}

bool MultiChannelMemorySystem::addBarrier(int chanId)
{
    numFence[chanId]++;
    return channels[chanId]->addBarrier();
}

bool MultiChannelMemorySystem::addTransaction(bool isWrite, uint64_t addr, BurstType* data)
{
    unsigned channelNumber = findChannelNumber(addr);
    return channels[channelNumber]->addTransaction(isWrite, addr, data);
}

bool MultiChannelMemorySystem::addTransaction(bool isWrite, uint64_t addr, const std::string& tag,
                                              BurstType* data)
{
    unsigned channelNumber = findChannelNumber(addr);
    return channels[channelNumber]->addTransaction(isWrite, addr, tag, data);
}

//using data mode, we can do ...

void MultiChannelMemorySystem::printStats(bool finalStats)
{
    uint64_t cyclesElapsed;
    MemoryController* mem_ctrl;
    unsigned IDD3N, IDD3NC, IDD3NQ;
    float Vpp, Vddc, Vddq;

    IDD3N = getConfigParam(UINT, "IDD3N");
    IDD3NC = getConfigParam(UINT, "IDD3NC");
    IDD3NQ = getConfigParam(UINT, "IDD3NQ");
    Vpp = getConfigParam(FLOAT, "Vpp");
    Vddc = getConfigParam(FLOAT, "Vddc");
    Vddq = getConfigParam(FLOAT, "Vddq");

    bool printEnergy = false;
    uint64_t total_burstEnergy = 0;
    uint64_t total_actpreEnergy = 0;
    uint64_t total_aluPIMEnergy = 0;
    uint64_t total_energy = 0;
    double total_burstPower = 0.0;
    double total_actprePower = 0.0;
    double total_aluPIMPower = 0.0;
    double total_power = 0.0;
    double total_bandwidth = 0.0;
    uint64_t total_num_mac = 0;
    uint64_t totalReads = 0;
    uint64_t totalWrites = 0;

    (*csvOut) << "ms" << currentClockCycle * configuration->tCK * 1E-6;
    for (size_t i = 0; i < configuration->NUM_CHANS; i++)
    {
        PRINTC(PRINT_CHAN_STAT, "==== Channel [" << i << "] ====");
        channels[i]->printStats(finalStats);
        PRINTC(PRINT_CHAN_STAT, "//// Channel [" << i << "] ////");
    }

    for (size_t i = 0; i < getConfigParam(UINT, "NUM_CHANS"); i++)
    {
        mem_ctrl = channels[i]->memoryController;
        cyclesElapsed = (mem_ctrl->currentClockCycle % configuration->EPOCH_LENGTH == 0)
                            ? configuration->EPOCH_LENGTH
                            : mem_ctrl->currentClockCycle % configuration->EPOCH_LENGTH;
        for (size_t r = 0; r < getConfigParam(UINT, "NUM_RANKS"); r++)
        {
            total_burstEnergy += mem_ctrl->burstEnergy[r];
            total_actpreEnergy += mem_ctrl->actpreEnergy[r];
            total_aluPIMEnergy += mem_ctrl->aluPIMEnergy[r];
            total_burstPower += ((double)mem_ctrl->burstEnergy[r] / cyclesElapsed / 1000.0);
            total_actprePower += ((double)mem_ctrl->actpreEnergy[r] / cyclesElapsed / 1000.0);
            total_aluPIMPower += ((double)mem_ctrl->aluPIMEnergy[r] / cyclesElapsed / 1000.0);
        }
        totalReads += mem_ctrl->totalReads;
        totalWrites += mem_ctrl->totalWrites;

        total_bandwidth += mem_ctrl->totalBandwidth;

        mem_ctrl->resetStats();
    }

    backgroundPower = (IDD3N * Vpp + IDD3NC * Vddc + IDD3NQ * Vddq) / 1000.0;
    total_energy = total_burstEnergy + total_actpreEnergy + total_aluPIMEnergy;
    total_power = backgroundPower + total_burstPower + total_actprePower + total_aluPIMPower;

    PRINT("//// Simulation Results ////");
    PRINT("        Total Simulated Cycle (assuming 1GHz)  : " << currentClockCycle);
    PRINT("        Total Simulated Time (s)  : " << currentClockCycle * configuration->tCK * 1E-9);
    if (printEnergy)
    {
        PRINT("        Background Power(watts)   : " << backgroundPower);

        PRINT("        Total Power(watts)   : " << total_power);
        if (total_energy * 1E-9 < 1)
        {
            PRINT("        Total Energy(uJ)     : " << total_energy * 1E-6);
        }
        else
        {
            PRINT("        Total Energy(mJ)     : " << total_energy * 1E-9);
        }
    }
    unsigned JEDEC_DATA_BUS_BITS = getConfigParam(UINT, "JEDEC_DATA_BUS_BITS");
    unsigned BL = getConfigParam(UINT, "BL");
    PRINT("        Total Bandwidth(GB/s): " << (totalReads + totalWrites) * JEDEC_DATA_BUS_BITS *
                                                   BL / 8 /
                                                   (currentClockCycle * configuration->tCK));
    if (total_num_mac > 0)
    {
        PRINT("        Total Mac            : " << FormatWithCommas<uint64_t>(total_num_mac));
        PRINT("        TOPS/s            : "
              << (total_num_mac * 2 / 1e12) / (currentClockCycle * configuration->tCK * 1E-9));
    }

    PRINT("");

    csvOut->finalize();
}

void MultiChannelMemorySystem::RegisterCallbacks(
    TransactionCompleteCB* readDone, TransactionCompleteCB* writeDone,
    void (*reportPower)(double bgpower, double burstpower, double refreshpower, double actprepower))
{
    for (size_t i = 0; i < configuration->NUM_CHANS; i++)
    {
        channels[i]->RegisterCallbacks(readDone, writeDone, reportPower);
    }
}

int MultiChannelMemorySystem::hasPendingTransactions()
{
    int num = 0;
    for (auto chan : channels)
    {
        num += chan->numOnTheFlyTransactions;
    }
    return num;
}

bool MultiChannelMemorySystem::willAcceptTransaction(uint64_t addr)
{
    unsigned chan, rank, bank, row, col;
    addrMapping->addressMapping(addr, chan, rank, bank, row, col);
    return channels[chan]->WillAcceptTransaction();
}

bool MultiChannelMemorySystem::willAcceptTransaction()
{
    for (size_t c = 0; c < configuration->NUM_CHANS; c++)
    {
        if (!channels[c]->WillAcceptTransaction())
        {
            return false;
        }
    }
    return true;
}
