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

#include "BusPacket.h"

using namespace DRAMSim;
using namespace std;

BusPacket::BusPacket(BusPacketType packtype, uint64_t physicalAddr, unsigned col, unsigned rw,
                     unsigned r, unsigned b, BurstType* dat, ostream& simLog)
    : dramsimLog(simLog),
      busPacketType(packtype),
      column(col),
      row(rw),
      bank(b),
      rank(r),
      physicalAddress(physicalAddr),
      data(dat)
{
}
BusPacket::BusPacket(BusPacketType packtype, uint64_t physicalAddr, unsigned col, unsigned rw,
                     unsigned r, unsigned b, BurstType* dat, ostream& simLog, std::string tg)
    : dramsimLog(simLog),
      busPacketType(packtype),
      column(col),
      row(rw),
      bank(b),
      rank(r),
      physicalAddress(physicalAddr),
      data(dat),
      tag(tg)
{
}

/*BusPacket::BusPacket(BusPacketType packtype, uint64_t physicalAddr, unsigned col, unsigned rw, unsigned r,
              unsigned b, unsigned s, BurstType* dat, ostream& simLog, std::string tg)
    : dramsimLog(simLog),
      BusPacketType(packtype),
      column(col),
      row(rw),
      subarray(s),
      bank(b),
      physicalAddress(physicalAddr),
      data(dat),
      tag(tg)
{
} //nothing help b*/
void BusPacket::print(uint64_t currentClockCycle, bool dataStart)
{
    if (VERIFICATION_OUTPUT)
    {
        switch (busPacketType)
        {
            case READ:
                cmd_verify_out << currentClockCycle << ": read (" << rank << "," << bank << ","
                               << column << ",0);" << endl;
                break;
            case WRITE:
                cmd_verify_out << currentClockCycle << ": write (" << rank << "," << bank << ","
                               << column << ",0 , 0, 'h0);" << endl;
                break;
            case ACTIVATE:
                cmd_verify_out << currentClockCycle << ": activate (" << rank << "," << bank << ","
                               << row << ");" << endl;
                break;
            case PRECHARGE:
                cmd_verify_out << currentClockCycle << ": precharge (" << rank << "," << bank << ","
                               << row << ");" << endl;
                break;
            case REF:
                cmd_verify_out << currentClockCycle << ": refresh (" << rank << ");" << endl;
                break;
            case RFCSB:
                cmd_verify_out << currentClockCycle << ": refresh single bank (" << rank << ","
                               << bank << ");" << endl;
                break;
            case DATA:
                // TODO: data verification?
                break;
            default:
                ERROR("Trying to print unknown kind of bus packet");
                exit(-1);
        }
    }
}
void BusPacket::print()
{
    switch (busPacketType)
    {
        case READ:
            PRINT("BP [READ] pa[0x" << hex << physicalAddress << dec << "] r[" << rank << "] b["
                                    << bank << "] row[" << row << "] col[" << column << "]");
            break;
        case WRITE:
            PRINT("BP [WRITE] pa[0x" << hex << physicalAddress << dec << "] r[" << rank << "] b["
                                     << bank << "] row[" << row << "] col[" << column << "]");
            break;
        case ACTIVATE:
            PRINT("BP [ACT] pa[0x" << hex << physicalAddress << dec << "] r[" << rank << "] b["
                                   << bank << "] row[" << row << "] col[" << column << "]");
            break;
        case PRECHARGE:
            PRINT("BP [PRE] pa[0x" << hex << physicalAddress << dec << "] r[" << rank << "] b["
                                   << bank << "] row[" << row << "] col[" << column << "]");
            break;
        case REF:
            PRINT("BP [REF] pa[0x" << hex << physicalAddress << dec << "] r[" << rank << "] b["
                                   << bank << "] row[" << row << "] col[" << column << "]");
            break;
        case RFCSB:
            PRINT("BP [RFCSB] pa[0x" << hex << physicalAddress << dec << "] r[" << rank << "] b["
                                     << bank << "] row[" << row << "] col[" << column << "]");
            break;
        case DATA:
            PRINTN("BP [DATA] pa[0x" << hex << physicalAddress << dec << "] r[" << rank << "] b["
                                     << bank << "] row[" << row << "] col[" << column << "] data["
                                     << data->fp16ToStr() << "]=");
            printData();
            PRINT("");
            break;
        default:
            ERROR("Trying to print unknown kind of bus packet");
            exit(-1);
    }
}
void BusPacket::printData() const
{
    PRINTN(data->fp16ToStr());
}
