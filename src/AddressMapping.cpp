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

#include <bitset>
#include <iostream>

#include "AddressMapping.h"
#include "SystemConfiguration.h"
#include "Utils.h"

using namespace std;

namespace DRAMSim
{
AddrMapping::AddrMapping()
{
    transactionSize = getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") / 8 * getConfigParam(UINT, "BL");
    // ex: (64 bit bus width) x (8 Burst Length) - 1 = 64 bytes - 1 = 63 = 0x3f mask
    transactionMask = transactionSize - 1;
    channelBitWidth = uLog2(getConfigParam(UINT, "NUM_CHANS"));
    rankBitWidth = uLog2(getConfigParam(UINT, "NUM_RANKS"));
    bankBitWidth = uLog2(getConfigParam(UINT, "NUM_BANKS"));
    bankgroupBitWidth = uLog2(getConfigParam(UINT, "NUM_BANK_GROUPS"));
    //subarrayBitWidth = uLog2(getConfigParam(UINT, "NUM_SUBARRAYS")); //how we make, how about we use ...
    rowBitWidth = uLog2(getConfigParam(UINT, "NUM_ROWS"));
    colBitWidth = uLog2(getConfigParam(UINT, "NUM_COLS"));
    byteOffsetWidth = uLog2(getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") / 8);
    unsigned throw_away_bits = uLog2(transactionSize);
    colLowBitWidth = (throw_away_bits - byteOffsetWidth);
    colHighBitWidth = colBitWidth - colLowBitWidth;
    num_chans_ = getConfigParam(UINT, "NUM_CHANS");
    num_bank_per_bg_ = getConfigParam(UINT, "NUM_BANKS") / getConfigParam(UINT, "NUM_BANK_GROUPS");
    addressMappingScheme = PIMConfiguration::getAddressMappingScheme();
}

unsigned AddrMapping::bankgroupId(int bank)
{
    /* TODO : shcha : Updating an address scheme may require changes. */
    return (bank / num_bank_per_bg_);
}

bool AddrMapping::isSameBankgroup(int bank0, int bank1)
{
    return bankgroupId(bank0) == bankgroupId(bank1);
}
unsigned AddrMapping::findsubarray(unsigned row)
{
    if(row < 0x2000)  return 0;
    else if(row<0x4000) return 1;
    else if(row<0x6000) return 2;
    else return 3;
}

bool AddrMapping::isSameSubarray(int row, int sub)
{
    return findsubarray(row) == sub;
}

void AddrMapping::addressMapping(uint64_t physicalAddress, unsigned& newTransactionChan,
                                 unsigned& newTransactionRank, unsigned& newTransactionBank,
                                 unsigned& newTransactionRow, unsigned& newTransactionColumn)
{
    if ((physicalAddress & transactionMask) != 0)
    {
        DEBUG("WARNING: address 0x" << std::hex << physicalAddress << std::dec
                                    << " is not aligned to the request size of "
                                    << transactionSize);
    }
    // each burst will contain JEDEC_DATA_BUS_BITS/8 bytes of data, so the bottom
    // bits (3 bits for a single channel DDR system) are thrown away before mapping the other bits
    physicalAddress >>= byteOffsetWidth;

    // The next thing we have to consider is that when a request is made for a
    // we've taken into account the granulaity of a single burst by shifting
    // off the bottom 3 bits, but a transaction has to take into account the
    // burst length (i.e. the requests will be aligned to cache line sizes which
    // should be equal to transactionSize above).
    //
    // Since the column address increments internally on bursts, the bottom n
    // bits of the column (colLow) have to be zero in order to account for the
    // total size of the transaction. These n bits should be shifted off the
    // address and also subtracted from the total column width.
    //
    // I am having a hard time explaining the reasoning here, but it comes down
    // this: for a 64 byte transaction, the bottom 6 bits of the address must be
    // zero. These zero bits must be made up of the byte offset (3 bits) and
    // also
    // from the bottom bits of the column
    //
    // For example: cowLowBits = log2(64bytes) - 3 bits = 3 bits

    physicalAddress >>= colLowBitWidth;

    if (DEBUG_ADDR_MAP)
    {
        DEBUG("Bit widths: ch:" << channelBitWidth << " r:" << rankBitWidth << " b:" << bankBitWidth
                                << " row:" << rowBitWidth << " colLow:" << colLowBitWidth
                                << " colHigh:" << colHighBitWidth << " off:" << byteOffsetWidth
                                << " Total:"
                                << (channelBitWidth + rankBitWidth + bankBitWidth + rowBitWidth +
                                    colLowBitWidth + colHighBitWidth + byteOffsetWidth));
    }

    // perform various address mapping schemes
    if (addressMappingScheme == Scheme1)
    {
        // chan:rank:row:col:bank
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth);
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
    }
    else if (addressMappingScheme == Scheme2)
    {
        // chan:row:col:bank:rank
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth);
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
    }
    else if (addressMappingScheme == Scheme3)
    {
        // chan:rank:bank:col:row
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth);
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
    }
    else if (addressMappingScheme == Scheme4)
    {
        // chan:rank:bank:row:col
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth);
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
    }
    else if (addressMappingScheme == Scheme5)
    {
        // chan:row:col:rank:bank
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth);
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
    }
    else if (addressMappingScheme == Scheme6)
    {
        // chan:row:bank:rank:col
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth);
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
    }
    // clone of scheme 5, but channel moved to lower bits
    else if (addressMappingScheme == Scheme7) //how about using this logic?
    {
        // row:col:rank:bank:chan
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth);
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
    }
    else if (addressMappingScheme == Scheme8)
    {
        // rank:row:col:bg:bank:chan
        newTransactionChan = diffBitWidth(&physicalAddress, channelBitWidth);
        newTransactionBank = diffBitWidth(&physicalAddress, bankBitWidth - bankgroupBitWidth);
        newTransactionBank |= diffBitWidth(&physicalAddress, bankgroupBitWidth)
                              << (bankBitWidth - bankgroupBitWidth);
        newTransactionColumn = diffBitWidth(&physicalAddress, colHighBitWidth);
        newTransactionRow = diffBitWidth(&physicalAddress, rowBitWidth);
        newTransactionRank = diffBitWidth(&physicalAddress, rankBitWidth);
    }

    else
    {
        ERROR("== Error - Unknown Address Mapping Scheme");
        exit(-1);
    }

    if (DEBUG_ADDR_MAP)
    {
        DEBUG("Mapped Ch=" << newTransactionChan << " Rank=" << newTransactionRank
                           << " Bank=" << newTransactionBank << " Row=" << newTransactionRow
                           << " Col=" << newTransactionColumn << "\n");
    }
}
};  // namespace DRAMSim
