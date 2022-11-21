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

#include <iomanip>

#include "AddressMapping.h"
#include "tests/PIMKernel.h"

unsigned PIMAddrManager::maskByBit(unsigned value, int start, int end)
{
    int length = start - end + 1;
    value = value >> end;
    return value & ((1 << length) - 1);
}

uint64_t PIMAddrManager::addrGen(unsigned chan, unsigned rank, unsigned bankgroup, unsigned bank,
                                 unsigned row, unsigned col)
{
    uint64_t addr = 0;
    if (address_mapping_scheme_ == Scheme8)
    {
        addr = rank;

        addr <<= num_row_bits_;
        addr |= row;

        addr <<= num_col_bits_;
        addr |= col;

        addr <<= num_bankgroup_bits_;
        addr |= bankgroup;

        addr <<= num_bank_bits_;
        addr |= bank;

        addr <<= num_chan_bits_;
        addr |= chan;

        addr <<= num_offset_bits_;
    }
    else
    {
        cerr << "Fatal: Not supported address scheme for PIM controller" << endl;
    }
    return addr;
}

uint64_t PIMAddrManager::addrGenSafe(unsigned chan, unsigned rank, unsigned bankgroup,
                                     unsigned bank, unsigned& row, unsigned& col)
{
    while (col >= num_cols_per_bl_)
    {
        row++;
        col -= (num_cols_per_bl_);
    }

    if (row >= num_rows_)
    {
        cerr << "row overflow" << endl;
    }
    return addrGen(chan, rank, bankgroup, bank, row, col);
}
