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

#include "gtest/gtest.h"
#include "tests/TestCases.h"

/*
 * MemTest:
 * memory bandwidth test for normal hbm
 */

using namespace DRAMSim;

TEST_F(MemBandwidthFixture, hbm_read_bandwidth)
{
    setDataSize(128 * 1024 * 64);  // in bytes
    uint64_t bw = measureCycle(false);
    float effective_bw_ratio = 0.8;
    EXPECT_TRUE(bw > 256 * effective_bw_ratio);
}

TEST_F(MemBandwidthFixture, hbm_write_bandwidth)
{
    setDataSize(256 * 1024 * 64);  // in bytes
    uint64_t bw = measureCycle(true);
    float effective_bw_ratio = 0.8;
    EXPECT_TRUE(bw > 256 * effective_bw_ratio);
}
