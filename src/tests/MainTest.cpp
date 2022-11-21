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
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <random>

#include "Burst.h"
#include "MultiChannelMemorySystem.h"
#include "gtest/gtest.h"
#include "tests/PIMKernel.h"
#include "tests/TestCases.h"

using namespace std;

int main(int argc, char* argv[])
{
    if (argc > 1)
    {
        ::testing::InitGoogleTest(&argc, argv);
    }
    else
    {
        ::testing::InitGoogleTest(&argc, argv);
        testing::GTEST_FLAG(filter) = "-*bw*.*";
    }

    return RUN_ALL_TESTS();
}
