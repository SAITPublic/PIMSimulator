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

#include "tests/PIMCmdGen.h"

#include <memory>

vector<PIMCmd> PIMCmdGen::getPIMCmds(KernelType ktype, int num_jump_to_be_taken,
                                     int num_jump_to_be_taken_odd_bank,
                                     int num_jump_to_be_taken_even_bank)
{
    unique_ptr<IPIMCmd> pim_kernel = nullptr;
    switch (ktype)
    {
        /*
        case KernelType::BN:
            pim_kernel = make_unique<BatchNormPIMKernel>(ktype);
            break;
        */
        case KernelType::RELU:
            pim_kernel = make_unique<ActPIMKernel>(ktype);
            break;
        case KernelType::MUL:
            pim_kernel = make_unique<EltwisePIMKernel>(ktype);
            break;
        case KernelType::ADD:
            pim_kernel = make_unique<EltwisePIMKernel>(ktype);
            break;
        case KernelType::GEMV:
            pim_kernel = make_unique<GemvPIMKernel>(ktype);
            break;
        case KernelType::GEMVTREE:
            pim_kernel = make_unique<GemvPIMKernel>(ktype);
            break;
        default:
            throw invalid_argument("Invalid kernel type");
    }
    return pim_kernel->generateKernel(num_jump_to_be_taken, num_jump_to_be_taken_odd_bank,
                                      num_jump_to_be_taken_even_bank);
}
