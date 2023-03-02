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

#ifndef __PIM_BENCH_TEST_CASE_H__
#define __PIM_BENCH_TEST_CASE_H__

#include <memory>
#include <string>

#include "tests/TestCases.h"

using namespace DRAMSim;

class PIMBenchTestCase
{
  public:
    PIMBenchTestCase(KernelType k, unsigned b, unsigned out, unsigned in)
        : kernel_type_(k), batch_(b), out_(out), in_(in)
    {
        mem_ = make_shared<MultiChannelMemorySystem>("ini/HBM2_samsung_2M_16B_x64.ini",
                                                     "system_hbm_64ch.ini", ".", "example_app",
                                                     256 * 64 * 2);
        pim_mem_ = make_shared<MultiChannelMemorySystem>("ini/HBM2_samsung_2M_16B_x64.ini",
                                                         "system_hbm_64ch.ini", ".", "example_app",
                                                         256 * 64 * 2);
        // # of pim channel = 64, # of pim rank = 1
        kernel_ = make_shared<PIMKernel>(pim_mem_, 64, 1);
        dim_data_ = new DataDim(kernel_type_, batch_, out_, in_, false);
    }

    virtual ~PIMBenchTestCase()
    {
        mem_->printStats(true);
        pim_mem_->printStats(true);
        delete dim_data_;
    }

    virtual uint64_t measureCycle(bool is_pim_) = 0;

    void run(shared_ptr<MultiChannelMemorySystem> mem_, uint64_t *cycle)
    {
        while (mem_->hasPendingTransactions())
        {
            mem_->update();
            (*cycle)++;
        }
    }

    uint64_t genMemTraffic(shared_ptr<MultiChannelMemorySystem> mem_, bool is_write,
                           uint32_t data_size_in_bytes, uint64_t starting_addr)
    {
        unsigned basic_stride =
            (getConfigParam(UINT, "JEDEC_DATA_BUS_BITS") * getConfigParam(UINT, "BL") / 8);
        BurstType null_bst;
        uint64_t addr;

        for (addr = starting_addr; addr < starting_addr + data_size_in_bytes; addr += basic_stride)
        {
            mem_->addTransaction(is_write, addr, &null_bst);
        }
        return addr;
    }

    string kernelTypetoStr(KernelType k)
    {
        if (k == KernelType::GEMV)
        {
            return string{"GEMV"};
        }
        else if (k == KernelType::MUL)
        {
            return string{"MUL"};
        }
        else if (k == KernelType::ADD)
        {
            return string{"ADD"};
        }
        else if (k == KernelType::RELU)
        {
            return string{"RELU"};
        }
        else
        {
            throw invalid_argument("Invalid kernel type");
        }
    }

    void printTestMessage(bool is_pim_)
    {
        cout << "  " << kernelTypetoStr(kernel_type_) << " (PIM "
             << (is_pim_ ? "enabled)" : "disabled)") << endl;
        dim_data_->printDim(kernel_type_);
    }

  protected:
    KernelType kernel_type_;
    unsigned batch_;
    unsigned in_;
    unsigned out_;
    bool is_pim_;

    shared_ptr<PIMKernel> kernel_;
    shared_ptr<MultiChannelMemorySystem> mem_, pim_mem_;
    DataDim *dim_data_;
};

class GemvPIMBenchTest : public PIMBenchTestCase
{
  public:
    GemvPIMBenchTest(KernelType k, unsigned b, unsigned out, unsigned in)
        : PIMBenchTestCase(k, b, out, in)
    {
    }

    uint64_t measureCycle(bool is_pim_)
    {
        uint64_t cycle = 0;
        uint64_t starting_addr = 0;

        if (is_pim_ == true)
        {
            kernel_->executeGemv(&dim_data_->weight_npbst_, &dim_data_->input_npbst_, false);
            kernel_->runPIM();
            cycle = kernel_->getCycle();
        }
        else
        {
            uint32_t input_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->input_dim_, dim_data_->batch_size_);
            uint32_t output_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->output_dim_, dim_data_->batch_size_);
            uint32_t weight_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->output_dim_, dim_data_->input_dim_);
            starting_addr = genMemTraffic(mem_, false, weight_data_size_in_byte, starting_addr);
            starting_addr = genMemTraffic(mem_, false, input_data_size_in_byte, starting_addr);
            run(mem_, &cycle);
            genMemTraffic(mem_, true, output_data_size_in_byte, starting_addr);  // result-vec
            run(mem_, &cycle);
        }
        return cycle;
    }
};

class EltPIMBenchTest : public PIMBenchTestCase
{
  public:
    EltPIMBenchTest(KernelType k, unsigned b, unsigned out, unsigned in)
        : PIMBenchTestCase(k, b, out, in)
    {
        input_row0_ = 0;
        input_row1_ = 128;
        result_row_ = 256;
    }

    uint64_t measureCycle(bool is_pim_ = false)
    {
        uint64_t cycle = 0;
        uint64_t starting_addr = 0;

        if (is_pim_ == true)
        {
            kernel_->executeEltwise(dim_data_->output_npbst_.getTotalDim(), pimBankType::ALL_BANK,
                                    kernel_type_, input_row0_, result_row_, input_row1_);
            kernel_->runPIM();
            cycle = kernel_->getCycle();
        }
        else
        {
            uint32_t input_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->input_dim_, dim_data_->batch_size_);
            uint32_t input1_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->input_dim_, dim_data_->batch_size_);
            uint32_t output_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->output_dim_, dim_data_->batch_size_);
            starting_addr = genMemTraffic(mem_, false, input_data_size_in_byte, starting_addr);
            starting_addr = genMemTraffic(mem_, false, input1_data_size_in_byte, starting_addr);
            run(mem_, &cycle);
            genMemTraffic(mem_, true, output_data_size_in_byte, starting_addr);  // result-vec
            run(mem_, &cycle);
        }
        return cycle;
    }

  private:
    // for PIM
    unsigned input_row0_;
    unsigned input_row1_;
    unsigned result_row_;
};

class ActPIMBenchTest : public PIMBenchTestCase
{
  public:
    ActPIMBenchTest(KernelType k, unsigned b, unsigned out, unsigned in)
        : PIMBenchTestCase(k, b, out, in)
    {
        input_row0_ = 0;
        result_row_ = 256;
    }

    uint64_t measureCycle(bool is_pim_ = false)
    {
        uint64_t cycle = 0;
        uint64_t starting_addr = 0;

        if (is_pim_ == true)
        {
            kernel_->executeEltwise(dim_data_->output_npbst_.getTotalDim(), pimBankType::ALL_BANK,
                                    KernelType::RELU, input_row0_, result_row_, 0);
            kernel_->runPIM();
            cycle = kernel_->getCycle();
        }
        else
        {
            uint32_t input_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->input_dim_, dim_data_->batch_size_);
            uint32_t output_data_size_in_byte =
                dim_data_->getDataSize(dim_data_->output_dim_, dim_data_->batch_size_);
            starting_addr = genMemTraffic(mem_, false, input_data_size_in_byte, starting_addr);
            run(mem_, &cycle);
            genMemTraffic(mem_, true, output_data_size_in_byte, starting_addr);  // result-vec
            run(mem_, &cycle);
        }
        return cycle;
    }

  private:
    // for PIM
    unsigned input_row0_;
    unsigned result_row_;
};

class PIMBenchFixture : public testing::Test
{
  public:
    virtual void SetUp()
    {
        pim_cycle_ = 0;
        non_pim_cycle_ = 0;
        perfTest = nullptr;
        printTestMessage();
    }

    virtual void TearDown()
    {
        printResult(non_pim_cycle_ / static_cast<float>(pim_cycle_));
        delete perfTest;
    }

    void setPIMBenchTestCase(KernelType k, unsigned out, unsigned in, unsigned batch = 1)
    {
        if (k == KernelType::GEMV)
        {
            perfTest = new GemvPIMBenchTest(k, batch, out, in);
        }
        else if (k == KernelType::MUL || k == KernelType::ADD)
        {
            perfTest = new EltPIMBenchTest(k, batch, out, in);
        }
        else if (k == KernelType::RELU)
        {
            perfTest = new ActPIMBenchTest(k, batch, out, in);
        }
        else
        {
            throw invalid_argument("Invalid kernel type");
        }
    }

    void executePIMKernel(void)
    {
        perfTest->printTestMessage(true);
        pim_cycle_ = perfTest->measureCycle(true);
        printStats(pim_cycle_);
    }

    void executeKernel(void)
    {
        perfTest->printTestMessage(false);
        non_pim_cycle_ = perfTest->measureCycle(false);
        printStats(non_pim_cycle_);
    }

    void expectPIMBench(float expected_perf_gain)
    {
        EXPECT_TRUE((float)non_pim_cycle_ / pim_cycle_ > expected_perf_gain)
            << (float)non_pim_cycle_ / pim_cycle_ << endl;
    }

    void printTestMessage()
    {
        cout << ">>Performance Test" << endl;
    }

    void printStats(uint64_t cycle)
    {
        cout << "> Test Results " << endl;
        cout << "> Cycle : " << cycle << endl;
        cout << endl;
    }

    void printResult(float gain)
    {
        cout << "> Speed-up : " << gain << endl;
    }

  private:
    uint64_t pim_cycle_, non_pim_cycle_;
    PIMBenchTestCase *perfTest;
};

#endif /*__PIM_BENCH_TEST_CASE_H__*/
