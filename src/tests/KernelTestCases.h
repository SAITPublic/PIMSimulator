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

#ifndef __KERNEL_TEST_CASES_H__
#define __KERNEL_TEST_CASES_H__

#include <memory>
#include <vector>

#include "tests/TestCases.h"

using namespace DRAMSim;

// A predicate-formatter for asserting that two integers are mutually prime.
::testing::AssertionResult fp16EqualHelper(const char* m_expr, const char* n_expr, fp16 m, fp16 n);
::testing::AssertionResult fp16BstEqualHelper(const char* m_expr, const char* n_expr,
                                              DRAMSim::BurstType mb, DRAMSim::BurstType nb);
#define EXPECT_FP16_BST_EQ(val1, val2) EXPECT_PRED_FORMAT2(fp16BstEqualHelper, val1, val2)
#define EXPECT_FP16_EQ(val1, val2) EXPECT_PRED_FORMAT2(fp16EqualHelper, val1, val2)

class TestStats
{
  private:
    unsigned num_test_passed_;
    unsigned num_test_warning_;
    unsigned num_test_failed_;

    vector<unsigned> fail_idx_;
    vector<float> fail_data_sim_;
    vector<float> fail_data_numpy_;

  public:
    TestStats() {}
    unsigned getNumPassed()
    {
        return num_test_passed_;
    }
    unsigned getNumWarning()
    {
        return num_test_warning_;
    }
    unsigned getNumFailed()
    {
        return num_test_failed_;
    }

    void IncNumPassed(unsigned i = 1)
    {
        num_test_passed_ += i;
    }
    void IncNumWarning(unsigned i = 1)
    {
        num_test_warning_ += i;
    }
    void IncNumFailed(unsigned i = 1)
    {
        num_test_failed_ += i;
    }

    void clear()
    {
        num_test_passed_ = 0;
        num_test_warning_ = 0;
        num_test_failed_ = 0;
        fail_idx_.clear();
        fail_data_sim_.clear();
        fail_data_numpy_.clear();
    };

    void insertToFailVector(unsigned idx, float m, float n)
    {
        fail_idx_.push_back(idx);
        fail_data_sim_.push_back(m);
        fail_data_numpy_.push_back(n);
    }

    void printFailVector()
    {
        for (int i = 0; i < fail_idx_.size(); i++)
        {
            cout << " idx : " << fail_idx_[i] << " sim : " << fail_data_sim_[i]
                 << " npy : " << fail_data_numpy_[i] << endl;
        }
    };
};

TestStats testStats;

class PIMKernelFixture : public testing::Test
{
  public:
    PIMKernelFixture() {}
    ~PIMKernelFixture() {}

    virtual void SetUp()
    {
        printTestMessage();
    }

    virtual void TearDown()
    {
        printResult();
        printFailVector();
    }

    BurstType* getResultPIM(KernelType kn_type, DataDim* dim_data, shared_ptr<PIMKernel> kernel,
                            BurstType* result)
    {
        switch (kn_type)
        {
            case KernelType::GEMV:
            {
                kernel->preloadGemv(&dim_data->weight_npbst_);
                kernel->executeGemv(&dim_data->weight_npbst_, &dim_data->input_npbst_, false);
                unsigned end_col = kernel->getResultColGemv(
                    dim_data->dimTobShape(dim_data->input_dim_), dim_data->output_dim_);
                result = new BurstType[dim_data->output_dim_ * dim_data->batch_size_];
                kernel->readResult(result, pimBankType::ODD_BANK,
                                   dim_data->output_dim_ * dim_data->batch_size_, 0, 0, end_col);
                break;
            }
            case KernelType::ADD:
            case KernelType::MUL:
            {
                int input_row0 = 0;
                int input_row1 = 128;
                int result_row = 256;
                kernel->preloadNoReplacement(&dim_data->input_npbst_, input_row0, 0);
                kernel->preloadNoReplacement(&dim_data->input1_npbst_, input_row1, 0);
                kernel->executeEltwise(dim_data->dimTobShape(dim_data->output_dim_),
                                       pimBankType::ALL_BANK, kn_type, input_row0, result_row,
                                       input_row1);
                result = new BurstType[dim_data->output_dim_];
                kernel->readData(result, dim_data->dimTobShape(dim_data->output_dim_), result_row,
                                 0);
                break;
            }
            case KernelType::RELU:
            {
                int input_row0 = 0;
                int result_row = 256;
                kernel->preloadNoReplacement(&dim_data->input_npbst_, input_row0, 0);
                kernel->executeEltwise(dim_data->dimTobShape(dim_data->output_dim_),
                                       pimBankType::ALL_BANK, kn_type, input_row0, result_row);
                result = new BurstType[dim_data->output_dim_];
                kernel->readData(result, dim_data->dimTobShape(dim_data->output_dim_), result_row,
                                 0);
                break;
            }
            case KernelType::GEMVTREE:
            default:
            {
                ERROR("== Error - Unknown KernelType trying to run");
                break;
            }
        }
        kernel->runPIM();
        return result;
    }

    void expectAccuracy(KernelType kn_type, int num_tests, NumpyBurstType precalculated_result,
                        uint32_t stride = 16)
    {
        switch (kn_type)
        {
            case KernelType::GEMV:
            {
                for (int i = 0; i < num_tests; i++)
                {
                    EXPECT_FP16_EQ(result_[i].fp16ReduceSum(),
                                   precalculated_result.getBurst(0).fp16Data_[i]);
                    reduced_result_[i / stride].fp16Data_[i % stride] = result_[i].fp16ReduceSum();
                }
                return;
            }
            case KernelType::ADD:
            case KernelType::MUL:
            case KernelType::RELU:
            {
                for (int i = 0; i < num_tests; i++)
                {
                    EXPECT_FP16_BST_EQ(result_[i], precalculated_result.getBurst(i));
                }
                return;
            }
            default:
            {
                ERROR("== Error - Unknown KernelType trying to run");
                return;
            }
        }
    }

    shared_ptr<PIMKernel> make_pim_kernel()
    {
        shared_ptr<MultiChannelMemorySystem> mem = make_shared<MultiChannelMemorySystem>(
            "ini/HBM2_samsung_2M_16B_x64.ini", "system_hbm_64ch.ini", ".", "example_app",
            256 * 64 * 2);
        int numPIMChan = 64;
        int numPIMRank = 1;
        shared_ptr<PIMKernel> kernel = make_shared<PIMKernel>(mem, numPIMChan, numPIMRank);

        return kernel;
    }

    /* result data */
    BurstType* result_;
    BurstType* reduced_result_;

    /* stats */
    void testStatsClear()
    {
        testStats.clear();
    }

    void printTestMessage()
    {
        cout << ">>PIM Kernel Accuraccy Test" << endl;
    }

    void printFailVector()
    {
        testStats.printFailVector();
    }

    void printResult()
    {
        cout << endl;
        cout << "> Test Result" << endl;
        cout << "  simulated output comparison via pre-calculated values" << endl;
        cout << "> passed : " << testStats.getNumPassed() << endl;
        cout << "> failed : " << testStats.getNumFailed() << endl;
    }
};

#define GET_NUM_TESTS() \
    testStats.getNumPassed() + testStats.getNumWarning() + testStats.getNumFailed()
#define INC_NUM_PASSED() testStats.IncNumPassed(1)
#define INC_NUM_WARNING() testStats.IncNumWarning(1)
#define INC_NUM_FAILED() testStats.IncNumFailed(1)
#define INSERT_TO_FAILED_VECTOR(var1, var2) \
    testStats.insertToFailVector(GET_NUM_TESTS(), var1, var2)

// A predicate-formatter for asserting that two integers are mutually prime.
::testing::AssertionResult fp16EqualHelper(const char* m_expr, const char* n_expr, fp16 m, fp16 n)
{
    fp16i mi(m);
    fp16i ni(n);
    unsigned cur_idx = GET_NUM_TESTS();
    if (fp16Equal(m, n, 4, 0.01))
    {
        INC_NUM_PASSED();
        return ::testing::AssertionSuccess();
    }
    else if (fp16Equal(m, n, 256, 0.7))
    {
        INC_NUM_PASSED();
        return ::testing::AssertionSuccess();
    }
    else
    {
        INSERT_TO_FAILED_VECTOR(convertH2F(m), convertH2F(n));
        INC_NUM_FAILED();
        return ::testing::AssertionFailure()
               << cur_idx << m_expr << " and " << n_expr << " (" << convertH2F(m) << " and "
               << convertH2F(n) << ") are not same " << mi.ival << " " << ni.ival;
    }
}

::testing::AssertionResult fp16BstEqualHelper(const char* m_expr, const char* n_expr, BurstType mb,
                                              BurstType nb)
{
    for (int i = 0; i < 16; i++)
    {
        fp16 m = mb.fp16Data_[i];
        fp16 n = nb.fp16Data_[i];
        fp16i mi(m);
        fp16i ni(n);

        if (fp16Equal(m, n, 4, 0.01))
        {
            INC_NUM_PASSED();
        }
        else if (fp16Equal(m, n, 256, 0.7))
        {
            INC_NUM_PASSED();
        }
        else
        {
            INC_NUM_FAILED();
            INSERT_TO_FAILED_VECTOR(convertH2F(m), convertH2F(n));
            return ::testing::AssertionFailure()
                   << m_expr << " and " << n_expr << " (" << convertH2F(m) << " and "
                   << convertH2F(n) << ") are not same " << mi.ival << " " << ni.ival;
        }
    }

    return ::testing::AssertionSuccess();
}
#endif
