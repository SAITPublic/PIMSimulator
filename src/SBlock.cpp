#include <sstream>
#include <string>

#include "SBlock.h"
#include "PrintMacros.h"
#include "SystemConfiguration.h"
#include "half.h"

using namespace DRAMSim;

void SBlock::add(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst) 
{
    if (pimPrecision_ == FP16)
    {
        for (int i = 0; i < 16; i++)
        {
            dstBst.fp16Data_[i] = src0Bst.fp16Data_[i] + src1Bst.fp16Data_[i];
        }
    }
    else if (pimPrecision_ == FP32)
    {
        for (int i = 0; i < 8; i++)
        {
            dstBst.fp32Data_[i] = src0Bst.fp32Data_[i] + src1Bst.fp32Data_[i];
        }
    }
    else
        dstBst = src0Bst + src1Bst;
}

void SBlock::mul(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst)
{
    if (pimPrecision_ == FP16)
    {
        for (int i = 0; i < 16; i++)
        {
            dstBst.fp16Data_[i] = src0Bst.fp16Data_[i] * src1Bst.fp16Data_[i];
        }
    }
    else if (pimPrecision_ == FP32)
    {
        for (int i = 0; i < 8; i++)
        {
            dstBst.fp32Data_[i] = src0Bst.fp32Data_[i] * src1Bst.fp32Data_[i];
        }
    }
    else
        dstBst = src0Bst * src1Bst;
}

void SBlock::mac(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst)
{
    if (pimPrecision_ == FP16)
    {
        for (int i = 0; i < 16; i++)
        {
            dstBst.fp16Data_[i] = src0Bst.fp16Data_[i] * src1Bst.fp16Data_[i] + dstBst.fp16Data_[i];
        }

        DEBUG("MAC " << src0Bst.hexToStr2() << "*+" << src1Bst.hexToStr2() << ""
                     << dstBst.hexToStr2());
    }
    else if (pimPrecision_ == FP32)
    {
        for (int i = 0; i < 8; i++)
        {
            dstBst.fp32Data_[i] = src0Bst.fp32Data_[i] * src1Bst.fp32Data_[i] + dstBst.fp32Data_[i];
        }
    }
    else
        dstBst = src0Bst * src1Bst + dstBst;
}
void SBlock::burstmax(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst) //don't need solely..
{
    //maybe using this as expf or gelu or some nonlinear functions to ....
    if (pimPrecision_ == FP16)
    {
        for (int i = 0; i < 16; i++)
        {
            dstBst.fp16Data_[i] = (src0Bst.fp16Data_[i] > src1Bst.fp16Data_[i])?src0Bst.fp16Data_[i]:src1Bst.fp16Data_[i];
        }
    }
}
std::string SBlock::print()
{
    stringstream ss;
    ss << "[BLF]" << blf.binToStr();
    for (int i = 0; i < 4; i++) ss << grf[i].binToStr();

    return ss.str();
}