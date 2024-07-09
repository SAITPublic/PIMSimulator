#ifndef __S_BLOCK_HPP__
#define __S_BLOCK_HPP__

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>

#include "Burst.h"
#include "SystemConfiguration.h"

using namespace std;

namespace DRAMSim{
class SBlock{
  public:
    SBlock()
    {
        pimPrecision_ = PIMConfiguration::getPIMPrecision();
    }
    SBlock(const PIMPrecision& pimPrecision) : pimPrecision_(pimPrecision) {}

    BurstType grf[4];
    BurstType blf;

    void add(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst);
    void mac(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst);
    void mul(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst);
    void burstmax(BurstType& dstBst, BurstType& src0Bst, BurstType& src1Bst);
    std::string print();
  private:
    PIMPrecision pimPrecision_;
};

}
#endif