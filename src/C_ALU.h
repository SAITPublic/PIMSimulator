#ifndef C_ALU_H
#define C_ALU_H

#include <vector>

#include "Burst.h"
#include "tests/KernelAddrGen.h"
#include "SystemConfiguration.h"

using namespace DRAMSim;
using namespace std;
class C_ALU
{
public:

    C_ALU(){
        pim_precision_ = PIMConfiguration::getPIMPrecision();
        C_REG = new BurstType();
        S_REG = new fp16();
    };    
    C_ALU(PIMPrecision pimprecision)
        :pim_precision_(pimprecision)
    {
        C_REG = new BurstType();
        S_REG = new fp16();
    };
    ~C_ALU();
    void AdderTree(fp16 C_REG); //use from kernel.cpp
    void load(); //how?
    void accum(BurstType& burst, bool is_neg);
    void adderTree();
    void zeroize();
    void C_max(BurstType& src0Bst);
    void setmax();

    BurstType* C_REG;
    PIMPrecision pim_precision_;
    fp16* S_REG;
private:
    int chan_id;
    int rank_id;
    
//main problem, how to add transaction to this channel level logic
};
#endif
