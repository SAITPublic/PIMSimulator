#include <vector>

#include "C_ALU.h"
//#include "tests/KernelAddrGen.h"
#include "Burst.h"

using namespace std;
using namespace DRAMSim;

C_ALU::~C_ALU()
{
    delete C_REG;
    delete S_REG;
}
/*void C_ALU::addertree(BurstType* result, int output_dim, int num_tile, int step, fp16* temp) //we got 16 types of things and measure
//prototype: output_dim = 1, num_tile = 16, step = 0
{
    if (num_tile == 1)
        return;

    int iter = num_tile / 2; //example we got num_tile for 
    if (step == 0)
    {
        for (int i = 0; i < iter; i++)
        {
            
            temp[i] = result[2 * i * output_dim].fp16AdderTree() +
                      result[(2 * i + 1) * output_dim].fp16AdderTree();
        }
    }
    else
    {
        for (int i = 0; i < iter; i++) temp[i] = temp[i * 2] + temp[i * 2 + 1];

        if (num_tile % 2 == 1)
            temp[iter] = temp[num_tile];
    }

    adderTree(result, output_dim, ceil(double(num_tile) / (double)2), step + 1, temp);
    return;
}*/
void C_ALU::adderTree()
{
    for(int i = 0; i < 16; i++)
    {
        if(S_REG != nullptr)
            *S_REG = *S_REG + C_REG->fp16Data_[i];
    }
    return;
}
void C_ALU::accum(BurstType& burst, bool is_neg){
    if(pim_precision_==FP16){
        for(int fp  = 0; fp < 16; fp++)
        {
            if(is_neg)  C_REG->fp16Data_[fp] -= burst.fp16Data_[fp];
            else    C_REG->fp16Data_[fp] += burst.fp16Data_[fp];
        }
    }
    else if(pim_precision_ == FP32){
        for(int fp = 0; fp < 8; fp++)
        {
            if(is_neg)  C_REG->fp32Data_[fp] -= burst.fp32Data_[fp];
            else    C_REG->fp32Data_[fp] += burst.fp16Data_[fp];
        }

    }
}
void C_ALU::zeroize()
{
    S_REG = 0;
    return;
}

void C_ALU::C_max(BurstType& src0Bst)
{
    if(pim_precision_ == FP16)
    {
        for(int fp = 0; fp < 16; fp++)
        {
            C_REG->fp16Data_[fp] = (C_REG->fp16Data_[fp] > src0Bst.fp16Data_[fp])?C_REG->fp16Data_[fp]:src0Bst.fp16Data_[fp];
        }
    }
}
void C_ALU::setmax()
{
    for(int fp = 0; fp < 16; fp++)
    {
        S_REG = (C_REG->fp16Data_[fp] > *S_REG)?&C_REG->fp16Data_[fp]:S_REG;
    }
}
//how about just put c_alu outside the bank...