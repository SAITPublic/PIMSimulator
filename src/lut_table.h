#ifndef LUT_TABLE_H
#define LUT_TABLE_H

#include <cmath>
#include <string>
#include <vector>

#include "tests/KernelAddrGen.h"
#include "FP16.h"
#include "half.h"
#include "Burst.h"

using namespace DRAMSim;
using namespace std;
class lut_table
{
private:
public:
    lut_table()
    {};
    lut_table(int secnum, fp16 lowerbound, fp16 lowerbound_, fp16 upperbound, fp16 upperbound_){
        sec_num = secnum;
        low_bound_gelu = lowerbound;
        up_bound_gelu = upperbound;
        low_bound_exp = lowerbound_;
        up_bound_exp = upperbound_;
        for(int i = 0; i < secnum / 4; i++){
            sets.push_back(new BurstType());
        }
    };
    ~lut_table(){
        for(int i = 0; i < sets.size(); i++){
            delete sets[i];
        }
        sets.clear();
    };
    void fill(layerType layertype, PIMPrecision pim_cmds);
    //void change(vector<fp16> slopes, vector<fp16> intercepts);
    //vector<fp16> slopes, intercepts;
    vector<BurstType*> sets;
    int sec_num;
    fp16 low_bound_gelu, up_bound_gelu, low_bound_exp, up_bound_exp;
};
#endif
