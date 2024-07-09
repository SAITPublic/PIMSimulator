#include "lut_table.h"

#define GELU(x) 0.5 * x * (1 + tanhf(sqrtf(2/M_PI) * (x + 0.044715*pow(x, 3))))

using namespace DRAMSim;
using namespace std;
void lut_table::fill(layerType layertype, PIMPrecision pimprecision){
    int size = sec_num / 16;
    fp16 former = convertF2H(float(0));
    fp16 latter = convertF2H(float(0));
    fp16 slope  = convertF2H(float(0));
    fp16 intercept = convertF2H(float(0));
    int b_idx = 0;
    for(int s = 0; s<size; s++)
    {
        for(int bst  = 0; bst < 16; bst++)
        {
            b_idx  = bst+16*s;
            former = (layertype==layerType::GELU)?b_idx * (up_bound_gelu- low_bound_gelu) / convertF2H(float(sec_num)): b_idx * (up_bound_exp - low_bound_exp) / convertF2H(float(sec_num));
            latter = (layertype == layerType::GELU)?former + (up_bound_gelu - low_bound_gelu) / convertF2H(float(sec_num)) : former + (up_bound_exp - low_bound_exp) /convertF2H(float(sec_num));
            if(b_idx == 0){
                sets[0]->fp16Data_[0] = convertF2H(float(0.0));
                sets[size]->fp16Data_[0] = convertF2H(float(-65504.0));
                sets[2*size]->fp16Data_[0] = convertF2H(float(0.0));
                sets[3*size]->fp16Data_[0] = convertF2H(float(-65504.0));
            }
            else if(b_idx == sec_num){
                sets[s]->fp16Data_[15] = convertF2H(float(0));
                sets[s+ size]->fp16Data_[15] = convertF2H(float(65504.0));
                sets[s+ 2* size]->fp16Data_[15] =convertF2H(float(0.0));
                sets[s + 3*size]->fp16Data_[15] = convertF2H(float(65504.0));
            }
            else{
                switch (layertype)
                {
                case layerType::GELU: //scope as -4, 4 
                    slope =  convertF2H((GELU(convertH2F(latter)) - GELU(convertH2F(former))) / pow((convertH2F(latter)-convertH2F(former)), 2));
                    intercept = convertF2H(expf(former)) - slope * former;
                    sets[s]->fp16Data_[bst] = slope;
                    sets[s+size]->fp16Data_[bst] = intercept;
                    break;
                case layerType::EXP: //scope as -11.0898, 11.0898 
                    slope = convertF2H((expf(convertH2F(latter)) - expf(convertH2F(former))) / pow((convertH2F(latter) - convertH2F(former)), 2));
                    intercept = convertF2H(expf(former)) - slope * former;
                    sets[s + 2*size]->fp16Data_[bst] = slope;
                    sets[s + 3*size]->fp16Data_[bst] = intercept;
                    break;
                default:
                    break;
                }
            } 
        }
    }
}

//slope/intercept model seems like 
//make lut_table as numpyburst how?
