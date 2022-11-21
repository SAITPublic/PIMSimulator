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

#include "FP16.h"

float convertH2F(fp16 val)
{
    float val1 = val;
    return val1;
}

fp16 convertF2H(float val)
{
    fp16 val0(val);
    return val0;
}

bool fp16Equal(fp16 A, fp16 B, int maxUlpsDiff, float maxFsdiff)
{
    fp16i Ai;
    Ai.fval = A;
    fp16i Bi;
    Bi.fval = B;

    if ((Ai.ival & (1 << 15)) != (Bi.ival & (1 << 15)))
    {
        if (A == B)
            return true;
    }

    // Find the difference in ULPs.
    int ulpsDiff = abs(Ai.ival - Bi.ival);
    float fsDiff = abs(convertH2F(Ai.fval) - convertH2F(Bi.fval));
    if (ulpsDiff <= maxUlpsDiff)
        return true;
    else if (fsDiff < maxFsdiff)
        return true;
    return false;
}
