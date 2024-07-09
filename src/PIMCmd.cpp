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

#include "PIMCmd.h"

namespace DRAMSim
{
bool operator==(const PIMCmd& lhs, const PIMCmd& rhs)
{
    return lhs.toInt() == rhs.toInt();
}

bool operator!=(const PIMCmd& lhs, const PIMCmd& rhs)
{
    return lhs.toInt() != rhs.toInt();
}

void PIMCmd::fromInt(uint32_t val)
{
    type_ = PIMCmdType(fromBit(val, 4, 28));
    switch (type_)
    {
        case PIMCmdType::EXIT:
            break;

        case PIMCmdType::NOP:
            loopCounter_ = fromBit(val, 11, 0);
            break;

        case PIMCmdType::JUMP:
            loopCounter_ = fromBit(val, 17, 11);
            loopOffset_ = fromBit(val, 11, 0);
            break;

        case PIMCmdType::FILL:
            dst_ = PIMOpdType(fromBit(val, 3, 25));
            src0_ = PIMOpdType(fromBit(val, 3, 22));
            dstIdx_ = fromBit(val, 4, 8);
            src0Idx_ = fromBit(val, 4, 4);
            src1Idx_ = fromBit(val, 4, 0);
            isRelu_ = fromBit(val, 1, 12);
            break;
        case PIMCmdType::MOV:
            dst_ = PIMOpdType(fromBit(val, 3, 25));
            src0_ = PIMOpdType(fromBit(val, 3, 22));
            isRelu_ = fromBit(val, 1, 12);
            dstIdx_ = fromBit(val, 4, 8);
            src0Idx_ = fromBit(val, 4, 4);
            src1Idx_ = fromBit(val, 4, 0);
            break;

        case PIMCmdType::MAD:
            src2_ = PIMOpdType(fromBit(val, 3, 16));
            dst_ = PIMOpdType(fromBit(val, 3, 25));
            src0_ = PIMOpdType(fromBit(val, 3, 22));
            src1_ = PIMOpdType(fromBit(val, 3, 19));
            isAuto_ = fromBit(val, 1, 15);
            dstIdx_ = fromBit(val, 4, 8);
            src0Idx_ = fromBit(val, 4, 4);
            src1Idx_ = fromBit(val, 4, 0);
            break;
        case PIMCmdType::ADD:
            dst_ = PIMOpdType(fromBit(val, 3, 25));
            src0_ = PIMOpdType(fromBit(val, 3, 22));
            src1_ = PIMOpdType(fromBit(val, 3, 19));
            isAuto_ = fromBit(val, 1, 15);
            dstIdx_ = fromBit(val, 4, 8);
            src0Idx_ = fromBit(val, 4, 4);
            src1Idx_ = fromBit(val, 4, 0);
            break;
        case PIMCmdType::MAX:
            dst_ = PIMOpdType(fromBit(val, 3, 25));
            src0_ = PIMOpdType(fromBit(val, 3, 22));
            src1_ = PIMOpdType(fromBit(val, 3, 19));
            isAuto_ = fromBit(val, 1, 15);
            dstIdx_ = fromBit(val, 4, 8);
            src0Idx_ = fromBit(val, 4, 4);
            src1Idx_ = fromBit(val, 4, 0);
            break;
        case PIMCmdType::MUL:
            dst_ = PIMOpdType(fromBit(val, 3, 25));
            src0_ = PIMOpdType(fromBit(val, 3, 22));
            src1_ = PIMOpdType(fromBit(val, 3, 19));
            isAuto_ = fromBit(val, 1, 15);
            dstIdx_ = fromBit(val, 4, 8);
            src0Idx_ = fromBit(val, 4, 4);
            src1Idx_ = fromBit(val, 4, 0);
            break;
        case PIMCmdType::MAC:
            dst_ = PIMOpdType(fromBit(val, 3, 25));
            src0_ = PIMOpdType(fromBit(val, 3, 22));
            src1_ = PIMOpdType(fromBit(val, 3, 19));
            isAuto_ = fromBit(val, 1, 15);
            dstIdx_ = fromBit(val, 4, 8);
            src0Idx_ = fromBit(val, 4, 4);
            src1Idx_ = fromBit(val, 4, 0);
            break;

        default:
            break;
    }
}

void PIMCmd::validationCheck() const
{
    /*if (type_ == PIMCmdType::MOV || type_ == PIMCmdType::FILL)
    {
        if (dst_ == PIMOpdType::EVEN_BANK || dst_ == PIMOpdType::ODD_BANK)
        {
            if (src0_ == PIMOpdType::GRF_A || src0_ == PIMOpdType::GRF_B ||
                src1_ == PIMOpdType::GRF_A || src1_ == PIMOpdType::GRF_B ||
                src2_ == PIMOpdType::GRF_A || src2_ == PIMOpdType::GRF_B)
            {
                std::cerr << "ERROR) Invalid in ISA 1.0 " << toStr() << std::endl;
                exit(-1);
            }
        }
        /*
           if (src0_ == PIMOpdType::EVEN_BANK || src0_ == PIMOpdType::ODD_BANK ||
           src1_ == PIMOpdType::EVEN_BANK || src1_ == PIMOpdType::ODD_BANK ||
           src2_ == PIMOpdType::EVEN_BANK || src2_ == PIMOpdType::ODD_BANK){
           if (dst_ == PIMOpdType::GRF_A || dst_ == PIMOpdType::GRF_B){
           std::cerr << "ERROR) Invalid in ISA 1.0 " << toStr() << std::endl;
           exit(-1);
           }
           }

    } */
}

uint32_t PIMCmd::toInt() const
{
    validationCheck();
    uint32_t val = toBit(int(type_), 4, 28);
    switch (type_)
    {
        case PIMCmdType::EXIT:
            break;

        case PIMCmdType::NOP:
            val |= toBit(loopCounter_, 11, 0);
            break;

        case PIMCmdType::JUMP:
            val |= toBit(loopCounter_, 17, 11);
            val |= toBit(loopOffset_, 11, 0);
            break;

        case PIMCmdType::FILL:
        case PIMCmdType::MOV:
            val |= toBit(int(dst_), 3, 25);
            val |= toBit(int(src0_), 3, 22);
            val |= toBit(dstIdx_, 4, 8);
            val |= toBit(src0Idx_, 4, 4);
            val |= toBit(src1Idx_, 4, 0);
            val |= toBit(isRelu_, 1, 12);
            break;

        case PIMCmdType::MAD:
            val |= toBit(int(src2_), 3, 16);
            val |= toBit(int(dst_), 3, 25);
            val |= toBit(int(src0_), 3, 22);
            val |= toBit(int(src1_), 3, 19);
            val |= toBit(isAuto_, 1, 15);
            val |= toBit(dstIdx_, 4, 8);
            val |= toBit(src0Idx_, 4, 4);
            val |= toBit(src1Idx_, 4, 0);
            break;

        case PIMCmdType::ADD:
            val |= toBit(int(dst_), 3, 25);
            val |= toBit(int(src0_), 3, 22);
            val |= toBit(int(src1_), 3, 19);
            val |= toBit(isAuto_, 1, 15);
            val |= toBit(dstIdx_, 4, 8);
            val |= toBit(src0Idx_, 4, 4);
            val |= toBit(src1Idx_, 4, 0);
            break;
        case PIMCmdType::MAX:
            val |= toBit(int(dst_), 3, 25);
            val |= toBit(int(src0_), 3, 22);
            val |= toBit(int(src1_), 3, 19);
            val |= toBit(isAuto_, 1, 15);
            val |= toBit(dstIdx_, 4, 8);
            val |= toBit(src0Idx_, 4, 4);
            val |= toBit(src1Idx_, 4, 0);
            break;
        case PIMCmdType::MUL:
            val |= toBit(int(dst_), 3, 25);
            val |= toBit(int(src0_), 3, 22);
            val |= toBit(int(src1_), 3, 19);
            val |= toBit(isAuto_, 1, 15);
            val |= toBit(dstIdx_, 4, 8);
            val |= toBit(src0Idx_, 4, 4);
            val |= toBit(src1Idx_, 4, 0);
            break;

        case PIMCmdType::MAC:
            val |= toBit(int(dst_), 3, 25);
            val |= toBit(int(src0_), 3, 22);
            val |= toBit(int(src1_), 3, 19);
            val |= toBit(isAuto_, 1, 15);
            val |= toBit(dstIdx_, 4, 8);
            val |= toBit(src0Idx_, 4, 4);
            val |= toBit(src1Idx_, 4, 0);
            break;

        default:
            break;
    }

    return val;
}

std::string PIMCmd::toStr() const
{
    stringstream ss;
    ss << cmdToStr(type_) << " ";
    switch (type_)
    {
        case PIMCmdType::EXIT:
            break;

        case PIMCmdType::NOP:
            ss << loopCounter_ + 1 << "x";
            break;

        case PIMCmdType::JUMP:
            ss << loopCounter_ << "x ";
            ss << "[PC - " << loopOffset_ << "]";
            break;

        case PIMCmdType::FILL:
        case PIMCmdType::MOV:
            ss << opdToStr(dst_, dstIdx_) << ", ";
            ss << opdToStr(src0_, src0Idx_);
            if (isRelu_)
                ss << ", relu";
            break;

        case PIMCmdType::ADD:
        case PIMCmdType::MUL:
        case PIMCmdType::MAC:
            ss << opdToStr(dst_, dstIdx_) << ", ";
            ss << opdToStr(src0_, src0Idx_) << ", ";
            ss << opdToStr(src1_, src1Idx_);
            break;

        case PIMCmdType::MAD:
            ss << opdToStr(dst_, dstIdx_) << ", ";
            ss << opdToStr(src0_, src0Idx_) << ", ";
            ss << opdToStr(src1_, src1Idx_) << ", ";
            ss << opdToStr(src2_, src1Idx_);
            break;

        default:
            break;
    }
    if (isAuto_)
    {
        ss << ", auto";
    }
    return ss.str();
}
}  // namespace DRAMSim
