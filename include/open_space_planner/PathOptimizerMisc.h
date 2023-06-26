/***********************************
 * File Name   : PathOptimizerMisc.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/

#ifndef OPEN_SPACE_PATH_OPTIMIZER_MISC_NLP_H
#define OPEN_SPACE_PATH_OPTIMIZER_MISC_NLP_H

namespace path_optimizer
{
    enum class OptimizeType : uint8_t
    {
        Smooth,
        CloseToRef,
        AvoidCollision,
        Curvature,
    };
}

#endif
