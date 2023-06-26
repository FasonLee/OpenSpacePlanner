/***********************************
 * File Name   : IaOptimizer.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/
#ifndef OPEN_SPACE_PLANNER_IA_OPTIMIZER_H
#define OPEN_SPACE_PLANNER_IA_OPTIMIZER_H

#include "HybridAStarPlanner.h"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

namespace path_optimizer
{
    class IaOptimizer
    {
    public:
        /** default constructor */
        IaOptimizer(const hybrid_a_star::HybridAStarParam &_param);

        /** default destructor */
        virtual ~IaOptimizer() = default;

        // bool init();

        bool optimize(const std::vector<Pose> &_inputPath, std::vector<Pose> &_outputPath_o);

    private:
        std::string mLogName;
        HybridAStarParam mHybridAStarParam;
    };
}
#endif // OPEN_SPACE_PLANNER_IA_OPTIMIZER_H