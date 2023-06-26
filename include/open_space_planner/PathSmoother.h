/***********************************
 * File Name   : PathSmoother.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/

#ifndef OPEN_SPACE_PATH_SMOOTHER_H
#define OPEN_SPACE_PATH_SMOOTHER_H

#include "PathOptIpoptNlp.h"
#include "PathOptimizerMisc.h"
#include "Pose.h"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

using namespace open_space_utils;

namespace path_optimizer
{
    class PathSmoother
    {
    public:
        /** default constructor */
        PathSmoother();

        /** default destructor */
        virtual ~PathSmoother() = default;

        bool init(std::string _logName);

        bool smooth(const std::vector<Pose> &_inputPath, std::vector<Pose> &_outputPath_o, const OptimizeType &_optType);

        // void set_nearest_obs_pose(const std::vector<Pose> &_nearestObsPose);

    private:
        std::string mLogName;
        ApplicationReturnStatus mIpoptStatus;
        Ipopt::SmartPtr<IpoptApplication> mIpoptApp;

        // std::vector<Pose> mNearestObsPose;
    };
}
#endif