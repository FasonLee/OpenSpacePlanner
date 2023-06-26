/***********************************
 * File Name   : PathSmoother.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/

#include "PathSmoother.h"

namespace path_optimizer
{
    PathSmoother::PathSmoother()
    {
    }

    bool PathSmoother::init(std::string _logName)
    {
        mLogName = _logName;
        mIpoptApp = IpoptApplicationFactory();
        // mIpoptApp->Options()->SetStringValue("print_level", "0");
        // mIpoptApp->Options()->SetStringValue("hassian_approximation", "limited-memory");
        // mIpoptApp->Options()->SetStringValue("limited_memory_update_type", "bfgs");
        // mIpoptApp->Options()->SetNumericValue("max_cpu_time", (double)m_max_ipopt_cpu_time);
        // TODO: 区分本地和QNX的ipopt地址
        mIpoptStatus = mIpoptApp->Initialize("./test/path_smoother_ipopt.opt", true);

        return true;
    }

    bool PathSmoother::smooth(const std::vector<Pose> &_inputPath, std::vector<Pose> &_outputPath_o, const OptimizeType &_optType)
    {
        printf("========== PathSmoother::smooth, optType: %u ==========\n", static_cast<std::underlying_type<OptimizeType>::type>(_optType));
        printf("_inputPath.size: %lu.\n", _inputPath.size());
        std::vector<float> refX;
        std::vector<float> refY;
        // std::vector<float> nearestObsX;
        // std::vector<float> nearestObsY;
        std::vector<float> optResult;
        int solvedFlag = 0;

        if (_inputPath.size() < 5)
        {
            return false;
        }
        if (static_cast<std::underlying_type<OptimizeType>::type>(_optType) == 0)
        {
            printf("ERROR, optType is null.\n");
            return false;
        }
        // if (static_cast<std::underlying_type<OptimizeType>::type>(_optType) & (1 << static_cast<std::underlying_type<OptimizeType>::type>(OptimizeType::AvoidCollision)))
        // {
        //     if (mNearestObsPose.size() != _inputPath.size())
        //     {
        //         printf("ERROR, mNearestObsPose.size != _inputPath.size, check if mNearestObsPose correctly initialized.\n");
        //         return false;
        //     }
        //     else
        //     {
        //         printf("mNearestObsPose.size: %lu\n", mNearestObsPose.size());
        //         for (auto &obs : mNearestObsPose)
        //         {
        //             nearestObsX.emplace_back(obs.pt.x);
        //             nearestObsY.emplace_back(obs.pt.y);
        //         }
        //     }
        // }

        for (size_t i = 0; i < _inputPath.size(); ++i)
        {
            refX.emplace_back(_inputPath[i].pt.x);
            refY.emplace_back(_inputPath[i].pt.y);
        }

        float qSmooth = 1.0f;
        float qRef = 1.0f;
        // float qCollision = 1.0f;
        // float dMax = 0.6f;

        Ipopt::SmartPtr<PathOptIpoptNlp> pathOptimizerNlp = new PathOptIpoptNlp();
        pathOptimizerNlp->set_ref_path(refX, refY);
        // pathOptimizerNlp->set_nearest_obs(nearestObsX, nearestObsY);
        pathOptimizerNlp->set_Q_smooth(qSmooth);
        pathOptimizerNlp->set_Q_ref(qRef);
        // pathOptimizerNlp->set_Q_collision_d_max(qCollision, dMax);
        pathOptimizerNlp->set_opt_param(2, _inputPath.size(), _optType);

        mIpoptStatus = mIpoptApp->OptimizeTNLP(pathOptimizerNlp);

        if (mIpoptStatus == Solve_Succeeded)
        {
            solvedFlag = 1;
            // Retrieve some statistics about the solve
            Index iterCount = mIpoptApp->Statistics()->IterationCount();
            printf("*** The problem solved in %d.\n", iterCount);
            // std::cout << "*** The problem solved in " << iterCount << " iterations!" << std::endl;
            // Number final_obj = app->Statistics()->FinalObjective();
            // std::cout << "*** The final value of the objective function is " << final_obj << '.'<< std::endl;
        }
        else if (mIpoptStatus == Maximum_Iterations_Exceeded)
        {
            printf("******************** Optimization: Maximum Number of Iterations Exceeded.\n");
        }
        else if (mIpoptStatus == Maximum_CpuTime_Exceeded)
        {
            printf("******************** Optimization: Maximum CPU time exceeded.\n");
        }
        else if (mIpoptStatus == Search_Direction_Becomes_Too_Small)
        {
            printf("******************** Optimization: Search Direction is becoming Too Small.\n");
        }
        else if (mIpoptStatus == Solved_To_Acceptable_Level)
        {
            printf("******************** Optimization: Solved To Acceptable Level.\n");
        }
        else if (mIpoptStatus == Feasible_Point_Found)
        {
            printf("******************** Optimization: Feasible point for square problem found.\n");
        }
        else if (mIpoptStatus == Diverging_Iterates)
        {
            printf("******************** Optimization: Iterates diverging; problem might be unbounded.\n");
        }
        else if (mIpoptStatus == Restoration_Failed)
        {
            printf("******************** Optimization: Restoration Failed!\n");
        }
        else if (mIpoptStatus == Error_In_Step_Computation)
        {
            printf("******************** Optimization: Error in step computation (regularization becomes too large?)!\n");
        }
        else if (mIpoptStatus == Infeasible_Problem_Detected)
        {
            printf("******************** Optimization: Converged to a point of local infeasibility. Problem may be infeasible.\n");
        }
        else if (mIpoptStatus == User_Requested_Stop)
        {
            printf("******************** Optimization: Stopping optimization at current point as requested by user.\n");
        }
        else if (mIpoptStatus == Invalid_Number_Detected)
        {
            printf("******************** Optimization: Invalid number in NLP function or derivative detected.\n");
        }
        else if (mIpoptStatus == Internal_Error)
        {
            printf("******************** Optimization: Unknown SolverReturn value - Notify IPOPT Authors.\n");
        }
        else
        {
            printf("******************** Optimization Solve Failed: %d ********************\n", mIpoptStatus);
        }

        Number totalCpuTime = mIpoptApp->Statistics()->TotalCpuTime();
        printf("*** The total cpu time is %f.\n", totalCpuTime);
        Number totalSysTime = mIpoptApp->Statistics()->TotalSysTime();
        printf("*** The total sys time is %f.\n", totalSysTime);
        Number totalWallclockTime = mIpoptApp->Statistics()->TotalWallclockTime();
        printf("*** The total wallclock time is %f.\n", totalWallclockTime);
        Index iterCnt = mIpoptApp->Statistics()->IterationCount();
        printf("*** The iterCnt is %u.\n", iterCnt);
        pathOptimizerNlp->get_opt_res(optResult);
        printf("optResult.size: %lu.\n", optResult.size());

        _outputPath_o.clear();
        for (size_t i = 0; i < optResult.size() - 1; i = i + 2)
        {
            Pose tmp{optResult[i], optResult[i + 1]};
            _outputPath_o.push_back(tmp);
        }
        printf("_outputPath_o.size: %lu.\n", _outputPath_o.size());
        // for (size_t i = 0; i < _outputPath_o.size(); ++i)
        // {
        //     printf("optResult[%lu]: %f, %f -> %f, %f\n", i, refX[i], refY[i], _outputPath_o[i].pt.x, _outputPath_o[i].pt.y);
        // }

        if (solvedFlag == 1)
            return true;
        else
            return false;
    }

    // void PathOptimizer::set_nearest_obs_pose(const std::vector<Pose> &_nearestObsPose)
    // {
    //     mNearestObsPose = _nearestObsPose;
    // }
}