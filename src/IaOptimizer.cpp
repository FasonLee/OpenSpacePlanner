/***********************************
 * File Name   : IaOptimizer.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/

#include "IaOptimizer.h"
#include "AStarPlanner.h"
#include "PathSmoother.h"
// include collision checker headers
#include "Map.h"
#include "Timer.h"

using namespace std;

namespace path_optimizer
{
    IaOptimizer::IaOptimizer(const hybrid_a_star::HybridAStarParam &_param)
    {
        mHybridAStarParam = _param;
        mLogName = _param.logName;
    }

    // bool IaOptimizer::init()
    // {
    // }

    bool IaOptimizer::optimize(const std::vector<Pose> &_inputPath, std::vector<Pose> &_outputPath_o)
    {
        printf("========== IaOptimizer, _inputPath.size: %lu ==========\n", _inputPath.size());
        // if (_inputPath.size() < 5) // TODO, specified false condition
        // {
        //     return false;
        // }

        Pose startPose = _inputPath[0];
        Pose goalPose = _inputPath.back();
        Grid startGrid = g_planning_map.poseToGrid(startPose);
        Grid goalGrid = g_planning_map.poseToGrid(goalPose);
        printf("startPose: (%f, %f, %f), goalPose: (%f, %f, %f)\n", startPose.pt.x, startPose.pt.y, startPose.heading, goalPose.pt.x, goalPose.pt.y, goalPose.heading);
        printf("startGrid: (%d, %d), goalGrid: (%d, %d)\n", startGrid.x, startGrid.y, goalGrid.x, goalGrid.y);

        /* A* */
        uint64_t aStarStartTs = open_space_utils::Timer::getTimestampUS();
        grid_a_star::AStarPlanner aStarPlanner;
        if (1)
        {
            vector<Grid> aStarOutputPath;
            grid_a_star::ObsTypeInvolved obsType = grid_a_star::ObsTypeInvolved::ObstacleOnly;
            // grid_a_star::ObsTypeInvolved obsType = grid_a_star::ObsTypeInvolved::ObstacleInflated;
            grid_a_star::HeuType heuType = grid_a_star::HeuType::None;
            grid_a_star::ExpandType expandType = grid_a_star::ExpandType::EightNeighbor;
            aStarPlanner.init(mLogName);
            aStarPlanner.planning(goalGrid, startGrid, aStarOutputPath, obsType, heuType, expandType);
            // show cost map
            if (0)
            {
                cv::Mat costMapMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
                std::vector<std::vector<grid_a_star::Cost>> *costMapPtr;
                costMapPtr = aStarPlanner.getCostMap();
                for (size_t i = 0; i < costMapPtr->size(); ++i)
                {
                    for (size_t j = 0; j < costMapPtr->at(i).size(); ++j)
                    {
                        if (costMapPtr->at(i)[j].obsValue == 255) // obs
                        {
                            cv::circle(costMapMat, cv::Point(i, g_planning_map.getSize().y - 1 - j), 0, cv::Scalar(64, 64, 64), CV_FILLED);
                        }
                        else if (costMapPtr->at(i)[j].gValue < 10000)
                        {
                            cv::circle(costMapMat, cv::Point(i, g_planning_map.getSize().y - 1 - j), 0, cv::Scalar(0, 255, 0), CV_FILLED);
                        }
                    }
                }
                std::string costMapWriteStr = "/tmp/OpenSpacePlannerDebug/a_star_cost_map_out.bmp";
                cv::imwrite(costMapWriteStr, costMapMat);
                printf("show A* cost map done..\n");
            }
            uint64_t aStarEndTs = open_space_utils::Timer::getTimestampUS();
            float aStarPeriod = float(aStarEndTs - aStarStartTs) * 0.001; // unit: ms
            printf("A* done, aStarPeriod: %0.3f[ms]\n", aStarPeriod);
        }

        /* hybrid A* */
        uint64_t hybridAStarStartTs = open_space_utils::Timer::getTimestampUS();
        vector<Pose> hybridAStarPath;
        int8_t hybridAStarFlag;
        hybrid_a_star::HybridAStarPlanner hybridAStarPlanner(mHybridAStarParam);
        // hybrid_a_star::HeuType heuType = hybrid_a_star::HeuType::Diagonal;
        // hybrid_a_star::HeuType heuType = hybrid_a_star::HeuType::Euclidean;
        hybrid_a_star::HeuType heuType = hybrid_a_star::HeuType::AStarGValue;
        hybridAStarPlanner.init();
        hybridAStarFlag = hybridAStarPlanner.planning(startPose, goalPose, hybridAStarPath, heuType, aStarPlanner.getCostMap());
        uint64_t hybridAStarEndTs = open_space_utils::Timer::getTimestampUS();
        float hybridAStarPeriod = float(hybridAStarEndTs - hybridAStarStartTs) * 0.001; // unit: ms
        if (hybridAStarFlag == 1)
        {
            printf("hybrid A* done, path.size: %lu, hybridAStarPeriod: %0.3f[ms]\n", hybridAStarPath.size(), hybridAStarPeriod);
        }
        else
        {
            printf("hybrid A* failed, error code: %d\n", hybridAStarFlag);
            return false;
        }

        /* path smoother */
        uint64_t smootherStartTs = open_space_utils::Timer::getTimestampUS();
        _outputPath_o.clear();
        _outputPath_o = hybridAStarPath;
        cv::Mat debugMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
        if (1) // due to computation resource, default turn off smoother process
        {
            // vector<Pose> pathForSmooth = hybridAStarPath;
            // vector<size_t> anchoredIndex;
            // anchoredIndex.push_back(0);
            // anchoredIndex.push_back(hybridAStarPath.size() - 1);
            {
                vector<Pose> pathForOpt;
                pathForOpt = hybridAStarPath;
                printf("path smoothing, input path size: %lu\n", pathForOpt.size());
                /* show pic*/
                for (size_t i = 0; i < g_planning_map.getSize().x; i++)
                {
                    for (size_t j = 0; j < g_planning_map.getSize().y; j++)
                    {
                        if (g_planning_map.cell[i][j] > 200) // TODO
                        {
                            cv::circle(debugMat, cv::Point(i, g_planning_map.getSize().y - 1 - j), 0, cv::Scalar(64, 64, 64), CV_FILLED);
                        }
                    }
                }
                cv::circle(debugMat, cv::Point(startGrid.x, g_planning_map.getSize().y - 1 - startGrid.y),
                           10, cv::Scalar(0, 255, 0), CV_FILLED); // green
                cv::circle(debugMat, cv::Point(startGrid.x, g_planning_map.getSize().y - 1 - startGrid.y),
                           0, cv::Scalar(0, 128, 0), CV_FILLED); // green
                cv::circle(debugMat, cv::Point(goalGrid.x, g_planning_map.getSize().y - 1 - goalGrid.y),
                           10, cv::Scalar(0, 0, 255), CV_FILLED); // red
                cv::circle(debugMat, cv::Point(goalGrid.x, g_planning_map.getSize().y - 1 - goalGrid.y),
                           0, cv::Scalar(0, 0, 128), CV_FILLED); // red
                for (auto &pp : pathForOpt)
                {
                    cv::circle(debugMat, cv::Point(g_planning_map.poseToGrid(pp).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(pp).y), 0, cv::Scalar(140, 180, 210), CV_FILLED); // light coffe
                }
                /* optimizing */
                vector<Pose> optimizedPath;
                bool optimizeFlag = false;
                path_optimizer::PathSmoother pathSmoother;
                pathSmoother.init(mLogName);
                path_optimizer::OptimizeType optType;
                uint8_t optTypeUint8 = static_cast<std::underlying_type<path_optimizer::OptimizeType>::type>(optType);
                optTypeUint8 = 0;
                optTypeUint8 |= (1 << static_cast<std::underlying_type<path_optimizer::OptimizeType>::type>(path_optimizer::OptimizeType::Smooth));
                optType = static_cast<path_optimizer::OptimizeType>(optTypeUint8);
                optimizeFlag = pathSmoother.smooth(pathForOpt, optimizedPath, optType);
                if (optimizeFlag)
                {
                    printf("optimize succeded\n");
                    // collision check
                    bool pathFeasible = true;
                    for (size_t i = 0; i < optimizedPath.size(); ++i)
                    {
                        if (0) // TODO: collision check
                        {
                            pathFeasible = false;
                            break;
                        }
                    }
                    if (pathFeasible)
                    {
                        printf("optimized path feasible\n");
                    }
                    else
                    {
                        printf("optimized path infeasible\n");
                    }
                }
                else
                {
                    printf("optimize failed\n");
                }
                _outputPath_o = optimizedPath; // TODO, move to upper
            }
            uint64_t smootherEndTs = open_space_utils::Timer::getTimestampUS();
            float smootherPeriod = float(smootherEndTs - smootherStartTs) * 0.001; // unit: ms
            printf("smootherPeriod: %0.3f[ms]\n", smootherPeriod);
        }

        if (_outputPath_o.empty())
        {
            printf("_outputPath_o is empty\n");
            return false;
        }

        // /* show result*/
        for (auto pp : _outputPath_o)
        {
            cv::circle(debugMat, cv::Point(g_planning_map.poseToGrid(pp).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(pp).y), 0, cv::Scalar(0, 165, 255), CV_FILLED); // orange
        }
        std::string writeStr = "/tmp/OpenSpacePlannerDebug/path_optimized.bmp";
        cv::imwrite(writeStr, debugMat);
        return true;
    }
}