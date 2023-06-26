/***********************************
 * File Name   : TestHybridAStarPlanner.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/
#include "Map.h"
#include "GeometryFunc.h"
#include "AStarPlanner.h"
#include "HybridAStarPlanner.h"
#include "PathSmoother.h"
#include "IaOptimizer.h"

#include <sys/stat.h>
#include <dirent.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace open_space_utils;
using namespace std;

static std::string logName;

void init_log()
{
    logName = "hybridAStarPlanner";
    printf("log initialized...\n");
    return;
}

int main(int argc, char **argv)
{
    printf("test Hybrid A Star initialized...\n");

    std::string exec_path = argv[0];
    boost::filesystem::path path(exec_path);
    std::string filename = path.parent_path().parent_path().parent_path().append("./test/test_map/map.pgm").string();
    std::cout << filename << std::endl;

    Grid mapSize{400, 400};
    float resolution = 0.05;
    Point origin{0.0f, 0.0f};
    g_planning_map.setMapInfo(resolution, mapSize, origin, true);
    g_planning_map.loadMapFromFilePgm(filename);
    cv::Mat map = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC1, cv::Scalar(255));
    for (size_t i = 0; i < map.rows; ++i)
    {
        for (size_t j = 0; j < map.cols; ++j)
        {
            map.at<unsigned char>(i, j) = g_planning_map.cell[j][g_planning_map.getSize().y - i];
        }
    }
    std::string writeStr = "/tmp/OpenSpacePlannerDebug/loaded_map_2.bmp";
    cv::imwrite(writeStr, map);
    printf("map loaded...\n");

    float wheelBase = 1.989;
    float maxKappa = 1.0 / 6.0;
    float axisLength = 3.0;
    float basicThresholdLongitude = 5.033 - 0.94;
    float basicThresholdLateral = 0.5 * 1.989;
    HybridAStarParam hybridAStarParam{logName, maxKappa, axisLength, basicThresholdLongitude, basicThresholdLateral, wheelBase};

#if 0
    Grid startGrid{50, 50};
    Grid goalGrid{380, 380};
    Pose startPose = g_planning_map.gridToPose(startGrid);
    startPose.heading = deg2rad(90);
    Pose goalPose = g_planning_map.gridToPose(goalGrid);
    std::vector<Pose> inputPath;
    inputPath.push_back(startPose);
    inputPath.push_back(goalPose);
    std::vector<Pose> result;
    path_optimizer::IaOptimizer IaOptimizer(hybridAStarParam);
    IaOptimizer.optimize(inputPath, result);
    printf("IaOptimizer done..\n");
#else
    /* test A* */
    grid_a_star::AStarPlanner aStarPlanner;
    if (0)
    {
        Grid aStarStartGrid{380, 380}; // tune hybrid A* use A* gValue, reverse start and goal
        Grid aStarGoalGrid{50, 50};    // tune hybrid A* use A* gValue, reverse start and goal
        vector<Grid> aStarOutputPath;
        grid_a_star::ObsTypeInvolved obsType = grid_a_star::ObsTypeInvolved::ObstacleOnly;
        // grid_a_star::ObsTypeInvolved obsType = grid_a_star::ObsTypeInvolved::ObstacleInflated;
        grid_a_star::HeuType heuType = grid_a_star::HeuType::None;
        // grid_a_star::HeuType heuType = grid_a_star::HeuType::Diagonal;
        // grid_a_star::HeuType heuType = grid_a_star::HeuType::Euclidean;
        grid_a_star::ExpandType expandType = grid_a_star::ExpandType::EightNeighbor;
        aStarPlanner.init(logName);
        aStarPlanner.planning(aStarStartGrid, aStarGoalGrid, aStarOutputPath, obsType, heuType, expandType);
        printf("A* done..\n");
        // show cost map
        cv::Mat costMapMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
        std::vector<std::vector<grid_a_star::Cost>> *costMapPtr;
        costMapPtr = aStarPlanner.getCostMap();
        for (size_t i = 0; i < costMapPtr->size(); ++i)
        {
            for (size_t j = 0; j < costMapPtr->at(i).size(); ++j)
            {
                if (costMapPtr->at(i)[j].obsValue == 255) // obs
                {
                    cv::circle(costMapMat, cv::Point(j, g_planning_map.getSize().y - 1 - j), 0, cv::Scalar(64, 64, 64), CV_FILLED);
                }
                else if (costMapPtr->at(i)[j].gValue < 10000)
                {
                    cv::circle(costMapMat, cv::Point(j, g_planning_map.getSize().y - 1 - j), 0, cv::Scalar(0, 255, 0), CV_FILLED);
                }
            }
        }
        std::string costMapWriteStr = "/tmp/OpenSpacePlanner/a_star_cost_map_out.bmp";
        cv::imwrite(costMapWriteStr, costMapMat);
        printf("show A* cost map done..\n");
    }

    /* test hybrid A* */
    if (1)
    {
        Grid startGrid{50, 50};
        Grid goalGrid{350, 350};
        vector<Grid> outputGridPath;
        Pose startPose = g_planning_map.gridToPose(startGrid);
        startPose.heading = deg2rad(90);
        Pose goalPose = g_planning_map.gridToPose(goalGrid);
        vector<Pose> outputPath;

        hybrid_a_star::HybridAStarPlanner hybridAStarPlanner(hybridAStarParam);
        // hybrid_a_star::HeuType heuType = hybrid_a_star::HeuType::Diagonal;
        hybrid_a_star::HeuType heuType = hybrid_a_star::HeuType::Euclidean;
        // hybrid_a_star::HeuType heuType = hybrid_a_star::HeuType::AStarGValue;
        printf("start: (%d, %d), (%f, %f, %f)\n", startGrid.x, startGrid.y, startPose.pt.x, startPose.pt.y, startPose.heading);
        printf("goal: (%d, %d), (%f, %f, %f)\n", goalGrid.x, goalGrid.y, goalPose.pt.x, goalPose.pt.y, goalPose.heading);
        hybridAStarPlanner.init();
        hybridAStarPlanner.planning(startPose, goalPose, outputPath, heuType, aStarPlanner.getCostMap());
        printf("hybrid A* done..\n");

        /* write HA path to csv */
        vector<double> r_x, r_y, r_heading;
        for (auto &it : outputPath) {
            r_x.emplace_back(it.pt.x);
            r_y.emplace_back(it.pt.y);
            r_heading.emplace_back(it.heading);
        }

        ofstream outFile;
        outFile.open("HA_path.csv", std::ios::out);
        outFile << "x_value" << ',' << "y_value" << ',' << "heading_value" << '\n';
        for (size_t i = 0; i < r_x.size(); ++i)
        {
            outFile << r_x.at(i) << ',' << r_y.at(i) << ',' << r_heading.at(i) << '\n';
        }
        outFile.close();

        /* test path optimizer */
        if (0)
        {
            vector<Pose> pathForOpt;
            // for (size_t i = 0; i < 100; ++i)
            // {
            //     pathForOpt.push_back(outputPath[i]);
            // }
            pathForOpt = outputPath;
            printf("path optimizing, input path size: %lu\n", pathForOpt.size());
            /* show pic*/
            cv::Mat debugMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
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
            pathSmoother.init(logName);
            path_optimizer::OptimizeType optType;
            uint8_t optTypeUint8 = static_cast<std::underlying_type<path_optimizer::OptimizeType>::type>(optType);
            optTypeUint8 |= (1 << static_cast<std::underlying_type<path_optimizer::OptimizeType>::type>(path_optimizer::OptimizeType::Smooth));
            // optTypeUint8 |= (1 << static_cast<std::underlying_type<path_optimizer::OptimizeType>::type>(path_optimizer::OptimizeType::CloseToRef));
            // if (1)
            // {
            //     optTypeUint8 |= (1 << static_cast<std::underlying_type<path_optimizer::OptimizeType>::type>(path_optimizer::OptimizeType::AvoidCollision));
            //     std::vector<Pose> nearestObsPose;
            //     float searchWindowDist = 10.0f;
            //     int searchWindowSize = (int)ceilf(searchWindowDist / 0.05f);
            //     int cnt = 0;
            //     for (auto &pp : pathForOpt)
            //     {
            //         cnt++;
            //         // printf("cnt: %d\n", cnt);
            //         Grid grid = g_planning_map.poseToGrid(pp);

            //         // TODO: level up, search from small window, and then large window
            //         int xLower = max(0, grid.x - searchWindowSize);
            //         int xUpper = min(g_planning_map.getSize().x, grid.x + searchWindowSize);
            //         int yLower = max(0, grid.y - searchWindowSize);
            //         int yUpper = min(g_planning_map.getSize().y, grid.y + searchWindowSize);
            //         // float minDist = searchWindowDist; // default
            //         float minDist = FLT_MAX; // default
            //         // int xLower = 0;
            //         // int xUpper = g_planning_map.getSize().x - 1;
            //         // int yLower = 0;
            //         // int yUpper = g_planning_map.getSize().y - 1;
            //         Grid nearestObsGrid;
            //         for (int i = xLower; i <= xUpper; ++i)
            //         {
            //             for (int j = yLower; j <= yUpper; ++j)
            //             {
            //                 if (g_planning_map.cell[i][j] > 200) // TODO
            //                 {
            //                     float tmp_dist = hypot(i - grid.x, j - grid.y);
            //                     if (tmp_dist < minDist)
            //                     {
            //                         minDist = tmp_dist;
            //                         nearestObsGrid.x = i;
            //                         nearestObsGrid.y = j;
            //                     }
            //                 }
            //             }
            //         }
            //         nearestObsPose.push_back(g_planning_map.gridToPose(nearestObsGrid));
            //     }
            //     path_optimizer.set_nearest_obs_pose(nearestObsPose);
            //     printf("set_nearest_obs_pose done, size: %lu\n", nearestObsPose.size());
            // }
            optType = static_cast<path_optimizer::OptimizeType>(optTypeUint8);
            optimizeFlag = pathSmoother.smooth(pathForOpt, optimizedPath, optType);
            if (optimizeFlag)
                printf("optimize succeded\n");
            else
                printf("optimize failed\n");

            /* show result*/
            for (auto pp : optimizedPath)
            {
                cv::circle(debugMat, cv::Point(g_planning_map.poseToGrid(pp).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(pp).y), 0, cv::Scalar(0, 165, 255), CV_FILLED); // orange
            }
            std::string writeStr = "/tmp/OpenSpacePlannerDebug/path_optimized.bmp";
            cv::imwrite(writeStr, debugMat);
            printf("show path optimized done..\n");
        }
    }
#endif

    while (1)
    {
        // usleep(100000);
    }

    return 0;
}

// int remove_directory(const char *path)
// {
//     DIR *d = opendir(path);
//     size_t path_len = strlen(path);
//     int r = -1;

//     if (d)
//     {
//         struct dirent *p;

//         r = 0;

//         while (!r && (p = readdir(d)))
//         {
//             int r2 = -1;
//             char *buf;
//             size_t len;

//             /* Skip the names "." and ".." as we don't want to recurse on them. */
//             if (!strcmp(p->d_name, ".") || !strcmp(p->d_name, ".."))
//             {
//                 continue;
//             }

//             len = path_len + strlen(p->d_name) + 2;
//             buf = (char *)malloc(len);

//             if (buf)
//             {
//                 struct stat statbuf;

//                 snprintf(buf, len, "%s/%s", path, p->d_name);

//                 if (!stat(buf, &statbuf))
//                 {
//                     if (S_ISDIR(statbuf.st_mode))
//                     {
//                         r2 = remove_directory(buf);
//                     }
//                     else
//                     {
//                         r2 = unlink(buf);
//                     }
//                 }

//                 free(buf);
//             }

//             r = r2;
//         }

//         closedir(d);
//     }
//     else
//     {
//     }

//     if (!r)
//     {
//         r = rmdir(path);
//     }

//     return r;
// }