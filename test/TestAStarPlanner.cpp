/***********************************
 * File Name   : TestAStarPlanner.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description :
 ***********************************/
#include "Map.h"
#include "AStarPlanner.h"

#include <sys/stat.h>
#include <dirent.h>
#include <opencv2/opencv.hpp>

using namespace open_space_utils;
using namespace open_space_map;
using namespace std;

static std::string logName;

void init_log()
{
    logName = "a_star_planner";
    printf("log initialized...\n");
    return;
}

int main()
{
    printf("test a_star_planner initialized...\n");
    std::string filename = "./test/test_map/test_map.pgm";
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
    /* for creat test map */
    // cv::Mat creat_map = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC1, cv::Scalar(0));
    // for (size_t i = 0; i < creat_map.rows; ++i)
    // {
    //     for (size_t j = 0; j < creat_map.cols; ++j)
    //     {
    //         if (i >= 75 && i <= 125 && j >= 275 && j <= 325)
    //         {
    //             creat_map.at<unsigned char>(i, j) = 255;
    //         }
    //     }
    // }
    // std::string writeStr = "/tmp/OpenSpacePlannerDebug/test_map.pgm";
    // cv::imwrite(writeStr, creat_map);
    printf("map loaded...\n");

    float maxKappa = 1 / (0.5 * 0.4);
    float axisLength = 0.7;
    float basicThresholdLongitude = 0.8;
    float basicThresholdLateral = 0.6;
    float wheelBase = 0.4;

    /* test A* */
    grid_a_star::AStarPlanner a_star_planner;
    if (1)
    {
        Grid a_star_start_grid{50, 50}; // tune hybrid A* use A* gValue, reverse start and goal
        Grid a_star_goal_grid{380, 380};   // tune hybrid A* use A* gValue, reverse start and goal
        vector<Grid> a_star_output_path;
        grid_a_star::ObsTypeInvolved obsType = grid_a_star::ObsTypeInvolved::ObstacleOnly;
        // grid_a_star::ObsTypeInvolved obsType = grid_a_star::ObsTypeInvolved::ObstacleInflated;
        grid_a_star::HeuType heuType = grid_a_star::HeuType::None;
        // grid_a_star::HeuType heuType = grid_a_star::HeuType::Diagonal;
        // grid_a_star::HeuType heuType = grid_a_star::HeuType::Euclidean;
        grid_a_star::ExpandType expandType = grid_a_star::ExpandType::EightNeighbor;
        a_star_planner.init(logName);
        a_star_planner.planning(a_star_start_grid, a_star_goal_grid, a_star_output_path, obsType, heuType, expandType);
        printf("A* done..\n");
        // show cost map
        cv::Mat costMapMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
        std::vector<std::vector<grid_a_star::Cost>> *costMapPtr;
        costMapPtr = a_star_planner.getCostMap();
        for (size_t i = 0; i < costMapPtr->size(); ++i)
        {
            for (size_t j = 0; j < costMapPtr->at(i).size(); ++j)
            {
                if (costMapPtr->at(i)[j].obsValue == 255) // obs
                {
                    cv::circle(costMapMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(64, 64, 64), CV_FILLED);
                }
                else if (costMapPtr->at(i)[j].gValue < 10000)
                {
                    cv::circle(costMapMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(0, 255, 0), CV_FILLED);
                }
            }
        }
        std::string costMapWriteStr = "/tmp/OpenSpacePlannerDebug/a_star_cost_map_out.bmp";
        cv::imwrite(costMapWriteStr, costMapMat);
        printf("show A* cost map done..\n");
    }

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