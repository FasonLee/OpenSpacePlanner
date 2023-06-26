/***********************************
 * File Name   : AStarPlanner.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description :
 ***********************************/

#include "AStarPlanner.h"
// #include "a_star_planner_log.h"
#include "Map.h"
#include "Timer.h"
#include <opencv2/opencv.hpp>

using namespace open_space_utils;
using namespace open_space_map;

#define DEBUG_A_STAR
static uint64_t testTime = 0;
static uint64_t updateAdjacent = 0;
static uint64_t othersInExpand = 0;
namespace grid_a_star
{
    AStarPlanner::AStarPlanner()
    {
    }

    bool AStarPlanner::init(std::string _logName)
    {
        mLogName = _logName;
        return true;
    }

    int8_t AStarPlanner::planning(const Grid &_startGrid, const Grid &_goalGrid, std::vector<Grid> &_GridPath_o,
                           ObsTypeInvolved _obsType, HeuType _heuType, ExpandType _expandType)
    {
        std::string timeStr = open_space_utils::Timer::getReadableTimestampUS();
        uint64_t gridNum = g_planning_map.getSize().x * g_planning_map.getSize().y;
        mHeuType = _heuType;
        mExpandType = _expandType;
        printf("===== A* planning, gridNum: %lu, obsType: %u, heu_type: %u, expandType: %u\n",
               gridNum, static_cast<std::underlying_type<ObsTypeInvolved>::type>(_obsType), static_cast<std::underlying_type<HeuType>::type>(mHeuType), static_cast<std::underlying_type<ExpandType>::type>(mExpandType));
        std::priority_queue<AStarGrid> openList;
        // std::priority_queue<std::pair<std::string, float>, std::vector<std::pair<std::string, float>>, cmp> openList;
#ifdef DEBUG_A_STAR
        cv::Mat debugMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
        std::string writeStr = "/tmp/OpenSpacePlannerDebug/" + timeStr + "_a_star_1.bmp";
#endif
        uint64_t costMapStartTs = open_space_utils::Timer::getTimestampUS();
        mCostMap.clear();
        mCostMap.reserve(g_planning_map.getSize().x);
        Cost initCost;
        for (size_t i = 0; i < g_planning_map.getSize().x; i++)
        {
            std::vector<Cost> tmp;
            tmp.reserve(g_planning_map.getSize().y);
            for (size_t j = 0; j < g_planning_map.getSize().y; j++)
            {
                if (g_planning_map.cell[i][j] > 200) // TODO
                {
#ifdef DEBUG_A_STAR
                    cv::circle(debugMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(128, 128, 128), CV_FILLED);
#endif
                    initCost.obsValue = 255;
                    initCost.extraCost = 255;
                }
//                 else if (tmp_cell.cost == INFLATED_OBSTACLE && _obsType >= ObsTypeInvolved::ObstacleInflated)
//                 {
// #ifdef DEBUG_A_STAR
//                     cv::circle(debugMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(160, 160, 160), CV_FILLED);
// #endif
//                     initCost.obsValue = 255;
//                     initCost.extraCost = 255;
//                 }
//                 else if (tmp_cell.cost > FREE_SPACE && _obsType >= ObsTypeInvolved::ObstacleWithGradient)
//                 {
// #ifdef DEBUG_A_STAR
//                     cv::circle(debugMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(200, 200, 200), CV_FILLED);
// #endif
//                     int k = 3; // adjust gradient cost influence
//                     initCost.obsValue = tmp_cell.cost * k;
//                     initCost.extraCost = tmp_cell.cost * k;
//                 }
                else
                {
                    initCost.obsValue = 0;
                    initCost.extraCost = 0;
                }
                tmp.push_back(initCost);
            }
            mCostMap.push_back(tmp);
        }

        uint64_t costMapDoneTs = open_space_utils::Timer::getTimestampUS();
        float costMapPeriod = float(costMapDoneTs - costMapStartTs) * 0.001; // unit: ms
        printf("A* cost map period: %0.3f [ms]\n", costMapPeriod);

#ifdef DEBUG_A_STAR
        cv::circle(debugMat, cv::Point(_startGrid.x, g_planning_map.getSize().y- 1 - _startGrid.y),
                   10, cv::Scalar(0, 255, 0), CV_FILLED); // green
        cv::circle(debugMat, cv::Point(_startGrid.x, g_planning_map.getSize().y- 1 - _startGrid.y),
                   0, cv::Scalar(0, 128, 0), CV_FILLED); // green
        cv::circle(debugMat, cv::Point(_goalGrid.x, g_planning_map.getSize().y - 1 - _goalGrid.y),
                   10, cv::Scalar(0, 0, 255), CV_FILLED); // red
        cv::circle(debugMat, cv::Point(_goalGrid.x, g_planning_map.getSize().y - 1 - _goalGrid.y),
                   0, cv::Scalar(0, 0, 128), CV_FILLED); // red
        writeStr = "/tmp/OpenSpacePlannerDebug/" + timeStr + "_a_star_1.bmp";
        cv::imwrite(writeStr, debugMat);
#endif

        if (!g_planning_map.inRange(_startGrid))
        {
            printf("ERROR, start_grid not in range\n");
            return 0;
        }
        else if (!g_planning_map.inRange(_goalGrid))
        {
            printf("ERROR, goal_grid not in range\n");
            return 0;
        }
        else if (mCostMap[_startGrid.x][_startGrid.y].obsValue == 255)
        {
            printf("ERROR, start_grid is obstacle\n");
            return 0;
        }
        else if (mCostMap[_goalGrid.x][_goalGrid.y].obsValue == 255)
        {
            printf("ERROR, goal_grid is obstacle\n");
            return 0;
        }
        else
        {
            // mGoalNode.reset(new AStarNode(_goalGrid.x, _goalGrid.y));
            // mStartNode.reset(new AStarNode(_startGrid.x, _startGrid.y));
            // mStartNode->setGCost(0.0f);
            // mStartNode->setHCost(computeHeuCost(mStartNode));
            // // put start in openList
            // mOpenSet.emplace(mStartNode->getIndex(), mStartNode);
            // openList.emplace(mStartNode->getIndex(), mStartNode->getFCost());

            mGoalGrid.x = _goalGrid.x;
            mGoalGrid.y = _goalGrid.y;
            mStartGrid.x = _startGrid.x;
            mStartGrid.y = _startGrid.y;
            mCostMap[_startGrid.x][_startGrid.y].gValue = 0;
            mCostMap[_startGrid.x][_startGrid.y].hValue = computeHeuCost(_startGrid.x, _startGrid.y);
            mCostMap[_startGrid.x][_startGrid.y].hValue += mCostMap[_startGrid.x][_startGrid.y].extraCost;
            mCostMap[_startGrid.x][_startGrid.y].isVisited = 1;
            mStartGrid.fValue = mCostMap[_startGrid.x][_startGrid.y].gValue + mCostMap[_startGrid.x][_startGrid.y].hValue;
            openList.push(mStartGrid);
        }

        // search path
        uint64_t aStarPlanningStartTs = open_space_utils::Timer::getTimestampUS();
        uint64_t cnt = 0;
        uint8_t expandDirNum = 0;
        if (mExpandType == ExpandType::EightNeighbor)
        {
            mAdjacentGrids.reserve(8);
            expandDirNum = 8;
        }
        else if (mExpandType == ExpandType::FourNeighbor)
        {
            expandDirNum = 4;
            mAdjacentGrids.reserve(4);
        }
        uint64_t sumTimeExpandNode = 0;
        uint64_t sumTimeOthers = 0;
        // std::vector<AStarGrid> close_set; // for debug
        // uint64_t show_expand_pic = 0; // for debug
        while (!openList.empty())
        {
            cnt++;
            // pop best from openList
            AStarGrid coreGrid = openList.top();
            openList.pop();
            // const std::string current_id = openList.top().first;
            // openList.pop();
            // std::shared_ptr<AStarNode> current_node = mOpenSet[current_id];
            // // ???是否在m_open_set中删除current_id的Node?
            // mOpenSet.erase(current_id);
            // if (cnt < 200)
            // {
            //     printf("coreGrid[%d][%d], g: %0.3f, h: %0.3f, f: %0.3f\n",
            //                             coreGrid.x, coreGrid.y, mCostMap[coreGrid.x][coreGrid.y].gValue, mCostMap[coreGrid.x][coreGrid.y].hValue, coreGrid.fValue);
            // }

            // #ifdef DEBUG_A_STAR
            //             // debug
            //             close_set.push_back(coreGrid);
            //             if (show_expand_pic > 1000)
            //             {
            //                 show_expand_pic = 0;
            //                 cv::Mat temp = debugMat.clone();
            //                 for (auto &grid : close_set)
            //                 {
            //                     cv::circle(temp, cv::Point(g_planning_map.getSize().y - 1 - grid.y, g_planning_map.getSize().x - 1 - grid.x),
            //                                0, cv::Scalar(238, 238, 175), CV_FILLED); // light blue
            //                 }
            //                 // cv::circle(temp, cv::Point(g_planning_map.getSize().y - 1 - coreGrid.y, g_planning_map.getSize().x - 1 - coreGrid.x),
            //                 //            0, cv::Scalar(255, 0, 0), CV_FILLED); // green
            //                 Grid parentGrid{-1, -1};
            //                 Grid currentGrid{coreGrid.x, coreGrid.y};
            //                 while (1)
            //                 {
            //                     parentGrid = mCostMap[currentGrid.x][currentGrid.y].parentGrid;
            //                     if (parentGrid.x < 0)
            //                     {
            //                         break;
            //                     }
            //                     else
            //                     {
            //                         cv::circle(temp, cv::Point(g_planning_map.getSize().y - 1 - currentGrid.y, g_planning_map.getSize().x - 1 - currentGrid.x), 0, cv::Scalar(0, 0, 0), CV_FILLED);
            //                     }
            //                     currentGrid = parentGrid;
            //                 }
            //                 cv::imshow("expand_pic", temp);
            //                 cv::waitKey(0);
            //             }
            //             else
            //             {
            //                 show_expand_pic++;
            //             }
            // #endif

            // put best in closed_list
            mCostMap[coreGrid.x][coreGrid.y].isVisited = 2;
            // mCloseSet.emplace(current_node->getIndex(), current_node);

            // if goal
            if (coreGrid.x == _goalGrid.x && coreGrid.y == _goalGrid.y)
            {
                // printf("coreGrid[%d][%d], parentGrid[%d][%d]\n", coreGrid.x, coreGrid.y, mCostMap[coreGrid.x][coreGrid.y].parentGrid.x, mCostMap[coreGrid.x][coreGrid.y].parentGrid.x);
                if (mHeuType != HeuType::None)
                {
                    break;
                }
            }
            // if (current_node->getX() == _goalGrid.x && current_node->getY() == _goalGrid.y)
            // {
            //     mSearchFinalNode = current_node;
            //     break;
            // }

            // #ifdef DEBUG_A_STAR
            //         printf("coreGrid: (%d, %d) cost: %d, g: %f, h: %f, f: %f\n",
            //                coreGrid.grid.x, coreGrid.grid.y,
            //                mCostMap[coreGrid.grid.x][coreGrid.grid.y].obsValue,
            //                mCostMap[coreGrid.grid.x][coreGrid.grid.y].gValue,
            //                mCostMap[coreGrid.grid.x][coreGrid.grid.y].hValue,
            //                coreGrid.fValue);
            //         cv::Mat tmp = debugMat.clone();
            //         cv::circle(tmp, cv::Point(g_planning_map.getSize().y - 1 - coreGrid.grid.y, g_planning_map.getSize().x - 1 - coreGrid.grid.x),
            //                    10, cv::Scalar(255, 0, 0), CV_FILLED); // blue
            // #endif
            expandCoreGrid(coreGrid, openList);

            // AStarGrid next_node;
            // for (size_t i = 0; i < expandDirNum; ++i)
            // {
            //     uint64_t ts1 = open_space_utils::Timer::getTimestampUS();
            //     next_node = expandNode(coreGrid, i);
            //     uint64_t ts2 = open_space_utils::Timer::getTimestampUS();
            //     sumTimeExpandNode += (ts2 - ts1);
            //     uint64_t ts_3 = 0;
            //     // out of bounds
            //     if (next_node.grid.x > g_planning_map.getSize().x ||
            //         next_node.grid.x < 0 ||
            //         next_node.grid.y > g_planning_map.getSize().y ||
            //         next_node.grid.y < 0)
            //     {
            //         ts_3 = open_space_utils::Timer::getTimestampUS();
            //         sumTimeOthers += (ts_3 - ts2);
            //         continue;
            //     }
            //     // check if already in mCloseSet
            //     if (mCostMap[next_node.grid.x][next_node.grid.y].isVisited == 2)
            //     {
            //         // printf("already in mCloseSet\n");
            //         ts_3 = open_space_utils::Timer::getTimestampUS();
            //         sumTimeOthers += (ts_3 - ts2);
            //         continue;
            //     }
            //     // collision check
            //     // uint64_t ts_3 = open_space_utils::Timer::getTimestampUS();
            //     // bool valid = is_node_valid(next_node);
            //     // uint64_t ts_4 = open_space_utils::Timer::getTimestampUS();
            //     // sum_time_is_node_valid += (ts_4 - ts_3);
            //     if (mCostMap[next_node.grid.x][next_node.grid.y].obsValue == 255)
            //     {
            //         // printf("next_node not valid\n");
            //         ts_3 = open_space_utils::Timer::getTimestampUS();
            //         sumTimeOthers += (ts_3 - ts2);
            //         continue;
            //     }
            //     float tmpGValue = mCostMap[coreGrid.grid.x][coreGrid.grid.y].gValue + getPiecewiseCost(coreGrid.grid, next_node.grid);
            //     if (mCostMap[next_node.grid.x][next_node.grid.y].isVisited == 1)
            //     {
            //         if (tmpGValue < mCostMap[next_node.grid.x][next_node.grid.y].gValue)
            //         {
            //             mCostMap[next_node.grid.x][next_node.grid.y].gValue = tmpGValue;
            //             next_node.fValue = tmpGValue + mCostMap[next_node.grid.x][next_node.grid.y].hValue;
            //             mCostMap[next_node.grid.x][next_node.grid.y].parentGrid = coreGrid.grid;
            //         }
            //     }
            //     else
            //     {
            //         // explored_node_num++;
            //         mCostMap[next_node.grid.x][next_node.grid.y].gValue = tmpGValue;
            //         mCostMap[next_node.grid.x][next_node.grid.y].hValue = getPiecewiseCost(next_node.grid, _goalGrid);
            //         mCostMap[next_node.grid.x][next_node.grid.y].hValue += mCostMap[next_node.grid.x][next_node.grid.y].extraCost;
            //         next_node.fValue = tmpGValue + mCostMap[next_node.grid.x][next_node.grid.y].hValue;
            //         mCostMap[next_node.grid.x][next_node.grid.y].parentGrid = coreGrid.grid;
            //         openList.push(next_node);
            //         mCostMap[next_node.grid.x][next_node.grid.y].isVisited = 1;
            //     }
            //     ts_3 = open_space_utils::Timer::getTimestampUS();
            //     sumTimeOthers += (ts_3 - ts2);
            // }

            // new expand
            // for (size_t i = 0; i < expandDirNum; ++i)
            // {
            //     uint64_t ts1 = open_space_utils::Timer::getTimestampUS();
            //     std::shared_ptr<AStarNode> next_node = expandNode(current_node, i);
            //     uint64_t ts2 = open_space_utils::Timer::getTimestampUS();
            //     sumTimeExpandNode += (ts2 - ts1);
            //     uint64_t ts_3 = 0;
            //     // out of bounds
            //     if (next_node == nullptr)
            //     {
            //         // printf("next_node == nullptr\n");
            //         ts_3 = open_space_utils::Timer::getTimestampUS();
            //         sumTimeOthers += (ts_3 - ts2);
            //         continue;
            //     }
            //     // check if already in mCloseSet
            //     if (mCloseSet.find(next_node->getIndex()) != mCloseSet.end())
            //     {
            //         // printf("already in mCloseSet\n");
            //         ts_3 = open_space_utils::Timer::getTimestampUS();
            //         sumTimeOthers += (ts_3 - ts2);
            //         continue;
            //     }
            //     // collision check
            //     // uint64_t ts_3 = open_space_utils::Timer::getTimestampUS();
            //     // bool valid = is_node_valid(next_node);
            //     // uint64_t ts_4 = open_space_utils::Timer::getTimestampUS();
            //     // sum_time_is_node_valid += (ts_4 - ts_3);
            //     if (mCostMap[next_node->getX()][next_node->getY()].obsValue == 255)
            //     {
            //         // printf("next_node not valid\n");
            //         ts_3 = open_space_utils::Timer::getTimestampUS();
            //         sumTimeOthers += (ts_3 - ts2);
            //         continue;
            //     }
            //     auto it = mOpenSet.find(next_node->getIndex());
            //     if (it == mOpenSet.end())
            //     {
            //         // explored_node_num++;
            //         next_node->setGCost(current_node->getGCost() + getPiecewiseCost(current_node, next_node));
            //         next_node->setHCost(computeHeuCost(next_node));
            //         mOpenSet.emplace(next_node->getIndex(), next_node);
            //         openList.emplace(next_node->getIndex(), next_node->getFCost());
            //     }
            //     else
            //     {
            //         next_node = it->second;
            //         float tmp_g_cost = current_node->getGCost() + getPiecewiseCost(current_node, next_node);
            //         if (tmp_g_cost < next_node->getGCost())
            //         {
            //             // printf("update g_cost: %f -> %f\n", next_node->getGCost(), tmp_g_cost);
            //             next_node->setGCost(tmp_g_cost);
            //             next_node->setPreNode(current_node);
            //         }
            //     }
            //     ts_3 = open_space_utils::Timer::getTimestampUS();
            //     sumTimeOthers += (ts_3 - ts2);
            // }
            // #ifdef DEBUG_A_STAR
            // printf("planning cnt: %d, openList.size: %lu\n", cnt, openList.size());
            //         imshow("coreGrid", tmp);
            //         waitKey();
            // #endif
        }
        printf("search done, cnt: %lu\n", cnt);
        uint64_t aStarPlanningDoneTs = open_space_utils::Timer::getTimestampUS();
        float searchPeriod = float(aStarPlanningDoneTs - aStarPlanningStartTs) * 0.001; // unit: ms

        // output path
        uint64_t outputStartTs = open_space_utils::Timer::getTimestampUS();
        std::vector<Grid> reversePath;
        Grid currentGrid = _goalGrid;
        Grid parentGrid{-1, -1};
        while (1)
        {
            parentGrid = mCostMap[currentGrid.x][currentGrid.y].parentGrid;
            if (parentGrid.x < 0)
            {
                printf("[%d][%d], planning failed, -2\n", currentGrid.x, currentGrid.y);
                return -2;
            }
            else
            {
                reversePath.push_back(currentGrid);
                if (parentGrid.x == _startGrid.x && parentGrid.y == _startGrid.y)
                {
                    printf("goal found\n");
                    reversePath.push_back(parentGrid);
                    break;
                }
            }
            currentGrid = parentGrid;
        }
        for (int i = (int)reversePath.size() - 1; i >= 0; i--)
        {
            _GridPath_o.push_back(reversePath[i]);
#ifdef DEBUG_A_STAR
            cv::circle(debugMat, cv::Point(reversePath[i].x, g_planning_map.getSize().y - 1 - reversePath[i].y), 0, cv::Scalar(0, 0, 0), CV_FILLED);
#endif
        }

        // new_output
        //         if (mSearchFinalNode == nullptr)
        //         {
        //             return -2;
        //         }
        //         if (!getOutputPath(mOutputPath))
        //         {
        //             return -3;
        //         }
        //         for (size_t i = 0; i < mOutputPath.size(); ++i)
        //         {
        // #ifdef DEBUG_A_STAR
        //             cv::circle(debugMat, cv::Point(g_planning_map.getSize().y - 1 - mOutputPath[i].y, g_planning_map.getSize().x - 1 - mOutputPath[i].x), 0, cv::Scalar(0, 0, 0), CV_FILLED);
        // #endif
        //         }

        printf("output A* path size: %lu\n", _GridPath_o.size());

#ifdef DEBUG_A_STAR
        writeStr = "/tmp/OpenSpacePlannerDebug/" + timeStr + "_a_star_2.bmp";
        cv::imwrite(writeStr, debugMat);
#endif
        uint64_t outputDoneTs = open_space_utils::Timer::getTimestampUS();
        float outputPeriod = float(outputDoneTs - outputStartTs) * 0.001; // unit: ms
        float expandNodePeriod = float(sumTimeExpandNode) * 0.001;        // unit: ms
        float othersPeriod = float(sumTimeOthers) * 0.001;
        float testPeriod = float(testTime) * 0.001;
        float updateAdjacentGridsPeriod = float(updateAdjacent) * 0.001;
        float othersInExpandPeriod = float(othersInExpand) * 0.001;

        float aStarTotalPeriod = float(outputDoneTs - costMapStartTs) * 0.001; // unit: ms
        printf("A* [ms]: costMapPeriod: %0.3f, searchPeriod: %0.3f, outputPeriod: %0.3f, total_period: %0.3f\n",
               costMapPeriod, searchPeriod, outputPeriod, aStarTotalPeriod);
        printf("A* [ms]: othersInExpandPeriod: %0.3f, updateAdjacentGridsPeriod: %0.3f\n",
               othersInExpandPeriod, updateAdjacentGridsPeriod);
        // printf("A* [ms]: expandNodePeriod: %0.3f, othersPeriod: %0.3f, testPeriod: %0.3f\n",
        //                         expandNodePeriod, othersPeriod, testPeriod);

        // show cost_map
        // #ifdef DEBUG_A_STAR
        //         cv::Mat costMapMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
        //         for (size_t i = 0; i < mCostMap.size(); ++i)
        //         {
        //             for (size_t j = 0; j < mCostMap[i].size(); ++j)
        //             {
        //                 if (mCostMap[i][j].obsValue == 255) // obs
        //                 {
        //                     cv::circle(costMapMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(64, 64, 64), CV_FILLED);
        //                 }
        //                 else if (mCostMap[i][j].gValue < 10000)
        //                 {
        //                     cv::circle(costMapMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(0, 255, 0), CV_FILLED);
        //                 }
        //             }
        //         }
        //         std::string costMapWriteStr = "/tmp/OpenSpacePlannerDebuga_star_cost_map.bmp";
        //         cv::imwrite(costMapWriteStr, costMapMat);
        // #endif

        return 1;
    }

    void AStarPlanner::expandCoreGrid(const AStarGrid &_coreGrid, std::priority_queue<AStarGrid> &_openList_io)
    {
        updateAdjacentGrids(_coreGrid); // hValue updated
        // printf("updateAdjacentGrids done, coreGrid[%d, %d]\n", _coreGrid.x, _coreGrid.y);

        uint64_t ts1 = open_space_utils::Timer::getTimestampUS();
        for (size_t i = 0; i < mAdjacentGrids.size(); i++)
        {
            // printf("mAdjacentGrids[%lu]: [%d, %d], isVisited: %u, obs: %d, g: %0.3f, h: %0.3f, f: %0.3f\n", i,
            //        mAdjacentGrids[i].x, mAdjacentGrids[i].y,
            //        mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].isVisited,
            //        mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].obsValue,
            //        mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].gValue,
            //        mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].hValue,
            //        mAdjacentGrids[i].fValue);

            if (mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].obsValue != 255)
            {
                float tmpGValue = mCostMap[_coreGrid.x][_coreGrid.y].gValue +
                                  getPiecewiseCost(_coreGrid.x, _coreGrid.y, mAdjacentGrids[i].x, mAdjacentGrids[i].y);
                // printf("tmpGValue: %f, gValue: %f\n", tmpGValue, mCostMap[mAdjacentGrids[i].grid.x][mAdjacentGrids[i].grid.y].gValue);
                if (mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].isVisited == 1)
                {
                    if (tmpGValue < mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].gValue)
                    {
                        mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].gValue = tmpGValue;
                        mAdjacentGrids[i].fValue = tmpGValue + mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].hValue;
                        mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].parentGrid = Grid{_coreGrid.x, _coreGrid.y}; // TODO further optimize
                    }
                }
                else if (mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].isVisited == 2)
                {
                    // printf("mAdjacentGrids[%d][%d], g_before: %0.3f, g: %0.3f\n",
                    //        mAdjacentGrids[i].x, mAdjacentGrids[i].y, mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].gValue, tmpGValue);
                    // if (mHeuType == HeuType::None && tmpGValue < mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].gValue)
                    // {
                    //     mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].gValue = tmpGValue;
                    //     mAdjacentGrids[i].fValue = tmpGValue + mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].hValue;
                    //     mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].parentGrid = Grid{_coreGrid.x, _coreGrid.y}; // TODO further optimize
                    //     _openList_io.push(mAdjacentGrids[i]);
                    //     mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].isVisited = 1;
                    // }
                    // do nothing
                }
                else
                {
                    mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].gValue = tmpGValue;
                    mAdjacentGrids[i].fValue = tmpGValue + mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].hValue;
                    mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].parentGrid = Grid{_coreGrid.x, _coreGrid.y}; // TODO further optimize
                    _openList_io.push(mAdjacentGrids[i]);
                    mCostMap[mAdjacentGrids[i].x][mAdjacentGrids[i].y].isVisited = 1;
                }
            }
        }
        uint64_t ts2 = open_space_utils::Timer::getTimestampUS();
        othersInExpand += (ts2 - ts1);
        return;
    }

    void AStarPlanner::updateAdjacentGrids(const AStarGrid &_coreGrid)
    {
        uint64_t startTs = open_space_utils::Timer::getTimestampUS();
        mAdjacentGrids.clear();
        int xMin = _coreGrid.x - 1;
        int xMax = _coreGrid.x + 1;
        int yMin = _coreGrid.y - 1;
        int yMax = _coreGrid.y + 1;

        if (xMin >= 0)
        {
            if (mExpandType == ExpandType::EightNeighbor)
            {
                if (yMin >= 0)
                {
                    if (mCostMap[xMin][yMin].hValue < 0)
                    {
                        mCostMap[xMin][yMin].hValue = computeHeuCost(xMin, yMin);
                        mCostMap[xMin][yMin].hValue += mCostMap[xMin][yMin].extraCost;
                    }
                    mAdjacentGrids.emplace_back(xMin, yMin, mCostMap[xMin][yMin].gValue + mCostMap[xMin][yMin].hValue);
                }
            }
            {
                if (mCostMap[xMin][_coreGrid.y].hValue < 0)
                {
                    mCostMap[xMin][_coreGrid.y].hValue = computeHeuCost(xMin, _coreGrid.y);
                    mCostMap[xMin][_coreGrid.y].hValue += mCostMap[xMin][_coreGrid.y].extraCost;
                }
                mAdjacentGrids.emplace_back(xMin, _coreGrid.y, mCostMap[xMin][_coreGrid.y].gValue + mCostMap[xMin][_coreGrid.y].hValue);
            }
            if (mExpandType == ExpandType::EightNeighbor)
            {
                if (yMax < g_planning_map.getSize().y)
                {
                    if (mCostMap[xMin][yMax].hValue < 0)
                    {
                        mCostMap[xMin][yMax].hValue = computeHeuCost(xMin, yMax);
                        mCostMap[xMin][yMax].hValue += mCostMap[xMin][yMax].extraCost;
                    }
                    mAdjacentGrids.emplace_back(xMin, yMax, mCostMap[xMin][yMax].gValue + mCostMap[xMin][yMax].hValue);
                }
            }
        }
        {
            if (yMin >= 0)
            {
                if (mCostMap[_coreGrid.x][yMin].hValue < 0)
                {
                    mCostMap[_coreGrid.x][yMin].hValue = computeHeuCost(_coreGrid.x, yMin);
                    mCostMap[_coreGrid.x][yMin].hValue += mCostMap[_coreGrid.x][yMin].extraCost;
                }
                mAdjacentGrids.emplace_back(_coreGrid.x, yMin, mCostMap[_coreGrid.x][yMin].gValue + mCostMap[_coreGrid.x][yMin].hValue);
            }
            if (yMax < g_planning_map.getSize().y)
            {
                if (mCostMap[_coreGrid.x][yMax].hValue < 0)
                {
                    mCostMap[_coreGrid.x][yMax].hValue = computeHeuCost(_coreGrid.x, yMax);
                    mCostMap[_coreGrid.x][yMax].hValue += mCostMap[_coreGrid.x][yMax].extraCost;
                }
                mAdjacentGrids.emplace_back(_coreGrid.x, yMax, mCostMap[_coreGrid.x][yMax].gValue + mCostMap[_coreGrid.x][yMax].hValue);
            }
        }
        if (xMax < g_planning_map.getSize().x)
        {
            if (mExpandType == ExpandType::EightNeighbor)
            {
                if (yMin >= 0)
                {
                    if (mCostMap[xMax][yMin].hValue < 0)
                    {
                        mCostMap[xMax][yMin].hValue = computeHeuCost(xMax, yMin);
                        mCostMap[xMax][yMin].hValue += mCostMap[xMax][yMin].extraCost;
                    }
                    mAdjacentGrids.emplace_back(xMax, yMin, mCostMap[xMax][yMin].gValue + mCostMap[xMax][yMin].hValue);
                }
            }
            {
                if (mCostMap[xMax][_coreGrid.y].hValue < 0)
                {
                    mCostMap[xMax][_coreGrid.y].hValue = computeHeuCost(xMax, _coreGrid.y);
                    mCostMap[xMax][_coreGrid.y].hValue += mCostMap[xMax][_coreGrid.y].extraCost;
                }
                mAdjacentGrids.emplace_back(xMax, _coreGrid.y, mCostMap[xMax][_coreGrid.y].gValue + mCostMap[xMax][_coreGrid.y].hValue);
            }
            if (mExpandType == ExpandType::EightNeighbor)
            {
                if (yMax < g_planning_map.getSize().y)
                {
                    if (mCostMap[xMax][yMax].hValue < 0)
                    {
                        mCostMap[xMax][yMax].hValue = computeHeuCost(xMax, yMax);
                        mCostMap[xMax][yMax].hValue += mCostMap[xMax][yMax].extraCost;
                    }
                    mAdjacentGrids.emplace_back(xMax, yMax, mCostMap[xMax][yMax].gValue + mCostMap[xMax][yMax].hValue);
                }
            }
        }
        uint64_t endTs = open_space_utils::Timer::getTimestampUS();
        updateAdjacent = updateAdjacent + endTs - startTs;
        return;
    }

    std::vector<std::vector<Cost>> *AStarPlanner::getCostMap()
    {
        return &mCostMap;
    }

    // float AStarPlanner::getPiecewiseCost(std::shared_ptr<AStarNode> _currentNode,
    //                                 std::shared_ptr<AStarNode> _nextNode)
    // {
    //     if (mExpandType == ExpandType::FourNeighbor || (mExpandType == ExpandType::EightNeighbor && mHeuType == HeuType::Manhattan))
    //     {
    //         return get_manhattan_distance(_currentNode, _nextNode);
    //     }
    //     else if (mExpandType == ExpandType::EightNeighbor && mHeuType == HeuType::Diagonal)
    //     {
    //         return getDiagonalDistance(_currentNode, _nextNode);
    //     }
    // }

    // bool AStarPlanner::getOutputPath(std::vector<Grid> &_outputPath_o)
    // {
    //     std::shared_ptr<AStarNode> current_node = mSearchFinalNode;
    //     while (current_node->getPreMode() != nullptr)
    //     {
    //         Grid tmp{current_node->getX(), current_node->getY()};
    //         _outputPath_o.push_back(tmp);
    //         current_node = current_node->getPreMode();
    //     }
    //     std::reverse(_outputPath_o.begin(), _outputPath_o.end());

    //     return true;
    // }

    // std::shared_ptr<AStarNode> AStarPlanner::expandNode(std::shared_ptr<AStarNode> _currentNode, size_t _nextNodeIndex)
    // {
    //     int next_node_x;
    //     int next_node_y;
    //     if (mExpandType == ExpandType::EightNeighbor)
    //     {
    //         /*
    //                 ^x
    //         0  1  2 |
    //         3  *  4 |
    //         5  6  7 |
    //                 |
    //         y<-------
    //         */
    //         if (_nextNodeIndex == 0)
    //         {
    //             next_node_x = _currentNode->getX() + 1;
    //             next_node_y = _currentNode->getY() + 1;
    //         }
    //         else if (_nextNodeIndex == 1)
    //         {
    //             next_node_x = _currentNode->getX() + 1;
    //             next_node_y = _currentNode->getY();
    //         }
    //         else if (_nextNodeIndex == 2)
    //         {
    //             next_node_x = _currentNode->getX() + 1;
    //             next_node_y = _currentNode->getY() - 1;
    //         }
    //         else if (_nextNodeIndex == 3)
    //         {
    //             next_node_x = _currentNode->getX();
    //             next_node_y = _currentNode->getY() + 1;
    //         }
    //         else if (_nextNodeIndex == 4)
    //         {
    //             next_node_x = _currentNode->getX();
    //             next_node_y = _currentNode->getY() - 1;
    //         }
    //         else if (_nextNodeIndex == 5)
    //         {
    //             next_node_x = _currentNode->getX() - 1;
    //             next_node_y = _currentNode->getY() + 1;
    //         }
    //         else if (_nextNodeIndex == 6)
    //         {
    //             next_node_x = _currentNode->getX() - 1;
    //             next_node_y = _currentNode->getY();
    //         }
    //         else if (_nextNodeIndex == 7)
    //         {
    //             next_node_x = _currentNode->getX() - 1;
    //             next_node_y = _currentNode->getY() - 1;
    //         }
    //     }
    //     else if (mExpandType == ExpandType::FourNeighbor)
    //     {
    //         /*
    //                 ^x
    //            0    |
    //         1  *  2 |
    //            3    |
    //                 |
    //         y<-------
    //         */
    //         if (_nextNodeIndex == 0)
    //         {
    //             next_node_x = _currentNode->getX() + 1;
    //             next_node_y = _currentNode->getY();
    //         }
    //         else if (_nextNodeIndex == 1)
    //         {
    //             next_node_x = _currentNode->getX();
    //             next_node_y = _currentNode->getY() + 1;
    //         }
    //         else if (_nextNodeIndex == 2)
    //         {
    //             next_node_x = _currentNode->getX();
    //             next_node_y = _currentNode->getY() - 1;
    //         }
    //         else if (_nextNodeIndex == 3)
    //         {
    //             next_node_x = _currentNode->getX() - 1;
    //             next_node_y = _currentNode->getY();
    //         }
    //     }
    //     if (next_node_x > g_planning_map.getSize().x ||
    //         next_node_x < 0 ||
    //         next_node_y > g_planning_map.getSize().y ||
    //         next_node_y < 0)
    //     {
    //         return nullptr;
    //     }
    //     uint64_t test1_ts = open_space_utils::Timer::getTimestampUS();

    //     std::shared_ptr<AStarNode> next_node = std::shared_ptr<AStarNode>(
    //         new AStarNode(next_node_x, next_node_y));
    //     uint64_t test2_ts = open_space_utils::Timer::getTimestampUS();
    //     next_node->setPreNode(_currentNode);

    //     testTime += (test2_ts - test1_ts);
    //     return next_node;
    // }

    // AStarGrid AStarPlanner::expandNode(const AStarGrid &_coreGrid, size_t _nextNodeIndex)
    // {
    //     uint64_t test1_ts = open_space_utils::Timer::getTimestampUS();
    //     int next_node_x;
    //     int next_node_y;
    //     if (mExpandType == ExpandType::EightNeighbor)
    //     {
    //         /*
    //                 ^x
    //         0  1  2 |
    //         3  *  4 |
    //         5  6  7 |
    //                 |
    //         y<-------
    //         */
    //         if (_nextNodeIndex == 0)
    //         {
    //             next_node_x = _coreGrid.grid.x + 1;
    //             next_node_y = _coreGrid.grid.y + 1;
    //         }
    //         else if (_nextNodeIndex == 1)
    //         {
    //             next_node_x = _coreGrid.grid.x + 1;
    //             next_node_y = _coreGrid.grid.y;
    //         }
    //         else if (_nextNodeIndex == 2)
    //         {
    //             next_node_x = _coreGrid.grid.x + 1;
    //             next_node_y = _coreGrid.grid.y - 1;
    //         }
    //         else if (_nextNodeIndex == 3)
    //         {
    //             next_node_x = _coreGrid.grid.x;
    //             next_node_y = _coreGrid.grid.y + 1;
    //         }
    //         else if (_nextNodeIndex == 4)
    //         {
    //             next_node_x = _coreGrid.grid.x;
    //             next_node_y = _coreGrid.grid.y - 1;
    //         }
    //         else if (_nextNodeIndex == 5)
    //         {
    //             next_node_x = _coreGrid.grid.x - 1;
    //             next_node_y = _coreGrid.grid.y + 1;
    //         }
    //         else if (_nextNodeIndex == 6)
    //         {
    //             next_node_x = _coreGrid.grid.x - 1;
    //             next_node_y = _coreGrid.grid.y;
    //         }
    //         else if (_nextNodeIndex == 7)
    //         {
    //             next_node_x = _coreGrid.grid.x - 1;
    //             next_node_y = _coreGrid.grid.y - 1;
    //         }
    //     }
    //     else if (mExpandType == ExpandType::FourNeighbor)
    //     {
    //         /*
    //                 ^x
    //            0    |
    //         1  *  2 |
    //            3    |
    //                 |
    //         y<-------
    //         */
    //         if (_nextNodeIndex == 0)
    //         {
    //             next_node_x = _coreGrid.grid.x + 1;
    //             next_node_y = _coreGrid.grid.y;
    //         }
    //         else if (_nextNodeIndex == 1)
    //         {
    //             next_node_x = _coreGrid.grid.x;
    //             next_node_y = _coreGrid.grid.y + 1;
    //         }
    //         else if (_nextNodeIndex == 2)
    //         {
    //             next_node_x = _coreGrid.grid.x;
    //             next_node_y = _coreGrid.grid.y - 1;
    //         }
    //         else if (_nextNodeIndex == 3)
    //         {
    //             next_node_x = _coreGrid.grid.x - 1;
    //             next_node_y = _coreGrid.grid.y;
    //         }
    //     }

    //     uint64_t test2_ts = open_space_utils::Timer::getTimestampUS();

    //     testTime += (test2_ts - test1_ts);
    //     return AStarGrid{Grid{next_node_x, next_node_y}};
    // }
}