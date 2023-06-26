/***********************************
 * File Name   : AStarPlanner.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description :
 ***********************************/

#ifndef OPEN_SPACE_PLANNER_A_STAR_PLANNER_H
#define OPEN_SPACE_PLANNER_A_STAR_PLANNER_H

#include "AStarNode.h"
#include "Grid.h"
#include "float.h"
#include "queue"

using namespace open_space_utils;

namespace grid_a_star
{
    enum class HeuType : uint8_t
    {
        None,
        Manhattan,
        Diagonal,
        Euclidean,
    };

    enum class ExpandType : uint8_t
    {
        FourNeighbor,
        EightNeighbor,
    };

    enum class ObsTypeInvolved : uint8_t
    {
        ObstacleOnly,
        // ObstacleInflated,
        // ObstacleWithGradient,
    };

    struct AStarGrid
    {
        int x;
        int y;
        float fValue = -1.0f;
        // std::shared_ptr<AStarGrid> preNode = nullptr;
        bool operator<(const AStarGrid &a) const
        {
            return fValue > a.fValue; // 小顶堆
        }
        AStarGrid() {}
        AStarGrid(int _x, int _y, float _fValue) : x(_x), y(_y), fValue(_fValue) {}
    };

    struct Cost
    {
        uint8_t obsValue = 0;
        float gValue = FLT_MAX;
        float hValue = -1.0f;
        // float fValue = -1.0f; // unused
        float extraCost = 0.0f;
        uint8_t isVisited = 0; // 0-not in open/closed list, 1-in open list, 2-in closed list
        open_space_utils::Grid parentGrid{-1, -1};
    };

    class AStarPlanner
    {
    public:
        AStarPlanner();
        virtual ~AStarPlanner() = default;

        bool init(std::string _logName);

        /**
         * @brief  A*路径规划
         * @param  _startGrid       起点栅格
         * @param  _goalGrid        终点栅格
         * @param  _GridPath_o 输出规划路径
         * @param  _obsType         考虑障碍物类型
         * @param  _heuType         heuristic类型
         * @param  _expandType      扩张类型
         * @retval 1为规划成功，否则为各种错误
         */
        int8_t planning(const Grid &_startGrid, const Grid &_goalGrid, std::vector<Grid> &_GridPath_o,
                        ObsTypeInvolved _obsType, HeuType _heuType, ExpandType _expandType);

        std::vector<std::vector<Cost>> *getCostMap();

    private:
        /**
         * @brief  A*更新周围栅格
         * @param  _coreGrid        当前栅格
         * @param  _openList_io     A*的open_list
         * @retval None
         */
        void expandCoreGrid(const AStarGrid &_coreGrid, std::priority_queue<AStarGrid> &_openList_io);

        /**
         * @brief  A*得到周围栅格
         * @param  _coreGrid        当前栅格
         * @retval 周围栅格
         */
        void updateAdjacentGrids(const AStarGrid &_coreGrid);

        inline float getPiecewiseCost(const int &_x_1, const int &_y_1, const int &_x_2, const int &_y_2)
        {
            return getDiagonalDistance(_x_1, _y_1, _x_2, _y_2);
            // return getEuclideanDistance(_x_1, _y_1, _x_2, _y_2);
        }

        // float getPiecewiseCost(std::shared_ptr<AStarNode> _currentNode,
        //                          std::shared_ptr<AStarNode> _nextNode);

        // std::shared_ptr<AStarNode> expandNode(std::shared_ptr<AStarNode> _currentNode, size_t _nextNodeIndex);

        // AStarGrid expandNode(const AStarGrid &_coreGrid, size_t _nextNodeIndex);

        // bool getOutputPath(std::vector<Grid> &_outputPath_o);

        inline float computeHeuCost(const int &_x, const int &_y)
        {
            if (mExpandType == ExpandType::FourNeighbor || (mExpandType == ExpandType::EightNeighbor && mHeuType == HeuType::Manhattan))
            {
                return get_manhattan_distance(_x, _y, mGoalGrid.x, mGoalGrid.y);
            }
            else if (mExpandType == ExpandType::EightNeighbor && mHeuType == HeuType::Diagonal)
            {
                return getDiagonalDistance(_x, _y, mGoalGrid.x, mGoalGrid.y);
            }
            else if (mExpandType == ExpandType::EightNeighbor && mHeuType == HeuType::Euclidean)
            {
                return getEuclideanDistance(_x, _y, mGoalGrid.x, mGoalGrid.y);
            }
            else
            {
                return 0.0f;
            }
        }

        // inline float computeHeuCost(std::shared_ptr<AStarNode> _currentNode)
        // {
        //     if (mExpandType == ExpandType::FourNeighbor || (mExpandType == ExpandType::EightNeighbor && mHeuType == HeuType::Manhattan))
        //     {
        //         return get_manhattan_distance(_currentNode, mGoalNode);
        //     }
        //     else if (mExpandType == ExpandType::EightNeighbor && mHeuType == HeuType::Diagonal)
        //     {
        //         return getDiagonalDistance(_currentNode, mGoalNode);
        //     }
        //     else
        //     {
        //         return 0.0f;
        //     }
        // }

        // inline float get_manhattan_distance(std::shared_ptr<AStarNode> _node_1, std::shared_ptr<AStarNode> _node_2)
        // {
        //     return (float)abs((_node_1->getX() - _node_2->getX()) + abs(_node_1->getY() - _node_2->getY()));
        // };

        inline float get_manhattan_distance(const int &_x_1, const int &_y_1, const int &_x_2, const int &_y_2)
        {
            return (float)(abs(_x_1 - _x_2) + abs(_y_1 - _y_2));
        };

        // inline float getDiagonalDistance(std::shared_ptr<AStarNode> _node_1, std::shared_ptr<AStarNode> _node_2)
        // {
        //     int dx = abs(_node_1->getX() - _node_2->getX());
        //     int dy = abs(_node_1->getY() - _node_2->getY());
        //     return (float)(dx + dy) + (D2 - 2) * (float)std::min(dx, dy);
        // };

        inline float getDiagonalDistance(const int &_x_1, const int &_y_1, const int &_x_2, const int &_y_2)
        {
            int dx = abs(_x_1 - _x_2);
            int dy = abs(_y_1 - _y_2);
            return (float)(dx + dy) + (D2 - 2) * (float)std::min(dx, dy);
        };

        inline float getEuclideanDistance(const int &_x_1, const int &_y_1, const int &_x_2, const int &_y_2)
        {
            return (float)hypot(_x_1 - _x_2, _y_1 - _y_2);
        };

        // struct cmp
        // {
        //     bool operator()(const std::pair<std::string, float> &left,
        //                     const std::pair<std::string, float> &right) const
        //     {
        //         return left.second >= right.second;
        //     }
        // };

        std::string mLogName;
        HeuType mHeuType = HeuType::Diagonal;               // default set to diagonal distance
        ExpandType mExpandType = ExpandType::EightNeighbor; // default set to eight neighbor
        float D2 = 1.414f;

        std::vector<std::vector<Cost>> mCostMap; // A*维护的数据地图
        AStarGrid mStartGrid;
        AStarGrid mGoalGrid;
        std::vector<AStarGrid> mAdjacentGrids;

        // std::unordered_map<std::string, std::shared_ptr<AStarNode>> mOpenSet;
        // std::unordered_map<std::string, std::shared_ptr<AStarNode>> mCloseSet;
        // std::shared_ptr<AStarNode> mStartNode;
        // std::shared_ptr<AStarNode> mGoalNode;
        // std::shared_ptr<AStarNode> mSearchFinalNode;
        // std::vector<Grid> mOutputPath;
    };
}
#endif