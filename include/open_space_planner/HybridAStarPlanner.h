/***********************************
 * File Name   : HybridAStarPlanner.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description :
 ***********************************/

#ifndef OPEN_SPACE_PLANNER_HYBRID_A_STAR_PLANNER_H
#define OPEN_SPACE_PLANNER_HYBRID_A_STAR_PLANNER_H

#include "HybridAStarNode.h"
#include "ReedsSheppPath.h"
#include "AStarPlanner.h"
#include "Pose.h"
#include "queue"
#include <unordered_map>
#include <opencv2/opencv.hpp>

// using namespace open_space_utils;

namespace hybrid_a_star
{
    enum class HeuType : uint8_t
    {
        None,
        Manhattan,
        Diagonal,
        Euclidean,
        AStarGValue
    };

    struct HybridAStarParam
    {
        std::string logName;
        float maxKappa;
        float axisLength;
        float basicThresholdLongitude;
        float basicThresholdLateral;
        float wheelBase;
        bool dubinsOnly = true;
    };

    class HybridAStarPlanner
    {
    public:
        HybridAStarPlanner(const HybridAStarParam &_param);
        virtual ~HybridAStarPlanner() = default;

        bool init();
        int8_t planning(const Pose &_startPose, const Pose &_goalPose, std::vector<Pose> &_outputPath_o,
                        HeuType _heuType, std::vector<std::vector<grid_a_star::Cost> > *_aStarCostMapPtr = nullptr);

    private:
        std::shared_ptr<HybridAStarNode> expandNode(
            std::shared_ptr<HybridAStarNode> _currentNode, size_t _nextNodeIndex);

        bool analyticExpansion(std::shared_ptr<HybridAStarNode> _currentNode);

        bool reedsSheppPathCheck(const std::shared_ptr<reeds_shepp::ReedsSheppPath> _reedsSheppToEnd);

        std::shared_ptr<HybridAStarNode> loadRsPathInCloseSet(
            const std::shared_ptr<reeds_shepp::ReedsSheppPath> _reedsSheppToEnd,
            std::shared_ptr<HybridAStarNode> _currentNode);

        void computeNewNodeCost(std::shared_ptr<HybridAStarNode> _currentNode,
                                   std::shared_ptr<HybridAStarNode> _nextNode);

        inline float computeHeuCost(std::shared_ptr<HybridAStarNode> _currentNode)
        {
            int dx = abs(_currentNode->getGridX() - mEndNode->getGridX());
            int dy = abs(_currentNode->getGridY() - mEndNode->getGridY());

            if (mHeuType == HeuType::Manhattan)
            {
                return dx + dy;
            }
            else if (mHeuType == HeuType::Diagonal)
            {
                return (float)(dx + dy) + (D2 - 2) * (float)std::min(dx, dy);
            }
            else if (mHeuType == HeuType::Euclidean)
            {
                return hypot(dx, dy);
            }
            else if (mHeuType == HeuType::AStarGValue)
            {
                float tmp = mCostMapPtr->at(_currentNode->getGridX())[_currentNode->getGridY()].gValue * mNodeGridRes;
                // printf("[%d][%d].gValue: %f\n", _currentNode->getGridX(), _currentNode->getGridY(), tmp);
                if (tmp > 100000) // TODO: deal with the possible un-initialized value
                {
                    return hypot(dx, dy);
                }
                return tmp;
            }
            else
            {
                return 0.0f;
            }
        }

        float getPiecewiseCost(std::shared_ptr<HybridAStarNode> _currentNode,
                                 std::shared_ptr<HybridAStarNode> _nextNode);

        bool getOutputPath(std::vector<Pose> &_outputPath_o);

        bool isNodeValid(std::shared_ptr<HybridAStarNode> _node);

        struct cmp
        {
            bool operator()(const std::pair<std::string, float> &left,
                            const std::pair<std::string, float> &right) const
            {
                return left.second >= right.second;
            }
        };

        std::string mLogName;
        HeuType mHeuType = HeuType::Diagonal; // default set to diagonal distance
        float D2 = 1.414f;
        std::vector<std::vector<grid_a_star::Cost>> *mCostMapPtr = nullptr;

        std::unique_ptr<reeds_shepp::ReedsShepp> mReedsSheppGenerator;
        std::unordered_map<std::string, std::shared_ptr<HybridAStarNode>> mOpenSet;
        std::unordered_map<std::string, std::shared_ptr<HybridAStarNode>> mCloseSet;
        std::shared_ptr<HybridAStarNode> mStartNode;
        std::shared_ptr<HybridAStarNode> mEndNode;
        std::shared_ptr<HybridAStarNode> mSearchFinalNode;
        std::vector<Pose> mOutputPath;
        float mAStarRes;
        std::vector<float> mXyBounds;
        uint64_t mRspEveryRunCnt;
        float mRspRadius;

        // global map param
        float mGlobalMapOriginX;
        float mGlobalMapOriginY;
        float mGlobalMapRes;
        int mGlobalMapSizeX;
        int mGlobalMapSizeY;
        float mZeroGridFromOriginOffsetX;
        float mZeroGridFromOriginOffsetY;

        // vehicle param
        float mAxisLength;
        float mBasicThresholdLongitude;
        float mBasicThresholdLateral;
        float mWheelBase;

        // cost weight
        float mQForward;
        float mQBackward;
        float mQDirectionSwitch;
        float mQSteering;
        float mQDsteering;

        // expand related
        size_t mExpandNodeNum;
        float mMaxSteering;
        float mTravelledRes;
        float mNodeGridRes;
        float mNodeYawRes;

        cv::Mat mDebugMat;
    };
}
#endif