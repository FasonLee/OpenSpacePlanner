/***********************************
 * File Name   : HybridAStarPlanner.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description :
 ***********************************/

#include "HybridAStarPlanner.h"
#include "GeometryFunc.h"
#include "Map.h"
#include "Timer.h"

#define DEBUG_HYBRID_A_STAR
#define DEBUG_NODE_VALID

namespace hybrid_a_star
{
    HybridAStarPlanner::HybridAStarPlanner(const HybridAStarParam &_param)
    {
        mLogName = _param.logName;
        mReedsSheppGenerator = std::make_unique<reeds_shepp::ReedsShepp>(_param.maxKappa, _param.dubinsOnly);
        // vehicle param
        mAxisLength = _param.axisLength;
        mBasicThresholdLongitude = _param.basicThresholdLongitude;
        mBasicThresholdLateral = _param.basicThresholdLateral;
        mWheelBase = _param.wheelBase;
    }

    bool HybridAStarPlanner::init()
    {
        mAStarRes = 1.0f;
        mRspEveryRunCnt = 20;
        mRspRadius = mAStarRes * (float)mRspEveryRunCnt;

        // global map param
        mGlobalMapOriginX = g_planning_map.getOriginPt().x;
        mGlobalMapOriginY = g_planning_map.getOriginPt().y;
        mGlobalMapRes = g_planning_map.getResolution();
        mGlobalMapSizeX = g_planning_map.getSize().x;
        mGlobalMapSizeY = g_planning_map.getSize().y;
        mZeroGridFromOriginOffsetX = g_planning_map.getZeroGridOffset().x;
        mZeroGridFromOriginOffsetY = g_planning_map.getZeroGridOffset().x;
        mXyBounds.clear();
        mXyBounds.push_back(g_planning_map.getRange().pt[0].x);
        mXyBounds.push_back(g_planning_map.getRange().pt[2].x);
        mXyBounds.push_back(g_planning_map.getRange().pt[0].y);
        mXyBounds.push_back(g_planning_map.getRange().pt[2].y);
        printf("init(), m_global_map_origin: %f, %f\n", mGlobalMapOriginX, mGlobalMapOriginY);
        printf("init(), m_global_map_size: %d, %d, mGlobalMapRes: %f\n", mGlobalMapSizeX, mGlobalMapSizeY, mGlobalMapRes);

        // cost weight
        mQForward = 1.0f;
        mQBackward = 1.0f;
        mQDirectionSwitch = 10.0f;
        mQSteering = 0.0f;  // work well when set 10.0f
        mQDsteering = 0.0f; // can be set 2.0f

        // expand related
        mExpandNodeNum = 12;
        mMaxSteering = open_space_utils::deg2rad(40);
        // mMaxSteering = atan(2*mAxisLength / mWheelBase); // 单轮不动
        mTravelledRes = 0.05f;
        mNodeGridRes = 0.05f;
        mNodeYawRes = open_space_utils::deg2rad(5);
        printf("init(), mExpandNodeNum: %lu, mMaxSteering: %0.1f[deg]\n", mExpandNodeNum, open_space_utils::rad2deg(mMaxSteering));

        return true;
    }

    int8_t HybridAStarPlanner::planning(const Pose &_startPose, const Pose &_goalPose, std::vector<Pose> &_outputPath_o,
                                        HeuType _heuType, std::vector<std::vector<grid_a_star::Cost>> *_aStarCostMapPtr)
    {
        std::string timeStr = open_space_utils::Timer::getReadableTimestampUS();
        // clear
        mOutputPath.clear();
        mOpenSet.clear();
        mCloseSet.clear();
        mHeuType = _heuType;
        mCostMapPtr = _aStarCostMapPtr;
        printf("===== hybrid A* planning, heu_type: %u\n", static_cast<std::underlying_type<HeuType>::type>(mHeuType));
        std::priority_queue<std::pair<std::string, float>, std::vector<std::pair<std::string, float>>, cmp> openList;

        // init map related
#ifdef DEBUG_HYBRID_A_STAR
        mDebugMat = cv::Mat(g_planning_map.getSize().x, g_planning_map.getSize().y, CV_8UC3, cv::Scalar(255, 255, 255));
        std::string writeStr = "/tmp/OpenSpacePlannerDebug/" + timeStr + "_hybrid_a_star_1.bmp";
#endif
        uint64_t debugMapStartTs = open_space_utils::Timer::getTimestampUS();
        for (size_t i = 0; i < g_planning_map.getSize().x; i++)
        {
            for (size_t j = 0; j < g_planning_map.getSize().y; j++)
            {
                if (g_planning_map.cell[i][j] > 200) // TODO
                {
#ifdef DEBUG_HYBRID_A_STAR
                    cv::circle(mDebugMat, cv::Point(i, g_planning_map.getSize().y - 1 - j), 0, cv::Scalar(64, 64, 64), CV_FILLED);
#endif
                }
                //                 else if (tmp_cell.cost == LETHAL_OBSTACLE)
                //                 {
                // #ifdef DEBUG_HYBRID_A_STAR
                //                     cv::circle(mDebugMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(128, 128, 128), CV_FILLED);
                // #endif
                //                 }
                //                 else if (tmp_cell.cost == INSCRIBED_INFLATED_OBSTACLE && mode_ > 0)
                //                 {
                // #ifdef DEBUG_HYBRID_A_STAR
                //                     cv::circle(mDebugMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(160, 160, 160), CV_FILLED);
                // #endif
                //                 }
                //                 else if (tmp_cell.cost > FREE_SPACE && mode_ > 1)
                //                 {
                // #ifdef DEBUG_HYBRID_A_STAR
                //                     cv::circle(mDebugMat, cv::Point(i, g_planning_map.getSize().y- 1 - j), 0, cv::Scalar(200, 200, 200), CV_FILLED);
                // #endif
                //                 }
                //                 else
                //                 {
                //                 }
            }
        }

        uint64_t debugMapDoneTs = open_space_utils::Timer::getTimestampUS();
        float debugMapPeriod = float(debugMapDoneTs - debugMapStartTs) * 0.001; // unit: ms
        printf("debugMapPeriod: %0.3f [ms]\n", debugMapPeriod);

        // init start & goal
#ifdef DEBUG_HYBRID_A_STAR
        cv::circle(mDebugMat, cv::Point(g_planning_map.poseToGrid(_startPose).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(_startPose).y),
                   10, cv::Scalar(0, 255, 0), CV_FILLED); // green
        cv::circle(mDebugMat, cv::Point(g_planning_map.poseToGrid(_startPose).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(_startPose).y),
                   0, cv::Scalar(0, 128, 0), CV_FILLED); // green
        cv::circle(mDebugMat, cv::Point(g_planning_map.poseToGrid(_goalPose).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(_goalPose).y),
                   10, cv::Scalar(0, 0, 255), CV_FILLED); // red
        cv::circle(mDebugMat, cv::Point(g_planning_map.poseToGrid(_goalPose).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(_goalPose).y),
                   0, cv::Scalar(0, 0, 128), CV_FILLED); // red
        cv::imwrite(writeStr, mDebugMat);
#endif
        mEndNode.reset(
            new HybridAStarNode({_goalPose.pt.x}, {_goalPose.pt.y}, {_goalPose.heading}, mXyBounds, mNodeGridRes, mNodeYawRes));
        mStartNode.reset(
            new HybridAStarNode({_startPose.pt.x}, {_startPose.pt.y}, {_startPose.heading}, mXyBounds, mNodeGridRes, mNodeYawRes));
        mStartNode->setGCost(0.0f);
        mStartNode->setHCost(computeHeuCost(mStartNode));
        if (!g_planning_map.inRange(_startPose.pt))
        {
            printf("start_pose out of range...\n");
            return -3;
        }
        else if (!g_planning_map.inRange(_goalPose.pt))
        {
            printf("goal_pose out of range...\n");
            return -3;
        }
        else if (!isNodeValid(mStartNode))
        {
            printf("start_pose not valid...\n");
            return -4;
        }
        else if (!isNodeValid(mEndNode))
        {
            printf("goal_pose not valid...\n");
            return -4;
        }
        printf("start_pose: %0.2f, %0.2f, %0.1f[deg]\n", _startPose.pt.x, _startPose.pt.y, rad2deg(_startPose.heading));
        printf("goal_pose: %0.2f, %0.2f, %0.1f[deg]\n", _goalPose.pt.x, _goalPose.pt.y, rad2deg(_goalPose.heading));

        // put start in openList
        mOpenSet.emplace(mStartNode->getIndex(), mStartNode);
        openList.emplace(mStartNode->getIndex(), mStartNode->getFCost());

        // search
        uint64_t searchStartTs = open_space_utils::Timer::getTimestampUS();
        uint64_t cnt = 0;
        uint64_t sumTimeIsNodeValid = 0;
        uint64_t sumTimeExpandNode = 0;
        uint64_t sumTimeRsp = 0;
        uint64_t rspCnt = mRspEveryRunCnt;
        uint64_t rspRunNum = 0;
        while (!openList.empty())
        {
            cnt++;
            // printf("========= cnt: %lu, openList.size: %lu, mOpenSet.size: %lu, mCloseSet.size: %lu\n", cnt, openList.size(), mOpenSet.size(), mCloseSet.size());
            // pop best from openList
            const std::string currentId = openList.top().first;
            openList.pop();
            std::shared_ptr<HybridAStarNode> currentNode = mOpenSet[currentId];
            // ???是否在mOpenSet中删除current_id的Node?
            mOpenSet.erase(currentId);
            // printf("currentNode: %s\n", currentNode->getIndex().c_str());
            // printf("currentNode[%d][%d], g: %0.3f, h: %0.3f, f: %0.3f\n",
            //                         currentNode->getGridX(), currentNode->getGridY(), currentNode->getGCost(), currentNode->getHCost(), currentNode->getFCost());
            // printf("currentNode[%d][%d], g: %0.3f, h: %0.3f, f: %0.3f\n",
            //        currentNode->getGridX(), currentNode->getGridY(), currentNode->getGCost(), currentNode->getHCost(), currentNode->getFCost());

            uint64_t rspStartTs = open_space_utils::Timer::getTimestampUS();
            float tmpDis = hypot(currentNode->getY() - mEndNode->getY(), currentNode->getX() - mEndNode->getX());
            if (rspCnt == mRspEveryRunCnt || tmpDis < mRspRadius)
            {
                rspCnt = 0;
                rspRunNum++;
                if (analyticExpansion(currentNode))
                {
                    break;
                }
            }
            else
            {
                rspCnt++;
            }
            uint64_t rspEndTs = open_space_utils::Timer::getTimestampUS();
            sumTimeRsp += (rspEndTs - rspStartTs);

            // put best in closed_list
            mCloseSet.emplace(currentNode->getIndex(), currentNode);

            // if goal (TO LEVEL UP)
            if (fabs(currentNode->getX() - mEndNode->getX()) < 0.2f &&
                fabs(currentNode->getY() - mEndNode->getY()) < 0.2f)
            {
                mSearchFinalNode = currentNode;
                break;
            }

            // expand
            for (size_t i = 0; i < mExpandNodeNum; ++i)
            {
                uint64_t ts1 = open_space_utils::Timer::getTimestampUS();
                std::shared_ptr<HybridAStarNode> nextNode = expandNode(currentNode, i);
                uint64_t ts2 = open_space_utils::Timer::getTimestampUS();
                sumTimeExpandNode += (ts2 - ts1);
                // printf("expand: %lu|%lu, nextNode: %s\n", i, mExpandNodeNum, nextNode->getIndex().c_str());
                // out of bounds
                if (nextNode == nullptr)
                {
                    // printf("nextNode == nullptr\n");
                    continue;
                }
                // check if already in mCloseSet
                if (mCloseSet.find(nextNode->getIndex()) != mCloseSet.end())
                {
                    // printf("already in mCloseSet\n");
                    continue;
                }
                // collision check
                uint64_t ts3 = open_space_utils::Timer::getTimestampUS();
                bool valid = true;
                if (mHeuType == HeuType::AStarGValue &&
                    mCostMapPtr->at(nextNode->getGridX())[nextNode->getGridY()].obsValue == 0) // inflate radius must consider travelled length, namely mAStarRes
                {
                    // do nothing
                }
                else
                {
                    valid = isNodeValid(nextNode);
                }
                uint64_t ts4 = open_space_utils::Timer::getTimestampUS();
                sumTimeIsNodeValid += (ts4 - ts3);
                if (!valid)
                {
                    // printf("nextNode not valid\n");
                    continue;
                }
                auto it = mOpenSet.find(nextNode->getIndex());
                if (it == mOpenSet.end())
                {
                    // explored_node_num++;
                    // const double start_time = Clock::NowInSeconds();
                    computeNewNodeCost(currentNode, nextNode);
                    // const double end_time = Clock::NowInSeconds();
                    // heuristic_time += end_time - start_time;
                    mOpenSet.emplace(nextNode->getIndex(), nextNode);
                    openList.emplace(nextNode->getIndex(), nextNode->getFCost());
                }
                else
                {
                    nextNode = it->second;
                    float tmpGCost = currentNode->getGCost() + getPiecewiseCost(currentNode, nextNode);
                    if (tmpGCost < nextNode->getGCost())
                    {
                        // printf("update g_cost: %f -> %f\n", nextNode->getGCost(), tmpGCost);
                        nextNode->setGCost(tmpGCost);
                        nextNode->setPreNode(currentNode);
                    }
                }
            }
            // printf("tail openList.size: %lu, mOpenSet.size: %lu, mCloseSet.size: %lu\n", openList.size(), mOpenSet.size(), mCloseSet.size());
        }
        printf("search done, cnt: %lu\n", cnt);
        uint64_t searchEndTs = open_space_utils::Timer::getTimestampUS();

        if (mSearchFinalNode == nullptr)
        {
            return -1;
        }

        uint64_t output_start_ts = open_space_utils::Timer::getTimestampUS();
        if (!getOutputPath(mOutputPath))
        {
            return -2;
        }
        _outputPath_o.clear();
        _outputPath_o = mOutputPath;
        printf("output hybrid A* path size: %lu\n", mOutputPath.size());
#ifdef DEBUG_HYBRID_A_STAR
        for (auto pp : mOutputPath)
        {
            cv::circle(mDebugMat, cv::Point(g_planning_map.poseToGrid(pp).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(pp).y), 0, cv::Scalar(0, 0, 0), CV_FILLED);
        }
        writeStr = "/tmp/OpenSpacePlannerDebug/" + timeStr + "_hybrid_a_star_2.bmp";
        cv::imwrite(writeStr, mDebugMat);
#endif

        printf("goal found\n");
        uint64_t outputEndTs = open_space_utils::Timer::getTimestampUS();
        float searchPeriod = float(searchEndTs - searchStartTs) * 0.001;   // unit: ms
        float outputPeriod = float(outputEndTs - output_start_ts) * 0.001; // unit: ms
        float expandNodePeriod = float(sumTimeExpandNode) * 0.001;         // unit: ms
        float rspPeriod = float(sumTimeRsp) * 0.001;                       // unit: ms
        float isNodeValidPeriod = float(sumTimeIsNodeValid) * 0.001;       // unit: ms
        printf("hybrid A* [ms]: searchPeriod: %0.3f, outputPeriod: %0.3f\n", searchPeriod, outputPeriod);
        printf("hybrid A* [ms]: expandNodePeriod: %0.3f, rspPeriod: %0.3f [%lu], isNodeValidPeriod: %0.3f\n",
               expandNodePeriod, rspPeriod, rspRunNum, isNodeValidPeriod);

        return 1;
    }

    bool HybridAStarPlanner::analyticExpansion(std::shared_ptr<HybridAStarNode> _currentNode)
    {
        std::shared_ptr<reeds_shepp::ReedsSheppPath> reedsSheppToCheck =
            std::make_shared<reeds_shepp::ReedsSheppPath>();
        if (!mReedsSheppGenerator->ShortestRSP(_currentNode, mEndNode,
                                               reedsSheppToCheck))
        {
            printf("ShortestRSP failed\n");
            return false;
        }

        if (!reedsSheppPathCheck(reedsSheppToCheck))
        {
            return false;
        }

        printf("Reach the end configuration with Reed Sharp\n");

        // load the whole RSP as nodes and add to the close set
        mSearchFinalNode = loadRsPathInCloseSet(reedsSheppToCheck, _currentNode);
        return true;
    }

    bool HybridAStarPlanner::reedsSheppPathCheck(const std::shared_ptr<reeds_shepp::ReedsSheppPath> _reedsSheppToEnd)
    {
        std::shared_ptr<HybridAStarNode> node = std::shared_ptr<HybridAStarNode>(new HybridAStarNode(
            _reedsSheppToEnd->x, _reedsSheppToEnd->y, _reedsSheppToEnd->phi,
            mXyBounds, mNodeGridRes, mNodeYawRes));
        return isNodeValid(node);
    }

    std::shared_ptr<HybridAStarNode> HybridAStarPlanner::loadRsPathInCloseSet(
        const std::shared_ptr<reeds_shepp::ReedsSheppPath> _reedsSheppToEnd,
        std::shared_ptr<HybridAStarNode> _currentNode)
    {
        std::shared_ptr<HybridAStarNode> endNode = std::shared_ptr<HybridAStarNode>(new HybridAStarNode(
            _reedsSheppToEnd->x, _reedsSheppToEnd->y, _reedsSheppToEnd->phi,
            mXyBounds, mNodeGridRes, mNodeYawRes));
        endNode->setPreNode(_currentNode);
        mCloseSet.emplace(endNode->getIndex(), endNode);
        return endNode;
    }

    std::shared_ptr<HybridAStarNode> HybridAStarPlanner::expandNode(
        std::shared_ptr<HybridAStarNode> _currentNode, size_t _nextNodeIndex)
    {
        float steering = 0.0;
        float travelledDistance = 0.0;
        if (_nextNodeIndex < static_cast<float>(mExpandNodeNum) / 2)
        {
            steering =
                -mMaxSteering +
                (2 * mMaxSteering / (static_cast<float>(mExpandNodeNum) / 2 - 1)) *
                    static_cast<float>(_nextNodeIndex);
            travelledDistance = mTravelledRes;
        }
        else
        {
            size_t index = _nextNodeIndex - mExpandNodeNum / 2;
            steering =
                -mMaxSteering +
                (2 * mMaxSteering / (static_cast<float>(mExpandNodeNum) / 2 - 1)) *
                    static_cast<float>(index);
            travelledDistance = -mTravelledRes;
        }

        // get travelled poses
        std::vector<float> intermediateX;
        std::vector<float> intermediateY;
        std::vector<float> intermediateHeading;
        float lastX = _currentNode->getX();
        float lastY = _currentNode->getY();
        float lastHeading = _currentNode->getHeading();
        intermediateX.push_back(lastX);
        intermediateY.push_back(lastY);
        intermediateHeading.push_back(lastHeading);
        // printf("mAStarRes: %f, mTravelledRes: %f\n", mAStarRes, mTravelledRes);
        for (size_t i = 0; i < mAStarRes / mTravelledRes; ++i)
        {
            const float nextX = lastX + travelledDistance * std::cos(lastHeading);
            const float nextY = lastY + travelledDistance * std::sin(lastHeading);
            const float nextHeading = open_space_utils::wrapToPi(
                lastHeading +
                travelledDistance / mAxisLength * std::tan(steering));
            intermediateX.push_back(nextX);
            intermediateY.push_back(nextY);
            intermediateHeading.push_back(nextHeading);
            // printf("intermediateHeading: %f\n", open_space_utils::rad2deg(nextHeading));
            lastX = nextX;
            lastY = nextY;
            lastHeading = nextHeading;
        }
        // check if out of XY boundary
        if (intermediateX.back() > mXyBounds[1] ||
            intermediateX.back() < mXyBounds[0] ||
            intermediateY.back() > mXyBounds[3] ||
            intermediateY.back() < mXyBounds[2])
        {
            return nullptr;
        }
        std::shared_ptr<HybridAStarNode> nextNode = std::shared_ptr<HybridAStarNode>(
            new HybridAStarNode(intermediateX, intermediateY, intermediateHeading, mXyBounds,
                                mNodeGridRes, mNodeYawRes));
        nextNode->setPreNode(_currentNode);
        nextNode->setDirection(travelledDistance > 0.0f);
        nextNode->setSteering(steering);
        return nextNode;
    }

    void HybridAStarPlanner::computeNewNodeCost(std::shared_ptr<HybridAStarNode> _currentNode,
                                                std::shared_ptr<HybridAStarNode> _nextNode)
    {
        _nextNode->setGCost(_currentNode->getGCost() + getPiecewiseCost(_currentNode, _nextNode));

        // set h cost
        float optimalPathCost = 0.0f;
        optimalPathCost += computeHeuCost(_nextNode); // 先粗略如此设定
        _nextNode->setHCost(optimalPathCost);
    }

    // float HybridAStarPlanner::computeHeuCost(std::shared_ptr<HybridAStarNode> _currentNode)
    // {
    //     int dx = abs(_currentNode->getGridX() - mEndNode->getGridX());
    //     int dy = abs(_currentNode->getGridY() - mEndNode->getGridY());
    //     // diagonal distance
    //     // float D2 = 1.414f;
    //     // return (float)(dx + dy) + (D2 - 2) * (float)std::min(dx, dy);

    //     // manhattan distance
    //     // return dx + dy;

    //     // euclidean distance
    //     return hypot(dx, dy);
    // }

    float HybridAStarPlanner::getPiecewiseCost(std::shared_ptr<HybridAStarNode> _currentNode,
                                               std::shared_ptr<HybridAStarNode> _nextNode)
    {
        float piecewiseCost = 0.0;
        if (_nextNode->getDirection())
        {
            piecewiseCost += static_cast<float>(_nextNode->getTravelledStep() - 1) *
                             mTravelledRes * mQForward;
        }
        else
        {
            piecewiseCost += static_cast<float>(_nextNode->getTravelledStep() - 1) *
                             mTravelledRes * mQBackward;
        }
        if (_currentNode->getDirection() != _nextNode->getDirection())
        {
            piecewiseCost += mQDirectionSwitch;
        }
        piecewiseCost += mQSteering * std::abs(_nextNode->getSteering());
        piecewiseCost += mQDsteering *
                         std::abs(_nextNode->getSteering() - _currentNode->getSteering());
        // printf("[%d][%d]->[%d][%d], step: %lu, piecewiseCost: %f\n", _currentNode->getGridX(), _currentNode->getGridY(),
        //        _nextNode->getGridX(), _nextNode->getGridY(), _nextNode->getTravelledStep(), piecewiseCost);
        return piecewiseCost;
    }

    bool HybridAStarPlanner::getOutputPath(std::vector<Pose> &_outputPath_o)
    {
        std::shared_ptr<HybridAStarNode> currentNode = mSearchFinalNode;
        std::vector<float> outputPathX;
        std::vector<float> outputPathY;
        std::vector<float> outputPathHeading;
        while (currentNode->getPreMode() != nullptr)
        {
            std::vector<float> x = currentNode->getTravelledX();
            std::vector<float> y = currentNode->getTravelledY();
            std::vector<float> heading = currentNode->getTravelledHeading();
            if (x.empty() || y.empty() || heading.empty())
            {
                return false;
            }
            if (x.size() != y.size() || x.size() != heading.size())
            {
                return false;
            }
            std::reverse(x.begin(), x.end());
            std::reverse(y.begin(), y.end());
            std::reverse(heading.begin(), heading.end());
            x.pop_back();
            y.pop_back();
            heading.pop_back();
            outputPathX.insert(outputPathX.end(), x.begin(), x.end());
            outputPathY.insert(outputPathY.end(), y.begin(), y.end());
            outputPathHeading.insert(outputPathHeading.end(), heading.begin(), heading.end());
            currentNode = currentNode->getPreMode();
        }
        outputPathX.push_back(currentNode->getX());
        outputPathY.push_back(currentNode->getY());
        outputPathHeading.push_back(currentNode->getHeading());
        std::reverse(outputPathX.begin(), outputPathX.end());
        std::reverse(outputPathY.begin(), outputPathY.end());
        std::reverse(outputPathHeading.begin(), outputPathHeading.end());

        if (outputPathX.size() != outputPathY.size() ||
            outputPathX.size() != outputPathHeading.size())
        {
            return false;
        }

        for (size_t i = 0; i < outputPathX.size(); ++i)
        {
            Pose tmp{outputPathX[i], outputPathY[i], outputPathHeading[i]};
            _outputPath_o.push_back(tmp);
        }

        return true;
    }

    bool HybridAStarPlanner::isNodeValid(std::shared_ptr<HybridAStarNode> _node)
    {
        if (_node == nullptr)
        {
            printf("isNodeValid(), nullptr\n");
            return false;
        }

        size_t nodeStepSize = _node->getTravelledStep();
        const auto &travelledX = _node->getTravelledX();
        const auto &travelledY = _node->getTravelledY();
        const auto &travelledHeading = _node->getTravelledHeading();

        std::unordered_map<std::string, std::pair<int, int>> mapGridUnderPose;

        // The first {x, y, phi} is collision free unless they are start and end
        // configuration of search problem
        size_t checkStartIndex = 0;
        if (nodeStepSize == 1)
        {
            checkStartIndex = 0;
        }
        else
        {
            checkStartIndex = 1;
        }

        for (size_t i = checkStartIndex; i < nodeStepSize; ++i)
        {
            if (travelledX[i] > mXyBounds[1] || travelledX[i] < mXyBounds[0] ||
                travelledY[i] > mXyBounds[3] || travelledY[i] < mXyBounds[2])
            {
#ifdef DEBUG_NODE_VALID
                printf("(%f, %f) out of bound {(%f, %f), (%f, %f)}\n", travelledX[i], travelledY[i], mXyBounds[0], mXyBounds[1], mXyBounds[2], mXyBounds[3]);
#endif
                return false;
            }
#ifdef DEBUG_NODE_VALID
            // printf("travelled[%lu]: %f, %f, %f\n", i, travelledX[i], travelledY[i], open_space_utils::rad2deg(travelledHeading[i]));
#endif
            float tmpX = travelledX[i] - mGlobalMapOriginX;
            float tmpY = travelledY[i] - mGlobalMapOriginY;
            float cosTheta = cos(travelledHeading[i]);
            float sinTheta = sin(travelledHeading[i]);
            float longitude = -0.94f;
            bool breakLongitude = false;
            while (1)
            {
                if (longitude > mBasicThresholdLongitude)
                {
                    longitude = mBasicThresholdLongitude;
                    breakLongitude = true;
                }
                float lateral = -mBasicThresholdLateral;
                bool breakLateral = false;
                while (1)
                {
                    if (lateral > mBasicThresholdLateral)
                    {
                        lateral = mBasicThresholdLateral;
                        breakLateral = true;
                    }
                    float x = tmpX + longitude * cosTheta - lateral * sinTheta;
                    float y = tmpY + longitude * sinTheta + lateral * cosTheta;
                    int gridX = (int)roundf((x - mZeroGridFromOriginOffsetX) / mGlobalMapRes);
                    int gridY = (int)roundf((y - mZeroGridFromOriginOffsetY) / mGlobalMapRes);
                    int rowIdx = mGlobalMapSizeY - 1 - gridY;
                    int colIdx = gridX;
                    if (rowIdx >= 0 && rowIdx < mGlobalMapSizeX && colIdx >= 0 && colIdx < mGlobalMapSizeY)
                    {
                        std::string tmpStr = std::to_string(gridX) + " " + std::to_string(gridY);
                        auto it = mapGridUnderPose.find(tmpStr);
                        if (it == mapGridUnderPose.end())
                        {
                            mapGridUnderPose.emplace(tmpStr, std::make_pair(gridX, gridY));
                        }
                    }
                    if (breakLateral)
                    {
                        break;
                    }
                    else
                    {
                        lateral = lateral + mGlobalMapRes;
                    }
                }
                if (breakLongitude)
                {
                    break;
                }
                else
                {
                    longitude = longitude + mGlobalMapRes;
                }
            }
        }

        // 查询路径栅格点是否有干涉
#ifdef DEBUG_NODE_VALID
        cv::Mat debugMat = mDebugMat.clone();
        std::string nodeStr = _node->getIndex();
        std::string timeStr = open_space_utils::Timer::getReadableTimestampUS();
        std::string writeStr = "/tmp/OpenSpacePlannerDebug/" + timeStr + "_" + nodeStr + ".bmp";
#endif
        // printf("mapGridUnderPose.size: %lu\n", mapGridUnderPose.size());
        for (auto &[key, value] : mapGridUnderPose)
        {
#ifdef DEBUG_NODE_VALID
            cv::circle(debugMat, cv::Point(value.first, g_planning_map.getSize().y - 1 - value.second),
                       0, cv::Scalar(238, 238, 175), CV_FILLED); // light blue
#endif
            if (g_planning_map.cell[value.first][value.second] > 200) // TODO
            {
#ifdef DEBUG_NODE_VALID
                printf("infeasible grid of node (%d, %d)\n", _node->getGridX(), _node->getGridY());
#endif
                return false;
            }
        }
#ifdef DEBUG_NODE_VALID
        for (size_t i = checkStartIndex; i < nodeStepSize; ++i)
        {
            if (travelledX[i] > mXyBounds[1] || travelledX[i] < mXyBounds[0] ||
                travelledY[i] > mXyBounds[3] || travelledY[i] < mXyBounds[2])
            {
                continue;
            }
            Pose tmp{travelledX[i], travelledY[i]};
            cv::circle(debugMat, cv::Point(g_planning_map.poseToGrid(tmp).x, g_planning_map.getSize().y - 1 - g_planning_map.poseToGrid(tmp).y), 0, cv::Scalar(0, 0, 0), CV_FILLED);
        }
        cv::imwrite(writeStr, debugMat);
#endif
        return true;
    }
}