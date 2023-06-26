/***********************************
 * File Name   : HybridAStarNode.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description : 
***********************************/

#ifndef OPEN_SPACE_PLANNER_HYBRID_A_STAR_NODE_H
#define OPEN_SPACE_PLANNER_HYBRID_A_STAR_NODE_H

#include <vector>
#include <string>
#include <memory>

namespace hybrid_a_star
{
    class HybridAStarNode
    {
    public:
        HybridAStarNode(const std::vector<float> &_travelledX,
                        const std::vector<float> &_travelledY,
                        const std::vector<float> &_travelledHeading,
                        const std::vector<float> &_xyBounds,
                        const float &_gridResolution,
                        const float &_headingResolution);
        // HybridAStarNode(float _x, float _y, float _heading,
        //                 const std::vector<float> &_xyBounds,
        //                 const float &_gridResolution,
        //                 const float &_headingResolution);

        float getFCost() const { return mGCost + mHCost; }
        float getGCost() const { return mGCost; }
        float getHCost() const { return mHCost; }
        int getGridX() const { return mXGrid; }
        int getGridY() const { return mYGrid; }
        int getGridHeading() const { return mHeadingGrid; }
        float getX() const { return mX; }
        float getY() const { return mY; }
        float getHeading() const { return mHeading; }
        // bool operator==(const Node3d &right) const;
        const std::string &getIndex() const { return mIndex; }
        size_t getTravelledStep() const { return mTravelledStep; }
        bool getDirection() const { return mDirection; }
        float getSteering() const { return mSteering; }
        std::shared_ptr<HybridAStarNode> getPreMode() const { return mPreNode; }
        const std::vector<float> &getTravelledX() const { return mTravelledX; }
        const std::vector<float> &getTravelledY() const { return mTravelledY; }
        const std::vector<float> &getTravelledHeading() const { return mTravelledHeading; }
        void setPreNode(std::shared_ptr<HybridAStarNode> _preNode) { mPreNode = _preNode; }
        void setDirection(bool _direction) { mDirection = _direction; }
        void setSteering(float _steering) { mSteering = _steering; }
        void setGCost(float _cost) { mGCost = _cost; }
        void setHCost(float _cost) { mHCost = _cost; }

    private:
        static std::string computeStringIndex(int _xGrid, int _yGrid, int _headingGrid);

        float mX = 0.0f;
        float mY = 0.0f;
        float mHeading = 0.0f;
        size_t mTravelledStep = 1;
        std::vector<float> mTravelledX;
        std::vector<float> mTravelledY;
        std::vector<float> mTravelledHeading;
        int mXGrid = 0;
        int mYGrid = 0;
        int mHeadingGrid = 0;
        std::string mIndex;
        float mGCost = -1.0f;
        float mHCost = -1.0f;
        float mFCost = -1.0f;
        std::shared_ptr<HybridAStarNode> mPreNode = nullptr;
        float mSteering = 0.0f;
        bool mDirection = true; // true-forward, false-backward
    };
}
#endif