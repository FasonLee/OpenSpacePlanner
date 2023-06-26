/***********************************
 * File Name   : AStarNode.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description : 
***********************************/

#ifndef OPEN_SPACE_PLANNER_A_STAR_NODE_H
#define OPEN_SPACE_PLANNER_A_STAR_NODE_H

#include <vector>
#include <string>
#include <memory>

namespace grid_a_star
{
    class AStarNode
    {
    public:
        AStarNode(const int &_x, const int &_y);

        float getFCost() const { return mGCost + mHCost; }
        float getGCost() const { return mGCost; }
        // float GetHeuCost() const { return mHCost; }
        int getX() const { return mX; }
        int getY() const { return mY; }
        // bool operator==(const Node3d &right) const;
        const std::string &getIndex() const { return mIndex; }
        std::shared_ptr<AStarNode> getPreMode() const { return mPreNode; }
        void setPreNode(std::shared_ptr<AStarNode> _preNode) { mPreNode = _preNode; }
        void setGCost(float _cost) { mGCost = _cost; }
        void setHCost(float _cost) { mHCost = _cost; }


    private:

        static std::string computeStringIndex(const int &_x, const int &_y);

        int mX = 0;
        int mY = 0;
        std::string mIndex;
        float mGCost = -1.0f;
        float mHCost = -1.0f;
        float mFCost = -1.0f;
        std::shared_ptr<AStarNode> mPreNode = nullptr;
    };
}
#endif