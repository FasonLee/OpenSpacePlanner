/***********************************
 * File Name   : AStarNode.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description : 
***********************************/

#include "AStarNode.h"
#include "math.h"

namespace grid_a_star
{
    using namespace grid_a_star;

    AStarNode::AStarNode(const int &_x, const int &_y)
    {
        mX = _x;
        mY = _y;

        mIndex = computeStringIndex(mX, mY);
    }

    std::string AStarNode::computeStringIndex(const int &_x, const int &_y)
    {
        return std::to_string(_x) + " " + std::to_string(_y);
    }
}