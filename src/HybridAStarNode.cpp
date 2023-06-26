/***********************************
 * File Name   : HybridAStarNode.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description : 
***********************************/

#include "HybridAStarNode.h"
#include "math.h"

namespace hybrid_a_star
{
    using namespace hybrid_a_star;

    HybridAStarNode::HybridAStarNode(const std::vector<float> &_travelledX,
                                     const std::vector<float> &_travelledY,
                                     const std::vector<float> &_travelledHeading,
                                     const std::vector<float> &_xyBounds,
                                     const float &_gridResolution,
                                     const float &_headingResolution)
    {
        mX = _travelledX.back();
        mY = _travelledY.back();
        mHeading = _travelledHeading.back();

        // XYbounds in xmin, xmax, ymin, ymax
        mXGrid = static_cast<int>((mX - _xyBounds[0]) / _gridResolution);
        mYGrid = static_cast<int>((mY - _xyBounds[2]) / _gridResolution);
        mHeadingGrid = static_cast<int>((mHeading - (-M_PI)) / _headingResolution);

        mTravelledX = _travelledX;
        mTravelledY = _travelledY;
        mTravelledHeading = _travelledHeading;

        mIndex = computeStringIndex(mXGrid, mYGrid, mHeadingGrid);
        mTravelledStep = mTravelledX.size();
    }

    // HybridAStarNode::HybridAStarNode(float _x, float _y, float _heading,
    //                                  const std::vector<float> &_xyBounds,
    //                                  const float &_gridResolution,
    //                                  const float &_headingResolution)
    // {
    //     mX = _x;
    //     mY = _y;
    //     mHeading = _heading;

    //     mXGrid = static_cast<int>((mX - _xyBounds[0]) / _gridResolution);
    //     mYGrid = static_cast<int>((mY - _xyBounds[2]) / _gridResolution);
    //     mHeadingGrid = static_cast<int>((mHeading - (-M_PI)) / _headingResolution);

    //     mTravelledX.push_back(_x);
    //     mTravelledY.push_back(_y);
    //     mTravelledHeading.push_back(_heading);

    //     mIndex = computeStringIndex(mXGrid, mYGrid, mHeadingGrid);
    // }

    std::string HybridAStarNode::computeStringIndex(int _xGrid, int _yGrid, int _headingGrid)
    {
        return std::to_string(_xGrid) + " " + std::to_string(_yGrid) + " " + std::to_string(_headingGrid);
    }
}