/***********************************
 * File Name   : Pose.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description : 
***********************************/

#include "Pose.h"
#include "float.h"

namespace open_space_utils
{
    Pose::Pose()
        : pt(), heading(0.0f)
    {}

    Pose::Pose(const Point &_pt)
        : pt(_pt), heading(0.0f)
    {}
    
    Pose::Pose(const Point &_pt, float _theta)
        : pt(_pt), heading(_theta) 
    {}

    Pose::Pose(const Grid &grid)
        : pt(grid), heading(0.0f)
    {}

    Pose::Pose(const Grid &grid, float _theta)
        : pt(grid), heading(_theta)
    {}

    Pose::Pose(float x_, float y_)
        : pt(x_, y_), heading(0.0f)
    {}

    Pose::Pose(float x_, float y_, float _theta)
        : pt(x_, y_), heading(_theta)
    {}

    Pose::Pose(const Pose &p)
        : pt(p.pt), heading(p.heading)
    {}

    // std::ostream &operator<<(std::ostream &out, const Pose &pose)
    // {
    //     out << "{ " << pose.pt.x << ", " << pose.pt.y
    //         << ", " << pose.heading << " }";
    //     return out;
    // }

    bool operator==(const Pose &p1, const Pose &p2)
    {
        return p1.pt == p2.pt && (fabs(p1.heading - p2.heading) < FLT_EPSILON);
    }

    bool operator!=(const Pose &p1, const Pose &p2)
    {
        return !(p1 == p2);
    }
}