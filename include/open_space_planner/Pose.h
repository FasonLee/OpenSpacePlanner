/***********************************
 * File Name   : Pose.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description : 
***********************************/

#ifndef OPEN_SPACE_UTILS_GEOMETRY_POSE_H
#define OPEN_SPACE_UTILS_GEOMETRY_POSE_H

#include "Point.h"

namespace open_space_utils
{
    struct Grid;
    struct Point;
    struct Pose
    {
        Point pt;
        float heading;

        Pose();
        Pose(const Point &pt);
        Pose(const Point &pt, float heading);
        Pose(const Grid &grid);
        Pose(const Grid &grid, float heading);
        Pose(float x, float y);
        Pose(float x, float y, float heading);
        Pose(const Pose &p);

        std::string toString() const;
        const char* toCString() const;

        // friend std::ostream &operator<<(std::ostream &out, const Pose &pose);
        friend bool operator==(const Pose &p1, const Pose &p2);
        friend bool operator!=(const Pose &p1, const Pose &p2);
    };

    inline std::string Pose::toString() const
    {
        std::string str = "";
        str = "{ "
            + std::to_string(pt.x)
            + ", "
            + std::to_string(pt.y)
            + ", "
            + std::to_string(heading * 180.0f / M_PI)
            + " }";
        return str;
    }

    inline const char* Pose::toCString() const
    {
        return toString().c_str();
    }
}

#endif // OPEN_SPACE_UTILS_GEOMETRY_POSE_H
