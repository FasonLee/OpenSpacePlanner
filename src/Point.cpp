/***********************************
 * File Name   : Point.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-29
 * Description : 
***********************************/

#include "Point.h"
#include "float.h"

namespace open_space_utils
{
    Point::Point() 
        : x(0.0), y(0.0)
    {}

    Point::Point(float _x, float _y)
        : x(_x), y(_y)
    {}

    Point::Point(const Point &p)
    {
        x = p.x;
        y = p.y;
    }

    Point::Point(const Grid &grid)
        : x(static_cast<float>(grid.x)), y(static_cast<float>(grid.y))
    {}

    // std::ostream &operator<<(std::ostream &out, const Point &pt)
    // {
    //     out << "{ " << pt.x << ", " << pt.y << " }";
    //     return out;
    // }

    bool operator==(const Point &p1, const Point &p2)
    {
        return (fabs(p1.x - p2.x) < FLT_EPSILON)
            && (fabs(p1.y - p2.y) < FLT_EPSILON);
    }

    bool operator!=(const Point &p1, const Point &p2)
    {
        return (fabs(p1.x - p2.x) >= FLT_EPSILON)
            || (fabs(p1.y - p2.y) >= FLT_EPSILON);
    }

    Point operator-(const Point &p1, const Point &p2)
    {
        return Point(p1.x - p2.x, p1.y - p2.y);
    }

    Point operator+(const Point &p1, const Point &p2)
    {
        return Point(p1.x + p2.x, p1.y + p2.y);
    }

    Point operator*(const Point &p1, float factor)
    {
        return Point(p1.x * factor, p1.y * factor);
    }

    Point operator/(const Point &p1, float factor)
    {
        return Point(p1.x / factor, p1.y / factor);
    }

    // bool operator<(const Point &p1, const Point &p2)
    // {
    //     if (p1.x < p2.x)
    //         return true;
        
    //     return (p1.y < p2.y);
    // }
}