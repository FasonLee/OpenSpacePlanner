/***********************************
 * File Name   : Rect.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-30
 * Description : 
***********************************/

#ifndef OPEN_SPACE_UTILS_GEOMETRY_RECT_H
#define OPEN_SPACE_UTILS_GEOMETRY_RECT_H

#include "Point.h"

namespace open_space_utils
{
    struct Rect
    {
        Rect();
        /**
          * @brief  构造函数
          * @param  min_pt x, y最小
          * @param  max_pt x, y最大
          * @retval None
          */        
        Rect(const Point &min_pt, const Point &max_pt);

        /**
          * @brief  构造函数
          * @param  逆时针四个点: pt1(x, y最小), pt2(x最大, y最小), pt3(x最大, y最大), pt4(x最小, y最大)
          * @retval None
          */
        Rect(const Point &pt1, const Point &pt2, const Point &pt3, const Point &pt4);

        bool inRange(const Point &point) const;
        std::string toString(bool orthogonal = false) const;
        const char* toCString(bool orthogonal = false) const;

        // friend bool operator==(const Rect &p1, const Rect &p2);

        Point pt[4]; // 逆时针四个点

      private:
        
        float getCross(const Point &p1, const Point &p2, const Point &p) const;
    };

    inline bool Rect::inRange(const Point &point) const
    {
        return (getCross(pt[0], pt[1], point)
                * getCross(pt[2], pt[3], point) >= 0)
            && (getCross(pt[1], pt[2], point)
                * getCross(pt[3], pt[0], point) >= 0);
    }

    inline std::string Rect::toString(bool orthogonal) const
    {
        std::string str = "";
        if (orthogonal)
        {
            str = "{ "
                + pt[0].toString()
                + ", "
                + pt[2].toString()
                + " }";
        }
        else
        {
            str = "{ "
                + pt[0].toString()
                + ", "
                + pt[1].toString()
                + ", "
                + pt[2].toString()
                + ", "
                + pt[3].toString()
                + " }";
        }

        return str;
    }

    inline const char* Rect::toCString(bool orthogonal) const
    {
        return toString().c_str();
    }
}

#endif // OPEN_SPACE_UTILS_GEOMETRY_RECT_H