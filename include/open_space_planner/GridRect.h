/***********************************
 * File Name   : GridRect.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-30
 * Description : 
***********************************/

#ifndef OPEN_SPACE_UTILS_GEOMETRY_GRID_RECT_H
#define OPEN_SPACE_UTILS_GEOMETRY_GRID_RECT_H

#include "Grid.h"

namespace open_space_utils
{
    struct GridRect
    {
        Grid min_grid;
        Grid max_grid;

        GridRect();
        GridRect(const Grid &_min_grid, const Grid &_max_grid);
        
        std::string toString() const;
        const char* toCString() const;
        bool inRange(const Grid &grid) const;
        bool inRange(const GridRect &inner_rect) const;
        Grid getSize() const;
        // Grid getCenter() const;
        friend bool operator==(const GridRect &r1, const GridRect &r2);
    };

    inline std::string GridRect::toString() const
    {
        std::string str = "";
        str = "{ "
            + min_grid.toString()
            + ", "
            + max_grid.toString()
            + " }";
        return str;
    }

    inline const char* GridRect::toCString() const
    {
        return toString().c_str();
    }

    inline bool GridRect::inRange(const Grid &grid) const
    {
        return (grid.x >= min_grid.x && grid.x <= max_grid.x
            && grid.y >= min_grid.y && grid.y <= max_grid.y);
    }

    // inline Grid GridRect::getCenter() const
    // {
    //     return (max_grid + min_grid) * 0.5f;
    // }

    inline bool GridRect::inRange(const GridRect &inner_rect) const
    {
        return (inRange(inner_rect.min_grid) && inRange(inner_rect.max_grid));
    }

    inline Grid GridRect::getSize() const
    {
        return max_grid - min_grid + Grid{1, 1};
    }

}

#endif // OPEN_SPACE_UTILS_GEOMETRY_GRID_RECT_H