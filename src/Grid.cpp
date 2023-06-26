/***********************************
 * File Name   : Grid.c
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description : 
***********************************/

#include "Grid.h"

namespace open_space_utils
{
    Grid::Grid()
        : x(0), y(0)
    {}

    Grid::Grid(int32_t _x, int32_t _y)
        : x(_x), y(_y)
    {}

    // std::ostream &operator<<(std::ostream &out, const Grid &pt)
    // {
    //     out << "{ " << pt.x << ", " << pt.y << " }";
    //     return out;
    // }

    bool operator==(const Grid &grid1, const Grid &grid2)
    {
        return (grid1.x == grid2.x) && (grid1.y == grid2.y);
    }

    bool operator!=(const Grid &grid1, const Grid &grid2)
    {
        return (grid1.x != grid2.x) || (grid1.y != grid2.y);
    }

    Grid operator-(const Grid &grid1, const Grid &grid2)
    {
        return Grid(grid1.x - grid2.x, grid1.y - grid2.y);
    }

    Grid operator+(const Grid &grid1, const Grid &grid2)
    {
        return Grid(grid1.x + grid2.x, grid1.y + grid2.y);
    }

    Grid operator*(const Grid &grid, float factor)
    {
        return Grid(grid.x * factor, grid.y * factor);
    }

    Grid operator/(const Grid &grid, float factor)
    {
        return Grid(grid.x / factor, grid.y / factor);
    }

    // bool operator<(const Grid &g1, const Grid &g2)
    // {
    //     if (g1.x < g2.x)
    //         return true;
    //     else if (g1.x > g2.x)
    //         return false;
    //     else
    //         return g1.y < g2.y;
    // }
}

