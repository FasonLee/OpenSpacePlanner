/***********************************
 * File Name   : Grid.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-26
 * Description : 
***********************************/

#ifndef OPEN_SPACE_UTILS_GEOMETRY_GRID_H
#define OPEN_SPACE_UTILS_GEOMETRY_GRID_H

#include <cinttypes>
#include <ostream>
#include <cmath>
#include <string>

namespace open_space_utils
{
    struct Grid
    {
        int32_t x;
        int32_t y;

        Grid();
        Grid(int32_t x, int32_t y);

        std::string toString() const;
        const char* toCString() const;

        // friend std::ostream &operator<<(std::ostream &out, const Grid &pt); // forbidden due to coding style
        friend bool operator==(const Grid &grid1, const Grid &grid2);
        friend bool operator!=(const Grid &grid1, const Grid &grid2);
        friend Grid operator-(const Grid &grid1, const Grid &grid2);
        friend Grid operator+(const Grid &grid1, const Grid &grid2);
        friend Grid operator*(const Grid &grid, float factor); // forbidden due to coding style
        friend Grid operator/(const Grid &grid, float factor); // forbidden due to coding style
        // friend bool operator<(const Grid &g1, const Grid &g2);
    };

    inline std::string Grid::toString() const
    {
        std::string str = "";
        str = "{ "
            + std::to_string(x)
            + ", "
            + std::to_string(y)
            + " }";
        return str;
    }

    inline const char* Grid::toCString() const
    {
        return toString().c_str();
    }
}

#endif // OPEN_SPACE_UTILS_GEOMETRY_GRID_H