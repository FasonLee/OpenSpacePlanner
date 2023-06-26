/***********************************
 * File Name   : Map.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-30
 * Description :
 ***********************************/

#ifndef OPEN_SPACE_PLANNER_MAP_H
#define OPEN_SPACE_PLANNER_MAP_H

#include "Grid.h"
#include "Point.h"
#include "Pose.h"
#include "Rect.h"
#include "GridRect.h"
#include "queue"
#include <opencv2/opencv.hpp>
#include "SharedRecursiveMutex.h"

using namespace open_space_utils;
using MapSize = Grid;

#define MAP_SIZE_X 400
#define MAP_SIZE_Y 400

namespace open_space_map
{
    class Map
    {
    public:
        Map();
        virtual ~Map() = default;

        void setMapInfo(float _resolution, const MapSize &_size,
                        const Point &_origin_pt, bool center)
        {
            resolution = _resolution;
            isOriginInCenter = center;
            setSize(_size);
            setOriginPt(_origin_pt);
        }

        // TODO set cell value
        bool loadMapFromFilePgm(std::string _file_name, uint8_t _channel = CV_8UC1);
        bool loadMapFromMat(const cv::Mat &_map, uint8_t _channel = CV_8UC1);

        MapSize getSize() const { return size; }
        float getResolution() const { return resolution; }
        Point getOriginPt() const { return originPt; }
        Rect getRange() const { return range; }
        Point getZeroGridOffset() const { return zeroGridFromOriginOffset; }
        auto getCell() const { return &cell; }

        inline bool inRange(const Point &pt) const
        {
            return range.inRange(pt);
        }

        inline bool inRange(const Grid &grid) const
        {
            return gridRange.inRange(grid);
        }

        inline Grid poseToGrid(const Pose &pose) const
        {
            Point diff = pose.pt - originPt;
            diff = diff / resolution;
            Grid grid_diff{
                static_cast<int32_t>(roundf(diff.x)),
                static_cast<int32_t>(roundf(diff.y))};

            Grid grid = originGrid + grid_diff;
            if (!inRange(grid))
            {
                std::string err_str = pose.toString() + " => " + grid.toString() + " is out of map range, origin_pt = " + originPt.toString() + "map range = %s" + gridRange.toString();
                printf("%s\n", err_str.c_str());
                // TRIG_SEGFAULT();
                throw std::out_of_range(err_str.c_str());
            }
            return grid;
        }

        inline Pose gridToPose(const Grid &grid) const
        {
            Grid grid_diff = grid - originGrid;
            Point diff{grid_diff.x * resolution,
                       grid_diff.y * resolution};
            return {originPt + diff, 0.0f};
        }

        // ^ Y
        // |
        // |
        // |
        // |
        // |
        // grid(0,0) --------------> X
        uint8_t cell[MAP_SIZE_X][MAP_SIZE_Y]; // TODO: set as private member

    private:
        void setSize(const MapSize &_size)
        {
            size = _size;
            Grid zero_grid{0, 0};

            if (isOriginInCenter)
            {
                // TODO: talk to fussion and fix this
                // if (size.x % 2 == 0)
                //     size.x += 1;
                // if (size.y % 2 == 0)
                //     size.y += 1;
                originGrid = size / 2;
            }
            else
            {
                originGrid = Grid(0, 0);
            }

            zeroGridFromOriginOffset.x = (float)(zero_grid.x - originGrid.x) * resolution;
            zeroGridFromOriginOffset.y = (float)(zero_grid.y - originGrid.y) * resolution;

            gridRange.min_grid = Grid{0, 0};
            gridRange.max_grid = size - Grid{1, 1};
        }

        void setOriginPt(const Point &pt)
        {
            originPt = pt;
            if (isOriginInCenter)
            {
                Point half_size_in_meter{
                    0.5f * size.x,
                    0.5f * size.y};
                half_size_in_meter = half_size_in_meter * resolution;
                range = Rect{
                    originPt - half_size_in_meter,
                    originPt + half_size_in_meter};
            }
            else
            {
                Point size_in_meter{
                    (float)size.x,
                    (float)size.y};
                size_in_meter = size_in_meter * resolution;
                Point half_res_in_meter{
                    0.5f * resolution,
                    0.5f * resolution};
                range = Rect{
                    originPt - half_res_in_meter,
                    originPt + size_in_meter - half_res_in_meter};
            }
        }

        mutable SharedRecursiveMutex mMapMtx;
        float resolution;
        MapSize size;
        Point originPt;
        Grid originGrid;
        Point zeroGridFromOriginOffset;
        Rect range;
        GridRect gridRange;
        bool isOriginInCenter;
    };
}

extern open_space_map::Map g_planning_map;

#endif // OPEN_SPACE_PLANNER_MAP_H