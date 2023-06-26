/***********************************
 * File Name   : Map.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/

#include "Map.h"

using namespace open_space_utils;

open_space_map::Map g_planning_map;

namespace open_space_map
{
    Map::Map()
    {
    }

    bool Map::loadMapFromFilePgm(std::string _file_name, uint8_t _channel)
    {
        std::unique_lock<SharedRecursiveMutex> map_lock(mMapMtx);
        cv::Mat map = cv::imread(_file_name, 0);
        // cv::Mat srcCopy = map.clone();
        // img from fussion
        // -----------> X
        // |
        // |
        // |
        // |
        // |
        // Y
        // cv::flip(srcCopy, srcCopy, 0); // 参数1输入,参数2输出,0表示上下翻转
        // return loadMapFromMat(srcCopy, _channel);
        std::string writeStr = "/tmp/OpenSpacePlannerDebug/loaded_map.bmp";
        cv::imwrite(writeStr, map);
        return loadMapFromMat(map, _channel);
    }

    bool Map::loadMapFromMat(const cv::Mat &_map,
                             uint8_t _channel)
    {
        std::unique_lock<SharedRecursiveMutex> map_lock(mMapMtx);
        if (_map.type() != _channel)
        {
            printf("_channel is not match the input mat\n");
            return false;
        }
        if (_channel != CV_8UC1 && _channel != CV_8UC3)
        {
            printf("_channel not supported\n");
            return false;
        }
        if (_map.rows != size.y || _map.cols != size.x)
        {
            printf("map size does not match: rows: %d, y: %d, cols: %d, x: %d\n",
                   _map.rows, size.y, _map.cols, size.x);
            return false;
        }

        int32_t channel_cnt = 1;
        if (_channel == CV_8UC3)
        {
            channel_cnt = 3;
        }
        const uint8_t *row;

        // printf("rows: %d, cols: %d\n", _map.rows, _map.cols);
        for (int32_t i = 0; i < _map.rows; i++)
        {
            row = _map.ptr<uint8_t>(i); // i行的首位地址
            for (int32_t j = 0; j < _map.cols; j++)
            {
                const uint8_t *ptr = row + j * channel_cnt;
                // printf("[%d][%d]: %u\n", i, j, *ptr);
                cell[j][size.y - i] = *ptr; // trans to planning coordinate
            }
        }
        return true;
    }
}