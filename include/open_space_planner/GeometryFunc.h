/***********************************
 * File Name   : GeometryFunc.h
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description :
 ***********************************/

#ifndef OPEN_SPACE_PLANNER_GEOMETRY_FUNC_H
#define OPEN_SPACE_PLANNER_GEOMETRY_FUNC_H

// #include <cmath>
// #include <vector>
// #include <unordered_set>
// #include <unordered_map>
// #include <memory>
// #include <opencv2/core.hpp>

#include "Point.h"
#include "Pose.h"
#include "Grid.h"
#include "Rect.h"
#include "GridRect.h"

namespace open_space_utils
{
#define M_2PI (M_PI * 2.0)
    /**
     * @brief  角度转弧度
     * @param  deg     角度
     * @retval float   弧度
     */
    float deg2rad(float deg);

    /**
     * @brief  弧度转角度
     * @param  rad     弧度
     * @retval deg     角度
     */
    float rad2deg(float rad);

    /**
     * @brief  获取两point之间距离
     * @param  p1      第一个point
     *         p2      第二个point
     * @retval float   距离
     */
    float getDistance(const Point &p1, const Point &p2);

    /**
     * @brief  获取两grid之间距离
     * @param  grid1   第一个grid
     *         grid2   第二个grid
     * @retval float   距离
     */
    float getDistance(const Grid &grid1, const Grid &grid2);

    /**
     * @brief  获取两个grid之间代表的实际距离
     * @param  grid1       第一个grid
     *         grid2       第二个grid
     *         resolution  栅格点的分辨率
     * @retval float       距离
     */
    float getDistance(const Grid &grid1, const Grid &grid2, float resolution);

    /**
     * @brief  获取从点1到点2的角度
     * @param  pt1       第一个点
     *         pt2       第二个点
     * @retval float     角度(弧度)
     */
    float getAngleR(const Point &p1, const Point &p2);
    float getAngleR(const Grid &grid1, const Grid &grid2);

    // Point getPoint(const Point &pt, float distance, float angle_r);

    /**
     * @brief  将角度归一化到[0, 2*pi)
     * @param  angle_r     角度(弧度)
     * @retval float       归一化后的角度
     */
    float wrapTo2Pi(float angle_r);

    /**
     * @brief  将角度归一化到[-pi, pi)
     * @param  angle_r     角度(弧度)
     * @retval float       归一化后的角度
     */
    float wrapToPi(float angle_r);

    /**
     * @brief  判断角度是否在指定范围内
     * @param  angle_r     角度(弧度)
     * @param  range_min_r 起始角度范围(弧度，[-pi, pi))
     * @param  range_max_r 终止角度范围(弧度, [-pi, pi))
     * @retval bool        是否在角度范围内
     */
    // bool withinAngleRangeR(float angle_r, float range_min_r, float range_max_r);

    /**
     * @brief  angle between angle2 and angle1 (angle2 - angle1)
     * @param  angle1_r    角度1(弧度)
     * @param  angle2_r    角度2(弧度)
     * @retval float       归一化后的角度差[-pi, pi)
     */
    // float angleBetweenR(float angle1_r, float angle2_r);

    /**
     * @brief  get intersection of two rectangles
     * @param  rect1    Rectangle 1
     * @param  rect2    Rectangle 2
     * @param  intersect_rect    result rectangle
     * @retval bool     whether two rectangles overlap (false = no overlap)
     */
    bool getGridRectIntersection(const GridRect &rect1, const GridRect &rect2,
                                 GridRect &intersect_rect);

    /**
     * @brief  transform represent the transform from the original frame to target frame
     * @param  transform      transform pose (x, y, yaw)
     * @param  origin_pose    original pose (x, y, yaw)
     * @retval Pose     result pose
     */
    Pose transformFrame(const Pose &transform, const Pose &origin_pose);

    /**
     * @brief  reverse operation for transformFrame
     * @param  transform      transform pose (x, y, yaw)
     * @param  target_pose    target pose (x, y, yaw)
     * @retval Pose     result pose (original pose in transformFrame)
     */
    Pose reverseTransformFrame(const Pose &transform, const Pose &target_pose);

    /**
     * @brief  get the transformation of the frames (origin to target)
     * @param  pose_in_origin_frame
     * @param  pose_in_target_frame
     * @retval Pose     required transform
     */
    Pose getTransform(const Pose &pose_in_origin_frame,
                      const Pose &pose_in_target_frame);
    Pose reverseTransform(const Pose &transform);

    // inline Grid matPtToGrid(const cv::Point &pt, const GridRect &rect)
    // {
    //     return rect.max_grid - Grid{pt.y, pt.x};
    // }

    // inline cv::Point gridToMatPt(const Grid &grid, const GridRect &rect)
    // {
    //     cv::Point pt;
    //     pt.y = rect.max_grid.x - grid.x;
    //     pt.x = rect.max_grid.y - grid.y;
    //     return pt;
    // }
}

#endif // OPEN_SPACE_PLANNER_GEOMETRY_FUNC_H
