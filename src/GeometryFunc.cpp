/***********************************
 * File Name   : GeometryFunc.cpp
 * Author      : Wenzhe.Zhang
 * Date        : 2023-05-31
 * Description : 
***********************************/
#include "GeometryFunc.h"

// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>

namespace open_space_utils
{
    float deg2rad(float deg)
    {
        return deg * M_PI / 180.0;
    }

    float rad2deg(float rad)
    {
        return rad * 180.0 / M_PI;
    }

    float getDistance(const Point &p1, const Point &p2)
    {
        return hypot(p1.x - p2.x, p1.y - p2.y);
    }

    float getDistance(const Grid &grid1, const Grid &grid2)
    {
        return hypot(grid1.x - grid2.x, grid1.y - grid2.y);
    }

    float getDistance(const Grid &grid1, const Grid &grid2, float resolution)
    {
        return hypot(grid1.x - grid2.x, grid1.y - grid2.y) * resolution;
    }

    float getAngleR(const Point &p1, const Point &p2)
    {
        return wrapToPi(atan2f(p2.y - p1.y, p2.x - p1.x));
    }

    float getAngleR(const Grid &grid1, const Grid &grid2)
    {
        return wrapToPi(atan2f(grid2.y - grid1.y, grid2.x - grid1.x));
    }

    // Point getPoint(const Point &pt, float distance, float angle_r)
    // {
    //     return Point(pt.x + distance * cosf(angle_r),
    //         pt.y + distance * sinf(angle_r));
    // }

    float wrapTo2Pi(float angle_r)
    {
        if (angle_r >= 0.0 && angle_r < M_2PI)
            return angle_r;

        int cnt = (int)(angle_r / M_2PI);

        angle_r -= cnt * M_2PI;
        if (angle_r < 0)
            angle_r += M_2PI;
        return angle_r; 
    }

    float wrapToPi(float angle_r)
    {
        angle_r = wrapTo2Pi(angle_r);
        if (angle_r >= M_PI)
            angle_r -= M_2PI;
        return angle_r;
    }

    // bool withinAngleRangeR(float angle_r, float range_min_r, float range_max_r)
    // {
    //     angle_r = wrapTo2Pi(angle_r);
    //     range_min_r = wrapTo2Pi(range_min_r);
    //     range_max_r = wrapTo2Pi(range_max_r);

    //     if (range_max_r > range_min_r)
    //     {
    //         if (angle_r <= range_max_r && angle_r >= range_min_r)
    //             return true;
    //         else
    //             return false;
    //     }
    //     else if (range_max_r == range_min_r)
    //     {
    //         return false;
    //     }
    //     else
    //     {
    //         if (angle_r >= range_min_r || angle_r <= range_max_r)
    //             return true;
    //         else
    //             return false;
    //     }
    // }

    /* angle between angle2 and angle1 (angle2 - angle1) */
    // float angleBetweenR(float angle1_r, float angle2_r)
    // {
    //     float angle_diff_r = wrapTo2Pi(angle2_r - angle1_r);
    //     return angle_diff_r >= M_PI ? angle_diff_r - M_2PI : angle_diff_r;
    // }

    bool getGridRectIntersection(const GridRect &rect1, const GridRect &rect2,
        GridRect &intersect_rect)
    {
        Grid min_grid{
            std::max(rect1.min_grid.x, rect2.min_grid.x),
            std::max(rect1.min_grid.y, rect2.min_grid.y)
        };
        Grid max_grid{
            std::min(rect1.max_grid.x, rect2.max_grid.x),
            std::min(rect1.max_grid.y, rect2.max_grid.y)
        };

        if (min_grid.x > max_grid.x || min_grid.y > max_grid.y)
            return false;
        
        intersect_rect = GridRect{min_grid, max_grid};
        return true;
    }

    Pose transformFrame(const Pose &transform, const Pose &origin_pose)
    {
        Pose result;
        float costheta = cosf(transform.heading);
        float sintheta = sinf(transform.heading);
        result.pt.x = transform.pt.x + origin_pose.pt.x * costheta
            - origin_pose.pt.y * sintheta;
        result.pt.y = transform.pt.y + origin_pose.pt.x * sintheta
            + origin_pose.pt.y * costheta;
        result.heading = wrapToPi(transform.heading + origin_pose.heading);
        return result;
    }

    Pose reverseTransformFrame(const Pose &transform, const Pose &target_pose)
    {
        Pose result;
        float tmp_x = target_pose.pt.x - transform.pt.x;
        float tmp_y = target_pose.pt.y - transform.pt.y;
        result.pt.x = tmp_x * cosf(transform.heading) + tmp_y * sinf(transform.heading);
        result.pt.y = -tmp_x * sinf(transform.heading) + tmp_y * cosf(transform.heading);
        result.heading = wrapToPi(target_pose.heading - transform.heading);
        return result;
    }

    Pose getTransform(const Pose &pose_in_origin_frame,
        const Pose &pose_in_target_frame)
    {
        Pose origin = pose_in_origin_frame;
        Pose target = pose_in_target_frame;
        Pose transform;
        transform.heading = target.heading - origin.heading;
        transform.pt.x = target.pt.x - (origin.pt.x * cosf(transform.heading)
            - origin.pt.y * sinf(transform.heading));
        transform.pt.y = target.pt.y - (origin.pt.x * sinf(transform.heading)
            + origin.pt.y * cosf(transform.heading));
        return transform;
    }

    Pose reverseTransform(const Pose &transform)
    {
        Pose p1(0.0f, 0.0f, 0.0f);
        Pose p2 = transformFrame(transform, p1);
        Pose reverse_transform = getTransform(p2, p1);
        return reverse_transform;
    }
}
