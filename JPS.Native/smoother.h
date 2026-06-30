/*
 * smoother.h
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/PathSmoother.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_SMOOTHER_H
#define JPS_SMOOTHER_H

#include <stdbool.h>
#include "grid_map.h"
#include "pathfinder.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 连续格坐标点（格中心 = cx+0.5），仅用于显示，不参与整数寻路。 */
typedef struct jps_point_f
{
    float x;
    float y;
} jps_point_f;

/*
 * 超覆盖(supercover)直线视线检测：整数增量遍历线段经过的每一格，全部可走才算通视。
 * 对角穿越是否检查两侧由 JPS_ALLOW_CORNER_CUTTING 控制，与寻路移动规则保持一致。
 */
bool jps_line_of_sight(const jps_grid_map *m, int x0, int y0, int x1, int y1);

/*
 * 前向增量视线拉直（forward-incremental string pulling）路径平滑。
 * 输入整数格路径，输出连续格中心点。返回点数；*out_points 指向 malloc 的数组，
 * 调用方用 free() 释放（path_count==0 时 *out_points 为 NULL、返回 0）。
 */
int jps_smooth_path(const jps_grid_map *m, const jps_point *path, int path_count,
                    jps_point_f **out_points);

/* 将平滑结果写入调用方提供的缓冲 out_points（容量 capacity_points）。
 * 返回如果无限容量时产生的点数（即平滑后的实际点数）；实际写入不超过 capacity_points。
 */
int jps_smooth_path_into(const jps_grid_map *m, const jps_point *path, int path_count,
                         jps_point_f *out_points, int capacity_points);

#ifdef __cplusplus
}
#endif

#endif /* JPS_SMOOTHER_H */
