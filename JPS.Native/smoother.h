/*
 * smoother.h
 * JPS Pathfinding — internal C port of JPS.Core/Pathfinding/PathSmoother.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_SMOOTHER_H
#define JPS_SMOOTHER_H

#include <stdbool.h>
#include "grid_map.h"
#include "pathfinder.h"

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
bool jps__line_of_sight(const jps_grid_map *m, int x0, int y0, int x1, int y1);

/*
 * 将平滑结果写入调用方提供的缓冲 out_points（容量 capacity_points）。
 * 返回如果无限容量时产生的点数（即平滑后的实际点数）；实际写入不超过 capacity_points。
 */
int jps__smooth_path_into(const jps_grid_map *m, const jps_point *path, int path_count,
                          jps_point_f *out_points, int capacity_points);

#endif /* JPS_SMOOTHER_H */
