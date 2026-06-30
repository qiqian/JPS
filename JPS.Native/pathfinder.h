/*
 * pathfinder.h
 * JPS Pathfinding — 公共句柄之一：jps_pathfinder（C 端对应 JPS.Core/Pathfinding/JpsPathfinder.cs）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_PATHFINDER_H
#define JPS_PATHFINDER_H

#include "jps_export.h"
#include "system.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 整数格坐标（内部搜索与路径重建用；亦供 smoother）。 */
typedef struct jps_point
{
    int x;
    int y;
} jps_point;

/*
 * JPS 寻路器（公共句柄）：持有按地图尺寸一次性分配、跨多次查询复用的逐节点搜索状态，
 * 以及最近一次寻路的结果（路径 + 展开节点数）
 * 它不拥有地图——每次寻路绑定到传入的 jps_system，因此多个 jps_pathfinder 可共享
 * 同一个 jps_system。每个 pathfinder 只持有自己的搜索状态，互不干扰。
 */
typedef struct jps_pathfinder jps_pathfinder;

JPS_API jps_pathfinder *JPS_CALL jps_pathfinder_create(void);
JPS_API void JPS_CALL jps_pathfinder_destroy(jps_pathfinder *pf);

// 在 system 上从 (sx,sy) 到 (gx,gy) 寻路（禁止斜穿角，整数代价 1000/1414，octile 启发）
// 结果（路径 + 展开节点数）暂存于 pf，供随后 copy/查询
// 调用前需确保 system 已 jps_system_sync（尤其阻挡改动之后）
// 返回：>=0 = 路径格数（含起终点；start==goal 时为 1）；负值见 jps_export.h 的 JPS_ERR_
JPS_API int JPS_CALL jps_pathfinder_find_path(jps_pathfinder *pf, jps_system *system,
                                              int sx, int sy, int gx, int gy);

/* 最近一次寻路的路径格数（未找到/未调用为 0）。 */
JPS_API int JPS_CALL jps_pathfinder_path_count(const jps_pathfinder *pf);

/* 最近一次搜索展开（出队展开）的节点数，用于诊断/性能统计。 */
JPS_API int JPS_CALL jps_pathfinder_expanded_nodes(const jps_pathfinder *pf);

// 把最近一次找到的路径拷进调用方缓冲。out_xy 按 x0,y0,x1,y1,... 交错存放，
// capacity_points 为可容纳点数（out_xy 至少 capacity_points*2 个 int）。
// 返回实际写入点数 = min(path_count, capacity_points)。
JPS_API int JPS_CALL jps_pathfinder_copy_path(const jps_pathfinder *pf, int *out_xy, int capacity_points);

// 对最近一次找到的整数路径做视线拉直平滑，输出连续格中心点（cx+0.5, cy+0.5）。
// 需提供寻路所用的 system 以取得地图。out_xy 按 x0,y0,... 交错的 float。
// 返回实际写入点数（≤ 原路径点数）。无路径时返回 0
JPS_API int JPS_CALL jps_pathfinder_copy_smoothed_path(const jps_pathfinder *pf, const jps_system *system,
                                                       float *out_xy, int capacity_points);

#ifdef __cplusplus
}
#endif

#endif /* JPS_PATHFINDER_H */
