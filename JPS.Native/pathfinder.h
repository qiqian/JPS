/*
 * pathfinder.h
 * JPS Pathfinding — 公共句柄之一：jps_pathfinder（C 端对应 JPS.Core/Pathfinding/JpsPathfinder.cs）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_PATHFINDER_H
#define JPS_PATHFINDER_H

#include <stdint.h>
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

/* 当前 pathfinder 保留的 native 内存字节数估算：本体 + 搜索/堆/结果/平滑缓冲。NULL 返回 0。 */
JPS_API uint64_t JPS_CALL jps_pathfinder_memory_bytes(const jps_pathfinder *pf);

// 在 system 上从 (sx,sy) 到 (gx,gy) 寻路（禁止斜穿角，整数代价 1000/1414，octile 启发）
// 结果（compact path + smoothed path + 展开节点数）暂存于 pf，供随后 copy/查询
// 调用前需确保 system 已 jps_system_sync（尤其阻挡改动之后）
// 返回：>=0 = compact path 点数（含起终点；start==goal 时为 1）；成功时已同步完成路径平滑。负值见 jps_export.h 的 JPS_ERR_
JPS_API int JPS_CALL jps_pathfinder_find_path(jps_pathfinder *pf, jps_system *system,
                                              int sx, int sy, int gx, int gy);

/* 最近一次寻路的 compact path 点数（起点 + 跳点/拐点 + 终点；未找到/未调用为 0）。配 jps_pathfinder_copy_path。 */
JPS_API int JPS_CALL jps_pathfinder_path_count(const jps_pathfinder *pf);

/*
 * 最近一次寻路的**平滑**路径点数（视线拉直后的折线；未找到/未调用为 0）。配 jps_pathfinder_copy_smoothed_path。
 * 平滑在 find_path 成功时已计算并缓存；本函数只返回缓存点数。pf 非 const 仅用于兼容旧声明。
 */
JPS_API int JPS_CALL jps_pathfinder_smoothed_path_count(jps_pathfinder *pf);

// 把最近一次找到的 compact path 拷进调用方缓冲。JPS.Native 不暴露 expanded per-cell path。
// out_xy 按 x0,y0,x1,y1,... 交错存放，
// capacity_points 为可容纳点数（out_xy 至少 capacity_points*2 个 int）。
// 返回实际写入点数 = min(path_count, capacity_points)。
JPS_API int JPS_CALL jps_pathfinder_copy_path(const jps_pathfinder *pf, int *out_xy, int capacity_points);

// 把最近一次寻路的**平滑路径**拷进调用方缓冲：视线拉直后的连续格中心点（cx+0.5, cy+0.5），
// out_xy 按 x0,y0,x1,y1,... 交错的 float（至少 capacity_points*2 个）。
//
// 平滑在 find_path 成功后**自动完成并缓存**，
// 这里只是拷已算好的结果——**无二次计算、不需要 system**。返回实际写入点数 = min(smoothed_path_count, capacity_points)。
// 典型用法：n = jps_pathfinder_smoothed_path_count(pf); 分配 n*2 float; jps_pathfinder_copy_smoothed_path(pf, buf, n)。
// 无路径时返回 0；因内部会填充缓存，pf 非 const。
JPS_API int JPS_CALL jps_pathfinder_copy_smoothed_path(jps_pathfinder *pf, float *out_xy, int capacity_points);

#ifdef __cplusplus
}
#endif

#endif /* JPS_PATHFINDER_H */
