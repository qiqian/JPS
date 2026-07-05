/*
 * system.h
 * JPS Pathfinding — 公共句柄之一：jps_system（C 端对应 JPS.Core/Pathfinding/JpsSystem.cs）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_SYSTEM_H
#define JPS_SYSTEM_H

#include <stdint.h>
#include "grid_map.h"
#include "jump_point_cache.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * JPS 运行环境（公共句柄）：拥有一张地图与其对应的惰性跳点缓存。
 *
 * 把地图与缓存集中在这里，是为了让**多个 jps_pathfinder 共享同一个 jps_system**：
 * 每个 jps_pathfinder 只持有自己的逐节点搜索状态，地图/缓存都在 jps_system。
 *
 * 典型用法：
 *   s  = jps_system_create(w, h);
 *   jps_system_set_blocked(s, x, y, 1); ...   // 设置阻挡
 *   jps_system_sync(s);                        // 阻挡改动后同步缓存
 *   pf = jps_pathfinder_create();
 *   jps_pathfinder_find_path(pf, s, sx, sy, gx, gy);   // 可创建多个 pf 共用同一个 s
 *
 * 结构体定义对内部 .c 可见（pathfinder 需访问 map/cache）；对 C# 而言它是不透明指针。
 */
typedef struct jps_system
{
    jps_grid_map *map;             /* 拥有，destroy 时释放 */
    jps_jump_point_cache *cache;   /* 拥有，destroy 时释放 */
} jps_system;

 /* 整数格坐标（内部搜索与路径重建用；亦供 smoother）。非公共 ABI。 */
typedef struct jps_point
{
    int x;
    int y;
} jps_point;

#ifdef __cplusplus
}
#endif

#endif /* JPS_SYSTEM_H */
