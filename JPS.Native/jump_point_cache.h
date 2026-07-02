/*
 * jump_point_cache.h
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JumpPointCache.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_JUMP_POINT_CACHE_H
#define JPS_JUMP_POINT_CACHE_H

#include <stdbool.h>
#include <stdint.h>
#include "grid_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 单个格子的跳点缓存：4 方向带符号距离(int16) + 4 方向世代戳(uint8)。
 * 方向索引 0=E,1=W,2=S,3=N（即 jps_dir_index_of 的正交结果）。
 * dist >0 跳点距离，<=0 到墙距离；gen 等于当前有效世代即 clean。
 */
typedef struct jps_cell_jump
{
    int16_t dist[4];
    uint8_t gen[4];
} jps_cell_jump;

/*
 * Lazy cardinal jump-point cache. The C build always uses acquire/release
 * generation stamps, so multiple pathfinders may share one system/cache while
 * searching concurrently after a single-threaded sync.
 */
typedef struct jps_jump_point_cache
{
    int w;
    int h;
    int size;
    jps_cell_jump *cells;
    uint8_t *row_gen;
    uint8_t *col_gen;
    int *row_version;
    int *col_version;
    int map_version;
} jps_jump_point_cache;

jps_jump_point_cache *jps_jump_point_cache_create(void);
void jps_jump_point_cache_destroy(jps_jump_point_cache *c);

/* 每次搜索开始时调用：按尺寸准备缓冲，并在地图版本变化时同步受影响的行/列。 */
void jps_jump_point_cache_sync(jps_jump_point_cache *c, const jps_grid_map *m);

/*
 * 取 (x,y) 沿正交方向 (dx,dy) 的带符号跳点距离。
 * 命中 clean 直接读；未命中沿射线扫一次并把整段 run 一起洗白。
 */
int jps_jump_point_cache_cardinal_dist(jps_jump_point_cache *c, const jps_grid_map *m,
                                       int x, int y, int dx, int dy, int dir);

#ifdef __cplusplus
}
#endif

#endif /* JPS_JUMP_POINT_CACHE_H */
