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
 * Lazy cardinal jump-point cache（按方向 SoA：dist 与 gen 各拆成 4 个连续平面）。
 * 方向索引 0=E,1=W,2=S,3=N（即 jps_dir_index_of 的正交结果）。
 *
 * 布局：dist / gen 各长 4*size，方向主序——方向 dir 的第 idx 格 = dist[(size_t)dir*size + idx]、
 * gen[(size_t)dir*size + idx]。总字节数与旧 AoS(int16 dist[4]+uint8 gen[4]=12B/格)完全相同，
 * 但同一方向沿行(E/W)或沿列(S/N，转置后)相邻的格在平面内连续，故回写整段 run 时 dist 是一段连续 int16、
 * 可用 16 位车道 SIMD 一次生成并写 8 个（gen 是同值 line_gen 的连续段）。
 * dist >0 跳点距离，<=0 到墙距离；gen 等于当前行/列有效世代即 clean。
 *
 * 并发模型不变：写者先普通写 dist、再 release-store gen 发布；读者 acquire-load gen，命中再普通读 dist。
 * 单线程 Sync 后多个 jps_pathfinder 可共享同一缓存并行只读/惰性补写（补写值是固定地图的纯函数）。
 */
typedef struct jps_jump_point_cache
{
    int w;
    int h;
    int size;
    int16_t *dist;        /* 4 个方向平面拼接：dist[dir*size + idx] */
    uint8_t *gen;         /* 4 个方向平面拼接：gen[dir*size + idx] */
    uint8_t *row_gen;     /* 每行 E/W 有效世代（dir 0/1） */
    uint8_t *col_gen;     /* 每列 S/N 有效世代（dir 2/3） */
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
