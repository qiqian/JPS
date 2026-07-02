/*
 * grid_map.h
 * JPS Pathfinding — C port of JPS.Core/Models/GridMap.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_GRID_MAP_H
#define JPS_GRID_MAP_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 纯网格模型：只承载“地图本身”（尺寸、阻挡、版本号）。
 *
 * 存储：阻挡用位压缩，1 bit/格。**按行对齐到 64 位字**——每行独占 stride 个 uint64，
 * 行首恒落在字边界（行尾不足 64 格的部分为 padding，逻辑上视为阻挡）。
 * 这样同一行连续 64 格落在同一个字内、不跨行，水平方向的跳点扫描可一次按字（64 格）批处理。
 *
 * 与 C# GridMap.cs 严格一致：版本号在任何阻挡增删时自增，供惰性跳点缓存判失效。
 */
typedef struct jps_grid_map
{
    int width;
    int height;
    /* 阻挡布局的版本号：任何阻挡增删都自增。寻路器据此判断惰性跳点缓存是否失效。 */
    int version;
    /*
     * 行/列影响版本：单格变化只会影响 y-1..y+1 的水平跳点扫描，以及 x-1..x+1 的垂直跳点扫描。
     * JumpPointCache.Sync 据此把失效粒度从整表降到相关行/列。
     */
    int *row_version;
    int *col_version;
    /* 每行的 uint64 数，向上取整到偶数个 word，使行起点保持 128-bit 对齐。 */
    int stride;
    /* 每列的 uint64 数（列排布/转置位图用），同样偶数、128-bit 对齐。 */
    int col_stride;
    /*
     * 行排布位图（按行对齐）：第 y 行第 (x>>6) 个字 = blocked[y*stride + (x>>6)]，位 = x&63。
     * 供水平跳点扫描（同一行 128 格落在一个对齐单元内）。16 字节对齐分配。
     */
    uint64_t *blocked;
    /*
     * 列排布位图（转置，按列对齐）：第 x 列第 (y>>6) 个字 = col_blocked[x*col_stride + (y>>6)]，位 = y&63。
     * 与 blocked 内容等价、随之同步更新；供**垂直**跳点扫描把 128 格 y 落在一个对齐单元内，复用同一套 SIMD。
     */
    uint64_t *col_blocked;
} jps_grid_map;

/*
 * 创建地图。尺寸必须为正，且边长不超过 INT16_MAX（跳点缓存以 int16 存距离）。
 * 非法尺寸返回 NULL（对应 C# 构造抛 ArgumentOutOfRangeException）。
 */
jps_grid_map *jps_grid_map_create(int width, int height);
void jps_grid_map_destroy(jps_grid_map *m);

void jps_grid_map_set_blocked(jps_grid_map *m, int x, int y, bool blocked);
void jps_grid_map_clear_all(jps_grid_map *m);

/* ---- 热路径访问器：静态内联，与 C# 的属性/方法语义逐一对应 ---- */

static inline bool jps_grid_map_in_bounds(const jps_grid_map *m, int x, int y)
{
    return x >= 0 && x < m->width && y >= 0 && y < m->height;
}

/* 行对齐定位：第 y 行第 (x>>6) 个字、位 (x&63)。括号不可省（移位优先级低于加法）。 */
static inline bool jps__grid_map_get_bit(const jps_grid_map *m, int x, int y)
{
    return (m->blocked[(size_t)y * m->stride + (x >> 6)] & (1ULL << (x & 63))) != 0ULL;
}

static inline bool jps_grid_map_is_blocked(const jps_grid_map *m, int x, int y)
{
    return jps_grid_map_in_bounds(m, x, y) && jps__grid_map_get_bit(m, x, y);
}

static inline bool jps_grid_map_is_walkable(const jps_grid_map *m, int x, int y)
{
    return jps_grid_map_in_bounds(m, x, y) && !jps__grid_map_get_bit(m, x, y);
}

/*
 * 取第 y 行第 word_col 个字（64 格）的**可走位图**（1=可走，0=阻挡，即 blocked 取反）。
 * 越界是调用方的有意约定（取上/下行、跨字进位、扫描终止哨兵）：一律返回 0（整字全阻挡）。
 * (uint32_t) 强转使一次比较同时挡住“负数越界”和“≥上界越界”。
 */
static inline uint64_t jps_grid_map_walkable_word(const jps_grid_map *m, int word_col, int y)
{
    if ((uint32_t)y >= (uint32_t)m->height) return 0ULL;            /* 行越界（含 y<0）→ 全阻挡 */
    if ((uint32_t)word_col >= (uint32_t)m->stride) return 0ULL;     /* 字越界（含 word_col<0）→ 全阻挡 */
    return ~m->blocked[(size_t)y * m->stride + word_col];           /* 取反：阻挡=0，可走=1 */
}

#ifdef __cplusplus
}
#endif

#endif /* JPS_GRID_MAP_H */
