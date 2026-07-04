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
 * 存储：阻挡用位压缩，1 bit/格，按行对齐到 64 位字（同行连续 64 格落在一个字内、不跨行）。
 *
 * **哨兵带（guard band）**：位图四周补一圈恒“全阻挡”的哨兵，让跳点扫描与热路 ±1 邻查免边界分支——
 *   · 每条线两侧各 JPS_GUARD_WORDS 个哨兵字（凑成 16 字节对齐的哨兵单元，扫描按 128 位单元读）；
 *   · 上下各 1 条全阻挡哨兵线。
 * blocked/col_blocked 指向“真实(0,0)”；扫描/邻查的负偏移与超界访问都落到哨兵内存(恒 1) → 天然当墙，
 * 故 get_bit 用**有符号**下标（允许 x=-1/y=-1 落到负偏移）。stride 为**物理线宽**（含两侧哨兵字）。
 *
 * 与 C# GridMap.cs 语义一致：版本号在任何阻挡增删时自增，供惰性跳点缓存判失效。
 */

/* 每线单侧哨兵字数：左/右各一个 16 字节对齐的 128 位哨兵单元。 */
#define JPS_GUARD_WORDS 2

typedef struct jps_grid_map
{
    int width;
    int height;
    /* 阻挡布局的版本号：任何阻挡增删都自增。寻路器据此判断惰性跳点缓存是否失效。 */
    int version;
    /* 行/列影响版本：单格变化只影响 y-1..y+1 的水平扫描、x-1..x+1 的垂直扫描。Sync 据此降失效粒度。 */
    int *row_version;
    int *col_version;
    /* **物理**线宽（uint64 数）= 数据字数 + 2*JPS_GUARD_WORDS，偶数、128 位对齐。用于线寻址。 */
    int stride;      /* 行排布 */
    int col_stride;  /* 列排布（转置） */
    /*
     * 行/列排布位图，均指向**数据(0,0)**：第 y 行第 (x>>6) 个字 = blocked[y*stride + (x>>6)]，位 = x&63。
     * 负下标（x<0 / y<0）与超界（x≥width / y≥height）落在哨兵带上（恒阻挡）。
     */
    uint64_t *blocked;
    uint64_t *col_blocked;
    /* 16 字节对齐的分配基址（blocked/col_blocked 由此偏移一条哨兵线 + 左哨兵字得到）；仅供释放。 */
    uint64_t *blocked_alloc;
    uint64_t *col_blocked_alloc;
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

/*
 * 行对齐定位：第 y 行第 (x>>6) 个字、位 (x&63)。**有符号**下标——供哨兵版邻查传入 x=-1/y=-1：
 *   x=-1 → (x>>6)=-1、(x&63)=63 → 左哨兵字 -1 的最高位（恒 1）；y=-1 → 上哨兵线（恒 1）。
 * 依赖算术右移（目标编译器 MSVC/GCC/Clang 对负数 >> 均为算术移位）。
 */
static inline bool jps__grid_map_get_bit(const jps_grid_map *m, int x, int y)
{
    ptrdiff_t idx = (ptrdiff_t)y * m->stride + (x >> 6);
    return (m->blocked[idx] & (1ULL << (x & 63))) != 0ULL;
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
 * 哨兵版可走判定：**免边界检查**，依赖哨兵带兜底。仅可用于 x∈[-1,width]、y∈[-1,height]
 * （即以某个界内格为基准的 ±1 邻查）；越界坐标落在哨兵/padding 上（恒阻挡）→ 返回不可走。
 * 传入更远的越界坐标是未定义行为（超出哨兵带覆盖）。
 */
static inline bool jps_grid_map_is_walkable_g(const jps_grid_map *m, int x, int y)
{
    return !jps__grid_map_get_bit(m, x, y);
}

#ifdef __cplusplus
}
#endif

#endif /* JPS_GRID_MAP_H */
