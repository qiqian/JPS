/*
 * directions.h
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JpsDirections.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_DIRECTIONS_H
#define JPS_DIRECTIONS_H

#include <stdbool.h>
#include <stdint.h>
#include "grid_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define JPS_DIR_COUNT 8

/* 整数距离：横向 1000，斜向 1414（≈ √2 × 1000）。 */
#define JPS_CARDINAL_COST 1000
#define JPS_DIAGONAL_COST 1414

static inline int jps_dir_index_of(int dx, int dy)
{
    if (dx == 1 && dy == 0) return 0;
    if (dx == -1 && dy == 0) return 1;
    if (dx == 0 && dy == 1) return 2;
    if (dx == 0 && dy == -1) return 3;
    if (dx == 1 && dy == 1) return 4;
    if (dx == -1 && dy == 1) return 5;
    if (dx == 1 && dy == -1) return 6;
    if (dx == -1 && dy == -1) return 7;
    return -1;
}

static inline bool jps_is_diagonal(int dx, int dy) { return dx != 0 && dy != 0; }

static inline bool jps_is_diagonal_index(int index) { return index >= 4; }

/* 整数符号函数，对应 C# Math.Sign。 */
static inline int jps_sign(int v) { return (v > 0) - (v < 0); }

/*
 * 从 (x,y) 沿对角 (dx,dy) 走一步是否合法。
 * 默认（未定义 JPS_ALLOW_CORNER_CUTTING）：禁止斜穿角——目标格 + 两侧正交格都必须可走。
 * 定义 JPS_ALLOW_CORNER_CUTTING：恢复“允许斜穿拐角”——只要目标格可走。
 */
static inline bool jps_diagonal_allowed(const jps_grid_map *m, int x, int y, int dx, int dy)
{
#ifdef JPS_ALLOW_CORNER_CUTTING
    return jps_grid_map_is_walkable(m, x + dx, y + dy);
#else
    return jps_grid_map_is_walkable(m, x + dx, y + dy) &&
           jps_grid_map_is_walkable(m, x + dx, y) &&
           jps_grid_map_is_walkable(m, x, y + dy);
#endif
}

/* 八方向（octile）启发式，全整数；返回 int64 以与 g/优先级一致。 */
static inline int64_t jps_octile_heuristic(int x1, int y1, int x2, int y2)
{
    int dx = x1 > x2 ? x1 - x2 : x2 - x1;
    int dy = y1 > y2 ? y1 - y2 : y2 - y1;
    int mn = dx < dy ? dx : dy;
    int mx = dx < dy ? dy : dx;
    return (int64_t)(mx - mn) * JPS_CARDINAL_COST + (int64_t)mn * JPS_DIAGONAL_COST;
}

static inline int jps_move_cost(int dx, int dy)
{
    return jps_is_diagonal(dx, dy) ? JPS_DIAGONAL_COST : JPS_CARDINAL_COST;
}

#ifdef __cplusplus
}
#endif

#endif /* JPS_DIRECTIONS_H */
