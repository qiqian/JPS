/*
 * rules.h
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JpsRules.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_RULES_H
#define JPS_RULES_H

#include <stdbool.h>
#include "grid_map.h"
#include "directions.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * JPS 跳点 / 强迫邻居判定规则。默认禁止斜穿角（no-corner-cutting）；
 * 定义 JPS_ALLOW_CORNER_CUTTING 可恢复“允许斜穿拐角”。
 * 与 C# JpsRules.cs 严格一致。
 */

/*
 * 对角移动 (dx,dy) 到达 (x,y) 时的强迫邻居。
 * 不切角版（默认，SoCS'12）：对角**不产生**强迫邻居（恒 false）——
 *   论文原文："only straight steps from p to x may produce forced neighbours"。
 */
static inline bool jps_has_diagonal_forced_neighbor(const jps_grid_map *m, int x, int y, int dx, int dy)
{
#ifdef JPS_ALLOW_CORNER_CUTTING
    return (!jps_grid_map_is_walkable(m, x - dx, y) && jps_grid_map_is_walkable(m, x - dx, y + dy)) ||
           (!jps_grid_map_is_walkable(m, x, y - dy) && jps_grid_map_is_walkable(m, x + dx, y - dy));
#else
    (void)m; (void)x; (void)y; (void)dx; (void)dy;
    return false;
#endif
}

/*
 * 直线移动时的强迫邻居。
 * 不切角版（默认，SoCS'12）：强迫邻居是“墙刚结束”的正交格——身后侧被挡、当前侧可走。
 */
static inline bool jps_has_cardinal_forced_neighbor(const jps_grid_map *m, int x, int y, int dx, int dy)
{
#ifdef JPS_ALLOW_CORNER_CUTTING
    if (dy == 0)
    {
        return (!jps_grid_map_is_walkable(m, x, y + 1) && jps_grid_map_is_walkable(m, x + dx, y + 1)) ||
               (!jps_grid_map_is_walkable(m, x, y - 1) && jps_grid_map_is_walkable(m, x + dx, y - 1));
    }
    return (!jps_grid_map_is_walkable(m, x + 1, y) && jps_grid_map_is_walkable(m, x + 1, y + dy)) ||
           (!jps_grid_map_is_walkable(m, x - 1, y) && jps_grid_map_is_walkable(m, x - 1, y + dy));
#else
    if (dy == 0) /* 水平移动 */
    {
        return (jps_grid_map_is_walkable(m, x, y + 1) && !jps_grid_map_is_walkable(m, x - dx, y + 1)) ||
               (jps_grid_map_is_walkable(m, x, y - 1) && !jps_grid_map_is_walkable(m, x - dx, y - 1));
    }
    return (jps_grid_map_is_walkable(m, x + 1, y) && !jps_grid_map_is_walkable(m, x + 1, y - dy)) ||
           (jps_grid_map_is_walkable(m, x - 1, y) && !jps_grid_map_is_walkable(m, x - 1, y - dy));
#endif
}

static inline bool jps_is_jump_point(const jps_grid_map *m, int x, int y, int dx, int dy)
{
    if (jps_is_diagonal(dx, dy))
        return jps_has_diagonal_forced_neighbor(m, x, y, dx, dy);
    return jps_has_cardinal_forced_neighbor(m, x, y, dx, dy);
}

#ifdef __cplusplus
}
#endif

#endif /* JPS_RULES_H */
