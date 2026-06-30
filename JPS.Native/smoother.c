/*
 * smoother.c
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/PathSmoother.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include "smoother.h"
#include "directions.h"

bool jps_line_of_sight(const jps_grid_map *m, int x0, int y0, int x1, int y1)
{
    int nx, ny, sign_x, sign_y, x, y, ix, iy;

    if (!jps_grid_map_is_walkable(m, x0, y0))
        return false;

    nx = abs(x1 - x0);   /* 水平总步数 */
    ny = abs(y1 - y0);   /* 垂直总步数 */
    sign_x = jps_sign(x1 - x0);
    sign_y = jps_sign(y1 - y0);
    x = x0;
    y = y0;
    ix = 0;   /* 已走的水平步数 */
    iy = 0;   /* 已走的垂直步数 */

    while (ix < nx || iy < ny)
    {
        /* decision = (2*ix+1)*ny - (2*iy+1)*nx，比较水平/垂直推进的相对进度 */
        long long decision = (1LL + 2 * ix) * ny - (1LL + 2 * iy) * nx;

        if (decision == 0)
        {
            /* 正好穿过格点：对角推进 */
            x += sign_x;
            y += sign_y;
            ix++;
            iy++;
#ifndef JPS_ALLOW_CORNER_CUTTING
            /* 默认禁止斜穿角：对角穿越的两个共角格不能是阻挡 */
            if (!jps_grid_map_is_walkable(m, x - sign_x, y) || !jps_grid_map_is_walkable(m, x, y - sign_y))
                return false;
#endif
        }
        else if (decision < 0)
        {
            x += sign_x;   /* 水平推进 */
            ix++;
        }
        else
        {
            y += sign_y;   /* 垂直推进 */
            iy++;
        }

        if (!jps_grid_map_is_walkable(m, x, y))
            return false;   /* 经过的某格被挡 → 不通视 */
    }

    return true;
}

static jps_point_f jps__center(jps_point c)
{
    jps_point_f p;
    p.x = (float)c.x + 0.5f;
    p.y = (float)c.y + 0.5f;
    return p;
}

int jps_smooth_path(const jps_grid_map *m, const jps_point *path, int path_count,
                    jps_point_f **out_points)
{
    /* 为兼容旧 API，分配缓冲并调用 jps_smooth_path_into 写入，然后返回分配的数组。
     * 这样既保持向后兼容，又避免复制核心逻辑。 */
    int cap = path_count;
    jps_point_f *result;
    int produced;

    if (path_count == 0)
    {
        *out_points = NULL;
        return 0;
    }

    result = (jps_point_f *)malloc((size_t)cap * sizeof(jps_point_f));
    if (result == NULL)
    {
        *out_points = NULL;
        return 0;
    }

    produced = jps_smooth_path_into(m, path, path_count, result, cap);
    *out_points = result;
    return produced;
}

int jps_smooth_path_into(const jps_grid_map *m, const jps_point *path, int path_count,
                         jps_point_f *out_points, int capacity_points)
{
    int count = 0;
    int anchor, i;

    if (path_count == 0)
        return 0;

    /* 起点一定保留 */
    if (count < capacity_points) out_points[count] = jps__center(path[0]);
    count++;
    if (path_count == 1)
        return count;

    anchor = 0;
    for (i = 2; i < path_count; i++)
    {
        if (!jps_line_of_sight(m, path[anchor].x, path[anchor].y, path[i].x, path[i].y))
        {
            if (count < capacity_points) out_points[count] = jps__center(path[i - 1]);
            count++;
            anchor = i - 1;
        }
    }

    /* 终点一定保留 */
    if (count < capacity_points) out_points[count] = jps__center(path[path_count - 1]);
    count++;

    return count;
}
