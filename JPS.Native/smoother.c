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
    jps_point_f *result;
    int cap, count = 0;
    int anchor, i;

    if (path_count == 0)
    {
        *out_points = NULL;
        return 0;
    }

    cap = path_count;   /* 平滑后点数 ≤ 原路径点数 */
    result = (jps_point_f *)malloc((size_t)cap * sizeof(jps_point_f));

    /* 起点一定保留 */
    result[count++] = jps__center(path[0]);
    if (path_count == 1)
    {
        *out_points = result;
        return count;
    }

    anchor = 0;
    /* 从 i=2 开始：锚点与其相邻点(anchor+1)必然通视，无需检测 */
    for (i = 2; i < path_count; i++)
    {
        if (!jps_line_of_sight(m, path[anchor].x, path[anchor].y, path[i].x, path[i].y))
        {
            result[count++] = jps__center(path[i - 1]);
            anchor = i - 1;
        }
    }

    /* 终点一定保留 */
    result[count++] = jps__center(path[path_count - 1]);

    *out_points = result;
    return count;
}
