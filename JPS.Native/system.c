/*
 * system.c
 * JPS Pathfinding — 公共句柄之一：jps_system 的实现。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include "system.h"

jps_system *jps_system_create(int width, int height)
{
    jps_system *s = (jps_system *)malloc(sizeof(jps_system));
    if (s == NULL)
        return NULL;

    s->map = jps_grid_map_create(width, height);   /* 尺寸非法/内存不足 → NULL */
    if (s->map == NULL)
    {
        free(s);
        return NULL;
    }

    s->cache = jps_jump_point_cache_create();
    if (s->cache == NULL)
    {
        jps_grid_map_destroy(s->map);
        free(s);
        return NULL;
    }
    return s;
}

void jps_system_destroy(jps_system *s)
{
    if (s == NULL)
        return;
    jps_jump_point_cache_destroy(s->cache);
    jps_grid_map_destroy(s->map);
    free(s);
}

int jps_system_width(const jps_system *s)  { return s ? s->map->width  : 0; }
int jps_system_height(const jps_system *s) { return s ? s->map->height : 0; }

void jps_system_set_blocked(jps_system *s, int x, int y, int blocked)
{
    if (s == NULL)
        return;
    jps_grid_map_set_blocked(s->map, x, y, blocked != 0);
}

int jps_system_is_blocked(const jps_system *s, int x, int y)
{
    if (s == NULL)
        return 1;
    /* 越界按阻挡处理：等价于 !is_walkable（越界即不可走）。 */
    return jps_grid_map_is_walkable(s->map, x, y) ? 0 : 1;
}

void jps_system_clear_all(jps_system *s)
{
    if (s == NULL)
        return;
    jps_grid_map_clear_all(s->map);
}

void jps_system_set_blocked_buffer(jps_system *s, const uint8_t *cells, int count)
{
    int w, h, x, y, i;

    if (s == NULL || cells == NULL)
        return;

    w = s->map->width;
    h = s->map->height;
    if (count != w * h)
        return;   /* 尺寸不匹配，拒绝（避免越界读取） */

    i = 0;
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            jps_grid_map_set_blocked(s->map, x, y, cells[i++] != 0);
}

void jps_system_set_blocked_batch(jps_system *s, const int *xyv, int edit_count)
{
    int i;

    if (s == NULL || xyv == NULL || edit_count <= 0)
        return;

    for (i = 0; i < edit_count; i++)
        jps_grid_map_set_blocked(s->map, xyv[i * 3], xyv[i * 3 + 1], xyv[i * 3 + 2] != 0);
}

void jps_system_sync(jps_system *s)
{
    if (s == NULL)
        return;
    jps_jump_point_cache_sync(s->cache, s->map);
}
