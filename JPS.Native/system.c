/*
 * system.c
 * JPS Pathfinding — 公共句柄之一：jps_system 的实现。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include "jps.h"                 /* jps_system 前向声明 + 公共函数原型 */
#include "system.h"


static uint64_t jps__grid_map_memory_bytes(const jps_grid_map *m)
{
    uint64_t bytes;
    size_t row_words, col_words;

    if (m == NULL)
        return 0;

    bytes = (uint64_t)sizeof(*m);
    row_words = (size_t)(m->height + 2) * (size_t)m->stride;
    col_words = (size_t)(m->width + 2) * (size_t)m->col_stride;
    if (m->blocked_alloc != NULL)
        bytes += (uint64_t)(row_words * sizeof(uint64_t));
    if (m->col_blocked_alloc != NULL)
        bytes += (uint64_t)(col_words * sizeof(uint64_t));
    if (m->row_version != NULL)
        bytes += (uint64_t)((size_t)m->height * sizeof(int));
    if (m->col_version != NULL)
        bytes += (uint64_t)((size_t)m->width * sizeof(int));
    if (m->dirty_rows != NULL)
        bytes += (uint64_t)((size_t)m->height * sizeof(int));
    if (m->dirty_cols != NULL)
        bytes += (uint64_t)((size_t)m->width * sizeof(int));
    if (m->dirty_row_mark != NULL)
        bytes += (uint64_t)((size_t)m->height * sizeof(uint8_t));
    if (m->dirty_col_mark != NULL)
        bytes += (uint64_t)((size_t)m->width * sizeof(uint8_t));
    return bytes;
}

static uint64_t jps__jump_point_cache_memory_bytes(const jps_jump_point_cache *c)
{
    uint64_t bytes;

    if (c == NULL)
        return 0;

    bytes = (uint64_t)sizeof(*c);
    if (c->dist != NULL)
        bytes += (uint64_t)(4u * (size_t)c->size * sizeof(int16_t));
    if (c->gen != NULL)
        bytes += (uint64_t)(4u * (size_t)c->size * sizeof(uint8_t));
    if (c->row_gen != NULL)
        bytes += (uint64_t)((size_t)c->h * sizeof(uint8_t));
    if (c->col_gen != NULL)
        bytes += (uint64_t)((size_t)c->w * sizeof(uint8_t));
    if (c->row_version != NULL)
        bytes += (uint64_t)((size_t)c->h * sizeof(int));
    if (c->col_version != NULL)
        bytes += (uint64_t)((size_t)c->w * sizeof(int));
    return bytes;
}

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

uint64_t jps_system_memory_bytes(const jps_system *s)
{
    if (s == NULL)
        return 0;

    return (uint64_t)sizeof(*s)
        + jps__grid_map_memory_bytes(s->map)
        + jps__jump_point_cache_memory_bytes(s->cache);
}

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
    if (s == NULL || cells == NULL)
        return;

    jps_grid_map_set_blocked_buffer(s->map, cells, count);
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
