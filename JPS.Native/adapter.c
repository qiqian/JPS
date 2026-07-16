/*
 * adapter.c
 * JPS Pathfinding - native C adapter for obstacle padding and id-tracked dynamic rectangles.
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "jps.h"

typedef struct jps_adapter_obstacle
{
    int id;
    int x;
    int y;
    int width;
    int height;
} jps_adapter_obstacle;

typedef struct jps_adapter_rect
{
    int left;
    int top;
    int right;
    int bottom;
} jps_adapter_rect;

struct jps_adapter
{
    int width;
    int height;
    int obstacle_padding;
    size_t cell_count;
    uint8_t *static_blocked;
    uint32_t *coverage;
    jps_adapter_obstacle *obstacles;
    int obstacle_count;
    int obstacle_capacity;
    jps_system *system;
};

static jps_adapter_rect jps__adapter_empty_rect(void)
{
    jps_adapter_rect r = { 0, 0, -1, -1 };
    return r;
}

static int jps__adapter_rect_empty(jps_adapter_rect r)
{
    return r.left > r.right || r.top > r.bottom;
}

static jps_adapter_rect jps__adapter_intersect(jps_adapter_rect a, jps_adapter_rect b)
{
    jps_adapter_rect r;
    if (jps__adapter_rect_empty(a) || jps__adapter_rect_empty(b))
        return jps__adapter_empty_rect();
    r.left = a.left > b.left ? a.left : b.left;
    r.top = a.top > b.top ? a.top : b.top;
    r.right = a.right < b.right ? a.right : b.right;
    r.bottom = a.bottom < b.bottom ? a.bottom : b.bottom;
    return r;
}

static jps_adapter_rect jps__adapter_expanded_rect(const jps_adapter *a,
                                                   int x, int y, int width, int height)
{
    int64_t padding = a->obstacle_padding;
    int64_t left = (int64_t)x - padding;
    int64_t top = (int64_t)y - padding;
    int64_t right = (int64_t)x + (int64_t)width - 1 + padding;
    int64_t bottom = (int64_t)y + (int64_t)height - 1 + padding;
    jps_adapter_rect r;

    if (left < 0) left = 0;
    if (top < 0) top = 0;
    if (right >= a->width) right = a->width - 1;
    if (bottom >= a->height) bottom = a->height - 1;
    if (left > right || top > bottom)
        return jps__adapter_empty_rect();

    r.left = (int)left;
    r.top = (int)top;
    r.right = (int)right;
    r.bottom = (int)bottom;
    return r;
}

static void jps__adapter_change_coverage(jps_adapter *a, int x, int y, int delta)
{
    size_t index = (size_t)y * (size_t)a->width + (size_t)x;
    uint32_t before = a->coverage[index];
    uint32_t after;

    if (delta > 0)
    {
        /* Source count is bounded by map/dynamic-array sizes; saturate defensively. */
        if (before == UINT32_MAX)
            return;
        after = before + 1u;
    }
    else
    {
        /* Internal calls are paired; keep zero safe if state is ever inconsistent. */
        if (before == 0)
            return;
        after = before - 1u;
    }

    a->coverage[index] = after;
    if (before == 0 && after != 0)
        jps_system_set_blocked(a->system, x, y, 1);
    else if (before != 0 && after == 0)
        jps_system_set_blocked(a->system, x, y, 0);
}

static void jps__adapter_apply_rect(jps_adapter *a, jps_adapter_rect r, int delta)
{
    int x, y;
    if (jps__adapter_rect_empty(r))
        return;
    for (y = r.top; y <= r.bottom; y++)
        for (x = r.left; x <= r.right; x++)
            jps__adapter_change_coverage(a, x, y, delta);
}

static void jps__adapter_apply_expanded(jps_adapter *a,
                                        int x, int y, int width, int height, int delta)
{
    jps__adapter_apply_rect(a, jps__adapter_expanded_rect(a, x, y, width, height), delta);
}

static void jps__adapter_apply_difference(jps_adapter *a, jps_adapter_rect source,
                                          jps_adapter_rect subtract, int delta)
{
    jps_adapter_rect intersection;
    jps_adapter_rect strip;
    if (jps__adapter_rect_empty(source))
        return;

    intersection = jps__adapter_intersect(source, subtract);
    if (jps__adapter_rect_empty(intersection))
    {
        jps__adapter_apply_rect(a, source, delta);
        return;
    }

    strip = (jps_adapter_rect){ source.left, source.top, source.right, intersection.top - 1 };
    jps__adapter_apply_rect(a, strip, delta);
    strip = (jps_adapter_rect){ source.left, intersection.bottom + 1, source.right, source.bottom };
    jps__adapter_apply_rect(a, strip, delta);
    strip = (jps_adapter_rect){ source.left, intersection.top, intersection.left - 1, intersection.bottom };
    jps__adapter_apply_rect(a, strip, delta);
    strip = (jps_adapter_rect){ intersection.right + 1, intersection.top, source.right, intersection.bottom };
    jps__adapter_apply_rect(a, strip, delta);
}

static void jps__adapter_replace_expanded(jps_adapter *a,
                                          const jps_adapter_obstacle *old_obstacle,
                                          const jps_adapter_obstacle *new_obstacle)
{
    jps_adapter_rect old_rect = jps__adapter_expanded_rect(a, old_obstacle->x, old_obstacle->y,
                                                           old_obstacle->width, old_obstacle->height);
    jps_adapter_rect new_rect = jps__adapter_expanded_rect(a, new_obstacle->x, new_obstacle->y,
                                                           new_obstacle->width, new_obstacle->height);
    jps__adapter_apply_difference(a, new_rect, old_rect, 1);
    jps__adapter_apply_difference(a, old_rect, new_rect, -1);
}

static void jps__adapter_apply_boundary(jps_adapter *a)
{
    int x, y;
    int p = a->obstacle_padding;
    if (p == 0)
        return;
    for (y = 0; y < a->height; y++)
        for (x = 0; x < a->width; x++)
            if (x < p || x >= a->width - p || y < p || y >= a->height - p)
                jps__adapter_change_coverage(a, x, y, 1);
}

static void jps__adapter_rebuild(jps_adapter *a)
{
    int x, y, i;
    memset(a->coverage, 0, a->cell_count * sizeof(uint32_t));
    jps_system_clear_all(a->system);
    jps__adapter_apply_boundary(a);

    for (y = 0; y < a->height; y++)
        for (x = 0; x < a->width; x++)
            if (a->static_blocked[(size_t)y * (size_t)a->width + (size_t)x] != 0)
                jps__adapter_apply_expanded(a, x, y, 1, 1, 1);

    for (i = 0; i < a->obstacle_count; i++)
    {
        const jps_adapter_obstacle *o = &a->obstacles[i];
        jps__adapter_apply_expanded(a, o->x, o->y, o->width, o->height, 1);
    }
}

static int jps__adapter_find_obstacle(const jps_adapter *a, int id)
{
    int i;
    for (i = 0; i < a->obstacle_count; i++)
        if (a->obstacles[i].id == id)
            return i;
    return -1;
}

static int jps__adapter_reserve_obstacle(jps_adapter *a)
{
    int new_capacity;
    jps_adapter_obstacle *next;
    if (a->obstacle_count < a->obstacle_capacity)
        return 1;

    if (a->obstacle_capacity == 0)
        new_capacity = 8;
    else
    {
        if (a->obstacle_capacity > INT_MAX / 2)
            return 0;
        new_capacity = a->obstacle_capacity * 2;
    }

    if ((size_t)new_capacity > SIZE_MAX / sizeof(jps_adapter_obstacle))
        return 0;

    next = (jps_adapter_obstacle *)realloc(a->obstacles,
                                            (size_t)new_capacity * sizeof(jps_adapter_obstacle));
    if (next == NULL)
        return 0;
    a->obstacles = next;
    a->obstacle_capacity = new_capacity;
    return 1;
}

static jps_adapter *jps__adapter_create_core(int width, int height, int obstacle_padding,
                                             const uint8_t *cells, int count)
{
    jps_adapter *a;
    size_t cell_count;
    if (width <= 0 || height <= 0 || width > 32767 || height > 32767 || obstacle_padding < 0)
        return NULL;
    cell_count = (size_t)width * (size_t)height;
    if (cells != NULL && (count < 0 || (size_t)count != cell_count))
        return NULL;

    a = (jps_adapter *)calloc(1, sizeof(jps_adapter));
    if (a == NULL)
        return NULL;
    a->width = width;
    a->height = height;
    a->obstacle_padding = obstacle_padding;
    a->cell_count = cell_count;
    a->static_blocked = (uint8_t *)calloc(cell_count, sizeof(uint8_t));
    a->coverage = (uint32_t *)calloc(cell_count, sizeof(uint32_t));
    a->system = jps_system_create(width, height);
    if (a->static_blocked == NULL || a->coverage == NULL || a->system == NULL)
    {
        jps_adapter_destroy(a);
        return NULL;
    }

    if (cells != NULL)
    {
        size_t i;
        for (i = 0; i < cell_count; i++)
            a->static_blocked[i] = cells[i] != 0 ? 1u : 0u;
    }
    jps__adapter_rebuild(a);
    jps_system_sync(a->system);
    return a;
}

jps_adapter *jps_adapter_create(int width, int height, int obstacle_padding)
{
    return jps__adapter_create_core(width, height, obstacle_padding, NULL, 0);
}

jps_adapter *jps_adapter_create_from_buffer(int width, int height, int obstacle_padding,
                                             const uint8_t *cells, int count)
{
    if (cells == NULL)
        return NULL;
    return jps__adapter_create_core(width, height, obstacle_padding, cells, count);
}

void jps_adapter_destroy(jps_adapter *a)
{
    if (a == NULL)
        return;
    jps_system_destroy(a->system);
    free(a->obstacles);
    free(a->coverage);
    free(a->static_blocked);
    free(a);
}

int jps_adapter_width(const jps_adapter *a) { return a != NULL ? a->width : 0; }
int jps_adapter_height(const jps_adapter *a) { return a != NULL ? a->height : 0; }
int jps_adapter_obstacle_padding(const jps_adapter *a) { return a != NULL ? a->obstacle_padding : 0; }
int jps_adapter_dynamic_obstacle_count(const jps_adapter *a) { return a != NULL ? a->obstacle_count : 0; }

uint64_t jps_adapter_memory_bytes(const jps_adapter *a)
{
    if (a == NULL)
        return 0;
    return (uint64_t)sizeof(*a)
        + (uint64_t)a->cell_count * sizeof(uint8_t)
        + (uint64_t)a->cell_count * sizeof(uint32_t)
        + (uint64_t)(size_t)a->obstacle_capacity * sizeof(jps_adapter_obstacle)
        + jps_system_memory_bytes(a->system);
}

int jps_adapter_set_obstacle_padding(jps_adapter *a, int obstacle_padding)
{
    if (a == NULL)
        return JPS_ERR_NULL;
    if (obstacle_padding < 0)
        return JPS_ERR_INVALID_ARGUMENT;
    if (a->obstacle_padding == obstacle_padding)
        return 0;
    a->obstacle_padding = obstacle_padding;
    jps__adapter_rebuild(a);
    return 1;
}

int jps_adapter_set_static_blocked(jps_adapter *a, int x, int y, int blocked)
{
    size_t index;
    uint8_t value;
    if (a == NULL)
        return JPS_ERR_NULL;
    if (x < 0 || x >= a->width || y < 0 || y >= a->height)
        return 0;
    index = (size_t)y * (size_t)a->width + (size_t)x;
    value = blocked != 0 ? 1u : 0u;
    if (a->static_blocked[index] == value)
        return 0;
    a->static_blocked[index] = value;
    jps__adapter_apply_expanded(a, x, y, 1, 1, value != 0 ? 1 : -1);
    return 1;
}

int jps_adapter_is_static_blocked(const jps_adapter *a, int x, int y)
{
    if (a == NULL || x < 0 || x >= a->width || y < 0 || y >= a->height)
        return 0;
    return a->static_blocked[(size_t)y * (size_t)a->width + (size_t)x] != 0 ? 1 : 0;
}

int jps_adapter_is_blocked(const jps_adapter *a, int x, int y)
{
    if (a == NULL)
        return 1;
    return jps_system_is_blocked(a->system, x, y);
}

int jps_adapter_update_dynamic_obstacle(jps_adapter *a, int id,
                                        int x, int y, int width, int height)
{
    int index;
    int remove;
    jps_adapter_obstacle next;
    if (a == NULL)
        return JPS_ERR_NULL;
    remove = width == 0 && height == 0;
    if (!remove && (width <= 0 || height <= 0))
        return JPS_ERR_INVALID_ARGUMENT;

    index = jps__adapter_find_obstacle(a, id);
    if (remove)
    {
        const jps_adapter_obstacle *old;
        if (index < 0)
            return 0;
        old = &a->obstacles[index];
        jps__adapter_apply_expanded(a, old->x, old->y, old->width, old->height, -1);
        a->obstacle_count--;
        if (index != a->obstacle_count)
            a->obstacles[index] = a->obstacles[a->obstacle_count];
        return 1;
    }

    next = (jps_adapter_obstacle){ id, x, y, width, height };
    if (index >= 0)
    {
        jps_adapter_obstacle *old = &a->obstacles[index];
        if (old->x == x && old->y == y && old->width == width && old->height == height)
            return 0;
        jps__adapter_replace_expanded(a, old, &next);
        *old = next;
        return 1;
    }

    if (!jps__adapter_reserve_obstacle(a))
        return JPS_ERR_OUT_OF_MEMORY;
    jps__adapter_apply_expanded(a, x, y, width, height, 1);
    a->obstacles[a->obstacle_count++] = next;
    return 1;
}

int jps_adapter_get_dynamic_obstacle(const jps_adapter *a, int id,
                                     int *out_x, int *out_y, int *out_width, int *out_height)
{
    int index;
    const jps_adapter_obstacle *o;
    if (a == NULL)
        return JPS_ERR_NULL;
    index = jps__adapter_find_obstacle(a, id);
    if (index < 0)
        return 0;
    o = &a->obstacles[index];
    if (out_x != NULL) *out_x = o->x;
    if (out_y != NULL) *out_y = o->y;
    if (out_width != NULL) *out_width = o->width;
    if (out_height != NULL) *out_height = o->height;
    return 1;
}

int jps_adapter_clear_dynamic_obstacles(jps_adapter *a)
{
    int i;
    if (a == NULL)
        return JPS_ERR_NULL;
    if (a->obstacle_count == 0)
        return 0;
    for (i = 0; i < a->obstacle_count; i++)
    {
        const jps_adapter_obstacle *o = &a->obstacles[i];
        jps__adapter_apply_expanded(a, o->x, o->y, o->width, o->height, -1);
    }
    a->obstacle_count = 0;
    return 1;
}

jps_system *jps_adapter_system(jps_adapter *a)
{
    return a != NULL ? a->system : NULL;
}

void jps_adapter_sync(jps_adapter *a)
{
    if (a != NULL)
        jps_system_sync(a->system);
}
