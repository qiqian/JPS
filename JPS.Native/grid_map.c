/*
 * grid_map.c
 * JPS Pathfinding — C port of JPS.Core/Models/GridMap.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include <string.h>
#include "grid_map.h"

/* int16 距离上限：边长不能超过 INT16_MAX。 */
#define JPS_MAX_DIM 32767
#define JPS_SIMD_ALIGNMENT 16

static int jps__logical_stride(int width)
{
    return (width + 63) >> 6;
}

static int jps__simd_stride(int width)
{
    int words = jps__logical_stride(width);
    return (words + 1) & ~1;   /* two uint64 words = 128 bits */
}

static void *jps__aligned_alloc16(size_t size)
{
    uintptr_t base, aligned;
    void *raw = malloc(size + (JPS_SIMD_ALIGNMENT - 1) + sizeof(void *));
    if (raw == NULL)
        return NULL;
    base = (uintptr_t)raw + sizeof(void *);
    aligned = (base + (JPS_SIMD_ALIGNMENT - 1)) & ~(uintptr_t)(JPS_SIMD_ALIGNMENT - 1);
    ((void **)aligned)[-1] = raw;
    return (void *)aligned;
}

static void jps__aligned_free(void *p)
{
    if (p != NULL)
        free(((void **)p)[-1]);
}

/*
 * 把每行行尾字中的 padding 位（列 ≥ Width）置 1（阻挡）。
 * 这样 walkable_word 取反即得正确可走位，省去逐字屏蔽。
 */
static void jps__mark_padding_blocked(jps_grid_map *m)
{
    int words = jps__logical_stride(m->width);
    int valid_in_last_word = m->width - (words - 1) * 64;
    int last = words - 1;
    int y, w;

    for (y = 0; y < m->height; y++)
    {
        if (valid_in_last_word != 64)
        {
            uint64_t padding_mask = ~((1ULL << valid_in_last_word) - 1);
            m->blocked[(size_t)y * m->stride + last] |= padding_mask;
        }
        for (w = words; w < m->stride; w++)
            m->blocked[(size_t)y * m->stride + w] = ~0ULL;
    }
}
jps_grid_map *jps_grid_map_create(int width, int height)
{
    jps_grid_map *m;

    if (width <= 0 || height <= 0)
        return NULL;
    /* 跳点缓存用 int16 存距离，距离 ≤ max(宽,高)，故边长不能超过 INT16_MAX。 */
    if (width > JPS_MAX_DIM || height > JPS_MAX_DIM)
        return NULL;

    m = (jps_grid_map *)malloc(sizeof(jps_grid_map));
    if (m == NULL)
        return NULL;

    m->width = width;
    m->height = height;
    m->version = 0;
    m->stride = jps__simd_stride(width);           /* 128-bit aligned row stride */
    m->blocked = (uint64_t *)jps__aligned_alloc16((size_t)m->stride * height * sizeof(uint64_t));
    if (m->blocked == NULL)
    {
        free(m);
        return NULL;
    }

    /* 对齐分配器用 malloc，不清零 → 必须先清零，否则有效格残留垃圾（= 随机阻挡）。 */
    memset(m->blocked, 0, (size_t)m->stride * height * sizeof(uint64_t));
    jps__mark_padding_blocked(m);                  /* 行尾 padding 位预置为阻挡 */
    return m;
}

void jps_grid_map_destroy(jps_grid_map *m)
{
    if (m == NULL)
        return;
    jps__aligned_free(m->blocked);
    free(m);
}

void jps_grid_map_set_blocked(jps_grid_map *m, int x, int y, bool blocked)
{
    int word;
    uint64_t mask;

    if (!jps_grid_map_in_bounds(m, x, y))
        return;

    if (jps__grid_map_get_bit(m, x, y) == blocked)
        return;   /* 无变化，不动版本号 */

    word = (int)((size_t)y * m->stride + (x >> 6));
    mask = 1ULL << (x & 63);
    if (blocked)
        m->blocked[word] |= mask;
    else
        m->blocked[word] &= ~mask;

    m->version++;   /* 阻挡变化 → 惰性跳点缓存整体失效 */
}

void jps_grid_map_clear_all(jps_grid_map *m)
{
    memset(m->blocked, 0, (size_t)m->stride * m->height * sizeof(uint64_t));
    jps__mark_padding_blocked(m);   /* memset 把 padding 也抹成 0，需重新置 1 */
    m->version++;
}
