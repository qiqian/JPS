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

static int jps__logical_stride(int dim)
{
    return (dim + 63) >> 6;
}

static int jps__simd_stride(int dim)
{
    int words = jps__logical_stride(dim);
    return (words + 1) & ~1;   /* 向上取偶：两个 uint64 = 128 位对齐 */
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
 * 通用 padding：把一份“按线(line)排布”位图里每条线上 across ≥ valid_across 的位置 1（阻挡）。
 * 行排布：line=y、across=x、valid_across=width；列排布：line=x、across=y、valid_across=height。
 * 这样取反即得可走位，扫描无需逐字屏蔽；且对齐补出的整字也置为全阻挡。
 */
static void jps__mark_padding(uint64_t *bits, int stride, int n_lines, int valid_across)
{
    int first_pad_word = valid_across >> 6;
    int first_pad_bit = valid_across & 63;
    int line, w;

    for (line = 0; line < n_lines; line++)
    {
        uint64_t *lb = &bits[(size_t)line * stride];
        for (w = first_pad_word; w < stride; w++)
        {
            uint64_t mask;
            if (w > first_pad_word)      mask = ~0ULL;                          /* 整字全 padding */
            else if (first_pad_bit == 0) mask = ~0ULL;                          /* 恰好字边界 */
            else                         mask = ~((1ULL << first_pad_bit) - 1); /* 该字内 ≥ 有效位的部分 */
            lb[w] |= mask;
        }
    }
}

jps_grid_map *jps_grid_map_create(int width, int height)
{
    jps_grid_map *m;
    size_t row_bytes, col_bytes;

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
    m->stride = jps__simd_stride(width);      /* 行排布：每行 128 位对齐 */
    m->col_stride = jps__simd_stride(height); /* 列排布：每列 128 位对齐 */

    row_bytes = (size_t)m->stride * height * sizeof(uint64_t);
    col_bytes = (size_t)m->col_stride * width * sizeof(uint64_t);
    m->blocked = (uint64_t *)jps__aligned_alloc16(row_bytes);
    m->col_blocked = (uint64_t *)jps__aligned_alloc16(col_bytes);
    if (m->blocked == NULL || m->col_blocked == NULL)
    {
        jps__aligned_free(m->blocked);
        jps__aligned_free(m->col_blocked);
        free(m);
        return NULL;
    }

    /* 对齐分配器用 malloc，不清零 → 必须先清零，否则有效格残留垃圾（= 随机阻挡）。 */
    memset(m->blocked, 0, row_bytes);
    memset(m->col_blocked, 0, col_bytes);
    jps__mark_padding(m->blocked, m->stride, height, width);       /* 行排布 padding：x ≥ width */
    jps__mark_padding(m->col_blocked, m->col_stride, width, height); /* 列排布 padding：y ≥ height */
    return m;
}

void jps_grid_map_destroy(jps_grid_map *m)
{
    if (m == NULL)
        return;
    jps__aligned_free(m->blocked);
    jps__aligned_free(m->col_blocked);
    free(m);
}

void jps_grid_map_set_blocked(jps_grid_map *m, int x, int y, bool blocked)
{
    size_t rword, cword;
    uint64_t rmask, cmask;

    if (!jps_grid_map_in_bounds(m, x, y))
        return;

    if (jps__grid_map_get_bit(m, x, y) == blocked)
        return;   /* 无变化，不动版本号；行/列两份始终一致 */

    rword = (size_t)y * m->stride + (x >> 6);        /* 行排布位置 */
    rmask = 1ULL << (x & 63);
    cword = (size_t)x * m->col_stride + (y >> 6);    /* 列排布位置（转置） */
    cmask = 1ULL << (y & 63);
    if (blocked)
    {
        m->blocked[rword] |= rmask;
        m->col_blocked[cword] |= cmask;
    }
    else
    {
        m->blocked[rword] &= ~rmask;
        m->col_blocked[cword] &= ~cmask;
    }

    m->version++;   /* 阻挡变化 → 惰性跳点缓存整体失效 */
}

void jps_grid_map_clear_all(jps_grid_map *m)
{
    memset(m->blocked, 0, (size_t)m->stride * m->height * sizeof(uint64_t));
    memset(m->col_blocked, 0, (size_t)m->col_stride * m->width * sizeof(uint64_t));
    jps__mark_padding(m->blocked, m->stride, m->height, m->width);       /* memset 抹掉了 padding，需重置 */
    jps__mark_padding(m->col_blocked, m->col_stride, m->width, m->height);
    m->version++;
}
