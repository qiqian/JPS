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

static inline int jps__logical_stride(int dim)
{
    return (dim + 63) >> 6;
}

/* 数据字数：向上取偶（两个 uint64 = 128 位对齐），不含哨兵字。 */
static inline int jps__data_stride(int dim)
{
    int words = jps__logical_stride(dim);
    return (words + 1) & ~1;
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
 * 填充哨兵带 + 行尾 padding（位图须已整体清零）。origin 指向数据(0,0)；pstride=物理线宽；
 * n_lines=有效线数；data_stride=数据字数（=pstride-2*GUARD）；valid_across=有效跨度(width/height)。
 * 置 1（阻挡）的部分：上/下哨兵线整条、每数据线两侧哨兵字、行尾 across≥valid 的 padding。
 */
static void jps__fill_bitmap(uint64_t *origin, int pstride, int n_lines, int data_stride, int valid_across)
{
    int first_pad_word = valid_across >> 6;
    int first_pad_bit = valid_across & 63;
    int line;
    ptrdiff_t w;

    /* 上哨兵线(line=-1)、下哨兵线(line=n_lines)：整条物理线（含两侧哨兵字）全阻挡。 */
    for (w = -JPS_GUARD_WORDS; w < data_stride + JPS_GUARD_WORDS; w++)
    {
        origin[(ptrdiff_t)(-1) * pstride + w] = ~0ULL;
        origin[(ptrdiff_t)n_lines * pstride + w] = ~0ULL;
    }

    /* 每条数据线：左哨兵字 + (行尾 padding ∪ 右哨兵字)。 */
    for (line = 0; line < n_lines; line++)
    {
        uint64_t *lb = origin + (ptrdiff_t)line * pstride;
        for (w = -JPS_GUARD_WORDS; w < 0; w++)
            lb[w] = ~0ULL;                                  /* 左哨兵字 */
        for (w = first_pad_word; w < data_stride + JPS_GUARD_WORDS; w++)
        {                                                   /* padding + 右哨兵字 */
            uint64_t mask = (w == first_pad_word && first_pad_bit != 0)
                                ? ~((1ULL << first_pad_bit) - 1)   /* 该字内 ≥ 有效位的部分 */
                                : ~0ULL;                           /* 整字全阻挡 */
            lb[w] |= mask;
        }
    }
}

static void jps__mark_dirty_row(jps_grid_map *m, int y)
{
    if ((uint32_t)y >= (uint32_t)m->height)
        return;
    m->row_version[y]++;
    if (!m->dirty_all && !m->dirty_row_mark[y])
    {
        m->dirty_row_mark[y] = 1;
        m->dirty_rows[m->dirty_row_count++] = y;
    }
}

static void jps__mark_dirty_col(jps_grid_map *m, int x)
{
    if ((uint32_t)x >= (uint32_t)m->width)
        return;
    m->col_version[x]++;
    if (!m->dirty_all && !m->dirty_col_mark[x])
    {
        m->dirty_col_mark[x] = 1;
        m->dirty_cols[m->dirty_col_count++] = x;
    }
}

static void jps__bump_line_versions(jps_grid_map *m, int x, int y)
{
    int i;
    for (i = y - 1; i <= y + 1; i++)
        jps__mark_dirty_row(m, i);
    for (i = x - 1; i <= x + 1; i++)
        jps__mark_dirty_col(m, i);
}

static void jps__bump_all_line_versions(jps_grid_map *m)
{
    int i;
    for (i = 0; i < m->height; i++)
        m->row_version[i]++;
    for (i = 0; i < m->width; i++)
        m->col_version[i]++;
    m->dirty_all = true;
}

jps_grid_map *jps_grid_map_create(int width, int height)
{
    jps_grid_map *m;
    int data_stride_r, data_stride_c;
    size_t row_words, col_words;

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
    m->row_version = NULL;
    m->col_version = NULL;
    m->dirty_rows = NULL;
    m->dirty_cols = NULL;
    m->dirty_row_mark = NULL;
    m->dirty_col_mark = NULL;
    m->dirty_row_count = 0;
    m->dirty_col_count = 0;
    m->dirty_all = false;
    m->blocked = NULL;
    m->col_blocked = NULL;
    m->blocked_alloc = NULL;
    m->col_blocked_alloc = NULL;

    data_stride_r = jps__data_stride(width);
    data_stride_c = jps__data_stride(height);
    m->stride     = data_stride_r + 2 * JPS_GUARD_WORDS;   /* 物理行宽（含两侧哨兵字） */
    m->col_stride = data_stride_c + 2 * JPS_GUARD_WORDS;   /* 物理列宽 */

    row_words = (size_t)(height + 2) * m->stride;     /* +2 = 上下哨兵线 */
    col_words = (size_t)(width + 2) * m->col_stride;
    m->blocked_alloc     = (uint64_t *)jps__aligned_alloc16(row_words * sizeof(uint64_t));
    m->col_blocked_alloc = (uint64_t *)jps__aligned_alloc16(col_words * sizeof(uint64_t));
    m->row_version = (int *)calloc((size_t)height, sizeof(int));
    m->col_version = (int *)calloc((size_t)width, sizeof(int));
    m->dirty_rows = (int *)malloc((size_t)height * sizeof(int));
    m->dirty_cols = (int *)malloc((size_t)width * sizeof(int));
    m->dirty_row_mark = (uint8_t *)calloc((size_t)height, sizeof(uint8_t));
    m->dirty_col_mark = (uint8_t *)calloc((size_t)width, sizeof(uint8_t));
    if (m->blocked_alloc == NULL || m->col_blocked_alloc == NULL ||
        m->row_version == NULL || m->col_version == NULL ||
        m->dirty_rows == NULL || m->dirty_cols == NULL ||
        m->dirty_row_mark == NULL || m->dirty_col_mark == NULL)
    {
        jps__aligned_free(m->blocked_alloc);
        jps__aligned_free(m->col_blocked_alloc);
        free(m->row_version);
        free(m->col_version);
        free(m->dirty_rows);
        free(m->dirty_cols);
        free(m->dirty_row_mark);
        free(m->dirty_col_mark);
        free(m);
        return NULL;
    }

    /* origin = 跳过上哨兵线(1 条 = stride 字) + 左哨兵字，指向数据(0,0)。
     * 偏移 = stride + GUARD 为偶数（stride 偶）→ origin 保持 16 字节对齐。 */
    m->blocked     = m->blocked_alloc     + (size_t)m->stride     + JPS_GUARD_WORDS;
    m->col_blocked = m->col_blocked_alloc + (size_t)m->col_stride + JPS_GUARD_WORDS;

    memset(m->blocked_alloc, 0, row_words * sizeof(uint64_t));
    memset(m->col_blocked_alloc, 0, col_words * sizeof(uint64_t));
    jps__fill_bitmap(m->blocked,     m->stride,     height, data_stride_r, width);   /* 行排布 */
    jps__fill_bitmap(m->col_blocked, m->col_stride, width,  data_stride_c, height);  /* 列排布 */
    return m;
}

void jps_grid_map_destroy(jps_grid_map *m)
{
    if (m == NULL)
        return;
    jps__aligned_free(m->blocked_alloc);
    jps__aligned_free(m->col_blocked_alloc);
    free(m->row_version);
    free(m->col_version);
    free(m->dirty_rows);
    free(m->dirty_cols);
    free(m->dirty_row_mark);
    free(m->dirty_col_mark);
    free(m);
}

void jps_grid_map_set_blocked(jps_grid_map *m, int x, int y, bool blocked)
{
    ptrdiff_t rword, cword;
    uint64_t rmask, cmask;

    if (!jps_grid_map_in_bounds(m, x, y))
        return;

    if (jps__grid_map_get_bit(m, x, y) == blocked)
        return;   /* 无变化，不动版本号；行/列两份始终一致 */

    rword = (ptrdiff_t)y * m->stride + (x >> 6);        /* 行排布位置（物理线宽） */
    rmask = 1ULL << (x & 63);
    cword = (ptrdiff_t)x * m->col_stride + (y >> 6);    /* 列排布位置（转置） */
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

    m->version++;
    jps__bump_line_versions(m, x, y);
}

void jps_grid_map_set_blocked_buffer(jps_grid_map *m, const uint8_t *cells, int count)
{
    int row_data_words, col_data_words;
    int y, word, bit;
    bool changed = false;

    if (m == NULL || cells == NULL || count != m->width * m->height)
        return;

    row_data_words = jps__logical_stride(m->width);
    col_data_words = jps__logical_stride(m->height);

    for (y = 0; y < m->height; y++)
    {
        const uint8_t *src = cells + (ptrdiff_t)y * m->width;
        uint64_t *row = m->blocked + (ptrdiff_t)y * m->stride;
        for (word = 0; word < row_data_words; word++)
        {
            int base = word << 6;
            int bit_count = m->width - base;
            uint64_t bits = 0;
            uint64_t mask;
            uint64_t old_bits;
            if (bit_count > 64)
                bit_count = 64;
            mask = bit_count == 64 ? ~0ULL : ((1ULL << bit_count) - 1);
            for (bit = 0; bit < bit_count; bit++)
                if (src[base + bit] != 0)
                    bits |= 1ULL << bit;
            old_bits = row[word];
            if ((old_bits & mask) != bits)
            {
                uint64_t new_word = (old_bits & ~mask) | bits;
                uint64_t delta = (old_bits & mask) ^ bits; /* bits that changed in this word */
                row[word] = new_word;
                changed = true;

                /* update corresponding column words for changed bits */
                for (bit = 0; bit < bit_count; bit++)
                {
                    if ((delta >> bit) & 1ULL)
                    {
                        int xidx = base + bit;
                        uint64_t *col = m->col_blocked + (ptrdiff_t)xidx * m->col_stride;
                        int cword = y >> 6;
                        uint64_t cmask = 1ULL << (y & 63);
                        if ((bits >> bit) & 1ULL)
                            col[cword] |= cmask;
                        else
                            col[cword] &= ~cmask;
                    }
                }
            }
        }
    }

    /* column bits are updated inline while processing rows above */

    if (!changed)
        return;

    jps_grid_map_clear_dirty(m);
    m->version++;
    jps__bump_all_line_versions(m);
}

void jps_grid_map_clear_all(jps_grid_map *m)
{
    int data_stride_r = m->stride - 2 * JPS_GUARD_WORDS;
    int data_stride_c = m->col_stride - 2 * JPS_GUARD_WORDS;
    size_t row_words = (size_t)(m->height + 2) * m->stride;
    size_t col_words = (size_t)(m->width + 2) * m->col_stride;

    memset(m->blocked_alloc, 0, row_words * sizeof(uint64_t));
    memset(m->col_blocked_alloc, 0, col_words * sizeof(uint64_t));
    jps__fill_bitmap(m->blocked,     m->stride,     m->height, data_stride_r, m->width);
    jps__fill_bitmap(m->col_blocked, m->col_stride, m->width,  data_stride_c, m->height);
    m->version++;
    jps__bump_all_line_versions(m);
}

void jps_grid_map_clear_dirty(jps_grid_map *m)
{
    int i;
    if (m == NULL)
        return;
    for (i = 0; i < m->dirty_row_count; i++)
        m->dirty_row_mark[m->dirty_rows[i]] = 0;
    for (i = 0; i < m->dirty_col_count; i++)
        m->dirty_col_mark[m->dirty_cols[i]] = 0;
    m->dirty_row_count = 0;
    m->dirty_col_count = 0;
    m->dirty_all = false;
}
