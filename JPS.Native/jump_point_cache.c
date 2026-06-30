/*
 * jump_point_cache.c
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JumpPointCache.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include <string.h>
#include "jump_point_cache.h"
#include "directions.h"
#include "rules.h"

/* ---------------- 64 位位扫描（de Bruijn 序列实现） ----------------
 * 手写而非用编译器内建，确保 MSVC / NDK / iOS 行为一致、与 C# Bits 逐位等价。
 */

static const uint64_t JPS__DEBRUIJN = 0x03f79d71b4cb0a89ULL;

static const int JPS__BIT_INDEX[64] = {
     0,  1, 48,  2, 57, 49, 28,  3,
    61, 58, 50, 42, 38, 29, 17,  4,
    62, 55, 59, 36, 53, 51, 43, 22,
    45, 39, 33, 30, 24, 18, 12,  5,
    63, 47, 56, 27, 60, 41, 37, 16,
    54, 35, 52, 21, 44, 32, 23, 11,
    46, 26, 40, 15, 34, 20, 31, 10,
    25, 14, 19,  9, 13,  8,  7,  6,
};

/* 最低置位的下标（要求 x != 0）。 */
static int jps__lowest_set(uint64_t x)
{
    return JPS__BIT_INDEX[((x & (~x + 1)) * JPS__DEBRUIJN) >> 58];
}

/* 最高置位的下标（要求 x != 0）。 */
static int jps__highest_set(uint64_t x)
{
    x |= x >> 1; x |= x >> 2; x |= x >> 4; x |= x >> 8; x |= x >> 16; x |= x >> 32;
    x &= ~(x >> 1);   /* 仅保留最高位 */
    return JPS__BIT_INDEX[(x * JPS__DEBRUIJN) >> 58];
}

/* ---------------- 水平按字（uint64）强迫邻居掩码 ---------------- */

/* 向右（dx=+1）：jump(c) = walk(c,y) ∧ [walk(c,y-1)∧block(c-1,y-1) ∨ walk(c,y+1)∧block(c-1,y+1)]。 */
static uint64_t jps__forced_right(uint64_t walk_y, uint64_t walk_up, uint64_t walk_dn,
                                  uint64_t prev_up, uint64_t prev_dn)
{
    uint64_t blk_up_shift = (~walk_up << 1) | ((~prev_up) >> 63);   /* block(c-1, y-1) */
    uint64_t blk_dn_shift = (~walk_dn << 1) | ((~prev_dn) >> 63);   /* block(c-1, y+1) */
    return ((walk_up & blk_up_shift) | (walk_dn & blk_dn_shift)) & walk_y;
}

/* 向左（dx=-1）：邻居取 c+1 → 阻挡位右移 1；跨字时高位从后一字最低位补入。 */
static uint64_t jps__forced_left(uint64_t walk_y, uint64_t walk_up, uint64_t walk_dn,
                                 uint64_t next_up, uint64_t next_dn)
{
    uint64_t blk_up_shift = (~walk_up >> 1) | ((~next_up & 1ULL) << 63);   /* block(c+1, y-1) */
    uint64_t blk_dn_shift = (~walk_dn >> 1) | ((~next_dn & 1ULL) << 63);   /* block(c+1, y+1) */
    return ((walk_up & blk_up_shift) | (walk_dn & blk_dn_shift)) & walk_y;
}

/*
 * 从 (x,y) 沿水平方向 dx∈{+1,-1} 找最近的“停点”，语义与逐格扫描完全一致：
 *   · 撞墙（含越界/padding）→ (*out_s=到墙步数, *out_jump=false)
 *   · 出现强迫邻居（该格是跳点）→ (*out_s=到跳点步数, *out_jump=true)
 */
static void jps__horizontal_scan(const jps_grid_map *m, int x, int y, int dx,
                                 int *out_s, bool *out_jump)
{
    if (dx > 0)
    {
        int start_col = x + 1;                       /* 逐格循环先 +dx 再判，故从 x+1 起 */
        int w = start_col >> 6;
        uint64_t sub = ~0ULL << (start_col & 63);    /* 首字内只看 ≥ 起始位的列 */
        uint64_t prev_up = jps_grid_map_walkable_word(m, w - 1, y - 1);
        uint64_t prev_dn = jps_grid_map_walkable_word(m, w - 1, y + 1);
        for (;;)
        {
            uint64_t walk_y = jps_grid_map_walkable_word(m, w, y);
            uint64_t walk_up = jps_grid_map_walkable_word(m, w, y - 1);
            uint64_t walk_dn = jps_grid_map_walkable_word(m, w, y + 1);
            uint64_t jump = jps__forced_right(walk_y, walk_up, walk_dn, prev_up, prev_dn);
            uint64_t mm = (~walk_y | jump) & sub;    /* ~walk_y = 行 y 阻挡位；与 jump 互斥 */
            if (mm != 0ULL)
            {
                int b = jps__lowest_set(mm);
                int col = (w << 6) + b;
                bool is_jump = ((jump >> b) & 1ULL) != 0ULL;
                *out_s = col - x;                    /* 越界字会令 ~walk_y 命中，循环必终止 */
                *out_jump = is_jump;
                return;
            }
            prev_up = walk_up; prev_dn = walk_dn;
            w++;
            sub = ~0ULL;
        }
    }
    else
    {
        int start_col = x - 1;
        int w, sb;
        uint64_t sub, next_up, next_dn;
        if (start_col < 0)                           /* 左邻即越界 → 墙，步数 1 */
        {
            *out_s = 1;
            *out_jump = false;
            return;
        }
        w = start_col >> 6;
        sb = start_col & 63;
        sub = sb == 63 ? ~0ULL : ((1ULL << (sb + 1)) - 1);   /* 首字内只看 ≤ 起始位的列 */
        next_up = jps_grid_map_walkable_word(m, w + 1, y - 1);
        next_dn = jps_grid_map_walkable_word(m, w + 1, y + 1);
        for (;;)
        {
            uint64_t walk_y = jps_grid_map_walkable_word(m, w, y);
            uint64_t walk_up = jps_grid_map_walkable_word(m, w, y - 1);
            uint64_t walk_dn = jps_grid_map_walkable_word(m, w, y + 1);
            uint64_t jump = jps__forced_left(walk_y, walk_up, walk_dn, next_up, next_dn);
            uint64_t mm = (~walk_y | jump) & sub;
            if (mm != 0ULL)
            {
                int b = jps__highest_set(mm);
                int col = (w << 6) + b;
                bool is_jump = ((jump >> b) & 1ULL) != 0ULL;
                *out_s = x - col;
                *out_jump = is_jump;
                return;
            }
            next_up = walk_up; next_dn = walk_dn;
            w--;
            if (w < 0)                               /* 越过第 0 列 → 第 -1 列是墙，步数 x+1 */
            {
                *out_s = x + 1;
                *out_jump = false;
                return;
            }
            sub = ~0ULL;
        }
    }
}

/* ---------------- 生命周期 ---------------- */

jps_jump_point_cache *jps_jump_point_cache_create(void)
{
    jps_jump_point_cache *c = (jps_jump_point_cache *)malloc(sizeof(jps_jump_point_cache));
    if (c == NULL)
        return NULL;
    c->w = 0;
    c->size = 0;
    c->cells = NULL;
    c->valid_gen = 0;
    c->map_version = -1;
    return c;
}

void jps_jump_point_cache_destroy(jps_jump_point_cache *c)
{
    if (c == NULL)
        return;
    free(c->cells);
    free(c);
}

void jps_jump_point_cache_sync(jps_jump_point_cache *c, const jps_grid_map *m)
{
    if (c->w != m->width || c->size != m->width * m->height)
    {
        c->w = m->width;
        c->size = m->width * m->height;
        free(c->cells);
        c->cells = (jps_cell_jump *)calloc((size_t)c->size, sizeof(jps_cell_jump));
        c->valid_gen = 0;
        c->map_version = -1;
    }

    if (c->map_version != m->version)
    {
        if (c->valid_gen >= 255)
        {
            memset(c->cells, 0, (size_t)c->size * sizeof(jps_cell_jump));   /* 世代回绕：整体清零 → 全 dirty */
            c->valid_gen = 1;
        }
        else
        {
            c->valid_gen++;
        }
        c->map_version = m->version;
    }
}

int jps_jump_point_cache_cardinal_dist(jps_jump_point_cache *c, const jps_grid_map *m,
                                       int x, int y, int dx, int dy, int dir)
{
    int idx0 = y * c->w + x;
    int s;
    bool jump_found;
    int fx, fy, k;

    if (c->cells[idx0].gen[dir] == c->valid_gen)
        return c->cells[idx0].dist[dir];

    /* 扫描：从 (x,y) 沿方向找最近跳点或墙。 */
    if (dy == 0)
    {
        jps__horizontal_scan(m, x, y, dx, &s, &jump_found);
    }
    else
    {
        int ry = y;
        s = 0;
        jump_found = false;
        for (;;)
        {
            ry += dy;
            s++;
            if (!jps_grid_map_is_walkable(m, x, ry)) { jump_found = false; break; }
            if (jps_is_jump_point(m, x, ry, dx, dy)) { jump_found = true; break; }
        }
    }

    /* 回填整段 run（步 k=0..s-1 的可走格）。距离量级 ≤ max(W,H) ≤ INT16_MAX。 */
    fx = x; fy = y;
    for (k = 0; k <= s - 1; k++)
    {
        int ci = fy * c->w + fx;
        c->cells[ci].dist[dir] = (int16_t)(jump_found ? (s - k) : -((s - 1) - k));
        c->cells[ci].gen[dir] = c->valid_gen;
        fx += dx;
        fy += dy;
    }

    return c->cells[idx0].dist[dir];
}
