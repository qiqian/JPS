/*
 * jump_point_cache.c
 * JPS Pathfinding - C port of JPS.Core/Pathfinding/JumpPointCache.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include <string.h>
#include "jump_point_cache.h"
#include "directions.h"
#include "rules.h"
#include "jps_atomic.h"
#include "jps_simd.h"

#ifndef JPS_HAVE_SIMD
#  error "JPS.Native requires a 128-bit SIMD backend (SSE2 or NEON)."
#endif
/* ---------------- 64 位位扫描（使用编译器/平台内建指令） ----------------
 * 使用编译器内建或 MSVC 原语映射到高效指令（x86 bsf/bsr 或 ARM rbit+clz），
 * 在所有受支持的编译器上比 de Bruijn 查表更快且可维护。要求 x != 0。
 */

#if defined(_MSC_VER)
#  include <intrin.h>

static int jps__lowest_set(uint64_t x)
{
    unsigned long idx;
    _BitScanForward64(&idx, x); /* 返回最低置位索引 */
    return (int)idx;
}

static int jps__highest_set(uint64_t x)
{
    unsigned long idx;
    _BitScanReverse64(&idx, x); /* 返回最高置位索引 */
    return (int)idx;
}

#else /* GCC / Clang */

static int jps__lowest_set(uint64_t x)
{
    return (int)__builtin_ctzll(x); /* count trailing zeros */
}

static int jps__highest_set(uint64_t x)
{
    return 63 - (int)__builtin_clzll(x); /* 63 - count leading zeros */
}

#endif

/* ================= SIMD 水平扫描（一次 128 列 = 一个对齐的 128 位单元） =================
 *
 * 以“128 位对齐单元”（两个连续 uint64：低字 = word unit、高字 = word unit+1，unit 偶）为步长，
 * 直接用对齐向量加载读整单元（GridMap 保证行首 16 字节对齐、stride 偶数），取反一次得可走位。
 * 强迫邻居需把行 y±1 的阻挡位按列移 1（block(c∓1)）：单元内两字之间的进位由**整 128 位**移 1
 * 自动完成；跨单元的进位用相邻单元的边界字显式带入(cin)。起始列不在单元边界时，用 128 位 sub
 * 掩码屏蔽单元内 start_col 之前/之后的列。找到停点后按列序定位：向右先低字、向左先高字。
 */

/* 取行 y 第 w2 个对齐单元（words w2, w2+1）的可走位向量；行/字越界 → 全阻挡(全 0)。
 * 前提：w2 为偶数、stride 为偶数 → w2 在界内时 w2+1 也在同一行内，可安全对齐加载。 */
static inline jps_v128 jps__walkable_v128(const jps_grid_map *m, int w2, int y)
{
    if ((uint32_t)y >= (uint32_t)m->height) return jps_v_zero();     /* 行越界（含 y<0） */
    if ((uint32_t)w2 >= (uint32_t)m->stride) return jps_v_zero();    /* 字越界（含 w2<0） */
    return jps_v_not(jps_v_load(&m->blocked[(size_t)y * m->stride + w2]));
}

/* 保留 bit 0..k（k∈0..63）的低位掩码。 */
static inline uint64_t jps__mask_le(int k)
{
    return k >= 63 ? ~0ULL : ((1ULL << (k + 1)) - 1);
}

static void jps__horizontal_scan(const jps_grid_map *m, int x, int y, int dx,
                                 int *out_s, bool *out_jump)
{
    jps_v128 ones = jps_v_set2(~0ULL, ~0ULL);

    if (dx > 0)
    {
        int start_col = x + 1;                        /* 逐格循环先 +dx 再判，故从 x+1 起 */
        int unit = (start_col >> 7) << 1;             /* start_col 所在 128 位单元的低字下标（偶） */
        int off = start_col & 127;                    /* 单元内偏移 0..127 */
        /* 首单元 sub：保留单元内 ≥ off 的列（off<64 落在低字，否则落在高字）。 */
        jps_v128 sub = off < 64 ? jps_v_set2(~0ULL << off, ~0ULL)
                                : jps_v_set2(0ULL, ~0ULL << (off - 64));
        /* 进位源 = 单元低字左邻字(word unit-1) 的 y±1 行；之后每单元由本单元高字滚动复用。 */
        uint64_t prev_up = jps_grid_map_walkable_word(m, unit - 1, y - 1);
        uint64_t prev_dn = jps_grid_map_walkable_word(m, unit - 1, y + 1);
        for (;;)
        {
            jps_v128 walk_y  = jps__walkable_v128(m, unit, y);
            jps_v128 walk_up = jps__walkable_v128(m, unit, y - 1);
            jps_v128 walk_dn = jps__walkable_v128(m, unit, y + 1);

            /* block(c-1) = (~walk << 1)：单元内进位由整 128 位左移完成；低字进位 cin 来自左邻字 bit63。 */
            jps_v128 blk_up = jps_v_shl1(jps_v_not(walk_up), (~prev_up) >> 63);
            jps_v128 blk_dn = jps_v_shl1(jps_v_not(walk_dn), (~prev_dn) >> 63);
            jps_v128 jump = jps_v_and(jps_v_or(jps_v_and(walk_up, blk_up), jps_v_and(walk_dn, blk_dn)), walk_y);
            jps_v128 mm = jps_v_and(jps_v_or(jps_v_not(walk_y), jump), sub);   /* ~walk_y = 阻挡位，与 jump 互斥 */

            if (!jps_v_is_zero(mm))
            {
                uint64_t m0 = jps_v_lane(mm, 0);      /* 先低字(word unit，更小列) */
                uint64_t m1 = jps_v_lane(mm, 1);      /* 再高字(word unit+1) */
                if (m0 != 0ULL)
                {
                    int b = jps__lowest_set(m0);
                    *out_s = (unit * 64 + b) - x;
                    *out_jump = ((jps_v_lane(jump, 0) >> b) & 1ULL) != 0ULL;
                }
                else /* m1 必非 0 */
                {
                    int b = jps__lowest_set(m1);
                    *out_s = ((unit + 1) * 64 + b) - x;
                    *out_jump = ((jps_v_lane(jump, 1) >> b) & 1ULL) != 0ULL;
                }
                return;
            }

            prev_up = jps_v_lane(walk_up, 1);         /* 本单元高字(word unit+1) → 下一单元的进位源 */
            prev_dn = jps_v_lane(walk_dn, 1);
            unit += 2;
            sub = ones;                               /* 首单元后不再屏蔽（寄存器拷贝，非重建） */
        }
    }
    else
    {
        int start_col = x - 1;
        int unit, off;
        uint64_t next_up, next_dn;
        jps_v128 sub;
        if (start_col < 0)                            /* 左邻即越界 → 墙，步数 1 */
        {
            *out_s = 1;
            *out_jump = false;
            return;
        }
        unit = (start_col >> 7) << 1;                 /* start_col 所在 128 位单元的低字下标（偶） */
        off = start_col & 127;
        /* 首单元 sub：保留单元内 ≤ off 的列（off≥64 落在高字，否则落在低字）。 */
        sub = off >= 64 ? jps_v_set2(~0ULL, jps__mask_le(off - 64))
                        : jps_v_set2(jps__mask_le(off), 0ULL);
        /* 进位源 = 单元高字右邻字(word unit+2) 的 y±1 行；之后每单元由本单元低字滚动复用。 */
        next_up = jps_grid_map_walkable_word(m, unit + 2, y - 1);
        next_dn = jps_grid_map_walkable_word(m, unit + 2, y + 1);
        for (;;)
        {
            jps_v128 walk_y  = jps__walkable_v128(m, unit, y);
            jps_v128 walk_up = jps__walkable_v128(m, unit, y - 1);
            jps_v128 walk_dn = jps__walkable_v128(m, unit, y + 1);

            /* block(c+1) = (~walk >> 1)：单元内进位由整 128 位右移完成；高字进位 cin 来自右邻字 bit0。 */
            jps_v128 blk_up = jps_v_shr1(jps_v_not(walk_up), (~next_up) & 1ULL);
            jps_v128 blk_dn = jps_v_shr1(jps_v_not(walk_dn), (~next_dn) & 1ULL);
            jps_v128 jump = jps_v_and(jps_v_or(jps_v_and(walk_up, blk_up), jps_v_and(walk_dn, blk_dn)), walk_y);
            jps_v128 mm = jps_v_and(jps_v_or(jps_v_not(walk_y), jump), sub);

            if (!jps_v_is_zero(mm))
            {
                uint64_t m0 = jps_v_lane(mm, 0);      /* 低字(word unit) */
                uint64_t m1 = jps_v_lane(mm, 1);      /* 高字(word unit+1，更大列) */
                if (m1 != 0ULL)
                {
                    int b = jps__highest_set(m1);
                    *out_s = x - ((unit + 1) * 64 + b);
                    *out_jump = ((jps_v_lane(jump, 1) >> b) & 1ULL) != 0ULL;
                }
                else /* m0 必非 0 */
                {
                    int b = jps__highest_set(m0);
                    *out_s = x - (unit * 64 + b);      /* unit 可为负(左边界)；用乘法避免负数左移 UB */
                    *out_jump = ((jps_v_lane(jump, 0) >> b) & 1ULL) != 0ULL;
                }
                return;
            }

            next_up = jps_v_lane(walk_up, 0);         /* 本单元低字(word unit) → 下一单元的进位源 */
            next_dn = jps_v_lane(walk_dn, 0);
            unit -= 2;
            sub = ones;
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

    /* acquire 读世代戳：若看到 clean，则发布它的那次 release 写之前的 dist 写均已可见，普通读 dist 即安全。 */
    if (jps_gen_load_acquire(&c->cells[idx0].gen[dir]) == c->valid_gen)
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
        c->cells[ci].dist[dir] = (int16_t)(jump_found ? (s - k) : -((s - 1) - k));   /* 先普通写 dist */
        jps_gen_store_release(&c->cells[ci].gen[dir], c->valid_gen);                  /* 再 release 发布该格 */
        fx += dx;
        fy += dy;
    }

    return c->cells[idx0].dist[dir];   /* 本线程自己刚写的值，程序序可见，无需屏障 */
}