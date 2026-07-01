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

/*
 * 参数化位图访问器：bits/stride/n_lines 描述一份按“线(line)”排布的位图，扫描沿“跨向(across)”推进：
 *   行排布(原图)：line=y、across=x、stride=行 stride、n_lines=height —— 用于水平扫描；
 *   列排布(转置) ：line=x、across=y、stride=列 stride、n_lines=width  —— 用于垂直扫描。
 * 于是“垂直扫描”= 在转置位图上做同一套“水平扫描”，两者共用下面这份 SIMD 代码。
 */

/* 取第 line 条线第 w2 个对齐单元(words w2,w2+1)的可走位向量；线/字越界 → 全阻挡(全 0)。
 * 前提：w2 偶、stride 偶 → w2 在界内时 w2+1 也在同一条线内，可安全 16 字节对齐加载。 */
static inline jps_v128 jps__walk_v128(const uint64_t *bits, int stride, int n_lines, int line, int w2)
{
    if ((uint32_t)line >= (uint32_t)n_lines) return jps_v_zero();
    if ((uint32_t)w2 >= (uint32_t)stride) return jps_v_zero();
    return jps_v_not(jps_v_load(&bits[(size_t)line * stride + w2]));
}

/* 取第 line 条线第 word_col 个 64 位字的可走位（取反）；越界 → 0（全阻挡）。用于跨单元进位。 */
static inline uint64_t jps__walk_word(const uint64_t *bits, int stride, int n_lines, int line, int word_col)
{
    if ((uint32_t)line >= (uint32_t)n_lines) return 0ULL;
    if ((uint32_t)word_col >= (uint32_t)stride) return 0ULL;
    return ~bits[(size_t)line * stride + word_col];
}

/* 保留 bit 0..k（k∈0..63）的低位掩码。 */
static inline uint64_t jps__mask_le(int k)
{
    return k >= 63 ? ~0ULL : ((1ULL << (k + 1)) - 1);
}

/*
 * 沿第 line 条线的 across 方向 dir∈{+1,-1} 扫描最近“停点”（跳点 or 墙），语义与逐格扫描一致。
 * bits/stride/n_lines 选行排布或列排布，从而一套代码同时服务水平与垂直扫描。
 * pos 为当前 across 坐标（行扫=x，列扫=y）；返回步数 *out_s 与是否跳点 *out_jump。
 */
static void jps__scan_line(const uint64_t *bits, int stride, int n_lines,
                           int line, int pos, int dir, int *out_s, bool *out_jump)
{
    jps_v128 ones = jps_v_set2(~0ULL, ~0ULL);

    if (dir > 0)
    {
        int start = pos + 1;                          /* 逐格循环先 +dir 再判，故从 pos+1 起 */
        int unit = (start >> 7) << 1;                 /* start 所在 128 位单元的低字下标（偶） */
        int off = start & 127;                        /* 单元内偏移 0..127 */
        /* 首单元 sub：保留单元内 ≥ off 的列（off<64 落在低字，否则落在高字）。 */
        jps_v128 sub = off < 64 ? jps_v_set2(~0ULL << off, ~0ULL)
                                : jps_v_set2(0ULL, ~0ULL << (off - 64));
        /* 进位源 = 单元低字左邻字(unit-1) 的 line±1 两条线；之后每单元由本单元高字滚动复用。 */
        uint64_t prev_up = jps__walk_word(bits, stride, n_lines, line - 1, unit - 1);
        uint64_t prev_dn = jps__walk_word(bits, stride, n_lines, line + 1, unit - 1);
        for (;;)
        {
            jps_v128 walk_y  = jps__walk_v128(bits, stride, n_lines, line,     unit);
            jps_v128 walk_up = jps__walk_v128(bits, stride, n_lines, line - 1, unit);
            jps_v128 walk_dn = jps__walk_v128(bits, stride, n_lines, line + 1, unit);

            /* block(c-1) = (~walk << 1)：单元内进位由整 128 位左移完成；低字进位 cin 来自左邻字 bit63。 */
            jps_v128 blk_up = jps_v_shl1(jps_v_not(walk_up), (~prev_up) >> 63);
            jps_v128 blk_dn = jps_v_shl1(jps_v_not(walk_dn), (~prev_dn) >> 63);
            jps_v128 jump = jps_v_and(jps_v_or(jps_v_and(walk_up, blk_up), jps_v_and(walk_dn, blk_dn)), walk_y);
            jps_v128 mm = jps_v_and(jps_v_or(jps_v_not(walk_y), jump), sub);   /* ~walk_y = 阻挡位，与 jump 互斥 */

            if (!jps_v_is_zero(mm))
            {
                uint64_t m0 = jps_v_lane(mm, 0);      /* 先低字(word unit，更小坐标) */
                uint64_t m1 = jps_v_lane(mm, 1);      /* 再高字(word unit+1) */
                if (m0 != 0ULL)
                {
                    int b = jps__lowest_set(m0);
                    *out_s = (unit * 64 + b) - pos;
                    *out_jump = ((jps_v_lane(jump, 0) >> b) & 1ULL) != 0ULL;
                }
                else /* m1 必非 0 */
                {
                    int b = jps__lowest_set(m1);
                    *out_s = ((unit + 1) * 64 + b) - pos;
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
        int start = pos - 1;
        int unit, off;
        uint64_t next_up, next_dn;
        jps_v128 sub;
        if (start < 0)                                /* 前邻即越界 → 墙，步数 1 */
        {
            *out_s = 1;
            *out_jump = false;
            return;
        }
        unit = (start >> 7) << 1;                     /* start 所在 128 位单元的低字下标（偶） */
        off = start & 127;
        /* 首单元 sub：保留单元内 ≤ off 的列（off≥64 落在高字，否则落在低字）。 */
        sub = off >= 64 ? jps_v_set2(~0ULL, jps__mask_le(off - 64))
                        : jps_v_set2(jps__mask_le(off), 0ULL);
        /* 进位源 = 单元高字右邻字(unit+2) 的 line±1 两条线；之后每单元由本单元低字滚动复用。 */
        next_up = jps__walk_word(bits, stride, n_lines, line - 1, unit + 2);
        next_dn = jps__walk_word(bits, stride, n_lines, line + 1, unit + 2);
        for (;;)
        {
            jps_v128 walk_y  = jps__walk_v128(bits, stride, n_lines, line,     unit);
            jps_v128 walk_up = jps__walk_v128(bits, stride, n_lines, line - 1, unit);
            jps_v128 walk_dn = jps__walk_v128(bits, stride, n_lines, line + 1, unit);

            /* block(c+1) = (~walk >> 1)：单元内进位由整 128 位右移完成；高字进位 cin 来自右邻字 bit0。 */
            jps_v128 blk_up = jps_v_shr1(jps_v_not(walk_up), (~next_up) & 1ULL);
            jps_v128 blk_dn = jps_v_shr1(jps_v_not(walk_dn), (~next_dn) & 1ULL);
            jps_v128 jump = jps_v_and(jps_v_or(jps_v_and(walk_up, blk_up), jps_v_and(walk_dn, blk_dn)), walk_y);
            jps_v128 mm = jps_v_and(jps_v_or(jps_v_not(walk_y), jump), sub);

            if (!jps_v_is_zero(mm))
            {
                uint64_t m0 = jps_v_lane(mm, 0);      /* 低字(word unit) */
                uint64_t m1 = jps_v_lane(mm, 1);      /* 高字(word unit+1，更大坐标) */
                if (m1 != 0ULL)
                {
                    int b = jps__highest_set(m1);
                    *out_s = pos - ((unit + 1) * 64 + b);
                    *out_jump = ((jps_v_lane(jump, 1) >> b) & 1ULL) != 0ULL;
                }
                else /* m0 必非 0 */
                {
                    int b = jps__highest_set(m0);
                    *out_s = pos - (unit * 64 + b);   /* unit 可为负(边界)；用乘法避免负数左移 UB */
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

    /* 扫描：从 (x,y) 沿方向找最近跳点或墙。水平走行排布，垂直走列排布(转置) —— 两者共用 SIMD。 */
    if (dy == 0)
        jps__scan_line(m->blocked, m->stride, m->height, y, x, dx, &s, &jump_found);
    else
        jps__scan_line(m->col_blocked, m->col_stride, m->width, x, y, dy, &s, &jump_found);

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