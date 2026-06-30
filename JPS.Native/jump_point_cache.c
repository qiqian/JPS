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

/* ================= SIMD 水平扫描（一次 128 列 = 两个 64 位字） =================
 *
 * 与标量版语义逐位等价，但每步处理一对相邻字 [w, w+1]（向右）或 [w-1, w] 倒序（向左）。
 * 关键：强迫邻居要把行 y±1 的阻挡位按列移 1（block(c∓1)），相邻字之间的进位由**整 128 位**
 * 移 1 自动完成（低字最高位→高字最低位）；只有跨“块”的进位需用上一块的边界字显式带入(cin)。
 * 找到停点后按列序定位：向右先低字(更小列)、向左先高字(更大列)，其余与标量一致。
 */
static void jps__horizontal_scan(const jps_grid_map *m, int x, int y, int dx,
                                 int *out_s, bool *out_jump)
{
    if (dx > 0)
    {
        int start_col = x + 1;                        /* 逐格循环先 +dx 再判，故从 x+1 起 */
        int w = start_col >> 6;                       /* 起始字（本对的低字） */
        uint64_t sub0 = ~0ULL << (start_col & 63);    /* 首字内只看 ≥ 起始位的列 */
        /* 进位源 = 上一字（w-1）的 y±1 行；仅首对需要，之后由本对高字滚动复用。 */
        uint64_t prev_up = jps_grid_map_walkable_word(m, w - 1, y - 1);
        uint64_t prev_dn = jps_grid_map_walkable_word(m, w - 1, y + 1);
        jps_v128 sub = jps_v_set2(sub0, ~0ULL);       /* 仅首对的低字(word w)受 sub0 限制 */
        for (;;)
        {
            jps_v128 walk_y  = jps_v_set2(jps_grid_map_walkable_word(m, w, y),     jps_grid_map_walkable_word(m, w + 1, y));
            jps_v128 walk_up = jps_v_set2(jps_grid_map_walkable_word(m, w, y - 1), jps_grid_map_walkable_word(m, w + 1, y - 1));
            jps_v128 walk_dn = jps_v_set2(jps_grid_map_walkable_word(m, w, y + 1), jps_grid_map_walkable_word(m, w + 1, y + 1));

            /* block(c-1) = (~walk << 1)：跨字进位由整 128 位左移完成；首字进位 cin 来自上一字 bit63。 */
            jps_v128 blk_up = jps_v_shl1(jps_v_not(walk_up), (~prev_up) >> 63);
            jps_v128 blk_dn = jps_v_shl1(jps_v_not(walk_dn), (~prev_dn) >> 63);
            jps_v128 jump = jps_v_and(jps_v_or(jps_v_and(walk_up, blk_up), jps_v_and(walk_dn, blk_dn)), walk_y);
            jps_v128 mm = jps_v_and(jps_v_or(jps_v_not(walk_y), jump), sub);   /* ~walk_y = 阻挡位，与 jump 互斥 */

            if (!jps_v_is_zero(mm))
            {
                /* 一次性提取各 lane，减少 jps_v_lane 调用频率 */
                uint64_t m0 = jps_v_lane(mm, 0);      /* 先低字(word w，更小列) */
                uint64_t m1 = jps_v_lane(mm, 1);      /* 再高字(word w+1) */
                uint64_t j0 = jps_v_lane(jump, 0);
                uint64_t j1 = jps_v_lane(jump, 1);

                if (m0 != 0ULL)
                {
                    int b = jps__lowest_set(m0);
                    *out_s = ((w << 6) + b) - x;
                    *out_jump = ((j0 >> b) & 1ULL) != 0ULL;
                    return;
                }
                else /* m1 must be non-zero */
                {
                    int b = jps__lowest_set(m1);
                    *out_s = (((w + 1) << 6) + b) - x;
                    *out_jump = ((j1 >> b) & 1ULL) != 0ULL;
                    return;
                }
            }

            /* 提取高字作为下一对的进位源，一次调用完成 */
            prev_up = jps_v_lane(walk_up, 1);         /* 本对高字(word w+1) → 下一对的进位源 */
            prev_dn = jps_v_lane(walk_dn, 1);
            w += 2;
            sub = jps_v_set2(~0ULL, ~0ULL);
        }
    }
    else
    {
        int start_col = x - 1;
        int w, sb;
        uint64_t sub0, next_up, next_dn;
        jps_v128 sub;
        if (start_col < 0)                            /* 左邻即越界 → 墙，步数 1 */
        {
            *out_s = 1;
            *out_jump = false;
            return;
        }
        w = start_col >> 6;                           /* 起始字（本对的高字） */
        sb = start_col & 63;
        sub0 = sb == 63 ? ~0ULL : ((1ULL << (sb + 1)) - 1);   /* 首字内只看 ≤ 起始位的列 */
        /* 进位源 = 后一字（w+1）的 y±1 行；仅首对需要，之后由本对低字滚动复用。 */
        next_up = jps_grid_map_walkable_word(m, w + 1, y - 1);
        next_dn = jps_grid_map_walkable_word(m, w + 1, y + 1);
        sub = jps_v_set2(~0ULL, sub0);               /* 仅首对的高字(word w)受 sub0 限制 */
        for (;;)
        {
            /* 本对 = (低字 = word w-1, 高字 = word w)，从高列向低列扫。 */
            jps_v128 walk_y  = jps_v_set2(jps_grid_map_walkable_word(m, w - 1, y),     jps_grid_map_walkable_word(m, w, y));
            jps_v128 walk_up = jps_v_set2(jps_grid_map_walkable_word(m, w - 1, y - 1), jps_grid_map_walkable_word(m, w, y - 1));
            jps_v128 walk_dn = jps_v_set2(jps_grid_map_walkable_word(m, w - 1, y + 1), jps_grid_map_walkable_word(m, w, y + 1));

            /* block(c+1) = (~walk >> 1)：跨字进位由整 128 位右移完成；高字进位 cin 来自后一字 bit0。 */
            jps_v128 blk_up = jps_v_shr1(jps_v_not(walk_up), (~next_up) & 1ULL);
            jps_v128 blk_dn = jps_v_shr1(jps_v_not(walk_dn), (~next_dn) & 1ULL);
            jps_v128 jump = jps_v_and(jps_v_or(jps_v_and(walk_up, blk_up), jps_v_and(walk_dn, blk_dn)), walk_y);
            jps_v128 mm = jps_v_and(jps_v_or(jps_v_not(walk_y), jump), sub);

            if (!jps_v_is_zero(mm))
            {
                /* 一次性提取各 lane，减少 jps_v_lane 调用频率 */
                uint64_t m0 = jps_v_lane(mm, 0); /* 低字(word w-1 或 w) */
                uint64_t m1 = jps_v_lane(mm, 1); /* 高字(word w 或 w+1) */
                uint64_t j0 = jps_v_lane(jump, 0);
                uint64_t j1 = jps_v_lane(jump, 1);

                if (m1 != 0ULL)
                {
                    int b = jps__highest_set(m1);
                    *out_s = x - ((w << 6) + b);
                    *out_jump = ((j1 >> b) & 1ULL) != 0ULL;
                    return;
                }
                else /* m0 must be non-zero */
                {
                    int b = jps__highest_set(m0);
                    *out_s = x - (((w - 1) << 6) + b);
                    *out_jump = ((j0 >> b) & 1ULL) != 0ULL;
                    return;
                }
            }

            /* 提取本对低字作为下一对进位源，一次调用完成 */
            next_up = jps_v_lane(walk_up, 0);         /* 本对低字(word w-1) → 下一对的进位源 */
            next_dn = jps_v_lane(walk_dn, 0);
            w -= 2;
            sub = jps_v_set2(~0ULL, ~0ULL);
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