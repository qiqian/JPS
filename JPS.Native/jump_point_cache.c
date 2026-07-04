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
 * 参数化位图访问器：bits(数据(0,0))/pstride(物理线宽) 描述一份按“线(line)”排布的位图，扫描沿“跨向(across)”推进：
 *   行排布(原图)：line=y、across=x、pstride=行物理 stride —— 用于水平扫描；
 *   列排布(转置) ：line=x、across=y、pstride=列物理 stride —— 用于垂直扫描。
 * 于是“垂直扫描”= 在转置位图上做同一套“水平扫描”，两者共用下面这份 SIMD 代码。
 * 越界（line-1/n_lines、w2 超界）由 GridMap 哨兵带兜底（恒阻挡），访问器不判界。
 */

/* 哨兵版：直接对齐读第 line 条线第 w2 个 128 位单元(words w2,w2+1)的可走位（取反），**不判界**——
 * 越界访问由 GridMap 的哨兵带兜底（恒阻挡）。有符号下标，允许 line=-1/w2=-2 落到负偏移。
 * 前提：调用方保证 line∈[-1,n_lines]、w2∈[-2,data_stride]（哨兵带覆盖范围）；pstride 偶 → 单元 16 字节对齐。 */
static inline jps_v128 jps__walk_v128(const uint64_t *bits, int pstride, int line, int w2)
{
    return jps_v_not(jps_v_load(&bits[(ptrdiff_t)line * pstride + w2]));
}

/* 哨兵版单字读（取反），用于跨单元进位；越界由哨兵带兜底，不判界。 */
static inline uint64_t jps__walk_word(const uint64_t *bits, int pstride, int line, int word_col)
{
    return ~bits[(ptrdiff_t)line * pstride + word_col];
}

/* 保留 bit 0..k（k∈0..63）的低位掩码。 */
static inline uint64_t jps__mask_le(int k)
{
    return k >= 63 ? ~0ULL : ((1ULL << (k + 1)) - 1);
}

/*
 * 沿第 line 条线的 across 方向 dir∈{+1,-1} 扫描最近“停点”（跳点 or 墙），语义与逐格扫描一致。
 * bits 指向数据(0,0)、pstride 为物理线宽；行排布或列排布共用同一套代码。越界终止靠 GridMap 哨兵带兜底
 * （line-1/n_lines、w2 越界都落到全阻挡哨兵内存），故内层无任何边界分支。
 * pos 为当前 across 坐标（行扫=x，列扫=y）；返回步数 *out_s 与是否跳点 *out_jump。
 */
static void jps__scan_line(const uint64_t *bits, int pstride,
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
        uint64_t prev_up = jps__walk_word(bits, pstride, line - 1, unit - 1);
        uint64_t prev_dn = jps__walk_word(bits, pstride, line + 1, unit - 1);
        for (;;)
        {
            jps_v128 walk_y  = jps__walk_v128(bits, pstride, line,     unit);
            jps_v128 walk_up = jps__walk_v128(bits, pstride, line - 1, unit);
            jps_v128 walk_dn = jps__walk_v128(bits, pstride, line + 1, unit);

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
        next_up = jps__walk_word(bits, pstride, line - 1, unit + 2);
        next_dn = jps__walk_word(bits, pstride, line + 1, unit + 2);
        for (;;)
        {
            jps_v128 walk_y  = jps__walk_v128(bits, pstride, line,     unit);
            jps_v128 walk_up = jps__walk_v128(bits, pstride, line - 1, unit);
            jps_v128 walk_dn = jps__walk_v128(bits, pstride, line + 1, unit);

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
    c->h = 0;
    c->size = 0;
    c->dist = NULL;
    c->gen = NULL;
    c->row_gen = NULL;
    c->col_gen = NULL;
    c->row_version = NULL;
    c->col_version = NULL;
    c->map_version = -1;
    return c;
}

void jps_jump_point_cache_destroy(jps_jump_point_cache *c)
{
    if (c == NULL)
        return;
    free(c->dist);
    free(c->gen);
    free(c->row_gen);
    free(c->col_gen);
    free(c->row_version);
    free(c->col_version);
    free(c);
}

/* 世代回绕(到 uint8 上限 255)时把该行 E/W 两 gen 平面对应行整段清零(→0，必与复位后的 1 失配→全 dirty)。
 * 行在每个 gen 平面内连续，两次 memset 即可（dist 无需动，靠 gen 失配即判 dirty）。 */
static void jps__cache_invalidate_row(jps_jump_point_cache *c, int y)
{
    if (c->row_gen[y] >= 255)
    {
        size_t rowoff = (size_t)y * c->w;
        memset(c->gen + (size_t)0 * c->size + rowoff, 0, (size_t)c->w);   /* dir 0 = E */
        memset(c->gen + (size_t)1 * c->size + rowoff, 0, (size_t)c->w);   /* dir 1 = W */
        c->row_gen[y] = 1;
    }
    else
    {
        c->row_gen[y]++;
    }
}

/* 列在 gen 平面内跨步 w，非连续，逐格清零 S/N 两平面。 */
static void jps__cache_invalidate_col(jps_jump_point_cache *c, int x)
{
    if (c->col_gen[x] >= 255)
    {
        uint8_t *g2 = c->gen + (size_t)2 * c->size;   /* dir 2 = S */
        uint8_t *g3 = c->gen + (size_t)3 * c->size;   /* dir 3 = N */
        int y;
        for (y = 0; y < c->h; y++)
        {
            size_t idx = (size_t)y * c->w + x;
            g2[idx] = 0;
            g3[idx] = 0;
        }
        c->col_gen[x] = 1;
    }
    else
    {
        c->col_gen[x]++;
    }
}

void jps_jump_point_cache_sync(jps_jump_point_cache *c, const jps_grid_map *m)
{
    int i;

    if (c->w != m->width || c->h != m->height || c->size != m->width * m->height)
    {
        c->w = m->width;
        c->h = m->height;
        c->size = m->width * m->height;
        free(c->dist);
        free(c->gen);
        free(c->row_gen);
        free(c->col_gen);
        free(c->row_version);
        free(c->col_version);
        c->dist = (int16_t *)calloc((size_t)4 * c->size, sizeof(int16_t));   /* 4 个方向 dist 平面 */
        c->gen = (uint8_t *)calloc((size_t)4 * c->size, sizeof(uint8_t));    /* 4 个方向 gen 平面 */
        c->row_gen = (uint8_t *)calloc((size_t)c->h, sizeof(uint8_t));
        c->col_gen = (uint8_t *)calloc((size_t)c->w, sizeof(uint8_t));
        c->row_version = (int *)malloc((size_t)c->h * sizeof(int));
        c->col_version = (int *)malloc((size_t)c->w * sizeof(int));
        /* -1 哨兵：与地图侧初始版本 0 必不相等 → 首次 Sync 把每行/列都失效一遍，
         * 令所有 cell.gen(=0) 与行/列世代(→1) 失配，杜绝"未触碰的行读到 dist=0 垃圾"。 */
        for (i = 0; i < c->h; i++)
            c->row_version[i] = -1;
        for (i = 0; i < c->w; i++)
            c->col_version[i] = -1;
        c->map_version = -1;
    }

    if (c->map_version != m->version)
    {
        for (i = 0; i < c->h; i++)
        {
            if (c->row_version[i] != m->row_version[i])
            {
                jps__cache_invalidate_row(c, i);
                c->row_version[i] = m->row_version[i];
            }
        }
        for (i = 0; i < c->w; i++)
        {
            if (c->col_version[i] != m->col_version[i])
            {
                jps__cache_invalidate_col(c, i);
                c->col_version[i] = m->col_version[i];
            }
        }
        c->map_version = m->version;
    }
}

/*
 * 水平回写 dist：把等差数列 dist(t) = a + b*t（t=0..s-1，b=±1）写入连续地址 dst[0..s-1]。
 * 一段连续 int16 → 用 16 位车道 SIMD 一次生成并写 8 个；尾部不足 8 个标量补齐。
 * 只写 dist（普通写，非发布）；gen 的 release 发布由调用方另做，语义与旧逐格版一致。
 */
static void jps__backfill_dist_run(int16_t *dst, int s, int a, int b)
{
    int t = 0;
#ifdef JPS_HAVE_SIMD
    if (s >= 8)
    {
        jps_v128 vramp = b > 0 ? jps_v_setr_i16(0, 1, 2, 3, 4, 5, 6, 7)
                               : jps_v_setr_i16(0, -1, -2, -3, -4, -5, -6, -7);
        jps_v128 vstep = jps_v_set1_i16((int16_t)(8 * b));                  /* 每组 t 前进 8，dist 变 8*b */
        jps_v128 vcur  = jps_v_add_i16(jps_v_set1_i16((int16_t)a), vramp);  /* [a, a+b, ..., a+7b] */
        int groups = s & ~7;
        for (; t < groups; t += 8)
        {
            jps_v_storeu_i16(dst + t, vcur);
            vcur = jps_v_add_i16(vcur, vstep);
        }
    }
#endif
    for (; t < s; t++)
        dst[t] = (int16_t)(a + b * t);
}

int jps_jump_point_cache_cardinal_dist(jps_jump_point_cache *c, const jps_grid_map *m,
                                       int x, int y, int dx, int dy, int dir)
{
    int idx0 = y * c->w + x;
    int16_t *distp = c->dist + (size_t)dir * c->size;             /* 该方向的 dist 平面 */
    uint8_t *genp = c->gen + (size_t)dir * c->size;               /* 该方向的 gen 平面 */
    uint8_t line_gen = dy == 0 ? c->row_gen[y] : c->col_gen[x];   /* E/W → 行世代；S/N → 列世代 */
    int s, t;
    bool jump_found;

    /* acquire 读世代戳：若看到 clean，则发布它的那次 release 写之前的 dist 写均已可见，普通读 dist 即安全。 */
    if (jps_gen_load_acquire(&genp[idx0]) == line_gen)
        return distp[idx0];

    /* 扫描：从 (x,y) 沿方向找最近跳点或墙。水平走行排布，垂直走列排布(转置) —— 两者共用 SIMD。 */
    if (dy == 0)
        jps__scan_line(m->blocked, m->stride, y, x, dx, &s, &jump_found);
    else
        jps__scan_line(m->col_blocked, m->col_stride, x, y, dy, &s, &jump_found);

    /* 回填整段 run（步 k=0..s-1 的可走格）。距离量级 ≤ max(W,H) ≤ INT16_MAX。
     * 先写 dist（水平段可 SIMD），再逐格 release-store gen 发布——所有 dist 写在任何 gen 发布之前，
     * 故读者见某格 gen==line_gen 时其 dist 必已可见，acquire/release 语义与旧逐格版一致。 */
    if (dy == 0)
    {
        /* 水平：整段 run 在平面内连续。dist(k) 换算成按“升序地址位置 t”的等差 a + b*t：
         * E(dx>0) 地址随步 k 升，t=k；W(dx<0) 地址随 k 降，升序位置 t=(s-1)-k。 */
        int block_start, a, b;
        if (dx > 0)
        {
            block_start = idx0;
            a = jump_found ? s : -(s - 1);
            b = jump_found ? -1 : 1;
        }
        else
        {
            block_start = idx0 - (s - 1);
            a = jump_found ? 1 : 0;
            b = jump_found ? 1 : -1;
        }
        jps__backfill_dist_run(distp + block_start, s, a, b);
        for (t = 0; t < s; t++)
            jps_gen_store_release(&genp[block_start + t], line_gen);
    }
    else
    {
        /* 垂直：沿列跨步 w，地址非连续 → 标量逐格回填。 */
        int fy = y, k;
        for (k = 0; k <= s - 1; k++)
        {
            size_t ci = (size_t)fy * c->w + x;
            distp[ci] = (int16_t)(jump_found ? (s - k) : -((s - 1) - k));   /* 先普通写 dist */
            jps_gen_store_release(&genp[ci], line_gen);                      /* 再 release 发布该格 */
            fy += dy;
        }
    }

    return jump_found ? s : -(s - 1);   /* idx0(k=0) 处的 dist；本线程刚写，直接返回 */
}
