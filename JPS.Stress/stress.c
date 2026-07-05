/*
 * stress.c
 * JPS Pathfinding — 独立的 C 压力测试（只测 JPS C 原生实现，不经 C# / P/Invoke）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 *
 * 流程：读一张 MovingAI .map → 生成一批随机可走起终点 → 读该地图的全部 .scen 用例 →
 *       把「随机 + scen」合成一组测试对，对每一对：
 *          ① 在干净图上寻一次做参考（校验路径合法）；
 *          ② 在小窗口里随机翻转若干格（不碰起终点）+ Sync，冷寻一次（校验在改后图上合法）；
 *          ③ 还原那些格 + Sync，再寻一次（校验合法，且结果与①逐点一致 = 失效/还原确定性）。
 *       全程单线程，只用 jps.h 的公共接口。
 *
 * 校验（无需外部参考，全自洽）：
 *   · 合法性：compact path 相邻点为直线/对角段、逐格可走、默认禁止斜穿角、首尾即起终点；
 *   · 确定性：改图→还原后，同一图上的结果必须和改动前逐点相同（纯函数不变量）。
 * 任一项失败即报告并以非 0 退出，适合塞进 CI / 长跑 fuzz。
 *
 * 用法：
 *   stress <map.map> [--rand N] [--seed S] [--reps R] [--scen FILE] [--no-scen] [-q]
 *     --rand N     随机起终点对数（默认 1000）
 *     --seed S     RNG 种子（默认 12345，便于复现）
 *     --reps R     整组测试对重复跑 R 遍（默认 1）
 *     --scen FILE  指定 scen 文件（默认 <map.map>.scen）
 *     --no-scen    不读 scen，只跑随机对
 *     -q           安静模式（不打进度）
 */

#define _CRT_SECURE_NO_WARNINGS   /* Windows UCRT：让 fopen/sscanf 不报 deprecated；非 Windows 无影响 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "jps.h"   /* 公共 API；本工程用 -DJPS_STATIC 与 JPS.Native 源码一起编译 */

/* ---------------- 小工具 ---------------- */

static uint64_t g_rng = 12345u;
static uint64_t rnd_next(void)
{
    /* xorshift64* — 可复现、够随机 */
    uint64_t x = g_rng;
    x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
    g_rng = x;
    return x * 0x2545F4914F6CDD1DULL;
}
static int rnd_range(int n) { return n <= 0 ? 0 : (int)(rnd_next() % (uint64_t)n); }
static int imin(int a, int b) { return a < b ? a : b; }
static int imax(int a, int b) { return a > b ? a : b; }
static int isign(int v) { return (v > 0) - (v < 0); }

/* 读整个文件到内存（\0 结尾）。失败返回 NULL。 */
static char *read_file(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz < 0) { fclose(f); return NULL; }
    char *buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    buf[rd] = '\0';
    return buf;
}

/* 就地取下一行：去掉 \r\n 并 \0 截断，返回行首；无更多行返回 NULL。 */
static char *next_line(char **cur)
{
    if (**cur == '\0') return NULL;
    char *start = *cur;
    char *nl = strchr(start, '\n');
    if (nl)
    {
        char *end = nl;
        if (end > start && end[-1] == '\r') end--;
        *end = '\0';
        *cur = nl + 1;
    }
    else
    {
        *cur = start + strlen(start);
    }
    return start;
}

/* ---------------- MovingAI .map 解析 ---------------- */

/* 解析 .map → cells（行主序，0=可走，1=阻挡）。可走地形：'.' 'G' 'S'（与 MovingAI 约定一致）。 */
static int parse_map(const char *path, int *out_w, int *out_h, uint8_t **out_cells)
{
    char *buf = read_file(path);
    if (!buf) return 0;

    char *cur = buf, *line;
    int w = 0, h = 0;
    while ((line = next_line(&cur)) != NULL)
    {
        if (strncmp(line, "height", 6) == 0) sscanf(line + 6, "%d", &h);
        else if (strncmp(line, "width", 5) == 0) sscanf(line + 5, "%d", &w);
        else if (strncmp(line, "map", 3) == 0) break;
    }
    if (w <= 0 || h <= 0) { free(buf); return 0; }

    uint8_t *cells = (uint8_t *)malloc((size_t)w * (size_t)h);
    if (!cells) { free(buf); return 0; }

    int y = 0;
    for (; y < h; y++)
    {
        line = next_line(&cur);
        if (!line) break;
        int len = (int)strlen(line);
        for (int x = 0; x < w; x++)
        {
            char c = x < len ? line[x] : '@';   /* 短行末尾按阻挡 */
            int walkable = (c == '.' || c == 'G' || c == 'S');
            cells[(size_t)y * w + x] = walkable ? 0u : 1u;
        }
    }
    free(buf);
    if (y != h) { free(cells); return 0; }
    *out_w = w; *out_h = h; *out_cells = cells;
    return 1;
}

/* ---------------- 测试对 ---------------- */

typedef struct { int sx, sy, gx, gy; } pair_t;

typedef struct { pair_t *v; int n, cap; } pairs_t;
static void pairs_push(pairs_t *p, int sx, int sy, int gx, int gy)
{
    if (p->n == p->cap)
    {
        p->cap = p->cap ? p->cap * 2 : 1024;
        p->v = (pair_t *)realloc(p->v, (size_t)p->cap * sizeof(pair_t));
    }
    p->v[p->n].sx = sx; p->v[p->n].sy = sy; p->v[p->n].gx = gx; p->v[p->n].gy = gy;
    p->n++;
}

/* 读 <map>.map.scen 的全部用例，过滤：界内 + 起终点均可走。返回加入的条数。 */
static int load_scen(const char *path, int w, int h, const uint8_t *cells, pairs_t *out)
{
    char *buf = read_file(path);
    if (!buf) return -1;   /* 文件不存在/读失败 */

    int added = 0;
    char *cur = buf, *line;
    while ((line = next_line(&cur)) != NULL)
    {
        int bucket, mw, mh, sx, sy, gx, gy;
        double opt;
        char name[256];
        /* bucket map w h sx sy gx gy optimal */
        if (sscanf(line, "%d %255s %d %d %d %d %d %d %lf",
                   &bucket, name, &mw, &mh, &sx, &sy, &gx, &gy, &opt) != 9)
            continue;
        if (sx < 0 || sy < 0 || gx < 0 || gy < 0) continue;
        if (sx >= w || sy >= h || gx >= w || gy >= h) continue;
        if (cells[(size_t)sy * w + sx] || cells[(size_t)gy * w + gx]) continue;
        pairs_push(out, sx, sy, gx, gy);
        added++;
    }
    free(buf);
    return added;
}

/* ---------------- compact path 合法性校验（对 system 当前地图状态） ---------------- */

/* 相邻两点为直线/对角段、逐格可走、默认禁止斜穿角、首尾即起终点。合法返回 1。 */
static int path_is_legal(jps_system *s, const int *xy, int n, int sx, int sy, int gx, int gy)
{
    if (n < 1) return 0;
    if (xy[0] != sx || xy[1] != sy) return 0;
    if (xy[(n - 1) * 2] != gx || xy[(n - 1) * 2 + 1] != gy) return 0;

    for (int i = 0; i + 1 < n; i++)
    {
        int ax = xy[i * 2], ay = xy[i * 2 + 1];
        int bx = xy[(i + 1) * 2], by = xy[(i + 1) * 2 + 1];
        int dx = isign(bx - ax), dy = isign(by - ay);
        int adx = abs(bx - ax), ady = abs(by - ay);
        if (adx == 0 && ady == 0) return 0;                 /* 重复点 */
        if (adx != 0 && ady != 0 && adx != ady) return 0;   /* 非直线/非对角 */

        int steps = imax(adx, ady);
        int cx = ax, cy = ay;
        for (int k = 0; k < steps; k++)
        {
#ifndef JPS_ALLOW_CORNER_CUTTING
            if (dx != 0 && dy != 0)   /* 对角：两侧共角格不得阻挡 */
            {
                if (jps_system_is_blocked(s, cx + dx, cy) || jps_system_is_blocked(s, cx, cy + dy))
                    return 0;
            }
#endif
            cx += dx; cy += dy;
            if (jps_system_is_blocked(s, cx, cy)) return 0;   /* 途经格被挡 */
        }
        if (cx != bx || cy != by) return 0;
    }
    return 1;
}

/* ---------------- 主流程 ---------------- */

typedef struct { long total, found, no_path, err, illegal, nondet; } stats_t;

/* 可复用的 compact-path 拷贝缓冲（按点数增长）。 */
typedef struct { int *v; int cap_points; } ibuf_t;
static int *ibuf_get(ibuf_t *b, int points)
{
    if (points > b->cap_points)
    {
        b->cap_points = points * 2 + 64;
        b->v = (int *)realloc(b->v, (size_t)b->cap_points * 2 * sizeof(int));
    }
    return b->v;
}

int main(int argc, char **argv)
{
    const char *map_path = NULL;
    const char *scen_path = NULL;
    int rand_n = 1000;
    int reps = 1;
    int use_scen = 1;
    int quiet = 0;
    uint64_t seed = 12345u;

    for (int i = 1; i < argc; i++)
    {
        const char *a = argv[i];
        if (a[0] != '-') { if (!map_path) map_path = a; else { fprintf(stderr, "多余参数：%s\n", a); return 2; } }
        else if (!strcmp(a, "--rand") && i + 1 < argc) rand_n = atoi(argv[++i]);
        else if (!strcmp(a, "--seed") && i + 1 < argc) seed = strtoull(argv[++i], NULL, 10);
        else if (!strcmp(a, "--reps") && i + 1 < argc) reps = atoi(argv[++i]);
        else if (!strcmp(a, "--scen") && i + 1 < argc) scen_path = argv[++i];
        else if (!strcmp(a, "--no-scen")) use_scen = 0;
        else if (!strcmp(a, "-q")) quiet = 1;
        else if (!strcmp(a, "-h") || !strcmp(a, "--help"))
        {
            printf("用法: %s <map.map> [--rand N] [--seed S] [--reps R] [--scen FILE] [--no-scen] [-q]\n", argv[0]);
            return 0;
        }
        else { fprintf(stderr, "未知参数：%s\n", a); return 2; }
    }
    if (!map_path) { fprintf(stderr, "错误：需要一个 .map 文件。用法见 -h。\n"); return 2; }
    if (rand_n < 0) rand_n = 0;
    if (reps < 1) reps = 1;
    g_rng = seed ? seed : 1u;

    /* ---- 读地图 ---- */
    int w, h;
    uint8_t *cells;
    if (!parse_map(map_path, &w, &h, &cells))
    {
        fprintf(stderr, "错误：无法解析地图 %s\n", map_path);
        return 1;
    }

    /* 可走格清单（供随机取点） */
    long total_cells = (long)w * h;
    int *walk = (int *)malloc((size_t)total_cells * sizeof(int));
    int walk_n = 0;
    for (long i = 0; i < total_cells; i++)
        if (!cells[i]) walk[walk_n++] = (int)i;
    if (walk_n < 2)
    {
        fprintf(stderr, "错误：可走格不足（%d）。\n", walk_n);
        free(cells); free(walk);
        return 1;
    }

    /* ---- 组测试对：随机 + scen ---- */
    pairs_t pairs = {0};
    for (int i = 0; i < rand_n; i++)
    {
        int a = walk[rnd_range(walk_n)];
        int b = walk[rnd_range(walk_n)];
        if (a == b) { i--; continue; }
        pairs_push(&pairs, a % w, a / w, b % w, b / w);
    }
    int rand_added = pairs.n;

    int scen_added = 0;
    char scen_auto[1024];
    if (use_scen)
    {
        if (!scen_path)
        {
            snprintf(scen_auto, sizeof scen_auto, "%s.scen", map_path);   /* MovingAI 约定 <map>.map.scen */
            scen_path = scen_auto;
        }
        int r = load_scen(scen_path, w, h, cells, &pairs);
        if (r < 0) fprintf(stderr, "提示：读不到 scen 文件 %s，只跑随机对。\n", scen_path);
        else scen_added = r;
    }

    if (pairs.n == 0)
    {
        fprintf(stderr, "错误：没有可跑的测试对。\n");
        free(cells); free(walk); free(pairs.v);
        return 1;
    }

    /* ---- 建 system + 装图 + pathfinder ---- */
    jps_system *sys = jps_system_create(w, h);
    if (!sys) { fprintf(stderr, "错误：jps_system_create 失败。\n"); return 1; }
    jps_system_set_blocked_buffer(sys, cells, (int)total_cells);
    jps_system_sync(sys);
    jps_pathfinder *pf = jps_pathfinder_create();
    if (!pf) { fprintf(stderr, "错误：jps_pathfinder_create 失败。\n"); return 1; }

    /* 冷编辑参数（与 benchmark 一致） */
    int window = imin(16, imin(w, h));
    int edit_cap = imin(24, imax(1, window * window / 4));
    int *ex = (int *)malloc((size_t)edit_cap * sizeof(int));
    int *ey = (int *)malloc((size_t)edit_cap * sizeof(int));
    uint8_t *eold = (uint8_t *)malloc((size_t)edit_cap);
    int *xyv = (int *)malloc((size_t)edit_cap * 3 * sizeof(int));   /* set_blocked_batch 的 (x,y,blocked) 三元组 */

    ibuf_t ref = {0}, work = {0};

    printf("stress: map=%s (%dx%d, walkable=%d)  rand=%d  scen=%d  pairs=%d  reps=%d  seed=%llu\n",
           map_path, w, h, walk_n, rand_added, scen_added, pairs.n, reps, (unsigned long long)seed);
    fflush(stdout);

    stats_t st = {0};
    clock_t t0 = clock();

    for (int rep = 0; rep < reps; rep++)
    {
        for (int pi = 0; pi < pairs.n; pi++)
        {
            pair_t p = pairs.v[pi];
            st.total++;

            /* ① 干净图参考寻路 */
            int rn = jps_pathfinder_find_path(pf, sys, p.sx, p.sy, p.gx, p.gy);
            int ref_n = 0;
            if (rn == JPS_ERR_NO_PATH) st.no_path++;
            else if (rn < 0) { st.err++; }
            else
            {
                st.found++;
                ref_n = rn;
                int *rb = ibuf_get(&ref, ref_n);
                jps_pathfinder_copy_path(pf, rb, ref_n);
                if (!path_is_legal(sys, rb, ref_n, p.sx, p.sy, p.gx, p.gy)) st.illegal++;
                /* 顺带跑一下平滑访问器（find 内部已算，这里只取，练一下公共接口） */
                int sn = jps_pathfinder_smoothed_path_count(pf);
                if (sn > 0) { int *wb = ibuf_get(&work, sn); jps_pathfinder_copy_smoothed_path(pf, (float *)wb, imin(sn, work.cap_points)); }
            }

            /* ② 小窗口随机翻转若干格（避开起终点），Sync，冷寻，校验（对改后图） */
            int ecount = 0;
            int ox = rnd_range(w - window + 1);
            int oy = rnd_range(h - window + 1);
            int attempts = edit_cap * 20 + 200;
            for (int a = 0; a < attempts && ecount < edit_cap; a++)
            {
                int x = ox + rnd_range(window);
                int y = oy + rnd_range(window);
                if ((x == p.sx && y == p.sy) || (x == p.gx && y == p.gy)) continue;
                int dup = 0;
                for (int j = 0; j < ecount; j++) if (ex[j] == x && ey[j] == y) { dup = 1; break; }
                if (dup) continue;
                ex[ecount] = x; ey[ecount] = y;
                eold[ecount] = (uint8_t)(jps_system_is_blocked(sys, x, y) ? 1 : 0);
                ecount++;
            }
            for (int j = 0; j < ecount; j++)
            {
                xyv[j * 3] = ex[j]; xyv[j * 3 + 1] = ey[j]; xyv[j * 3 + 2] = eold[j] ? 0 : 1;   /* 翻转 */
            }
            jps_system_set_blocked_batch(sys, xyv, ecount);
            jps_system_sync(sys);

            int en = jps_pathfinder_find_path(pf, sys, p.sx, p.sy, p.gx, p.gy);
            if (en >= 1)
            {
                int *wb = ibuf_get(&work, en);
                jps_pathfinder_copy_path(pf, wb, en);
                if (!path_is_legal(sys, wb, en, p.sx, p.sy, p.gx, p.gy)) st.illegal++;
            }
            else if (en < 0 && en != JPS_ERR_NO_PATH) st.err++;

            /* ③ 还原，Sync，再寻，校验合法 + 与①逐点一致（失效/还原确定性） */
            for (int j = 0; j < ecount; j++)
            {
                xyv[j * 3] = ex[j]; xyv[j * 3 + 1] = ey[j]; xyv[j * 3 + 2] = eold[j];   /* 还原 */
            }
            jps_system_set_blocked_batch(sys, xyv, ecount);
            jps_system_sync(sys);

            int r2 = jps_pathfinder_find_path(pf, sys, p.sx, p.sy, p.gx, p.gy);
            if (r2 >= 1)
            {
                int *wb = ibuf_get(&work, r2);
                jps_pathfinder_copy_path(pf, wb, r2);
                if (!path_is_legal(sys, wb, r2, p.sx, p.sy, p.gx, p.gy)) st.illegal++;
                /* 确定性：与参考逐点一致（纯函数，churn 后必须不变） */
                if (rn >= 1)
                {
                    int *rb = ref.v;
                    if (r2 != ref_n || memcmp(wb, rb, (size_t)r2 * 2 * sizeof(int)) != 0) st.nondet++;
                }
            }
            else if (rn >= 1) st.nondet++;   /* 之前有路，还原后却没路 → 确定性坏了 */
            else if (r2 < 0 && r2 != JPS_ERR_NO_PATH) st.err++;

            if (!quiet && (st.total % 2000 == 0))
            {
                printf("\r  [%ld/%ld] found=%ld nopath=%ld illegal=%ld nondet=%ld err=%ld",
                       st.total, (long)pairs.n * reps, st.found, st.no_path, st.illegal, st.nondet, st.err);
                fflush(stdout);
            }
        }
    }

    double secs = (double)(clock() - t0) / CLOCKS_PER_SEC;
    if (!quiet) printf("\r");
    printf("done: pairs=%ld  found=%ld  no_path=%ld  illegal=%ld  nondet=%ld  err=%ld  (%.2fs, %.0f finds/s)\n",
           st.total, st.found, st.no_path, st.illegal, st.nondet, st.err,
           secs, secs > 0 ? (st.total * 3.0) / secs : 0.0);

    int bad = (st.illegal != 0) || (st.nondet != 0) || (st.err != 0);
    printf(bad ? "RESULT: FAIL ✗ (illegal/nondet/err 非零)\n" : "RESULT: PASS ✓\n");

    jps_pathfinder_destroy(pf);
    jps_system_destroy(sys);
    free(cells); free(walk); free(pairs.v);
    free(ex); free(ey); free(eold); free(xyv);
    free(ref.v); free(work.v);
    return bad ? 1 : 0;
}
