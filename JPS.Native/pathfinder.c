/*
 * pathfinder.c
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JpsPathfinder.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include <string.h>
#include "jps.h"
#include "smoother.h"
#include "min_heap.h"
#include "directions.h"
#include "rules.h"
#include "system.h"

_Static_assert(sizeof(jps_point_f) <= sizeof(uint64_t), "g_dir slot must hold one smoothed point");

 /* 索引顺序与 C# JpsDirections.All 严格一致。 */
static const int jps_dir_dx[JPS_DIR_COUNT] = { 1, -1, 0, 0, 1, -1, 1, -1 };
static const int jps_dir_dy[JPS_DIR_COUNT] = { 0, 0, 1, -1, 1, 1, -1, -1 };

/*
 * 一次“跳跃”的结果：从某格沿某方向跳到的目标格。
 * has_jump=false 表示该方向跳不到任何跳点/终点（撞墙）。
 */
typedef struct
{
    bool has_jump;
    int x;
    int y;
    int steps;   /* 跨越的格数（用于按 步数×单格代价 累计 g 值） */
} jps__jump_entry;

static const jps__jump_entry JPS__JUMP_NONE = { false, 0, 0, 0 };


/*
 * 寻路结果（纯内部状态，不进公共头）：path 是 compact path 动态数组，
 * path_count 为起点、跳点/拐点与终点的点数。
 * 缓冲可跨次查询复用。对外只通过 copy/count 等访问器暴露，不跨 ABI 传结构体。
 */
typedef struct
{
    bool success;
    jps_point *path;
    int path_count;
    int path_capacity;
    int expanded_nodes;
} jps_path_result;

/*
 * 逐节点搜索状态：按**访问频率**拆存（SoA），不按节点合并（AoS）——
 * mark 是被高频单独访问的（堆过期检查/邻居 closed 检查只碰它），保持独立密集数组：
 * uint16 一条 cache line 装 32 个节点的 mark；与冷字段合并会把最热数组稀释 3–4 倍（实测回退）。
 *
 * g、steps、parent_dir 打包进同一个 uint64（g_dir），取代原先独立的 steps 数组：
 *   位 [0,44)  = g 值：迷宫最优路径可途经 ~W×H 格，g 上限 ~1.5e12 < 2^41 ≪ 2^44，44 位足够；
 *   位 [44,60) = steps 到达该节点的跳跃步数（≤ max(W,H) ≤ 32767 < 2^16），父节点 = 当前 − dir×steps；
 *   位 [60,64) = parent_dir 到达方向（0..7）。
 *   · 展开读 g+dir、relax 写 g+steps+dir 本来就要摸这条 line，steps 塞进同字等于免费搭车——
 *     独立 steps 数组整个消失，relax 少一次 store，逐节点搜索态 12→10 B/格；
 *   · 哨兵：无父存 0xF 于最高 4 位，(int64_t)g_dir>>60 算术右移读出即 -1，parent_dir<0 判据照旧；
 *   · g_dir 自然 8 对齐、8 节点/line，无跨线、无非对齐访问。
 */
#define JPS__G_MASK      ((1ULL << 44) - 1)   /* g 的低 44 位 */
#define JPS__STEPS_SHIFT 44
#define JPS__STEPS_MASK  0xFFFFULL             /* steps 的 16 位 */
#define JPS__DIR_SHIFT   60

#define JPS__NO_DIR ((uint8_t)0xFu)   /* parent_dir 哨兵：无父（放最高 4 位，算术右移读出 -1） */

static inline int64_t jps__gd_g(uint64_t gd) { return (int64_t)(gd & JPS__G_MASK); }

static inline int jps__gd_steps(uint64_t gd) { return (int)((gd >> JPS__STEPS_SHIFT) & JPS__STEPS_MASK); }

static inline int8_t jps__gd_dir(uint64_t gd) { return (int8_t)((int64_t)gd >> JPS__DIR_SHIFT); }

static inline uint64_t jps__pack_gdir(int64_t g, int steps, uint8_t dir)
{
    return ((uint64_t)g & JPS__G_MASK)
         | (((uint64_t)steps & JPS__STEPS_MASK) << JPS__STEPS_SHIFT)
         | ((uint64_t)dir << JPS__DIR_SHIFT);
}

struct jps_pathfinder
{
    /* ---- 按地图尺寸一次性分配、跨多次查询复用的缓冲区 ---- */
    int w, h, size;
    uint64_t *g_dir;     /* 每节点：g + steps + 到达方向打包同字（见上方布局说明） */
    /*
     * 查询纪元（epoch）标记法——免清零地跨查询复用 mark 数组：
     *   mark == 2·epoch → 本次查询已入 open；== 2·epoch+1 → 已 closed；< 2·epoch → 未访问。
     *   epoch 每次 find_path 自增；mark 为 uint16 → 需 2·epoch+1 ≤ 65535，
     *   故 epoch 在 1..32767 循环，回绕时 memset 整个 mark（每 ~3.3 万次查询一次，摊薄可忽略）。
     *   0 是保留值：epoch 从 1 起、mark 写入值 ∈ 2..65535，mark==0（新 calloc/回绕后）恒为未访问。
     *
     * ⚠️ 与跳点缓存的行/列世代（jump_point_cache.h 的 row_gen/col_gen/gen 平面）是**两套独立机制**：
     *   那套随「地图改动」推进、跨查询存活，判定缓存条目失效；
     *   这套随「每次查询」推进、只属于本 pathfinder，判定节点访问状态。二者互不作用。
     */
    uint16_t *mark;      /* 访问状态（独立密集数组，32 节点/cache line） */
    int epoch;           /* 查询纪元，1..32767 循环（见上） */
    jps_min_heap open;
    int dir_buf[JPS_DIR_COUNT];

    jps_jump_point_cache *cache;   /* 当前查询绑定的共享跳点缓存 */
    jps_path_result result;        /* 最近一次寻路结果（供 copy/count 访问器读取） */

    int *rebuild_nodes;            /* 路径重建用父链节点栈，跨查询复用，避免每次 malloc/free */
    int rebuild_nodes_capacity;

    /* 平滑路径缓存：find_path 成功后复用 g_dir 内存窗口，copy_smoothed_path 直接拷。 */
    const jps_grid_map *smooth_map;   /* 寻路所用地图，供平滑 LOS 使用 */
    jps_point_f *smoothed;            /* g_dir 的别名，不拥有内存；搜索结束后才可覆盖 */
    int smoothed_count;
    int smoothed_capacity;
    bool smoothed_valid;              /* 本次寻路结果的平滑是否已算 */
};
typedef struct jps_pathfinder jps_pathfinder;

static void jps__ensure_smoothed(jps_pathfinder *pf);

/* ---------------- PathResult（内部） ---------------- */

static void jps__result_init(jps_path_result *r)
{
    r->success = false;
    r->path = NULL;
    r->path_count = 0;
    r->path_capacity = 0;
    r->expanded_nodes = 0;
}

static void jps__result_free(jps_path_result *r)
{
    free(r->path);
    r->path = NULL;
    r->path_count = 0;
    r->path_capacity = 0;
}

static void jps__result_reset(jps_path_result *r)
{
    r->success = false;
    r->path_count = 0;
    r->expanded_nodes = 0;
}

static void jps__ensure_path_capacity(jps_path_result *r, int count)
{
    int n;
    if (count <= r->path_capacity)
        return;

    n = r->path_capacity < 16 ? 16 : r->path_capacity * 2;
    while (n < count)
        n *= 2;
    r->path = (jps_point *)realloc(r->path, (size_t)n * sizeof(jps_point));
    r->path_capacity = n;
}

/* ---------------- 生命周期 ---------------- */

jps_pathfinder *jps_pathfinder_create(void)
{
    jps_pathfinder *pf = (jps_pathfinder *)malloc(sizeof(jps_pathfinder));
    if (pf == NULL)
        return NULL;
    pf->w = pf->h = pf->size = 0;
    pf->g_dir = NULL;
    pf->mark = NULL;
    pf->epoch = 0;
    jps_min_heap_init(&pf->open, 64);
    pf->cache = NULL;
    jps__result_init(&pf->result);
    pf->rebuild_nodes = NULL;
    pf->rebuild_nodes_capacity = 0;
    pf->smooth_map = NULL;
    pf->smoothed = NULL;
    pf->smoothed_count = 0;
    pf->smoothed_capacity = 0;
    pf->smoothed_valid = false;
    return pf;
}

void jps_pathfinder_destroy(jps_pathfinder *pf)
{
    if (pf == NULL)
        return;
    free(pf->g_dir);
    free(pf->mark);
    free(pf->rebuild_nodes);
    jps_min_heap_free(&pf->open);
    jps__result_free(&pf->result);
    free(pf);
}

uint64_t jps_pathfinder_memory_bytes(const jps_pathfinder *pf)
{
    uint64_t bytes;

    if (pf == NULL)
        return 0;

    bytes = (uint64_t)sizeof(*pf);
    if (pf->g_dir != NULL)
        bytes += (uint64_t)((size_t)pf->size * sizeof(uint64_t));
    if (pf->mark != NULL)
        bytes += (uint64_t)((size_t)pf->size * sizeof(uint16_t));
    if (pf->open.elem != NULL)
        bytes += (uint64_t)((size_t)pf->open.capacity * sizeof(int));
    if (pf->open.prio != NULL)
        bytes += (uint64_t)((size_t)pf->open.capacity * sizeof(int64_t));
    if (pf->result.path != NULL)
        bytes += (uint64_t)((size_t)pf->result.path_capacity * sizeof(jps_point));
    if (pf->rebuild_nodes != NULL)
        bytes += (uint64_t)((size_t)pf->rebuild_nodes_capacity * sizeof(int));
    return bytes;
}

static void jps__ensure_buffers(jps_pathfinder *pf, const jps_grid_map *m)
{
    if (pf->w == m->width && pf->size == m->width * m->height)
        return;

    pf->w = m->width;
    pf->h = m->height;
    pf->size = pf->w * pf->h;
    free(pf->g_dir);
    free(pf->mark);
    pf->smoothed = NULL;
    pf->smoothed_count = 0;
    pf->smoothed_capacity = 0;
    pf->smoothed_valid = false;
    pf->g_dir = (uint64_t *)malloc((size_t)pf->size * sizeof(uint64_t));
    /* mark 必须清零（calloc），使纪元标记方案（mark < 2·epoch 即未访问）对新缓冲成立；
     * g_dir 仅在本纪元 mark 命中后才被读取，无需清零。 */
    pf->mark = (uint16_t *)calloc((size_t)pf->size, sizeof(uint16_t));
    pf->epoch = 0;
}

static void jps__next_epoch(jps_pathfinder *pf)
{
    pf->epoch++;
    if (pf->epoch <= 32767)   /* mark 为 uint16：需 2·epoch+1 ≤ 65535 */
        return;
    /* 纪元回绕：清零 mark 即可——g_dir 仅在本纪元 mark 命中后才被读取。 */
    memset(pf->mark, 0, (size_t)pf->size * sizeof(uint16_t));
    pf->epoch = 1;
}

static inline int jps__id(const jps_pathfinder *pf, int x, int y) { return y * pf->w + x; }

/* 堆载荷用打包坐标 (y<<16)|x（边长 ≤ 32767 → x,y 各占 16 位，值 < 2^31 落在 int 正区间）。
 * 出队后用移位取回 (x,y)，免去 current%w / current/w 对运行期除数的真除法（div ≈ 20–40 周期，
 * 乘法还原 id 仅 ~3 周期）。堆只按 prio 排序、不比较载荷，故出队顺序与旧 id 编码逐位一致——
 * 不改变展开顺序，C≡C# 强一致不受影响。 */
static inline int jps__pack_xy(int x, int y) { return (y << 16) | x; }

/* ---------------- 正交跳跃 ---------------- */

static jps__jump_entry jps__cardinal_jump(jps_pathfinder *pf, const jps_grid_map *m,
                                          int x, int y, int dx, int dy, int dir, int gx, int gy)
{
    /* 先内联快探 clean 命中（一次 acquire 字节读 + int16 读），miss 才走完整慢路——
     * 与 jps__diagonal_jump 的正交子探测同构，省掉命中时的函数调用与平面基址/世代解算。 */
    const jps_jump_point_cache *c = pf->cache;
    uint8_t line_gen = dy == 0 ? c->row_gen[y] : c->col_gen[x];
    int idx = dy == 0 ? y * c->w + x : x * c->h + y;
    int dist, max_travel;
    if (!jps_jump_probe(c->dist + (size_t)dir * c->size, c->gen + (size_t)dir * c->size,
                        idx, line_gen, &dist))
        dist = jps_jump_point_cache_cardinal_dist(pf->cache, m, x, y, dx, dy, dir);
    max_travel = dist > 0 ? dist : -dist;

    /* 终点正好在这条射线上且可达 → 直接拦截 */
    bool goal_on_ray =
        (dy == 0 && gy == y && jps_sign(gx - x) == dx) ||
        (dx == 0 && gx == x && jps_sign(gy - y) == dy);
    if (goal_on_ray)
    {
        int dist_to_goal = dx != 0 ? abs(gx - x) : abs(gy - y);
        if (dist_to_goal <= max_travel)
        {
            jps__jump_entry e = { true, gx, gy, dist_to_goal };
            return e;
        }
    }

    if (dist > 0)
    {
        jps__jump_entry e = { true, x + dx * dist, y + dy * dist, dist };
        return e;
    }
    return JPS__JUMP_NONE;
}

/* ---------------- 对角：经典逐格扫描，复用正交 memo ---------------- */

static jps__jump_entry jps__diagonal_jump(jps_pathfinder *pf, const jps_grid_map *m,
                                          int x, int y, int dx, int dy, int gx, int gy)
{
    int cx = x, cy = y, steps = 0;
    int horizontal_dir = jps_dir_index_of(dx, 0);
    int vertical_dir = jps_dir_index_of(0, dy);

    /* 对角每步要探两条正交线的 memo，热路（缓存 clean）命中率极高。把两方向的
     * dist/gen 平面基址与行/列世代数组在循环外解出，循环内先内联快探（一次 acquire
     * 字节读 + 一次 int16 读），miss 才走完整慢路（扫描+回填）——省去每步两次函数
     * 调用与平面基址/世代的重复解算。 */
    const jps_jump_point_cache *c = pf->cache;
    const int16_t *dist_h = c->dist + (size_t)horizontal_dir * c->size;
    const uint8_t *gen_h  = c->gen  + (size_t)horizontal_dir * c->size;
    const int16_t *dist_v = c->dist + (size_t)vertical_dir * c->size;
    const uint8_t *gen_v  = c->gen  + (size_t)vertical_dir * c->size;

    for (;;)
    {
        int hd, vd, idx_h, idx_v;

        /* 默认禁止斜穿角：斜走一步需目标格 + 两侧正交格都可走 */
        if (!jps_diagonal_allowed(m, cx, cy, dx, dy))
            return JPS__JUMP_NONE;

        cx += dx;
        cy += dy;
        steps++;

        if (cx == gx && cy == gy)
        {
            jps__jump_entry e = { true, cx, cy, steps };
            return e;
        }
#ifdef JPS_ALLOW_CORNER_CUTTING
        if (jps_has_diagonal_forced_neighbor(m, cx, cy, dx, dy))
        {
            jps__jump_entry e = { true, cx, cy, steps };
            return e;
        }
#endif

        idx_h = cy * c->w + cx;   /* E/W 平面：行主序 */
        idx_v = cx * c->h + cy;   /* S/N 平面：列主序 */

        /* 正交分量子检测：先内联快探 clean 命中，miss 才调完整版。短路顺序与 C# 一致。 */
        if (!jps_jump_probe(dist_h, gen_h, idx_h, c->row_gen[cy], &hd))
            hd = jps_jump_point_cache_cardinal_dist(pf->cache, m, cx, cy, dx, 0, horizontal_dir);
        if (hd > 0) { jps__jump_entry e = { true, cx, cy, steps }; return e; }
        if (cy == gy && jps_sign(gx - cx) == dx && abs(gx - cx) <= -hd)
        { jps__jump_entry e = { true, cx, cy, steps }; return e; }

        if (!jps_jump_probe(dist_v, gen_v, idx_v, c->col_gen[cx], &vd))
            vd = jps_jump_point_cache_cardinal_dist(pf->cache, m, cx, cy, 0, dy, vertical_dir);
        if (vd > 0) { jps__jump_entry e = { true, cx, cy, steps }; return e; }
        if (cx == gx && jps_sign(gy - cy) == dy && abs(gy - cy) <= -vd)
        { jps__jump_entry e = { true, cx, cy, steps }; return e; }
    }
}

/* ---------------- 方向剪枝（写入 dir_buf，返回数量） ---------------- */

static int jps__fill_directions(jps_pathfinder *pf, const jps_grid_map *m, int x, int y, int8_t parent_dir)
{
    int n = 0;
    int pdx, pdy;

    /* 起点没有父（parent_dir<0）：探索全部 8 个方向。 */
    if (parent_dir < 0)
    {
        int i;
        for (i = 0; i < JPS_DIR_COUNT; i++)
        {
            int dx = jps_dir_dx[i];
            int dy = jps_dir_dy[i];
            bool allowed = jps_is_diagonal_index(i)
                ? jps_diagonal_allowed(m, x, y, dx, dy)
                : jps_grid_map_is_walkable_g(m, x + dx, y + dy);   /* (x,y) 界内、±1 邻查 → 哨兵版免检 */
            if (allowed)
                pf->dir_buf[n++] = i;
        }
        return n;
    }

    pdx = jps_dir_dx[parent_dir];
    pdy = jps_dir_dy[parent_dir];

#ifdef JPS_ALLOW_CORNER_CUTTING
    /* ============ 允许斜穿角 ============ */
    if (jps_is_diagonal(pdx, pdy))
    {
        pf->dir_buf[n++] = parent_dir;
        pf->dir_buf[n++] = jps_dir_index_of(pdx, 0);
        pf->dir_buf[n++] = jps_dir_index_of(0, pdy);

        if (!jps_grid_map_is_walkable_g(m, x - pdx, y))
            pf->dir_buf[n++] = jps_dir_index_of(-pdx, pdy);
        if (!jps_grid_map_is_walkable_g(m, x, y - pdy))
            pf->dir_buf[n++] = jps_dir_index_of(pdx, -pdy);

        return n;
    }

    pf->dir_buf[n++] = parent_dir;

    if (pdx != 0)
    {
        if (!jps_grid_map_is_walkable_g(m, x, y + 1)) pf->dir_buf[n++] = jps_dir_index_of(pdx, 1);
        if (!jps_grid_map_is_walkable_g(m, x, y - 1)) pf->dir_buf[n++] = jps_dir_index_of(pdx, -1);
    }
    else
    {
        if (!jps_grid_map_is_walkable_g(m, x + 1, y)) pf->dir_buf[n++] = jps_dir_index_of(1, pdy);
        if (!jps_grid_map_is_walkable_g(m, x - 1, y)) pf->dir_buf[n++] = jps_dir_index_of(-1, pdy);
    }

    return n;
#else
    /* ============ 禁止斜穿角（默认，SoCS'12）============ */
    if (jps_is_diagonal(pdx, pdy))
    {
        /* 对角来向 → 只有 3 个自然邻居；禁止切角时对角不产生强迫邻居。 */
        pf->dir_buf[n++] = parent_dir;
        pf->dir_buf[n++] = jps_dir_index_of(pdx, 0);
        pf->dir_buf[n++] = jps_dir_index_of(0, pdy);
        return n;
    }

    pf->dir_buf[n++] = parent_dir;

    if (pdx != 0)   /* 水平移动 */
    {
        if (jps_grid_map_is_walkable_g(m, x, y + 1) && !jps_grid_map_is_walkable_g(m, x - pdx, y + 1))
        {
            pf->dir_buf[n++] = jps_dir_index_of(0, 1);
            pf->dir_buf[n++] = jps_dir_index_of(pdx, 1);
        }
        if (jps_grid_map_is_walkable_g(m, x, y - 1) && !jps_grid_map_is_walkable_g(m, x - pdx, y - 1))
        {
            pf->dir_buf[n++] = jps_dir_index_of(0, -1);
            pf->dir_buf[n++] = jps_dir_index_of(pdx, -1);
        }
    }
    else            /* 垂直移动 */
    {
        if (jps_grid_map_is_walkable_g(m, x + 1, y) && !jps_grid_map_is_walkable_g(m, x + 1, y - pdy))
        {
            pf->dir_buf[n++] = jps_dir_index_of(1, 0);
            pf->dir_buf[n++] = jps_dir_index_of(1, pdy);
        }
        if (jps_grid_map_is_walkable_g(m, x - 1, y) && !jps_grid_map_is_walkable_g(m, x - 1, y - pdy))
        {
            pf->dir_buf[n++] = jps_dir_index_of(-1, 0);
            pf->dir_buf[n++] = jps_dir_index_of(-1, pdy);
        }
    }

    return n;
#endif
}

/* ---------------- 路径重建 ---------------- */

static void jps__ensure_rebuild_nodes(jps_pathfinder *pf, int count)
{
    int n;
    if (count <= pf->rebuild_nodes_capacity)
        return;

    n = pf->rebuild_nodes_capacity < 16 ? 16 : pf->rebuild_nodes_capacity * 2;
    while (n < count)
        n *= 2;
    pf->rebuild_nodes = (int *)realloc(pf->rebuild_nodes, (size_t)n * sizeof(int));
    pf->rebuild_nodes_capacity = n;
}

static void jps__reconstruct_path(jps_pathfinder *pf, int sx, int sy, int gx, int gy, jps_path_result *r)
{
    /* 沿父链收集 compact path（goal → start），再反向写出 start → goal。
     * 对外只暴露 compact path：起点、跳点/拐点、终点；不展开跳跃段中间格。
     * 直接沿 (x,y) 坐标回溯（父 = 当前 − dir×steps），nodes 存打包坐标——
     * 全程无除法：查 g_dir 用一次乘法定位，写出用移位解包。 */
    int *nodes = pf->rebuild_nodes;
    int nodes_count = 0;
    int cx = gx, cy = gy;
    int i, write;

    /* 收集 */
    for (;;)
    {
        if (nodes_count == pf->rebuild_nodes_capacity)
        {
            jps__ensure_rebuild_nodes(pf, nodes_count + 1);
            nodes = pf->rebuild_nodes;
        }
        nodes[nodes_count++] = jps__pack_xy(cx, cy);

        if (cx == sx && cy == sy)
            break;

        {
            uint64_t gd = pf->g_dir[cy * pf->w + cx];     /* 一次乘法定位，同 load 取来向与步数 */
            int pdir = jps__gd_dir(gd);                   /* 非起点必有父，不会是 -1 */
            int dx = jps_dir_dx[pdir];
            int dy = jps_dir_dy[pdir];
            int steps = jps__gd_steps(gd);
            cx -= dx * steps;
            cy -= dy * steps;
        }
    }

    /* compact path = JPS 原始跳点序列（起点、跳点/拐点、终点），不做共线合并、不展开中间格。
     * nodes 为 goal→start，反向写出 start→goal。 */
    jps__ensure_path_capacity(r, nodes_count);
    write = 0;
    for (i = nodes_count - 1; i >= 0; i--)
    {
        int packed = nodes[i];
        r->path[write].x = packed & 0xFFFF;
        r->path[write].y = packed >> 16;
        write++;
    }
    r->path_count = write;
}

/* ---------------- 入口 ---------------- */

int jps_pathfinder_find_path(jps_pathfinder *pf, jps_system *system,
                             int sx, int sy, int gx, int gy)
{
    jps_grid_map *map;
    jps_path_result *out;
    int open_mark, closed_mark;
    int start_id, goal_packed;
    int expanded_count = 0;
    int current;   /* 出队的打包坐标 (y<<16)|x */
    int64_t prio;

    if (pf == NULL || system == NULL)
        return JPS_ERR_NULL;

    map = system->map;
    out = &pf->result;
    jps__result_reset(out);
    pf->cache = system->cache;

    if (!jps_grid_map_in_bounds(map, sx, sy) || !jps_grid_map_in_bounds(map, gx, gy))
        return JPS_ERR_OUT_OF_BOUNDS;
    if (!jps_grid_map_is_walkable(map, sx, sy) || !jps_grid_map_is_walkable(map, gx, gy))
        return JPS_ERR_BLOCKED;

    jps__ensure_buffers(pf, map);
    jps__next_epoch(pf);   /* 缓存同步由 jps_system_sync 负责（调用方在寻路前完成） */

    open_mark = pf->epoch * 2;      /* 本纪元“已生成/在 open”标记 */
    closed_mark = open_mark + 1;    /* 本纪元“已展开/closed”标记 */

    start_id = jps__id(pf, sx, sy);
    goal_packed = jps__pack_xy(gx, gy);

    jps_min_heap_clear(&pf->open);
    pf->g_dir[start_id] = jps__pack_gdir(0, 0, JPS__NO_DIR);   /* g=0、steps=0，起点无来向 */
    pf->mark[start_id] = (uint16_t)open_mark;
    jps_min_heap_enqueue(&pf->open, jps__pack_xy(sx, sy), jps_octile_heuristic(sx, sy, gx, gy));

    while (jps_min_heap_try_dequeue(&pf->open, &current, &prio))
    {
        uint64_t cur_gd;
        int64_t cur_g;
        int cx, cy, id, dir_count, i;

        /* 出队为打包坐标：移位取 (x,y)，一次乘法还原线性索引（免 current%w / current/w 真除法）。 */
        cx = current & 0xFFFF;
        cy = current >> 16;
        id = cy * pf->w + cx;

        if (pf->mark[id] == closed_mark)
            continue;

        pf->mark[id] = (uint16_t)closed_mark;
        cur_gd = pf->g_dir[id];   /* 一次 load 同取 g 与来向；已 closed，g 不再变 */
        cur_g = jps__gd_g(cur_gd);
        expanded_count++;

        if (current == goal_packed)
        {
            jps__reconstruct_path(pf, sx, sy, gx, gy, out);
            out->success = true;
            out->expanded_nodes = expanded_count;
            pf->smooth_map = map;         /* 记住地图，供平滑 LOS 使用 */
            pf->smoothed_valid = false;
            jps__ensure_smoothed(pf);     /* benchmark 计时包含平滑；copy/count 只读缓存 */
            return out->path_count;
        }

        dir_count = jps__fill_directions(pf, map, cx, cy, jps__gd_dir(cur_gd));

        for (i = 0; i < dir_count; i++)
        {
            int idx = pf->dir_buf[i];
            int dx = jps_dir_dx[idx];
            int dy = jps_dir_dy[idx];
            bool diagonal = jps_is_diagonal_index(idx);
            jps__jump_entry jump = diagonal
                ? jps__diagonal_jump(pf, map, cx, cy, dx, dy, gx, gy)
                : jps__cardinal_jump(pf, map, cx, cy, dx, dy, idx, gx, gy);

            int nb_id, nb_mark;
            int64_t move_cost, tentative;
            bool first_seen;

            if (!jump.has_jump)
                continue;

            nb_id = jps__id(pf, jump.x, jump.y);
            nb_mark = pf->mark[nb_id];   /* 读一次，closed 判定与 first_seen 共用 */
            if (nb_mark == closed_mark)
                continue;

            move_cost = (int64_t)jump.steps * (diagonal ? JPS_DIAGONAL_COST : JPS_CARDINAL_COST);
            tentative = cur_g + move_cost;

            first_seen = nb_mark < open_mark;
            if (!first_seen && tentative >= jps__gd_g(pf->g_dir[nb_id]))
                continue;

            /* g、steps、dir 同字：一条 8 字节 store 同时写入三者（原独立 steps 数组已并入）。 */
            pf->g_dir[nb_id] = jps__pack_gdir(tentative, jump.steps, (uint8_t)idx);
            pf->mark[nb_id] = (uint16_t)open_mark;

            jps_min_heap_enqueue(&pf->open, jps__pack_xy(jump.x, jump.y),
                                 tentative + jps_octile_heuristic(jump.x, jump.y, gx, gy));
        }
    }

    out->success = false;
    out->expanded_nodes = expanded_count;
    return JPS_ERR_NO_PATH;
}

/* ---------------- 结果访问器（不跨 ABI 传结构体/堆） ---------------- */

int jps_pathfinder_path_count(const jps_pathfinder *pf)
{
    return (pf && pf->result.success) ? pf->result.path_count : 0;
}

int jps_pathfinder_copy_path(const jps_pathfinder *pf, int *out_xy, int capacity_points)
{
    int n, i;

    if (pf == NULL || out_xy == NULL || !pf->result.success)
        return 0;

    n = pf->result.path_count;
    if (n > capacity_points)
        n = capacity_points;

    for (i = 0; i < n; i++)
    {
        out_xy[i * 2]     = pf->result.path[i].x;
        out_xy[i * 2 + 1] = pf->result.path[i].y;
    }
    return n;
}

/* 平滑缓存：find_path 成功后立即算一次。前提：pf->result.success（path_count≥1）。
 * 搜索已经结束，g_dir 不再需要保留 g/parent_dir，可作为 smoothed path 输出缓冲复用。 */
static void jps__ensure_smoothed(jps_pathfinder *pf)
{
    if (pf->smoothed_valid)
        return;

    pf->smoothed = (jps_point_f *)(void *)pf->g_dir;
    pf->smoothed_capacity = pf->size;
    pf->smoothed_count = jps__smooth_path_into(pf->smooth_map, pf->result.path, pf->result.path_count,
                                               pf->smoothed, pf->smoothed_capacity);
    pf->smoothed_valid = true;
}

int jps_pathfinder_smoothed_path_count(jps_pathfinder *pf)
{
    if (pf == NULL || !pf->result.success)
        return 0;
    jps__ensure_smoothed(pf);   /* find_path 已算；保留兜底，已算则直接返回 */
    return pf->smoothed_count;
}

int jps_pathfinder_copy_smoothed_path(jps_pathfinder *pf, float *out_xy, int capacity_points)
{
    int n, i;

    if (pf == NULL || out_xy == NULL || !pf->result.success)
        return 0;

    jps__ensure_smoothed(pf);   /* 平滑已在此算好并缓存——无二次计算、无需 system */

    n = pf->smoothed_count;
    if (n > pf->smoothed_capacity)
        n = pf->smoothed_capacity;
    if (n > capacity_points)
        n = capacity_points;

    for (i = 0; i < n; i++)
    {
        out_xy[i * 2]     = pf->smoothed[i].x;
        out_xy[i * 2 + 1] = pf->smoothed[i].y;
    }
    return n;
}
