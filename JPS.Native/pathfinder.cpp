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

static_assert(sizeof(jps_point_f) <= sizeof(uint64_t), "g storage slot must hold one smoothed point");

/*
 * 一次“跳跃”跳到的目标格。跳跃函数返回 bool（是否跳到跳点/终点），
 * 命中时把 (x,y,steps) 写入调用方传入的这个结构；撞墙/无跳点返回 false，不写。
 */
typedef struct
{
    int x;
    int y;
    int steps;   /* 跨越的格数（用于按 步数×单格代价 累计 g 值） */
} jps__jump_entry;


/*
 * 寻路结果（纯内部状态，不进公共头）：path 是 packed_xy compact path 动态数组，
 * path_count 为起点、跳点/拐点与终点的点数。
 * 缓冲可跨次查询复用。对外只通过 copy/count 等访问器暴露，不跨 ABI 传结构体。
 */
typedef struct
{
    bool success;
    bool reached_goal;   /* true=真正到达 goal；false=返回的是离 goal 最近的可达点（find_path_nearest 未达时） */
    uint32_t *path;      /* 每点 (y<<16)|x；地图边长 <=32767，最高位恒为 0 */
    int path_count;
    int path_capacity;
} jps_path_result;

/*
 * 稀疏逐节点搜索状态：
 *   g_slot[id] == 0          → 本次查询未访问；
 *   g_slot[id] == +(slot+1)  → open；
 *   g_slot[id] == -(slot+1)  → closed。
 * g_slot 同时承担 dense id→slot 映射与原 mark 的 seen/closed 状态；查询切换时只遍历
 * slot_node[0..state_count) 把实际访问过的 slot 清 0，不再维护 2 B/格的 mark/epoch。
 *
 * 只有访问到的节点才在 g_storage 中占一个 uint64；g、steps、parent dx/dy 打包布局为：
 *   位 [0,44)  = g 值：迷宫最优路径可途经 ~W×H 格，g 上限 ~1.5e12 < 2^41 ≪ 2^44，44 位足够；
 *   位 [44,60) = steps 到达该节点的跳跃步数（≤ max(W,H) ≤ 32767 < 2^16），父节点 = 当前 − dir×steps；
 *   位 [60,62) = parent dx 到达方向编码（dx+1，取值 0..2；3 为无父哨兵）。
 *   位 [62,64) = parent dy 到达方向编码（dy+1，取值 0..2；3 为无父哨兵）。
 * g_storage / slot_node 按同一 slot 并行索引并跨查询保留峰值容量；slot_node 低 31 位存
 * packed_xy（最高位恒空），最高位兼作堆过期检查的 closed 位。堆直接存 slot，出堆无需
 * g_slot→g_storage 的依赖寻址。典型内存为 4N + 12V（N=地图格数，V=峰值访问节点容量）。
 *   · 哨兵：无父存 dx_code=dy_code=3；实际方向只用 0..2（dx/dy = code-1）；
 *   · slot 从 0 递增，地图最大 N<2^30，故 ±(slot+1) 安全落在 int32_t 范围。
 */
#define JPS__STEPS_SHIFT 44
#define JPS__G_MASK      ((1ULL << JPS__STEPS_SHIFT) - 1)   /* g 的低 44 位 */
#define JPS__STEPS_MASK  0xFFFFULL             /* steps 的 16 位 */
#define JPS__DX_SHIFT    60
#define JPS__DY_SHIFT    62

#define JPS__NO_DCODE ((uint8_t)3u)   /* parent dx/dy 哨兵：无父（dx_code=dy_code=3） */
#define JPS__SLOT_CLOSED 0x80000000u
#define JPS__PACKED_XY_MASK 0x7FFFFFFFu

static inline int64_t jps__gd_g(uint64_t gd) { return (int64_t)(gd & JPS__G_MASK); }

static inline int jps__gd_steps(uint64_t gd) { return (int)((gd >> JPS__STEPS_SHIFT) & JPS__STEPS_MASK); }

static inline uint8_t jps__dir_code(int d) { return (uint8_t)(d + 1); }

static inline int jps__dir_decode(uint64_t code) { return (int)code - 1; }

static inline bool jps__gd_has_parent(uint64_t gd)
{
    /* 根哨兵把 dx/dy code 同置 3 → 高 4 位(bits 60..63)=0xF；非根两 code 均 ∈0..2，高 4 位必 ≠0xF
     * （code==3 只由 root 打包产生，且必成对出现，故不存在"仅一个为 3"的中间态）。
     * 于是一次比较即可判有无父，等价原 dx_code!=3 && dy_code!=3。 */
    return (gd >> JPS__DX_SHIFT) != 0xFULL;
}

static inline int jps__gd_dx(uint64_t gd) { return jps__dir_decode((gd >> JPS__DX_SHIFT) & 3ULL); }

static inline int jps__gd_dy(uint64_t gd) { return jps__dir_decode(gd >> JPS__DY_SHIFT); }

static inline uint64_t jps__pack_gdir(int64_t g, int steps, int dx, int dy)
{
    return ((uint64_t)g & JPS__G_MASK)
         | (((uint64_t)steps & JPS__STEPS_MASK) << JPS__STEPS_SHIFT)
         | ((uint64_t)jps__dir_code(dx) << JPS__DX_SHIFT)
         | ((uint64_t)jps__dir_code(dy) << JPS__DY_SHIFT);
}

static inline uint64_t jps__pack_gdir_root(int64_t g, int steps)
{
    return ((uint64_t)g & JPS__G_MASK)
         | (((uint64_t)steps & JPS__STEPS_MASK) << JPS__STEPS_SHIFT)
         | ((uint64_t)JPS__NO_DCODE << JPS__DX_SHIFT)
         | ((uint64_t)JPS__NO_DCODE << JPS__DY_SHIFT);
}

struct jps__dir{
    int8_t dx : 2;
    int8_t dy : 2;
    uint8_t hdir : 3;
    uint8_t vdir : 3;
    uint8_t dir : 3;
	uint8_t diagonal : 1;
};
typedef struct jps__dir jps__dir;   
static_assert(sizeof(jps__dir) <= sizeof(uint16_t), "jps__dir too big");

/* 索引顺序与 C# JpsDirections.All 严格一致：E,W,S,N,SE,SW,NE,NW。 */
static constexpr jps__dir jps__dirs[JPS_DIR_COUNT] = {
    {  1,  0, 0, 7, 0, 0 },
    { -1,  0, 1, 7, 1, 0 },
    {  0,  1, 7, 2, 2, 0 },
    {  0, -1, 7, 3, 3, 0 },
    {  1,  1, 0, 2, 4, 1 },
    { -1,  1, 1, 2, 5, 1 },
    {  1, -1, 0, 3, 6, 1 },
    { -1, -1, 1, 3, 7, 1 }
};

static constexpr uint8_t jps__dir_by_delta[3][3] = {
    { 7, 1, 5 },
    { 3, 0xFFu, 2 },
    { 6, 0, 4 }
};

static inline constexpr jps__dir jps__dir_of(int dx, int dy)
{
    return jps__dirs[jps__dir_by_delta[dx + 1][dy + 1]];
}

struct jps_pathfinder
{
    /* ---- 按地图尺寸一次性分配、跨多次查询复用的缓冲区 ---- */
    int w, h, size;
    int32_t *g_slot;        /* 每格一个 slot tag：0 unseen，正 open，负 closed */
    uint64_t *g_storage;    /* 只存本次实际访问节点的 packed g/steps/parent */
    uint32_t *slot_node;    /* 与 g_storage 同 slot：低 31 位 packed_xy，最高位 closed */
    int state_count;
    int state_capacity;
    jps_min_heap open;

    jps_jump_point_cache *cache;   /* 当前查询绑定的共享跳点缓存 */
    jps_path_result result;        /* 最近一次寻路结果（供 copy/count 访问器读取） */

    /* 平滑路径缓存：find_path 成功后复用 g_storage 内存窗口，copy_smoothed_path 直接拷。 */
    jps_point_f *smoothed;            /* g_storage 的别名，不拥有内存；搜索结束后才可覆盖 */
    int smoothed_count;
    int smoothed_capacity;
};
typedef struct jps_pathfinder jps_pathfinder;

/* ---------------- PathResult（内部） ---------------- */

static void jps__result_init(jps_path_result *r)
{
    r->success = false;
    r->reached_goal = false;
    r->path = NULL;
    r->path_count = 0;
    r->path_capacity = 0;
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
    r->reached_goal = false;
    r->path_count = 0;
}

static void jps__ensure_path_capacity(jps_path_result *r, int count)
{
    int n;
    if (count <= r->path_capacity)
        return;

    n = r->path_capacity < 16 ? 16 : r->path_capacity * 2;
    while (n < count)
        n *= 2;
    r->path = (uint32_t *)realloc(r->path, (size_t)n * sizeof(uint32_t));
    r->path_capacity = n;
}

static void jps__ensure_state_capacity(jps_pathfinder *pf, int count)
{
    int n;
    if (count <= pf->state_capacity)
        return;

    n = pf->state_capacity < 64 ? 64 : pf->state_capacity * 2;
    while (n < count)
        n *= 2;
    pf->g_storage = (uint64_t *)realloc(pf->g_storage, (size_t)n * sizeof(uint64_t));
    pf->slot_node = (uint32_t *)realloc(pf->slot_node, (size_t)n * sizeof(uint32_t));
    pf->state_capacity = n;
}

static void jps__reset_sparse_state(jps_pathfinder *pf)
{
    int i;
    for (i = 0; i < pf->state_count; i++)
    {
        uint32_t packed = pf->slot_node[i] & JPS__PACKED_XY_MASK;
        int id = (int)(packed >> 16) * pf->w + (int)(packed & 0xFFFFu);
        pf->g_slot[id] = 0;
    }
    pf->state_count = 0;
}

static inline int jps__slot_from_tag(int32_t tag)
{
    return (tag < 0 ? -(int)tag : (int)tag) - 1;
}

static inline int jps__state_add(jps_pathfinder *pf, int id, uint32_t packed_xy, uint64_t gd)
{
    int slot = pf->state_count;
    if (slot == pf->state_capacity)
        jps__ensure_state_capacity(pf, slot + 1);
    pf->state_count = slot + 1;
    pf->slot_node[slot] = packed_xy;
    pf->g_storage[slot] = gd;
    pf->g_slot[id] = (int32_t)(slot + 1);
    return slot;
}

static inline uint64_t jps__state_get(const jps_pathfinder *pf, int id)
{
    return pf->g_storage[jps__slot_from_tag(pf->g_slot[id])];
}

/* ---------------- 生命周期 ---------------- */

jps_pathfinder *jps_pathfinder_create(void)
{
    jps_pathfinder *pf = (jps_pathfinder *)malloc(sizeof(jps_pathfinder));
    if (pf == NULL)
        return NULL;
    pf->w = pf->h = pf->size = 0;
    pf->g_slot = NULL;
    pf->g_storage = NULL;
    pf->slot_node = NULL;
    pf->state_count = 0;
    pf->state_capacity = 0;
    jps_min_heap_init(&pf->open, 64);
    pf->cache = NULL;
    jps__result_init(&pf->result);
    pf->smoothed = NULL;
    pf->smoothed_count = 0;
    pf->smoothed_capacity = 0;
    return pf;
}

void jps_pathfinder_destroy(jps_pathfinder *pf)
{
    if (pf == NULL)
        return;
    free(pf->g_slot);
    free(pf->g_storage);
    free(pf->slot_node);
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
    if (pf->g_slot != NULL)
        bytes += (uint64_t)((size_t)pf->size * sizeof(int32_t));
    if (pf->g_storage != NULL)
        bytes += (uint64_t)((size_t)pf->state_capacity * sizeof(uint64_t));
    if (pf->slot_node != NULL)
        bytes += (uint64_t)((size_t)pf->state_capacity * sizeof(uint32_t));
    if (pf->open.elem != NULL)
        bytes += (uint64_t)((size_t)pf->open.capacity * sizeof(int));
    if (pf->open.prio != NULL)
        bytes += (uint64_t)((size_t)pf->open.capacity * sizeof(int64_t));
    if (pf->result.path != NULL)
        bytes += (uint64_t)((size_t)pf->result.path_capacity * sizeof(uint32_t));
    return bytes;
}

static void jps__ensure_buffers(jps_pathfinder *pf, const jps_grid_map *m)
{
    if (pf->w == m->width && pf->size == m->width * m->height)
        return;

    pf->w = m->width;
    pf->h = m->height;
    pf->size = pf->w * pf->h;
    free(pf->g_slot);
    free(pf->g_storage);
    free(pf->slot_node);
    pf->smoothed = NULL;
    pf->smoothed_count = 0;
    pf->smoothed_capacity = 0;
    pf->g_slot = (int32_t *)calloc((size_t)pf->size, sizeof(int32_t));
    pf->g_storage = NULL;
    pf->slot_node = NULL;
    pf->state_count = 0;
    pf->state_capacity = 0;
}

static inline int jps__id(const jps_pathfinder *pf, int x, int y) { return y * pf->w + x; }

/* 坐标打包为 (y<<16)|x（边长 ≤32767，最高位恒为 0，可借给 slot_node 作 closed 位）。
 * 主搜索堆载荷直接存 sparse slot；出堆从紧凑 slot_node 解坐标、从 g_storage 取状态。
 * 堆只按 priority 排序、不比较载荷，因此载荷从坐标换成 slot 不改变展开顺序。 */
static inline int jps__pack_xy(int x, int y) { return (y << 16) | x; }

/* ---------------- 正交跳跃 ---------------- */

static bool jps__cardinal_jump(jps_pathfinder *pf, const jps_grid_map *m,
                               int x, int y, int dx, int dy, int dir, int gx, int gy,
                               jps__jump_entry *out)
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
            out->x = gx; out->y = gy; out->steps = dist_to_goal;
            return true;
        }
    }

    if (dist > 0)
    {
        out->x = x + dx * dist; out->y = y + dy * dist; out->steps = dist;
        return true;
    }
    return false;
}

/* ---------------- 对角：经典逐格扫描，复用正交 memo ---------------- */

/*
 * CheckGoal=false 的实例把三处 goal 检查（直接命中 + 两条正交射线拦截）整段 if constexpr 掉，
 * 供 goal 不在本对角扫掠楔形内时使用——每步省 ~3 次比较。楔形内用 CheckGoal=true 实例，与原逻辑逐位等价。
 */
template <bool CheckGoal>
static bool jps__diagonal_jump_impl(jps_pathfinder *pf, const jps_grid_map *m,
                               const int x, const int y, const int dx, const int dy,
                               const int horizontal_dir, const int vertical_dir, const int gx, const int gy,
                               jps__jump_entry *out)
{
    int cx = x, cy = y, steps = 0;

    /* 对角每步要探两条正交线的 memo，热路（缓存 clean）命中率极高。把两方向的
     * dist/gen 平面基址与行/列世代数组在循环外解出，循环内先内联快探（一次 acquire
     * 字节读 + 一次 int16 读），miss 才走完整慢路（扫描+回填）——省去每步两次函数
     * 调用与平面基址/世代的重复解算。 */
    const jps_jump_point_cache *c = pf->cache;
    const int16_t *dist_h = c->dist + (size_t)horizontal_dir * c->size;
    const uint8_t *gen_h  = c->gen  + (size_t)horizontal_dir * c->size;
    const int16_t *dist_v = c->dist + (size_t)vertical_dir * c->size;
    const uint8_t *gen_v  = c->gen  + (size_t)vertical_dir * c->size;

    /* 下标随每步 cx+=dx / cy+=dy 线性变化，增量恒定：把乘法换成循环内加法递推。
     * 初值取 (x,y) 处，进入循环体自增 dx/dy 后正好对应 (cx,cy)。 */
    int idx_h = y * c->w + x;      /* E/W 平面：行主序 */
    int idx_v = x * c->h + y;      /* S/N 平面：列主序 */
    const int didx_h = dy * c->w + dx;
    const int didx_v = dx * c->h + dy;

    for (;;)
    {
        int hd, vd;

        /* 默认禁止斜穿角：斜走一步需目标格 + 两侧正交格都可走 */
        if (!jps_diagonal_allowed(m, cx, cy, dx, dy))
            return false;

        cx += dx;
        cy += dy;
        idx_h += didx_h;
        idx_v += didx_v;
        steps++;

        if constexpr (CheckGoal)
        {
            if (cx == gx && cy == gy)
            {
                out->x = cx; out->y = cy; out->steps = steps;
                return true;
            }
        }
#ifdef JPS_ALLOW_CORNER_CUTTING
        if (jps_has_diagonal_forced_neighbor(m, cx, cy, dx, dy))
        {
            out->x = cx; out->y = cy; out->steps = steps;
            return true;
        }
#endif

        /* 正交分量子检测：先内联快探 clean 命中，miss 才调完整版。短路顺序与 C# 一致。 */
        if (!jps_jump_probe(dist_h, gen_h, idx_h, c->row_gen[cy], &hd))
            hd = jps_jump_point_cache_cardinal_dist(pf->cache, m, cx, cy, dx, 0, horizontal_dir);
        if (hd > 0) { out->x = cx; out->y = cy; out->steps = steps; return true; }
        if constexpr (CheckGoal)
        {
            if (cy == gy && jps_sign(gx - cx) == dx && abs(gx - cx) <= -hd)
            { out->x = cx; out->y = cy; out->steps = steps; return true; }
        }

        if (!jps_jump_probe(dist_v, gen_v, idx_v, c->col_gen[cx], &vd))
            vd = jps_jump_point_cache_cardinal_dist(pf->cache, m, cx, cy, 0, dy, vertical_dir);
        if (vd > 0) { out->x = cx; out->y = cy; out->steps = steps; return true; }
        if constexpr (CheckGoal)
        {
            if (cx == gx && jps_sign(gy - cy) == dy && abs(gy - cy) <= -vd)
            { out->x = cx; out->y = cy; out->steps = steps; return true; }
        }
    }
}

/*
 * 对角跳跃分派：目标在本对角扫掠楔形内（gx 在 dx 侧 且 gy 在 dy 侧）才可能触发 goal 检查——
 * 只有此时 cx 能达 gx、cy 能达 gy。否则三处 goal 检查恒不触发，走无检查实例。
 * 判定每次跳跃只算一次（非每步），开销可忽略。
 */
static bool jps__diagonal_jump(jps_pathfinder *pf, const jps_grid_map *m,
                               const int x, const int y, const int dx, const int dy,
                               const int horizontal_dir, const int vertical_dir, const int gx, const int gy,
                               jps__jump_entry *out)
{
    if (jps_sign(gx - x) == dx && jps_sign(gy - y) == dy)
        return jps__diagonal_jump_impl<true>(pf, m, x, y, dx, dy, horizontal_dir, vertical_dir, gx, gy, out);
    return jps__diagonal_jump_impl<false>(pf, m, x, y, dx, dy, horizontal_dir, vertical_dir, gx, gy, out);
}

/* ---------------- 方向剪枝（写入 dir_buf，返回数量） ---------------- */

static int jps__fill_directions(jps__dir* dir_buf, const jps_grid_map *m, int x, int y,
                                bool has_parent, int pdx, int pdy)
{
    int n = 0;

    /* 起点没有父：探索全部 8 个方向。 */
    if (!has_parent)
    {
        int i;
        for (i = 0; i < JPS_DIR_COUNT; i++)
        {
            jps__dir d = jps__dirs[i];
            int dx = d.dx;
            int dy = d.dy;
            bool allowed = d.diagonal
                ? jps_diagonal_allowed(m, x, y, dx, dy)
                : jps_grid_map_is_walkable_g(m, x + dx, y + dy);   /* (x,y) 界内、±1 邻查 → 哨兵版免检 */
            if (allowed)
                dir_buf[n++] = d;
        }
        return n;
    }

#ifdef JPS_ALLOW_CORNER_CUTTING
    /* ============ 允许斜穿角 ============ */
    if (jps_is_diagonal(pdx, pdy))
    {
        dir_buf[n++] = jps__dir_of(pdx, pdy);
        dir_buf[n++] = jps__dir_of(pdx, 0);
        dir_buf[n++] = jps__dir_of(0, pdy);

        if (!jps_grid_map_is_walkable_g(m, x - pdx, y))
            dir_buf[n++] = jps__dir_of(-pdx, pdy);
        if (!jps_grid_map_is_walkable_g(m, x, y - pdy))
            dir_buf[n++] = jps__dir_of(pdx, -pdy);

        return n;
    }

    dir_buf[n++] = jps__dir_of(pdx, pdy);

    if (pdx != 0)
    {
        if (!jps_grid_map_is_walkable_g(m, x, y + 1)) dir_buf[n++] = jps__dir_of(pdx, 1);
        if (!jps_grid_map_is_walkable_g(m, x, y - 1)) dir_buf[n++] = jps__dir_of(pdx, -1);
    }
    else
    {
        if (!jps_grid_map_is_walkable_g(m, x + 1, y)) dir_buf[n++] = jps__dir_of(1, pdy);
        if (!jps_grid_map_is_walkable_g(m, x - 1, y)) dir_buf[n++] = jps__dir_of(-1, pdy);
    }

    return n;
#else
    /* ============ 禁止斜穿角（默认，SoCS'12）============ */
    if (jps_is_diagonal(pdx, pdy))
    {
        /* 对角来向 → 只有 3 个自然邻居；禁止切角时对角不产生强迫邻居。 */
        dir_buf[n++] = jps__dir_of(pdx, pdy);
        dir_buf[n++] = jps__dir_of(pdx, 0);
        dir_buf[n++] = jps__dir_of(0, pdy);
        return n;
    }

    dir_buf[n++] = jps__dir_of(pdx, pdy);

    if (pdx != 0)   /* 水平移动 */
    {
        if (jps_grid_map_is_walkable_g(m, x, y + 1) && !jps_grid_map_is_walkable_g(m, x - pdx, y + 1))
        {
            dir_buf[n++] = jps__dir_of(0, 1);
            dir_buf[n++] = jps__dir_of(pdx, 1);
        }
        if (jps_grid_map_is_walkable_g(m, x, y - 1) && !jps_grid_map_is_walkable_g(m, x - pdx, y - 1))
        {
            dir_buf[n++] = jps__dir_of(0, -1);
            dir_buf[n++] = jps__dir_of(pdx, -1);
        }
    }
    else            /* 垂直移动 */
    {
        if (jps_grid_map_is_walkable_g(m, x + 1, y) && !jps_grid_map_is_walkable_g(m, x + 1, y - pdy))
        {
            dir_buf[n++] = jps__dir_of(1, 0);
            dir_buf[n++] = jps__dir_of(1, pdy);
        }
        if (jps_grid_map_is_walkable_g(m, x - 1, y) && !jps_grid_map_is_walkable_g(m, x - 1, y - pdy))
        {
            dir_buf[n++] = jps__dir_of(-1, 0);
            dir_buf[n++] = jps__dir_of(-1, pdy);
        }
    }

    return n;
#endif
}

/* ---------------- 路径重建 ---------------- */

static void jps__reverse_packed_path(uint32_t *path, int begin, int end)
{
    int left = begin;
    int right = end - 1;
    while (left < right)
    {
        uint32_t t = path[left];
        path[left++] = path[right];
        path[right--] = t;
    }
}

static void jps__reconstruct_path(jps_pathfinder *pf, int sx, int sy, int gx, int gy, jps_path_result *r)
{
    /* 沿父链收集 compact path（goal → start），再反向写出 start → goal。
     * 对外只暴露 compact path：起点、跳点/拐点、终点；不展开跳跃段中间格。
     * 直接沿 (x,y) 坐标回溯（父 = 当前 − dir×steps），结果数组本身存 packed_xy，
     * 收集后原地翻转，无第二份 rebuild 缓冲。 */
    int nodes_count = 0;
    int cx = gx, cy = gy;

    for (;;)
    {
        if (nodes_count == r->path_capacity)
            jps__ensure_path_capacity(r, nodes_count + 1);
        r->path[nodes_count++] = (uint32_t)jps__pack_xy(cx, cy);

        if (cx == sx && cy == sy)
            break;

        {
            uint64_t gd = jps__state_get(pf, cy * pf->w + cx);
            int dx = jps__gd_dx(gd);                      /* 非起点必有父，dx/dy code 不会是哨兵 */
            int dy = jps__gd_dy(gd);
            int steps = jps__gd_steps(gd);
            cx -= dx * steps;
            cy -= dy * steps;
        }
    }

    jps__reverse_packed_path(r->path, 0, nodes_count);
    r->path_count = nodes_count;
}

/* 平滑缓存：find_path 成功后立即算一次。搜索已经结束，g_storage 不再需要保留搜索状态；
 * 有效 compact path 的平滑输出点数不超过其输入点数，保证到 path_count 后即可原地复用。 */
static void jps__ensure_smoothed(jps_pathfinder* pf, const jps_grid_map* map)
{
    jps__ensure_state_capacity(pf, pf->result.path_count);
    pf->smoothed = (jps_point_f*)(void*)pf->g_storage;
    pf->smoothed_capacity = pf->state_capacity;
    pf->smoothed_count = jps__smooth_path_into(map, pf->result.path, pf->result.path_count,
        pf->smoothed, pf->smoothed_capacity);
}

/* ---------------- 入口 ---------------- */

/*
 * goal-snapping：goal 落在阻挡上时，按 Chebyshev 环由近及远扫描，返回**最近的可走格**——
 * 同环内取离 start 最近(octile 最小)者（即朝 start 一侧、自然的接近侧接触格），严格 tie-break
 * （同 h 保留扫描序先者，扫描序确定 → C≡C# 一致）。周围全被挡（半径超上限）返回 false。
 * 环由近及远 → 首个含可走格的环即最近；OOB 环格由 is_walkable 的界内判定天然跳过。
 */
static bool jps__snap_goal(const jps_grid_map *m, int sx, int sy, int gx, int gy,
                           int *out_x, int *out_y)
{
    int max_r = m->width > m->height ? m->width : m->height;
    int r;
    for (r = 1; r <= max_r; r++)
    {
        int64_t best_h = -1;   /* 哨兵：本环尚未找到 */
        int bx = 0, by = 0, yy, xx;
        for (yy = gy - r; yy <= gy + r; yy++)
        {
            int on_y_border = (yy == gy - r || yy == gy + r);   /* 上/下边整行；中间行只取左右两端 */
            for (xx = gx - r; xx <= gx + r; xx++)
            {
                int64_t h;
                if (!on_y_border && xx != gx - r && xx != gx + r)
                    continue;                                    /* 跳过环内部，只取边界 */
                if (!jps_grid_map_is_walkable(m, xx, yy))
                    continue;
                h = jps_octile_heuristic(xx, yy, sx, sy);
                if (best_h < 0 || h < best_h)                    /* 严格 <：同 h 保留先者 */
                {
                    best_h = h; bx = xx; by = yy;
                }
            }
        }
        if (best_h >= 0) { *out_x = bx; *out_y = by; return true; }
    }
    return false;
}

/*
 * 跳点粒度补偿：从最近跳点 (bx,by) 做**有界 greedy-best-first flood**（按 octile-to-goal 排序），
 * 在连通可达域内找 octile 最小的可达格——GBFS 遇死胡同会回探其他分支，故不像纯贪心那样卡局部最小
 * （最近可达格常在需先"绕远"才能到的方向）。上限 K 格封顶开销；连通域近 goal 的最近格通常远在 K 内。
 * 复用 open 堆 + 稀疏 state（存 BFS 父；主搜索父链已经 reconstruct，可先清掉）。
 * 找到最近格后沿 sparse state 父链回溯，把 (bx,by) 之后到最近格的这段逐格追加进 path。
 */
static void jps__nearest_refine(jps_pathfinder *pf, const jps_grid_map *m,
                                int bx, int by, int gx, int gy, jps_path_result *r)
{
    enum { JPS__REFINE_CAP = 4096 };
    int start_packed = jps__pack_xy(bx, by);
    int best_packed = start_packed;
    int64_t best_h = jps_octile_heuristic(bx, by, gx, gy);
    int visited = 0, cur, chain_begin, i, start_slot;
    int64_t prio;

    jps__reset_sparse_state(pf);                 /* 主搜索父链已重建；同一 arena 复用为 refine BFS */
    jps_min_heap_clear(&pf->open);
    start_slot = jps__state_add(pf, jps__id(pf, bx, by), (uint32_t)start_packed,
                                (uint64_t)(uint32_t)start_packed);
    jps_min_heap_enqueue(&pf->open, start_slot, best_h);

    while (visited < JPS__REFINE_CAP && jps_min_heap_try_dequeue(&pf->open, &cur, &prio))
    {
        uint32_t node_state = pf->slot_node[cur];
        int packed, cx, cy, id;
        if ((node_state & JPS__SLOT_CLOSED) != 0)
            continue;
        packed = (int)(node_state & JPS__PACKED_XY_MASK);
        pf->slot_node[cur] = node_state | JPS__SLOT_CLOSED;
        cx = packed & 0xFFFF;
        cy = packed >> 16;
        id = jps__id(pf, cx, cy);
        pf->g_slot[id] = -(int32_t)(cur + 1);
        visited++;
        if (prio < best_h) { best_h = prio; best_packed = packed; }
        for (i = 0; i < JPS_DIR_COUNT; i++)
        {
            int dx = jps__dirs[i].dx, dy = jps__dirs[i].dy;
            int nx = cx + dx, ny = cy + dy, nid, slot;
            bool ok = jps__dirs[i].diagonal ? jps_diagonal_allowed(m, cx, cy, dx, dy)
                                            : jps_grid_map_is_walkable(m, nx, ny);
            if (!ok)
                continue;
            nid = jps__id(pf, nx, ny);
            if (pf->g_slot[nid] != 0)
                continue;
            slot = jps__state_add(pf, nid, (uint32_t)jps__pack_xy(nx, ny),
                                  (uint64_t)(uint32_t)packed);         /* BFS 父 = packed */
            jps_min_heap_enqueue(&pf->open, slot, jps_octile_heuristic(nx, ny, gx, gy));
        }
    }

    if (best_packed == start_packed)
        return;   /* 起点即最近，无需追加 */

    /* 把 best→…→start（含 best、不含 start）直接追加进 packed 结果区，再只翻转新增后缀。 */
    chain_begin = r->path_count;
    cur = best_packed;
    while (cur != start_packed && r->path_count - chain_begin < JPS__REFINE_CAP)
    {
        if (r->path_count == r->path_capacity)
            jps__ensure_path_capacity(r, r->path_count + 1);
        r->path[r->path_count++] = (uint32_t)cur;
        cur = (int)(uint32_t)jps__state_get(pf, jps__id(pf, cur & 0xFFFF, cur >> 16));
    }
    jps__reverse_packed_path(r->path, chain_begin, r->path_count);
}

/* 搜索核心：allow_nearest=false 即经典严格 find_path；true 即 find_path_nearest——
 * goal 落在阻挡上时先 goal-snapping 到最近可走格再寻路，且搜索耗尽时返回展开过的、
 * 离(可能已 snap 的)目标最近(octile h 最小)的节点路径，再朝 goal 贪心下降贴近。 */
static int jps__find_path_core(jps_pathfinder *pf, jps_system *system,
                               int sx, int sy, int gx, int gy, bool allow_nearest)
{
    jps_grid_map *map;
    jps_path_result *out;
    int start_id, start_packed, start_slot, goal_packed;
    int current;   /* 出队的 sparse slot */
    int64_t prio;
    int best_packed;   /* 离 goal 最近的已展开节点（打包坐标）；nearest 兜底用 */
    int64_t best_h;

    if (pf == NULL || system == NULL)
        return JPS_ERR_NULL;

    map = system->map;
    out = &pf->result;
    jps__result_reset(out);
    pf->smoothed_count = 0;
    pf->cache = system->cache;

    if (!jps_grid_map_in_bounds(map, sx, sy) || !jps_grid_map_in_bounds(map, gx, gy))
        return JPS_ERR_OUT_OF_BOUNDS;
    if (!jps_grid_map_is_walkable(map, sx, sy))
        return JPS_ERR_BLOCKED;
    /* 严格模式要求 goal 可走；nearest 模式允许 goal 落在阻挡上（膨胀后 goal 进障碍的场景）。 */
    if (!allow_nearest && !jps_grid_map_is_walkable(map, gx, gy))
        return JPS_ERR_BLOCKED;

    /* nearest 模式 + goal 落在阻挡上：先 goal-snapping——把 goal 移到离它最近、朝 start 一侧的
     * 可走格，再照常寻路。够得到该 snap 目标 → reached_goal=1 停在接触格；够不到 → 退化为"离 snap
     * 目标最近的已展开点"、reached_goal=0。周围全被挡则维持原 goal（仍走最近已展开点兜底）。
     * 于是 reached_goal 之后表示"到达了这个**（可能已 snap 的）有效目标**"。 */
    if (allow_nearest && !jps_grid_map_is_walkable(map, gx, gy))
    {
        int sgx, sgy;
        if (jps__snap_goal(map, sx, sy, gx, gy, &sgx, &sgy))
        {
            gx = sgx;
            gy = sgy;
        }
    }

    jps__ensure_buffers(pf, map);
    jps__reset_sparse_state(pf);   /* 只清上一阶段实际访问过的 g_slot */

    start_id = jps__id(pf, sx, sy);
    start_packed = jps__pack_xy(sx, sy);
    goal_packed = jps__pack_xy(gx, gy);

    best_packed = start_packed;                            /* 起点必是首个展开点 → best 恒有效 */
    best_h = jps_octile_heuristic(sx, sy, gx, gy);

    jps_min_heap_clear(&pf->open);
    start_slot = jps__state_add(pf, start_id, (uint32_t)start_packed, jps__pack_gdir_root(0, 0));
    jps_min_heap_enqueue(&pf->open, start_slot, best_h);

    while (jps_min_heap_try_dequeue(&pf->open, &current, &prio))
    {
        uint64_t cur_gd;
        uint32_t node_state;
        int64_t cur_g;
        int current_packed, cx, cy, id, dir_count, i;

        node_state = pf->slot_node[current];
        if ((node_state & JPS__SLOT_CLOSED) != 0)
            continue;
        pf->slot_node[current] = node_state | JPS__SLOT_CLOSED;
        current_packed = (int)(node_state & JPS__PACKED_XY_MASK);
        cx = current_packed & 0xFFFF;
        cy = current_packed >> 16;
        id = jps__id(pf, cx, cy);
        pf->g_slot[id] = -(int32_t)(current + 1);
        cur_gd = pf->g_storage[current];
        cur_g = jps__gd_g(cur_gd);

        if (current_packed == goal_packed)
        {
            jps__reconstruct_path(pf, sx, sy, gx, gy, out);
            out->success = true;
            out->reached_goal = true;
            jps__ensure_smoothed(pf, map);     /* benchmark 计时包含平滑；copy/count 只读缓存 */
            return out->path_count;
        }

        /* nearest 兜底：记录离 goal 最近(octile h 最小)的已展开节点。严格 tie-break
         * （h 相等保留先展开者），展开序确定 → C≡C# 一致。仅 nearest 模式计。 */
        if (allow_nearest)
        {
            int64_t h = jps_octile_heuristic(cx, cy, gx, gy);
            if (h < best_h)
            {
                best_h = h;
                best_packed = current_packed;
            }
        }

        jps__dir dir_buf[JPS_DIR_COUNT];
        dir_count = jps__fill_directions(dir_buf, map, cx, cy,
                                         jps__gd_has_parent(cur_gd),
                                         jps__gd_dx(cur_gd), jps__gd_dy(cur_gd));

        for (i = 0; i < dir_count; i++)
        {
            const jps__dir& d = dir_buf[i];
            int dx = d.dx;
            int dy = d.dy;
            jps__jump_entry jump;
            bool has_jump = d.diagonal
                ? jps__diagonal_jump(pf, map, cx, cy, dx, dy, d.hdir, d.vdir, gx, gy, &jump)
                : jps__cardinal_jump(pf, map, cx, cy, dx, dy, d.dir, gx, gy, &jump);

            int nb_id, nb_slot;
            int32_t nb_tag;
            int64_t move_cost, tentative;
            bool first_seen;

            if (!has_jump)
                continue;

            nb_id = jps__id(pf, jump.x, jump.y);
            nb_tag = pf->g_slot[nb_id];   /* 0 unseen，正 open，负 closed */
            if (nb_tag < 0)
                continue;

            move_cost = (int64_t)jump.steps * (d.diagonal ? JPS_DIAGONAL_COST : JPS_CARDINAL_COST);
            tentative = cur_g + move_cost;

            first_seen = nb_tag == 0;
            if (!first_seen && tentative >= jps__gd_g(pf->g_storage[nb_tag - 1]))
                continue;

            /* 首见分配一个紧凑 slot；后续 decrease 只覆盖同一 slot。 */
            if (first_seen)
                nb_slot = jps__state_add(pf, nb_id, (uint32_t)jps__pack_xy(jump.x, jump.y),
                                         jps__pack_gdir(tentative, jump.steps, dx, dy));
            else
            {
                nb_slot = nb_tag - 1;
                pf->g_storage[nb_tag - 1] = jps__pack_gdir(tentative, jump.steps, dx, dy);
            }

            jps_min_heap_enqueue(&pf->open, nb_slot,
                                 tentative + jps_octile_heuristic(jump.x, jump.y, gx, gy));
        }
    }

    /* 搜索耗尽未达 goal。 */
    if (allow_nearest)
    {
        /* 先回溯到离 goal 最近的已展开节点（起点也在候选内，best_packed 恒有效），
         * 再从该跳点朝 goal 贪心下降到局部最近可达格，让落脚点真正贴近 goal。 */
        int bx = best_packed & 0xFFFF, by = best_packed >> 16;
        jps__reconstruct_path(pf, sx, sy, bx, by, out);
        jps__nearest_refine(pf, map, bx, by, gx, gy, out);
        out->success = true;
        out->reached_goal = false;
        jps__ensure_smoothed(pf, map);
        return out->path_count;
    }

    out->success = false;
    return JPS_ERR_NO_PATH;
}

int jps_pathfinder_find_path(jps_pathfinder *pf, jps_system *system,
                             int sx, int sy, int gx, int gy)
{
    return jps__find_path_core(pf, system, sx, sy, gx, gy, false);
}

/* 不可达兜底版：goal 可落在阻挡上；到不了 goal 时返回展开过的、离 goal 最近的节点路径。
 * 返回 >=1 = 路径点数（真到达或最近点——用 jps_pathfinder_reached_goal 区分）；<0 见 JPS_ERR_*。 */
int jps_pathfinder_find_path_nearest(jps_pathfinder *pf, jps_system *system,
                                     int sx, int sy, int gx, int gy)
{
    return jps__find_path_core(pf, system, sx, sy, gx, gy, true);
}

/* ---------------- 结果访问器（不跨 ABI 传结构体/堆） ---------------- */

int jps_pathfinder_path_count(const jps_pathfinder *pf)
{
    return (pf && pf->result.success) ? pf->result.path_count : 0;
}

/* 最近一次寻路是否真正到达 goal：1=到达；0=返回的是最近点（find_path_nearest 未达）或无结果。 */
int jps_pathfinder_reached_goal(const jps_pathfinder *pf)
{
    return (pf && pf->result.success && pf->result.reached_goal) ? 1 : 0;
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
        uint32_t packed = pf->result.path[i];
        out_xy[i * 2]     = (int)(packed & 0xFFFFu);
        out_xy[i * 2 + 1] = (int)(packed >> 16);
    }
    return n;
}

int jps_pathfinder_smoothed_path_count(jps_pathfinder *pf)
{
    if (pf == NULL || !pf->result.success)
        return 0;
    return pf->smoothed_count;
}

int jps_pathfinder_copy_smoothed_path(jps_pathfinder *pf, float *out_xy, int capacity_points)
{
    int n, i;

    if (pf == NULL || out_xy == NULL || !pf->result.success)
        return 0;

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
