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

static_assert(sizeof(jps_point_f) <= sizeof(uint64_t), "g_dir slot must hold one smoothed point");

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
 * 寻路结果（纯内部状态，不进公共头）：path 是 compact path 动态数组，
 * path_count 为起点、跳点/拐点与终点的点数。
 * 缓冲可跨次查询复用。对外只通过 copy/count 等访问器暴露，不跨 ABI 传结构体。
 */
typedef struct
{
    bool success;
    bool reached_goal;   /* true=真正到达 goal；false=返回的是离 goal 最近的可达点（find_path_nearest 未达时） */
    jps_point *path;
    int path_count;
    int path_capacity;
} jps_path_result;

/*
 * 逐节点搜索状态：按**访问频率**拆存（SoA），不按节点合并（AoS）——
 * mark 是被高频单独访问的（堆过期检查/邻居 closed 检查只碰它），保持独立密集数组：
 * uint16 一条 cache line 装 32 个节点的 mark；与冷字段合并会把最热数组稀释 3–4 倍（实测回退）。
 *
 * g、steps、parent dx/dy 打包进同一个 uint64（g_dir），取代原先独立的 steps 数组：
 *   位 [0,44)  = g 值：迷宫最优路径可途经 ~W×H 格，g 上限 ~1.5e12 < 2^41 ≪ 2^44，44 位足够；
 *   位 [44,60) = steps 到达该节点的跳跃步数（≤ max(W,H) ≤ 32767 < 2^16），父节点 = 当前 − dir×steps；
 *   位 [60,62) = parent dx 到达方向编码（dx+1，取值 0..2；3 为无父哨兵）。
 *   位 [62,64) = parent dy 到达方向编码（dy+1，取值 0..2；3 为无父哨兵）。
 *   · 展开读 g+方向、relax 写 g+steps+方向本来就要摸这条 line，steps 塞进同字等于免费搭车——
 *     独立 steps 数组整个消失，relax 少一次 store，逐节点搜索态 12→10 B/格；
 *   · 哨兵：无父存 dx_code=dy_code=3；实际方向只用 0..2（dx/dy = code-1）；
 *   · g_dir 自然 8 对齐、8 节点/line，无跨线、无非对齐访问。
 */
#define JPS__STEPS_SHIFT 44
#define JPS__G_MASK      ((1ULL << JPS__STEPS_SHIFT) - 1)   /* g 的低 44 位 */
#define JPS__STEPS_MASK  0xFFFFULL             /* steps 的 16 位 */
#define JPS__DX_SHIFT    60
#define JPS__DY_SHIFT    62

#define JPS__NO_DCODE ((uint8_t)3u)   /* parent dx/dy 哨兵：无父（dx_code=dy_code=3） */

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

    jps_jump_point_cache *cache;   /* 当前查询绑定的共享跳点缓存 */
    jps_path_result result;        /* 最近一次寻路结果（供 copy/count 访问器读取） */

    int *rebuild_nodes;            /* 路径重建用父链节点栈，跨查询复用，避免每次 malloc/free */
    int rebuild_nodes_capacity;

    /* 平滑路径缓存：find_path 成功后复用 g_dir 内存窗口，copy_smoothed_path 直接拷。 */
    jps_point_f *smoothed;            /* g_dir 的别名，不拥有内存；搜索结束后才可覆盖 */
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
    pf->smoothed = NULL;
    pf->smoothed_count = 0;
    pf->smoothed_capacity = 0;
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
            int dx = jps__gd_dx(gd);                      /* 非起点必有父，dx/dy code 不会是哨兵 */
            int dy = jps__gd_dy(gd);
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

/* 平滑缓存：find_path 成功后立即算一次。前提：pf->result.success（path_count≥1）。
 * 搜索已经结束，g_dir 不再需要保留 g/parent dx/dy，可作为 smoothed path 输出缓冲复用。 */
static void jps__ensure_smoothed(jps_pathfinder* pf, const jps_grid_map* map)
{
    pf->smoothed = (jps_point_f*)(void*)pf->g_dir;
    pf->smoothed_capacity = pf->size;
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
 * 复用 open 堆 + g_dir(存 BFS 父，主搜索数据已被 reconstruct 提走，可覆盖) + mark(fresh epoch 作 visited)。
 * 找到最近格后沿 g_dir 父链回溯，把 (bx,by) 之后到最近格的这段逐格追加进 path（平滑器后续拉直）。
 */
static void jps__nearest_refine(jps_pathfinder *pf, const jps_grid_map *m,
                                int bx, int by, int gx, int gy, jps_path_result *r)
{
    enum { JPS__REFINE_CAP = 4096 };
    int start_packed = jps__pack_xy(bx, by);
    int best_packed = start_packed;
    int64_t best_h = jps_octile_heuristic(bx, by, gx, gy);
    int visited = 0, cur, chain_len, i;
    int64_t prio;

    jps__next_epoch(pf);                         /* fresh epoch → 全新 visited 标记，不与主搜索冲突 */
    int visited_mark = pf->epoch * 2;

    jps_min_heap_clear(&pf->open);
    pf->mark[jps__id(pf, bx, by)] = (uint16_t)visited_mark;
    pf->g_dir[jps__id(pf, bx, by)] = (uint64_t)(uint32_t)start_packed;   /* 自指 = 无父哨兵 */
    jps_min_heap_enqueue(&pf->open, start_packed, best_h);

    while (visited < JPS__REFINE_CAP && jps_min_heap_try_dequeue(&pf->open, &cur, &prio))
    {
        int cx = cur & 0xFFFF, cy = cur >> 16;
        visited++;
        if (prio < best_h) { best_h = prio; best_packed = cur; }   /* prio 即该格 octile（入队即标记，无重复） */
        for (i = 0; i < JPS_DIR_COUNT; i++)
        {
            int dx = jps__dirs[i].dx, dy = jps__dirs[i].dy;
            int nx = cx + dx, ny = cy + dy, nid;
            bool ok = jps__dirs[i].diagonal ? jps_diagonal_allowed(m, cx, cy, dx, dy)
                                            : jps_grid_map_is_walkable(m, nx, ny);
            if (!ok)
                continue;
            nid = jps__id(pf, nx, ny);
            if (pf->mark[nid] == (uint16_t)visited_mark)
                continue;
            pf->mark[nid] = (uint16_t)visited_mark;
            pf->g_dir[nid] = (uint64_t)(uint32_t)cur;                 /* BFS 父 = cur */
            jps_min_heap_enqueue(&pf->open, jps__pack_xy(nx, ny), jps_octile_heuristic(nx, ny, gx, gy));
        }
    }

    if (best_packed == start_packed)
        return;   /* 起点即最近，无需追加 */

    /* 沿 g_dir 父链收集 best→…→start（含 best、不含 start），再逆序追加为 start 之后→best 的正向段。 */
    jps__ensure_rebuild_nodes(pf, JPS__REFINE_CAP);
    chain_len = 0;
    cur = best_packed;
    while (cur != start_packed && chain_len < JPS__REFINE_CAP)
    {
        pf->rebuild_nodes[chain_len++] = cur;
        cur = (int)(uint32_t)pf->g_dir[jps__id(pf, cur & 0xFFFF, cur >> 16)];
    }
    jps__ensure_path_capacity(r, r->path_count + chain_len);
    for (i = chain_len - 1; i >= 0; i--)   /* 逆序：靠近 start 的先追加 */
    {
        int p = pf->rebuild_nodes[i];
        r->path[r->path_count].x = p & 0xFFFF;
        r->path[r->path_count].y = p >> 16;
        r->path_count++;
    }
}

/* 搜索核心：allow_nearest=false 即经典严格 find_path；true 即 find_path_nearest——
 * goal 落在阻挡上时先 goal-snapping 到最近可走格再寻路，且搜索耗尽时返回展开过的、
 * 离(可能已 snap 的)目标最近(octile h 最小)的节点路径，再朝 goal 贪心下降贴近。 */
static int jps__find_path_core(jps_pathfinder *pf, jps_system *system,
                               int sx, int sy, int gx, int gy, bool allow_nearest)
{
    jps_grid_map *map;
    jps_path_result *out;
    int open_mark, closed_mark;
    int start_id, goal_packed;
    int current;   /* 出队的打包坐标 (y<<16)|x */
    int64_t prio;
    int best_packed;   /* 离 goal 最近的已展开节点（打包坐标）；nearest 兜底用 */
    int64_t best_h;

    if (pf == NULL || system == NULL)
        return JPS_ERR_NULL;

    map = system->map;
    out = &pf->result;
    jps__result_reset(out);
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
    jps__next_epoch(pf);   /* 缓存同步由 jps_system_sync 负责（调用方在寻路前完成） */

    open_mark = pf->epoch * 2;      /* 本纪元“已生成/在 open”标记 */
    closed_mark = open_mark + 1;    /* 本纪元“已展开/closed”标记 */

    start_id = jps__id(pf, sx, sy);
    goal_packed = jps__pack_xy(gx, gy);

    best_packed = jps__pack_xy(sx, sy);                    /* 起点必是首个展开点 → best 恒有效 */
    best_h = jps_octile_heuristic(sx, sy, gx, gy);

    jps_min_heap_clear(&pf->open);
    pf->g_dir[start_id] = jps__pack_gdir_root(0, 0);   /* g=0、steps=0，起点无来向 */
    pf->mark[start_id] = (uint16_t)open_mark;
    jps_min_heap_enqueue(&pf->open, jps__pack_xy(sx, sy), best_h);

    while (jps_min_heap_try_dequeue(&pf->open, &current, &prio))
    {
        uint64_t cur_gd;
        int64_t cur_g;
        int cx, cy, id, dir_count, i;

        /* 出队为打包坐标：移位取 (x,y)，一次乘法还原线性索引（免 current%w / current/w 真除法）。 */
        cx = current & 0xFFFF;
        cy = current >> 16;
        id = jps__id(pf, cx, cy);

        if (pf->mark[id] == closed_mark)
            continue;

        pf->mark[id] = (uint16_t)closed_mark;
        cur_gd = pf->g_dir[id];   /* 一次 load 同取 g 与来向；已 closed，g 不再变 */
        cur_g = jps__gd_g(cur_gd);

        if (current == goal_packed)
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
                best_packed = current;
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

            int nb_id, nb_mark;
            int64_t move_cost, tentative;
            bool first_seen;

            if (!has_jump)
                continue;

            nb_id = jps__id(pf, jump.x, jump.y);
            nb_mark = pf->mark[nb_id];   /* 读一次，closed 判定与 first_seen 共用 */
            if (nb_mark == closed_mark)
                continue;

            move_cost = (int64_t)jump.steps * (d.diagonal ? JPS_DIAGONAL_COST : JPS_CARDINAL_COST);
            tentative = cur_g + move_cost;

            first_seen = nb_mark < open_mark;
            if (!first_seen && tentative >= jps__gd_g(pf->g_dir[nb_id]))
                continue;

            /* g、steps、parent dx/dy 同字：一条 8 字节 store 同时写入三者（原独立 steps 数组已并入）。 */
            pf->g_dir[nb_id] = jps__pack_gdir(tentative, jump.steps, dx, dy);
            pf->mark[nb_id] = (uint16_t)open_mark;

            jps_min_heap_enqueue(&pf->open, jps__pack_xy(jump.x, jump.y),
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
        out_xy[i * 2]     = pf->result.path[i].x;
        out_xy[i * 2 + 1] = pf->result.path[i].y;
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
