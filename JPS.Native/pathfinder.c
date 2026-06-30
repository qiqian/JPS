/*
 * pathfinder.c
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JpsPathfinder.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "pathfinder.h"
#include "smoother.h"
#include "min_heap.h"
#include "directions.h"
#include "rules.h"

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
 * 寻路结果（纯内部状态，不进公共头）：path 是动态数组，path_count 个格坐标，
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

struct jps_pathfinder
{
    /* ---- 按地图尺寸一次性分配、跨多次查询复用的缓冲区 ---- */
    int w, h, size;
    int64_t *g;          /* 各节点已知最短代价 g */
    int8_t *parent_dir;  /* 到达该节点的方向索引（剪枝用 + 回溯反推父节点） */
    int16_t *parent_steps;/* 到达该节点的跳跃步数；父节点 = 当前 − dir×steps */
    int32_t *mark;       /* 访问状态：== 2·gen → open，== 2·gen+1 → closed */
    int gen;
    jps_min_heap open;
    int dir_buf[JPS_DIR_COUNT];

    jps_jump_point_cache *cache;   /* 当前查询绑定的共享跳点缓存 */
    jps_path_result result;        /* 最近一次寻路结果（供 copy/count 访问器读取） */
};

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

static void jps__path_push(jps_path_result *r, int x, int y)
{
    if (r->path_count == r->path_capacity)
    {
        int n = r->path_capacity < 16 ? 16 : r->path_capacity * 2;
        r->path = (jps_point *)realloc(r->path, (size_t)n * sizeof(jps_point));
        r->path_capacity = n;
    }
    r->path[r->path_count].x = x;
    r->path[r->path_count].y = y;
    r->path_count++;
}

/* ---------------- 生命周期 ---------------- */

jps_pathfinder *jps_pathfinder_create(void)
{
    jps_pathfinder *pf = (jps_pathfinder *)malloc(sizeof(jps_pathfinder));
    if (pf == NULL)
        return NULL;
    pf->w = pf->h = pf->size = 0;
    pf->g = NULL;
    pf->parent_dir = NULL;
    pf->parent_steps = NULL;
    pf->mark = NULL;
    pf->gen = 0;
    jps_min_heap_init(&pf->open, 64);
    pf->cache = NULL;
    jps__result_init(&pf->result);
    return pf;
}

void jps_pathfinder_destroy(jps_pathfinder *pf)
{
    if (pf == NULL)
        return;
    free(pf->g);
    free(pf->parent_dir);
    free(pf->parent_steps);
    free(pf->mark);
    jps_min_heap_free(&pf->open);
    jps__result_free(&pf->result);
    free(pf);
}

static void jps__ensure_buffers(jps_pathfinder *pf, const jps_grid_map *m)
{
    if (pf->w == m->width && pf->size == m->width * m->height)
        return;

    pf->w = m->width;
    pf->h = m->height;
    pf->size = pf->w * pf->h;
    free(pf->g);
    free(pf->parent_dir);
    free(pf->parent_steps);
    free(pf->mark);
    pf->g = (int64_t *)malloc((size_t)pf->size * sizeof(int64_t));
    pf->parent_dir = (int8_t *)malloc((size_t)pf->size * sizeof(int8_t));
    pf->parent_steps = (int16_t *)malloc((size_t)pf->size * sizeof(int16_t));
    pf->mark = (int32_t *)calloc((size_t)pf->size, sizeof(int32_t));
    pf->gen = 0;
}

static void jps__next_generation(jps_pathfinder *pf)
{
    pf->gen++;
    if (pf->gen <= (INT_MAX / 2) - 1)
        return;
    memset(pf->mark, 0, (size_t)pf->size * sizeof(int32_t));
    pf->gen = 1;
}

static int jps__id(const jps_pathfinder *pf, int x, int y) { return y * pf->w + x; }

/* ---------------- 正交跳跃 ---------------- */

static jps__jump_entry jps__cardinal_jump(jps_pathfinder *pf, const jps_grid_map *m,
                                          int x, int y, int dx, int dy, int dir, int gx, int gy)
{
    int dist = jps_jump_point_cache_cardinal_dist(pf->cache, m, x, y, dx, dy, dir);
    int max_travel = dist > 0 ? dist : -dist;

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
    for (;;)
    {
        int hd, vd;

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

        /* 正交分量子检测：直接读正交 memo（命中为 O(1)）。短路顺序与 C# 一致。 */
        hd = jps_jump_point_cache_cardinal_dist(pf->cache, m, cx, cy, dx, 0, horizontal_dir);
        if (hd > 0) { jps__jump_entry e = { true, cx, cy, steps }; return e; }
        if (cy == gy && jps_sign(gx - cx) == dx && abs(gx - cx) <= -hd)
        { jps__jump_entry e = { true, cx, cy, steps }; return e; }

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
                : jps_grid_map_is_walkable(m, x + dx, y + dy);
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

        if (!jps_grid_map_is_walkable(m, x - pdx, y))
            pf->dir_buf[n++] = jps_dir_index_of(-pdx, pdy);
        if (!jps_grid_map_is_walkable(m, x, y - pdy))
            pf->dir_buf[n++] = jps_dir_index_of(pdx, -pdy);

        return n;
    }

    pf->dir_buf[n++] = parent_dir;

    if (pdx != 0)
    {
        if (!jps_grid_map_is_walkable(m, x, y + 1)) pf->dir_buf[n++] = jps_dir_index_of(pdx, 1);
        if (!jps_grid_map_is_walkable(m, x, y - 1)) pf->dir_buf[n++] = jps_dir_index_of(pdx, -1);
    }
    else
    {
        if (!jps_grid_map_is_walkable(m, x + 1, y)) pf->dir_buf[n++] = jps_dir_index_of(1, pdy);
        if (!jps_grid_map_is_walkable(m, x - 1, y)) pf->dir_buf[n++] = jps_dir_index_of(-1, pdy);
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
        if (jps_grid_map_is_walkable(m, x, y + 1) && !jps_grid_map_is_walkable(m, x - pdx, y + 1))
        {
            pf->dir_buf[n++] = jps_dir_index_of(0, 1);
            pf->dir_buf[n++] = jps_dir_index_of(pdx, 1);
        }
        if (jps_grid_map_is_walkable(m, x, y - 1) && !jps_grid_map_is_walkable(m, x - pdx, y - 1))
        {
            pf->dir_buf[n++] = jps_dir_index_of(0, -1);
            pf->dir_buf[n++] = jps_dir_index_of(pdx, -1);
        }
    }
    else            /* 垂直移动 */
    {
        if (jps_grid_map_is_walkable(m, x + 1, y) && !jps_grid_map_is_walkable(m, x + 1, y - pdy))
        {
            pf->dir_buf[n++] = jps_dir_index_of(1, 0);
            pf->dir_buf[n++] = jps_dir_index_of(1, pdy);
        }
        if (jps_grid_map_is_walkable(m, x - 1, y) && !jps_grid_map_is_walkable(m, x - 1, y - pdy))
        {
            pf->dir_buf[n++] = jps_dir_index_of(-1, 0);
            pf->dir_buf[n++] = jps_dir_index_of(-1, pdy);
        }
    }

    return n;
#endif
}

/* ---------------- 路径重建 ---------------- */

static void jps__append_segment(jps_path_result *r, int fx, int fy, int tx, int ty)
{
    int dx = jps_sign(tx - fx);
    int dy = jps_sign(ty - fy);
    int x = fx, y = fy;

    if (r->path_count == 0 ||
        r->path[r->path_count - 1].x != fx || r->path[r->path_count - 1].y != fy)
        jps__path_push(r, fx, fy);

    while (x != tx || y != ty)
    {
        x += dx;
        y += dy;
        jps__path_push(r, x, y);
    }
}

static void jps__reconstruct_path(jps_pathfinder *pf, int start_id, int goal_id, jps_path_result *r)
{
    /* 先沿父链收集跳点节点（goal → start），再反转。 */
    int *nodes = NULL;
    int nodes_count = 0, nodes_cap = 0;
    int current = goal_id;
    int i;

    /* 收集 */
    for (;;)
    {
        if (nodes_count == nodes_cap)
        {
            nodes_cap = nodes_cap < 16 ? 16 : nodes_cap * 2;
            nodes = (int *)realloc(nodes, (size_t)nodes_cap * sizeof(int));
        }
        nodes[nodes_count++] = current;

        if (current == start_id)
            break;

        {
            int dx = jps_dir_dx[pf->parent_dir[current]];
            int dy = jps_dir_dy[pf->parent_dir[current]];
            int steps = pf->parent_steps[current];
            int cx = current % pf->w, cy = current / pf->w;
            current = (cy - dy * steps) * pf->w + (cx - dx * steps);
        }
    }

    /* 反转为 start → goal */
    for (i = 0; i < nodes_count / 2; i++)
    {
        int t = nodes[i];
        nodes[i] = nodes[nodes_count - 1 - i];
        nodes[nodes_count - 1 - i] = t;
    }

    r->path_count = 0;

    /* start==goal 退化：只有一个节点，段循环跑 0 次而漏掉它——直接补回该格。 */
    if (nodes_count == 1)
    {
        jps__path_push(r, goal_id % pf->w, goal_id / pf->w);
        free(nodes);
        return;
    }

    for (i = 0; i < nodes_count - 1; i++)
    {
        int from_id = nodes[i];
        int to_id = nodes[i + 1];
        jps__append_segment(r, from_id % pf->w, from_id / pf->w, to_id % pf->w, to_id / pf->w);
    }

    free(nodes);
}

/* ---------------- 入口 ---------------- */

int jps_pathfinder_find_path(jps_pathfinder *pf, jps_system *system,
                             int sx, int sy, int gx, int gy)
{
    jps_grid_map *map;
    jps_path_result *out;
    int open_mark, closed_mark;
    int start_id, goal_id;
    int expanded_count = 0;
    int current;
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
    jps__next_generation(pf);   /* 缓存同步由 jps_system_sync 负责（调用方在寻路前完成） */

    open_mark = pf->gen * 2;        /* 本代“已生成/在 open”标记 */
    closed_mark = open_mark + 1;    /* 本代“已展开/closed”标记 */

    start_id = jps__id(pf, sx, sy);
    goal_id = jps__id(pf, gx, gy);

    jps_min_heap_clear(&pf->open);
    pf->g[start_id] = 0;
    pf->mark[start_id] = open_mark;
    pf->parent_dir[start_id] = -1;   /* 起点无来向 */
    jps_min_heap_enqueue(&pf->open, start_id, jps_octile_heuristic(sx, sy, gx, gy));

    while (jps_min_heap_try_dequeue(&pf->open, &current, &prio))
    {
        int cx, cy, dir_count, i;

        if (pf->mark[current] == closed_mark)
            continue;

        pf->mark[current] = closed_mark;
        expanded_count++;

        cx = current % pf->w;
        cy = current / pf->w;

        if (current == goal_id)
        {
            jps__reconstruct_path(pf, start_id, goal_id, out);
            out->success = true;
            out->expanded_nodes = expanded_count;
            return out->path_count;
        }

        dir_count = jps__fill_directions(pf, map, cx, cy, pf->parent_dir[current]);

        for (i = 0; i < dir_count; i++)
        {
            int idx = pf->dir_buf[i];
            int dx = jps_dir_dx[idx];
            int dy = jps_dir_dy[idx];
            bool diagonal = jps_is_diagonal_index(idx);
            jps__jump_entry jump = diagonal
                ? jps__diagonal_jump(pf, map, cx, cy, dx, dy, gx, gy)
                : jps__cardinal_jump(pf, map, cx, cy, dx, dy, idx, gx, gy);

            int nb_id;
            int64_t move_cost, tentative;
            bool first_seen;

            if (!jump.has_jump)
                continue;

            nb_id = jps__id(pf, jump.x, jump.y);
            if (pf->mark[nb_id] == closed_mark)
                continue;

            move_cost = (int64_t)jump.steps * (diagonal ? JPS_DIAGONAL_COST : JPS_CARDINAL_COST);
            tentative = pf->g[current] + move_cost;

            first_seen = pf->mark[nb_id] < open_mark;
            if (!first_seen && tentative >= pf->g[nb_id])
                continue;

            pf->g[nb_id] = tentative;
            pf->mark[nb_id] = open_mark;
            pf->parent_dir[nb_id] = (int8_t)idx;
            pf->parent_steps[nb_id] = (int16_t)jump.steps;

            jps_min_heap_enqueue(&pf->open, nb_id,
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

int jps_pathfinder_expanded_nodes(const jps_pathfinder *pf)
{
    return pf ? pf->result.expanded_nodes : 0;
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

int jps_pathfinder_copy_smoothed_path(const jps_pathfinder *pf, const jps_system *system,
                                      float *out_xy, int capacity_points)
{
    jps_point_f *pts = NULL;
    int produced, n, i;

    if (pf == NULL || system == NULL || out_xy == NULL ||
        !pf->result.success || pf->result.path_count == 0)
        return 0;

    /* 直接将平滑结果写入调用者提供的 out_xy，避免内部 malloc。 */
    produced = jps_smooth_path_into(system->map, pf->result.path, pf->result.path_count,
                                     (jps_point_f *)out_xy, capacity_points);
    /* produced 是理论产生的点数；实际写入已受 capacity_points 限制，返回写入点数（<= capacity_points） */
    return produced < capacity_points ? produced : capacity_points;
}
