/*
 * jump_point_cache.h
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JumpPointCache.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_JUMP_POINT_CACHE_H
#define JPS_JUMP_POINT_CACHE_H

#include <stdbool.h>
#include <stdint.h>
#include "grid_map.h"
#include "jps_atomic.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Lazy cardinal jump-point cache（按方向 SoA：dist 与 gen 各拆成 4 个连续平面）。
 * 方向索引 0=E,1=W,2=S,3=N（即 jps_dir_index_of 的正交结果）。
 *
 * 布局：dist / gen 各长 4*size，方向主序——方向 dir 的第 idx 格 = dist[(size_t)dir*size + idx]、
 * gen[(size_t)dir*size + idx]。总字节数与旧 AoS(int16 dist[4]+uint8 gen[4]=12B/格)完全相同，
 * 但同一方向沿行(E/W)或沿列(S/N，转置后)相邻的格在平面内连续，故回写整段 run 时 dist 是一段连续 int16、
 * 可用 16 位车道 SIMD 一次生成并写 8 个（gen 是同值 line_gen 的连续段）。
 * dist >0 跳点距离，<=0 到墙距离；gen 等于当前行/列有效世代即 clean。
 *
 * 并发模型不变：写者先普通写 dist、再 release-store gen 发布；读者 acquire-load gen，命中再普通读 dist。
 * 单线程 Sync 后多个 jps_pathfinder 可共享同一缓存并行只读/惰性补写（补写值是固定地图的纯函数）。
 *
 * ---- 行/列世代失效机制（与 jps_pathfinder 的查询纪元 epoch/mark 是**两套独立机制**）----
 * 这套世代随「地图改动」推进、跨查询存活，判定缓存条目是否失效；
 * pathfinder 那套随「每次查询」推进、判定节点访问状态（见 pathfinder.c）。二者互不作用。
 *
 * 三层结构与不变量：
 *   · map 的 row_version/col_version（int）：set_blocked(x,y) 时 bump 行 y±1、列 x±1——
 *     单格变化只影响这些线上的水平/垂直扫描结果；
 *   · 本结构的 row_version/col_version 镜像上者，Sync 时逐线比对，不等 → bump 该线的
 *     row_gen/col_gen（有效世代）；初始化为 -1 哨兵（≠ 地图侧初始 0），保证首次 Sync
 *     把每条线都 bump 到 ≥1；
 *   · cell 侧 gen 平面（uint8）：某格某方向 clean ⇔ gen[dir*size+idx] == 对应线的有效世代。
 *
 * 关键不变量：**cell gen 的 0 是保留值（恒 dirty），line 有效世代只在 1..255 循环**。
 *   回绕（line gen 到 255 再失效）时把该线两个方向的 gen 平面 memset 为 0、line gen 复位为 1；
 *   一个循环周期内 cell 只会写入“当时的 line gen”(1..255)，而上个周期的同值已被 memset 清 0，
 *   故不存在跨周期的 stale-clean 别名。回绕代价 = 单线 memset，罕见且便宜。
 */
typedef struct jps_jump_point_cache
{
    int w;
    int h;
    int size;
    int16_t *dist;        /* 4 个方向平面拼接：dist[dir*size + idx] */
    uint8_t *gen;         /* 4 个方向平面拼接：gen[dir*size + idx]；0 = 恒 dirty（保留值） */
    uint8_t *row_gen;     /* 每行 E/W 有效世代（dir 0/1），1..255 循环 */
    uint8_t *col_gen;     /* 每列 S/N 有效世代（dir 2/3），1..255 循环 */
    int *row_version;     /* 已同步的地图行版本（-1 哨兵 → 首次 Sync 全失效） */
    int *col_version;     /* 已同步的地图列版本（同上） */
    int map_version;      /* 已同步的地图总版本（快速跳过无变化的 Sync） */
} jps_jump_point_cache;

jps_jump_point_cache *jps_jump_point_cache_create(void);
void jps_jump_point_cache_destroy(jps_jump_point_cache *c);

/* 每次搜索开始时调用：按尺寸准备缓冲，并在地图版本变化时同步受影响的行/列。 */
void jps_jump_point_cache_sync(jps_jump_point_cache *c, jps_grid_map *m);

/*
 * 取 (x,y) 沿正交方向 (dx,dy) 的带符号跳点距离。
 * 命中 clean 直接读；未命中沿射线扫一次并把整段 run 一起洗白。
 */
int jps_jump_point_cache_cardinal_dist(jps_jump_point_cache *c, const jps_grid_map *m,
                                       int x, int y, int dx, int dy, int dir);

/*
 * 热路内联快探：dir 平面上第 idx 格若 clean（gen==line_gen）则 *out 置 dist 并返回 true；
 * dirty 返回 false（调用方走完整慢路 cardinal_dist：扫描+回填）。
 * distp/genp 为该方向平面基址（c->dist/gen + dir*size）、line_gen 为该格所在行/列的有效世代，
 * 均由调用方在热循环外解出——省去每次探测的函数调用与基址/世代重复解算。
 * 内存序与完整版一致：acquire 读 gen，命中再普通读 dist（见 jps_atomic.h 的发布协议）。
 */
static inline bool jps_jump_probe(const int16_t *distp, const uint8_t *genp,
                                  int idx, uint8_t line_gen, int *out)
{
    if (jps_gen_load_acquire(&genp[idx]) != line_gen)
        return false;
    *out = distp[idx];
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* JPS_JUMP_POINT_CACHE_H */
