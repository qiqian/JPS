/*
 * system.h
 * JPS Pathfinding — 公共句柄之一：jps_system（C 端对应 JPS.Core/Pathfinding/JpsSystem.cs）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_SYSTEM_H
#define JPS_SYSTEM_H

#include <stdint.h>
#include "jps_export.h"
#include "grid_map.h"
#include "jump_point_cache.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * JPS 运行环境（公共句柄）：拥有一张地图与其对应的惰性跳点缓存。
 *
 * 把地图与缓存集中在这里，是为了让**多个 jps_pathfinder 共享同一个 jps_system**：
 * 每个 jps_pathfinder 只持有自己的逐节点搜索状态，地图/缓存都在 jps_system。
 *
 * 典型用法：
 *   s  = jps_system_create(w, h);
 *   jps_system_set_blocked(s, x, y, 1); ...   // 设置阻挡
 *   jps_system_sync(s);                        // 阻挡改动后同步缓存
 *   pf = jps_pathfinder_create();
 *   jps_pathfinder_find_path(pf, s, sx, sy, gx, gy);   // 可创建多个 pf 共用同一个 s
 *
 * 结构体定义对内部 .c 可见（pathfinder 需访问 map/cache）；对 C# 而言它是不透明指针。
 */
typedef struct jps_system
{
    jps_grid_map *map;             /* 拥有，destroy 时释放 */
    jps_jump_point_cache *cache;   /* 拥有，destroy 时释放 */
} jps_system;

/*
 * 创建一张 width×height 的空地图（全部可走）及其跳点缓存。
 * 尺寸必须为正且边长 ≤ 32767；非法尺寸或内存不足返回 NULL。
 */
JPS_API jps_system *JPS_CALL jps_system_create(int width, int height);

/* 销毁 system，释放其地图与缓存。传 NULL 安全无操作。 */
JPS_API void JPS_CALL jps_system_destroy(jps_system *s);

JPS_API int JPS_CALL jps_system_width(const jps_system *s);
JPS_API int JPS_CALL jps_system_height(const jps_system *s);

/* ---- 阻挡编辑（改动后需 jps_system_sync 才会令缓存失效生效） ---- */

/* 设置/清除单格阻挡（blocked 非 0 = 阻挡）。越界忽略。 */
JPS_API void JPS_CALL jps_system_set_blocked(jps_system *s, int x, int y, int blocked);

/* 查询单格是否阻挡：1=阻挡，0=可走（越界视为阻挡，返回 1）。 */
JPS_API int JPS_CALL jps_system_is_blocked(const jps_system *s, int x, int y);

/* 清空全部阻挡（整图复位为可走）。 */
JPS_API void JPS_CALL jps_system_clear_all(jps_system *s);

/*
 * 批量载入阻挡：cells 为行主序（长度 = width*height）的字节数组，0=可走、非 0=阻挡。
 * count 须等于 width*height，否则忽略。适合一次性刷整张阻挡图。
 */
JPS_API void JPS_CALL jps_system_set_blocked_buffer(jps_system *s, const uint8_t *cells, int count);

/*
 * 把缓存同步到当前地图：按地图版本号 O(1) 整体置脏。
 * 寻路前调用；尤其在改动阻挡之后、find_path 之前必须调用一次。
 */
JPS_API void JPS_CALL jps_system_sync(jps_system *s);

#ifdef __cplusplus
}
#endif

#endif /* JPS_SYSTEM_H */
