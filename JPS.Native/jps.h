/*
 * jps.h
 * JPS Pathfinding — 唯一对外发布的公共头（供 C / C++ / C# P/Invoke 调用）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 *
 * **集成只需这一个头文件**：导出宏、返回码、两个不透明句柄的前向声明与全部公共函数都在这里；
 * 结构体定义与内部模块（grid_map / jump_point_cache / pathfinder / smoother …）不对外暴露。
 *
 * 对外只有两个不透明句柄，与 C# 版 JpsSystem / JpsPathfinder 设计一致：
 *
 *   jps_system     —— 拥有地图 + 惰性跳点缓存；承载阻挡编辑与缓存同步。
 *   jps_pathfinder —— 持有逐节点搜索状态；寻路时绑定到某个 jps_system。
 *                     **多个 jps_pathfinder 可共享同一个 jps_system。**
 *
 * 典型用法（C 伪代码；C# 端用 DllImport 一一对应）：
 *
 *   jps_system *s = jps_system_create(w, h);
 *   jps_system_set_blocked(s, x, y, 1);        // 设置阻挡
 *   jps_system_sync(s);                         // 阻挡改动后同步缓存
 *
 *   jps_pathfinder *pf = jps_pathfinder_create();
 *   int n = jps_pathfinder_find_path(pf, s, sx, sy, gx, gy);   // n>=0 compact path 点数；成功时已完成平滑
 *   if (n > 0) {
 *       int *buf = malloc(n * 2 * sizeof(int));
 *       jps_pathfinder_copy_path(pf, buf, n);                  // 取出 compact path（x,y 交错）
 *       int sn = jps_pathfinder_smoothed_path_count(pf);
 *       float *sbuf = malloc(sn * 2 * sizeof(float));
 *       jps_pathfinder_copy_smoothed_path(pf, sbuf, sn);       // 取出平滑路径（x,y 交错）
 *   }
 *   // 可再创建更多 pf 共用同一个 s 并行/串行寻路
 *
 *   jps_pathfinder_destroy(pf);
 *   jps_system_destroy(s);
 *
 * 设计取舍：所有参数/返回值均为基本类型，不跨 ABI 传结构体；路径用
 * “find 计算并缓存 → copy 进调用方缓冲” 两步式取出，不让 C# 释放 C 堆。
 */

#ifndef JPS_H
#define JPS_H

#include <stdint.h>

/*
 * 链接修饰：
 *   - 定义 JPS_STATIC          → 既不 export 也不 import（静态库 / 单元测试 / 内部直连；iOS 静态库即此路）。
 *   - 定义 JPSNATIVE_EXPORTS   → 构建 DLL 本体时导出（见 vcxproj）。
 *   - 两者都不定义             → 消费方（C/C++）按导入处理。
 * 调用约定固定 __cdecl，C# 端 P/Invoke 默认即 Cdecl。
 * 非 Windows 用 visibility("default")，配合 -fvisibility=hidden 只导出这里标了 JPS_API 的公共接口。
 */
#if defined(JPS_STATIC)
#  define JPS_API
#  if defined(_WIN32)
#    define JPS_CALL __cdecl
#  else
#    define JPS_CALL
#  endif
#elif defined(_WIN32)
#  if defined(JPSNATIVE_EXPORTS)
#    define JPS_API __declspec(dllexport)
#  else
#    define JPS_API __declspec(dllimport)
#  endif
#  define JPS_CALL __cdecl
#else
#  define JPS_API __attribute__((visibility("default")))
#  define JPS_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* jps_pathfinder_find_path 的返回码：>=0 为找到的 compact path 点数；负值为错误。 */
enum
{
    JPS_ERR_NULL          = -1,   /* 句柄为 NULL */
    JPS_ERR_OUT_OF_BOUNDS = -2,   /* 起点或终点越界 */
    JPS_ERR_BLOCKED       = -3,   /* 起点或终点本身是阻挡 */
    JPS_ERR_NO_PATH       = -4    /* 搜索完毕未找到可达路径 */
};

/*
 * 两个不透明句柄：结构体定义在内部头（system.h / pathfinder.c），对消费方只是前向声明。
 * 一律通过下面的函数创建 / 使用 / 销毁，不跨 ABI 传结构体。
 */
typedef struct jps_system jps_system;
typedef struct jps_pathfinder jps_pathfinder;

/* ==================== jps_system：建图 / 改阻挡 / 同步 ==================== */

/*
 * 创建一张 width×height 的空地图（全部可走）及其跳点缓存。
 * 尺寸必须为正且边长 ≤ 32767；非法尺寸或内存不足返回 NULL。
 */
JPS_API jps_system *JPS_CALL jps_system_create(int width, int height);

/* 销毁 system，释放其地图与缓存。传 NULL 安全无操作。 */
JPS_API void JPS_CALL jps_system_destroy(jps_system *s);

JPS_API int JPS_CALL jps_system_width(const jps_system *s);
JPS_API int JPS_CALL jps_system_height(const jps_system *s);

/* 当前 system 保留的 native 内存字节数估算：system 本体 + 地图 + 共享跳点缓存。NULL 返回 0。 */
JPS_API uint64_t JPS_CALL jps_system_memory_bytes(const jps_system *s);

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
 * 批量应用稀疏阻挡增量：xyv 为 (x, y, blocked) 三元组连续排布，长度 = edit_count*3。
 * 一次调用应用全部改动（每格等价 jps_system_set_blocked），越界项忽略。
 * 相比逐格 set_blocked，跨 ABI 只需一次调用——适合动态地图每帧只改一小簇格子的场景。
 */
JPS_API void JPS_CALL jps_system_set_blocked_batch(jps_system *s, const int *xyv, int edit_count);

/*
 * 把缓存同步到当前地图：按地图版本号同步受影响的行/列。
 * 寻路前调用；尤其在改动阻挡之后、find_path 之前必须调用一次。
 */
JPS_API void JPS_CALL jps_system_sync(jps_system *s);

/* ==================== jps_pathfinder：寻路 / 取结果 ==================== */

JPS_API jps_pathfinder *JPS_CALL jps_pathfinder_create(void);
JPS_API void JPS_CALL jps_pathfinder_destroy(jps_pathfinder *pf);

/* 当前 pathfinder 保留的 native 内存字节数估算：本体 + 搜索/堆/结果/平滑缓冲。NULL 返回 0。 */
JPS_API uint64_t JPS_CALL jps_pathfinder_memory_bytes(const jps_pathfinder *pf);

/*
 * 在 system 上从 (sx,sy) 到 (gx,gy) 寻路（禁止斜穿角，整数代价 1000/1414，octile 启发）。
 * 结果（compact path + smoothed path + 展开节点数）暂存于 pf，供随后 copy / 查询。
 * 调用前需确保 system 已 jps_system_sync（尤其阻挡改动之后）。
 * 返回：>=0 = compact path 点数（含起终点；start==goal 时为 1）；成功时已同步完成路径平滑。
 *       负值见上方 JPS_ERR_*。
 */
JPS_API int JPS_CALL jps_pathfinder_find_path(jps_pathfinder *pf, jps_system *system,
                                              int sx, int sy, int gx, int gy);

/*
 * 兜底版寻路（内建 goal-snapping）：与 find_path 相同，但——
 *   1) 允许 gx,gy 落在阻挡上（膨胀后 goal 进障碍的大体型场景）；
 *   2) 若 gx,gy 被挡，先 **goal-snapping**：把目标移到离它最近、且朝 start 一侧的可走格（接近侧接触格），
 *      再对这个有效目标寻路——够得到就停在接触格（reached=1）；
 *   3) 连有效目标也到不了（被墙围死等）时不返回 NO_PATH，而是返回搜索展开过的、离有效目标最近
 *      （octile 启发最小）的节点路径（起点亦在候选内，故至少 1 点，reached=0）。
 * 返回：>=1 = compact path 点数（成功时已完成平滑；用 jps_pathfinder_reached_goal 区分到达/最近点）；
 *       <0 见 JPS_ERR_*（起点越界/被挡、goal 越界仍报错；goal 被挡在本函数中不再是错误）。
 * compact/smoothed path 与 reached 标志经现有 copy/count/reached_goal 访问器读取。
 * 路径末点即实际落脚格（goal 或其 snap）；调用方读末点即知停在哪。
 */
JPS_API int JPS_CALL jps_pathfinder_find_path_nearest(jps_pathfinder *pf, jps_system *system,
                                                      int sx, int sy, int gx, int gy);

/* 最近一次寻路的 compact path 点数（起点 + 跳点/拐点 + 终点；未找到/未调用为 0）。配 jps_pathfinder_copy_path。 */
JPS_API int JPS_CALL jps_pathfinder_path_count(const jps_pathfinder *pf);

/*
 * 最近一次寻路是否到达**有效目标**：1=到达（find_path 的 goal，或 find_path_nearest 的 goal/其 snap 接触格）；
 * 0=返回的是离目标最近的已展开点（find_path_nearest 未达）或无结果。
 * 供 find_path_nearest 调用方区分"到达可交互落脚格"与"尽力最近"；对严格 find_path 成功时恒为 1。
 */
JPS_API int JPS_CALL jps_pathfinder_reached_goal(const jps_pathfinder *pf);

/*
 * 最近一次寻路的**平滑**路径点数（视线拉直后的折线；未找到/未调用为 0）。配 jps_pathfinder_copy_smoothed_path。
 * 平滑在 find_path 成功时已计算并缓存；本函数只返回缓存点数。pf 非 const 仅用于兼容旧声明。
 */
JPS_API int JPS_CALL jps_pathfinder_smoothed_path_count(jps_pathfinder *pf);

/*
 * 把最近一次找到的 compact path 拷进调用方缓冲。不暴露 expanded per-cell path。
 * out_xy 按 x0,y0,x1,y1,... 交错存放，capacity_points 为可容纳点数（out_xy 至少 capacity_points*2 个 int）。
 * 返回实际写入点数 = min(path_count, capacity_points)。
 */
JPS_API int JPS_CALL jps_pathfinder_copy_path(const jps_pathfinder *pf, int *out_xy, int capacity_points);

/*
 * 把最近一次寻路的**平滑路径**拷进调用方缓冲：视线拉直后的连续格中心点（cx+0.5, cy+0.5），
 * out_xy 按 x0,y0,x1,y1,... 交错的 float（至少 capacity_points*2 个）。
 * 平滑在 find_path 成功后已算好并缓存，这里只拷——无二次计算、不需要 system。
 * 返回实际写入点数 = min(smoothed_path_count, capacity_points)；无路径返回 0。因内部会填充缓存，pf 非 const。
 */
JPS_API int JPS_CALL jps_pathfinder_copy_smoothed_path(jps_pathfinder *pf, float *out_xy, int capacity_points);

#ifdef __cplusplus
}
#endif

#endif /* JPS_H */
