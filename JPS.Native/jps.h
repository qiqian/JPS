/*
 * jps.h
 * JPS Pathfinding — DLL 公共总头（供 C# P/Invoke 调用）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 *
 * 对外只暴露两个不透明句柄，与 C# 版 JpsSystem / JpsPathfinder 设计一致：
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
 *   int n = jps_pathfinder_find_path(pf, s, sx, sy, gx, gy);   // n>=0 路径格数，<0 见 JPS_ERR_*
 *   if (n > 0) {
 *       int *buf = malloc(n * 2 * sizeof(int));
 *       jps_pathfinder_copy_path(pf, buf, n);                  // 取出路径（x,y 交错）
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

#include "jps_export.h"   /* JPS_API / JPS_CALL / JPS_ERR_* */
#include "system.h"       /* jps_system：建图 / 改阻挡 / sync */
#include "pathfinder.h"   /* jps_pathfinder：寻路 / 取结果 */

#endif /* JPS_H */
