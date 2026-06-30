/*
 * jps_export.h
 * JPS Pathfinding — DLL 导出宏与公共错误码（被各公共头共享）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_EXPORT_H
#define JPS_EXPORT_H

/*
 * 链接修饰：
 *   - 定义 JPS_STATIC          → 既不 export 也不 import（静态库 / 单元测试 / 内部直连）。
 *   - 定义 JPSNATIVE_EXPORTS   → 构建 DLL 本体时导出（见 vcxproj）。
 *   - 两者都不定义             → 消费方（C# 之外的 C/C++）按导入处理。
 * 调用约定固定 __cdecl，C# 端 P/Invoke 默认即 Cdecl。
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

/* jps_pathfinder_find_path 的返回码：>=0 为找到的路径格数；负值为错误。 */
enum
{
    JPS_ERR_NULL          = -1,   /* 句柄为 NULL */
    JPS_ERR_OUT_OF_BOUNDS = -2,   /* 起点或终点越界 */
    JPS_ERR_BLOCKED       = -3,   /* 起点或终点本身是阻挡 */
    JPS_ERR_NO_PATH       = -4    /* 搜索完毕未找到可达路径 */
};

#endif /* JPS_EXPORT_H */
