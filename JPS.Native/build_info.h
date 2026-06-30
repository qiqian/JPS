/*
 * build_info.h
 * JPS Pathfinding — C port of JPS.Core/Pathfinding/JpsBuildInfo.cs
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

#ifndef JPS_BUILD_INFO_H
#define JPS_BUILD_INFO_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 反映本库的编译期配置，供运行时诊断 / 打印。 */

/* 是否允许斜穿角：定义 JPS_ALLOW_CORNER_CUTTING 为 true；默认 false（禁止切角）。 */
static inline bool jps_build_corner_cutting(void)
{
#ifdef JPS_ALLOW_CORNER_CUTTING
    return true;
#else
    return false;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* JPS_BUILD_INFO_H */
