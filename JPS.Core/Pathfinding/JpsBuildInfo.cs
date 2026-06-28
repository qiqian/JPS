/*
 * JpsBuildInfo.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian. MIT License.
 */

namespace JPS.Pathfinding
{
    /// <summary>
    /// 反映 JPS.Core 的编译期配置，供运行时诊断 / 打印。
    /// 这两个常量由 JPS.Core 自身编译时的条件编译符号决定，因此准确反映核心库当前行为
    /// （而不是引用方项目的符号）。
    /// </summary>
    public static class JpsBuildInfo
    {
        /// <summary>是否允许斜穿角：定义 JPS_ALLOW_CORNER_CUTTING 为 true；默认 false（禁止切角）。</summary>
        public const bool CornerCutting =
#if JPS_ALLOW_CORNER_CUTTING
            true;
#else
            false;
#endif

        /// <summary>是否启用无锁多线程共享缓存：定义 JPS_CONCURRENT_CACHE 为 true；默认开启。</summary>
        public const bool ConcurrentCache =
#if JPS_CONCURRENT_CACHE
            true;
#else
            false;
#endif
    }
}
