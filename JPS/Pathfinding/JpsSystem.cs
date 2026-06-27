using JPS.Models;

namespace JPS.Pathfinding
{
    /// <summary>
    /// JPS 运行环境：持有一张 <see cref="GridMap"/> 与其对应的 <see cref="JumpPointCache"/>。
    ///
    /// 把跳点缓存从 <see cref="JpsPathfinder"/> 提出来放在这里，是为了**让多个 JpsPathfinder 共享同一份 cache**：
    /// 每个 JpsPathfinder 只持有自己的逐节点搜索状态（g/mark/open/...），而地图与缓存集中在 JpsSystem。
    ///
    /// 用法：
    ///  - 单线程：地图阻挡改动后调用 <see cref="Sync"/>，再 FindPath（本项目工具即如此）。
    ///  - 多线程共享（无锁）：在并行寻路前由**单线程**调用一次 <see cref="Sync"/>（此时地图固定、缓存版本固定），
    ///    随后多个 JpsPathfinder 各自在自己的线程上对同一 JpsSystem 寻路，只读/惰性补写共享缓存。
    ///    并发发布由 <see cref="JumpPointCache"/> 中受条件编译符号 <c>JPS_CONCURRENT_CACHE</c> 控制的全屏障保证
    ///    （定义该符号才线程安全；不定义则为单线程极速模式）。
    /// </summary>
    public sealed class JpsSystem
    {
        public GridMap Map { get; }
        public JumpPointCache Cache { get; }

        public JpsSystem(GridMap map)
        {
            Map = map;
            Cache = new JumpPointCache();
        }

        /// <summary>
        /// 把缓存同步到当前地图：按 <see cref="GridMap.Version"/> 整体置脏（O(1)，回绕时清零）。
        /// 单线程调用——尤其多线程共享时必须在并行寻路**之前**由单线程调用一次。
        /// </summary>
        public void Sync() => Cache.Sync(Map);
    }
}
