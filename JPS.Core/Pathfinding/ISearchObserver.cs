namespace JPS.Pathfinding
{
    /// <summary>
    /// 搜索过程的可观测钩子：寻路器在关键事件点回调它，用于可视化 / 诊断。
    ///
    /// 算法核心（JpsPathfinder / AStarPathfinder）**不持有任何可视化数据结构**，只在
    /// 展开 / 入队 / 扫描时发出事件；具体的收集、去重与存储全部由实现方（UI 层）负责。
    /// 不传 observer（null）时，这些事件完全不触发，算法是纯净的最短开销路径。
    /// </summary>
    public interface ISearchObserver
    {
        /// <summary>一个节点出队并被展开（它是跳点）。</summary>
        void OnExpand(int x, int y);

        /// <summary>一个节点首次入队（前沿候选；之后若被展开则不再算前沿）。</summary>
        void OnFrontier(int x, int y);

        /// <summary>跳跃射线扫过、但未进入开放列表的格子（“扫一眼就跳过/抛弃”）。</summary>
        void OnScan(int x, int y);
    }
}
