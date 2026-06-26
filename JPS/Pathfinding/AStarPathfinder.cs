using JPS.Models;

namespace JPS.Pathfinding;

/// <summary>
/// 标准 8 邻接 A* 寻路，作为 JPS 的对照组。
///
/// 与 JPS 的区别：A* 每展开一个节点，都要实时检查它周围 8 个邻居并逐个入队，
/// 不做任何“跳跃”，因此扩展的节点数远多于 JPS（会铺满一大片）。它不需要预计算。
///
/// 为公平对比，移动规则与本项目 JPS 一致（对角只要目标格可走即可，允许斜穿拐角），
/// 启发式同为整数八方向 octile 距离；性能手法也相同：
/// 扁平数组 + int 节点 + 代次戳免清零 + 缓冲区复用，全程整数运算。
/// </summary>
public sealed class AStarPathfinder
{
    /// <summary>每个方向的移动代价（正交 1000 / 对角 1414），按方向索引预存，省去循环内分支。</summary>
    private static readonly long[] DirCost = BuildDirCost();

    private static long[] BuildDirCost()
    {
        var costs = new long[JpsDirections.Count];
        for (int i = 0; i < JpsDirections.Count; i++)
        {
            var (dx, dy) = JpsDirections.All[i];
            costs[i] = JpsDirections.MoveCost(dx, dy);
        }
        return costs;
    }

    // ---- 与 JpsPathfinder 同款的复用缓冲区（按 id=y*W+x 索引）----
    private int _w, _h, _size;
    private long[] _g = [];          // 各节点已知最短代价 g
    private int[] _parent = [];      // 父节点 id（-1 表示无）
    private int[] _seenGen = [];     // 是否已生成（代次戳）
    private int[] _closedGen = [];   // 是否已展开（代次戳）
    private int _gen;                // 当前查询代次
    private readonly PriorityQueue<int, long> _open = new();

    private readonly List<int> _expandedIds = [];
    private readonly List<int> _generatedIds = [];

    /// <summary>
    /// 标准 A* 搜索。collectDebug 控制是否采集可视化数据（已扩展/前沿格子）。
    /// </summary>
    public PathResult FindPath(GridMap map, bool collectDebug = true)
    {
        if (!map.HasStart || !map.HasEnd)
            return new PathResult { Message = "请先设置起点和终点。" };

        if (!map.IsWalkable(map.StartX, map.StartY) || !map.IsWalkable(map.EndX, map.EndY))
            return new PathResult { Message = "起点或终点位于阻挡上。" };

        EnsureBuffers(map);
        NextGeneration();   // 代次自增，旧标记自动失效

        if (collectDebug)
        {
            _expandedIds.Clear();
            _generatedIds.Clear();
        }

        int gx = map.EndX, gy = map.EndY;
        int startId = Id(map.StartX, map.StartY);
        int goalId = Id(gx, gy);

        _open.Clear();
        _g[startId] = 0;
        _seenGen[startId] = _gen;
        _parent[startId] = -1;
        _open.Enqueue(startId, JpsDirections.OctileHeuristic(map.StartX, map.StartY, gx, gy));

        int expandedCount = 0;

        // A* 主循环：每次取出 f 最小的节点
        while (_open.TryDequeue(out int current, out _))
        {
            if (_closedGen[current] == _gen)
                continue;   // 惰性删除：已展开的重复入队项跳过

            _closedGen[current] = _gen;
            expandedCount++;
            if (collectDebug)
                _expandedIds.Add(current);

            if (current == goalId)
                return Success(startId, goalId, expandedCount, collectDebug);

            int cx = current % _w;
            int cy = current / _w;
            long gCur = _g[current];

            // 遍历 8 个邻居
            for (int i = 0; i < JpsDirections.Count; i++)
            {
                var (dx, dy) = JpsDirections.All[i];
                int nx = cx + dx;
                int ny = cy + dy;

                if (!map.IsWalkable(nx, ny))
                    continue;

                int nbId = ny * _w + nx;
                if (_closedGen[nbId] == _gen)
                    continue;

                // 松弛：首次到达或找到更短 g 才更新
                long tentative = gCur + DirCost[i];
                bool firstSeen = _seenGen[nbId] != _gen;
                if (!firstSeen && tentative >= _g[nbId])
                    continue;

                _g[nbId] = tentative;
                _seenGen[nbId] = _gen;
                _parent[nbId] = current;

                if (collectDebug && firstSeen)
                    _generatedIds.Add(nbId);

                // f = g + 启发值（octile，可采纳 → A* 最优）
                _open.Enqueue(nbId, tentative + JpsDirections.OctileHeuristic(nx, ny, gx, gy));
            }
        }

        return Failure(expandedCount, collectDebug);
    }

    /// <summary>找到终点：回溯路径并按需附带可视化数据。</summary>
    private PathResult Success(int startId, int goalId, int expandedCount, bool collectDebug)
    {
        var path = ReconstructPath(startId, goalId);

        if (!collectDebug)
        {
            return new PathResult
            {
                Success = true,
                Path = path,
                ExpandedNodes = expandedCount,
                Message = $"A*：扩展 {expandedCount}，路径 {path.Count} 格。"
            };
        }

        var expanded = ToPoints(_expandedIds);
        var frontier = FrontierPoints();
        return new PathResult
        {
            Success = true,
            Path = path,
            Expanded = expanded,
            Frontier = frontier,
            ExpandedNodes = expandedCount,
            Message = $"A*：扩展 {expanded.Count}，入队未扩展 {frontier.Count}，搜索合计 {expanded.Count + frontier.Count} 格，路径 {path.Count} 格。"
        };
    }

    /// <summary>开放列表耗尽仍未到终点：无解。</summary>
    private PathResult Failure(int expandedCount, bool collectDebug)
    {
        if (!collectDebug)
            return new PathResult { ExpandedNodes = expandedCount, Message = $"A*：未找到路径（扩展 {expandedCount}）。" };

        var expanded = ToPoints(_expandedIds);
        var frontier = FrontierPoints();
        return new PathResult
        {
            Expanded = expanded,
            Frontier = frontier,
            ExpandedNodes = expandedCount,
            Message = $"A*：未找到路径（扩展 {expanded.Count} 格）。"
        };
    }

    /// <summary>前沿 = 已生成但搜索结束时仍未展开（仍在 open 里）的节点。</summary>
    private List<(int X, int Y)> FrontierPoints()
    {
        var frontier = new List<(int X, int Y)>();
        foreach (int id in _generatedIds)
            if (_closedGen[id] != _gen)
                frontier.Add((id % _w, id / _w));
        return frontier;
    }

    /// <summary>把节点 id 列表还原为坐标列表。</summary>
    private List<(int X, int Y)> ToPoints(List<int> ids)
    {
        var pts = new List<(int X, int Y)>(ids.Count);
        foreach (int id in ids)
            pts.Add((id % _w, id / _w));
        return pts;
    }

    /// <summary>地图尺寸变化时才重新分配数组，否则复用。</summary>
    private void EnsureBuffers(GridMap map)
    {
        if (_w == map.Width && _size == map.Width * map.Height)
            return;

        _w = map.Width;
        _h = map.Height;
        _size = _w * _h;
        _g = new long[_size];
        _parent = new int[_size];
        _seenGen = new int[_size];
        _closedGen = new int[_size];
        _gen = 0;
    }

    /// <summary>代次自增，使上次查询的 seen/closed 标记自动失效，免去清零。</summary>
    private void NextGeneration()
    {
        _gen++;
        if (_gen != int.MaxValue)
            return;

        Array.Clear(_seenGen);
        Array.Clear(_closedGen);
        _gen = 1;
    }

    /// <summary>坐标 → 一维节点 id。</summary>
    private int Id(int x, int y) => y * _w + x;

    /// <summary>沿 _parent 从终点回溯到起点，反转得到逐格路径（A* 每步一格，无需补段）。</summary>
    private List<(int X, int Y)> ReconstructPath(int startId, int goalId)
    {
        var ids = new List<int> { goalId };
        int current = goalId;
        while (current != startId)
        {
            current = _parent[current];
            ids.Add(current);
        }
        ids.Reverse();

        var path = new List<(int X, int Y)>(ids.Count);
        foreach (int id in ids)
            path.Add((id % _w, id / _w));
        return path;
    }
}
