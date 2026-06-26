using JPS.Models;

namespace JPS.Pathfinding;

/// <summary>
/// 标准 8 邻接 A* 寻路，作为 JPS 的对照组。
/// 移动规则与本项目的 JPS 保持一致（对角只要目标格可走即可），以便公平比较。
/// 性能上与 JpsPathfinder 采用同样的手法：扁平数组 + int 节点 + 代次戳免清零 + buffer 复用。
/// </summary>
public sealed class AStarPathfinder
{
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

    private int _w, _h, _size;
    private long[] _g = [];
    private int[] _parent = [];
    private int[] _seenGen = [];
    private int[] _closedGen = [];
    private int _gen;
    private readonly PriorityQueue<int, long> _open = new();

    private readonly List<int> _expandedIds = [];
    private readonly List<int> _generatedIds = [];

    public PathResult FindPath(GridMap map, bool collectDebug = true)
    {
        if (!map.HasStart || !map.HasEnd)
            return new PathResult { Message = "请先设置起点和终点。" };

        if (!map.IsWalkable(map.StartX, map.StartY) || !map.IsWalkable(map.EndX, map.EndY))
            return new PathResult { Message = "起点或终点位于阻挡上。" };

        EnsureBuffers(map);
        NextGeneration();

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

        while (_open.TryDequeue(out int current, out _))
        {
            if (_closedGen[current] == _gen)
                continue;

            _closedGen[current] = _gen;
            expandedCount++;
            if (collectDebug)
                _expandedIds.Add(current);

            if (current == goalId)
                return Success(startId, goalId, expandedCount, collectDebug);

            int cx = current % _w;
            int cy = current / _w;
            long gCur = _g[current];

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

                long tentative = gCur + DirCost[i];
                bool firstSeen = _seenGen[nbId] != _gen;
                if (!firstSeen && tentative >= _g[nbId])
                    continue;

                _g[nbId] = tentative;
                _seenGen[nbId] = _gen;
                _parent[nbId] = current;

                if (collectDebug && firstSeen)
                    _generatedIds.Add(nbId);

                _open.Enqueue(nbId, tentative + JpsDirections.OctileHeuristic(nx, ny, gx, gy));
            }
        }

        return Failure(expandedCount, collectDebug);
    }

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

    private List<(int X, int Y)> FrontierPoints()
    {
        var frontier = new List<(int X, int Y)>();
        foreach (int id in _generatedIds)
            if (_closedGen[id] != _gen)
                frontier.Add((id % _w, id / _w));
        return frontier;
    }

    private List<(int X, int Y)> ToPoints(List<int> ids)
    {
        var pts = new List<(int X, int Y)>(ids.Count);
        foreach (int id in ids)
            pts.Add((id % _w, id / _w));
        return pts;
    }

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

    private void NextGeneration()
    {
        _gen++;
        if (_gen != int.MaxValue)
            return;

        Array.Clear(_seenGen);
        Array.Clear(_closedGen);
        _gen = 1;
    }

    private int Id(int x, int y) => y * _w + x;

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
