using JPS.Models;

namespace JPS.Pathfinding;

public readonly struct JumpEntry
{
    public readonly bool HasJump;
    public readonly int X;
    public readonly int Y;
    public readonly int Steps;

    public static JumpEntry None => default;

    public JumpEntry(int x, int y, int steps)
    {
        HasJump = true;
        X = x;
        Y = y;
        Steps = steps;
    }
}

public sealed class PathResult
{
    public bool Success { get; init; }
    public List<(int X, int Y)> Path { get; init; } = [];
    public List<(int X, int Y)> Expanded { get; init; } = [];
    public List<(int X, int Y)> Frontier { get; init; } = [];
    public List<(int X, int Y)> Scanned { get; init; } = [];
    public int ExpandedNodes { get; init; }
    public string Message { get; init; } = string.Empty;
}

public sealed class JpsPathfinder
{
    private readonly JpsPrecomputer _precomputer = new();

    public JumpCache? Cache { get; private set; }

    // ---- 按地图尺寸一次性分配、跨多次查询复用的缓冲区 ----
    private int _w, _h, _size;
    private long[] _g = [];
    private int[] _parent = [];
    private sbyte[] _parentDir = [];
    private int[] _seenGen = [];     // 节点是否已被生成（算过 g）
    private int[] _closedGen = [];   // 节点是否已出队展开
    private int[] _scanGen = [];     // 可视化：是否已记录为“扫描跳过”
    private int _gen;
    private readonly PriorityQueue<int, long> _open = new();
    private readonly int[] _dirBuf = new int[JpsDirections.Count];

    // 可视化用的收集列表（仅在 collectDebug 时填充）
    private readonly List<int> _expandedIds = [];
    private readonly List<int> _generatedIds = [];
    private readonly List<int> _scannedIds = [];

    public void RebuildCache(GridMap map) => Cache = _precomputer.Build(map);

    public PathResult FindPath(GridMap map, bool collectDebug = true)
    {
        if (!map.HasStart || !map.HasEnd)
            return new PathResult { Message = "请先设置起点和终点。" };

        if (!map.IsWalkable(map.StartX, map.StartY) || !map.IsWalkable(map.EndX, map.EndY))
            return new PathResult { Message = "起点或终点位于阻挡上。" };

        if (!map.IsPrecomputeValid || Cache == null)
            RebuildCache(map);

        var cache = Cache!;
        EnsureBuffers(map);
        NextGeneration();

        if (collectDebug)
        {
            _expandedIds.Clear();
            _generatedIds.Clear();
            _scannedIds.Clear();
        }

        int gx = map.EndX, gy = map.EndY;
        int startId = Id(map.StartX, map.StartY);
        int goalId = Id(gx, gy);

        _open.Clear();
        _g[startId] = 0;
        _seenGen[startId] = _gen;
        _parent[startId] = -1;
        _parentDir[startId] = -1;
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
                return Success(map, startId, goalId, expandedCount, collectDebug);

            int cx = current % _w;
            int cy = current / _w;
            int baseIdx = cache.Base(cx, cy);

            int dirCount = FillDirections(map, cx, cy, _parentDir[current]);

            for (int i = 0; i < dirCount; i++)
            {
                int idx = _dirBuf[i];
                var (dx, dy) = JpsDirections.All[idx];
                int dist = cache.GetAt(baseIdx, idx);

                JumpEntry jump = JpsDirections.IsDiagonal(dx, dy)
                    ? DiagonalJump(cache, cx, cy, dx, dy, gx, gy, dist)
                    : CardinalJump(cx, cy, dx, dy, gx, gy, dist);

                if (!jump.HasJump)
                {
                    if (collectDebug)
                        CollectFailedRay(map, cx, cy, dx, dy);
                    continue;
                }

                if (collectDebug)
                    CollectSkippedRay(cx, cy, jump.X, jump.Y);

                int nbId = Id(jump.X, jump.Y);
                if (_closedGen[nbId] == _gen)
                    continue;

                long moveCost = (long)jump.Steps *
                    (JpsDirections.IsDiagonal(dx, dy) ? JpsDirections.DiagonalCost : JpsDirections.CardinalCost);
                long tentative = _g[current] + moveCost;

                bool firstSeen = _seenGen[nbId] != _gen;
                if (!firstSeen && tentative >= _g[nbId])
                    continue;

                _g[nbId] = tentative;
                _seenGen[nbId] = _gen;
                _parent[nbId] = current;
                _parentDir[nbId] = (sbyte)idx;

                if (collectDebug && firstSeen)
                    _generatedIds.Add(nbId);

                long f = tentative + JpsDirections.OctileHeuristic(jump.X, jump.Y, gx, gy);
                _open.Enqueue(nbId, f);
            }
        }

        return Failure(expandedCount, collectDebug);
    }

    // ---------------- 结果构造 ----------------

    private PathResult Success(GridMap map, int startId, int goalId, int expandedCount, bool collectDebug)
    {
        var path = ReconstructPath(startId, goalId);

        if (!collectDebug)
        {
            return new PathResult
            {
                Success = true,
                Path = path,
                ExpandedNodes = expandedCount,
                Message = $"JPS：扩展 {expandedCount}，路径 {path.Count} 格。"
            };
        }

        var expanded = ToPoints(_expandedIds);
        var frontier = FrontierPoints();
        var scanned = ToPoints(_scannedIds);
        return new PathResult
        {
            Success = true,
            Path = path,
            Expanded = expanded,
            Frontier = frontier,
            Scanned = scanned,
            ExpandedNodes = expandedCount,
            Message = $"JPS：扩展 {expanded.Count}，入队未扩展 {frontier.Count}，扫描跳过 {scanned.Count} 格，路径 {path.Count} 格。"
        };
    }

    private PathResult Failure(int expandedCount, bool collectDebug)
    {
        if (!collectDebug)
            return new PathResult { ExpandedNodes = expandedCount, Message = $"JPS：未找到路径（扩展 {expandedCount}）。" };

        var expanded = ToPoints(_expandedIds);
        var frontier = FrontierPoints();
        var scanned = ToPoints(_scannedIds);
        return new PathResult
        {
            Expanded = expanded,
            Frontier = frontier,
            Scanned = scanned,
            ExpandedNodes = expandedCount,
            Message = $"JPS：未找到路径（扩展 {expanded.Count}，扫描跳过 {scanned.Count} 格）。"
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

    // ---------------- 缓冲区与代次 ----------------

    private void EnsureBuffers(GridMap map)
    {
        if (_w == map.Width && _size == map.Width * map.Height)
            return;

        _w = map.Width;
        _h = map.Height;
        _size = _w * _h;
        _g = new long[_size];
        _parent = new int[_size];
        _parentDir = new sbyte[_size];
        _seenGen = new int[_size];
        _closedGen = new int[_size];
        _scanGen = new int[_size];
        _gen = 0;
    }

    private void NextGeneration()
    {
        _gen++;
        if (_gen != int.MaxValue)
            return;

        // 代次号溢出（极罕见）：清零重来
        Array.Clear(_seenGen);
        Array.Clear(_closedGen);
        Array.Clear(_scanGen);
        _gen = 1;
    }

    private int Id(int x, int y) => y * _w + x;

    // ---------------- 方向剪枝（零分配，写入 _dirBuf，返回数量）----------------

    private int FillDirections(GridMap map, int x, int y, sbyte parentDir)
    {
        if (parentDir < 0)
        {
            for (int i = 0; i < JpsDirections.Count; i++)
                _dirBuf[i] = i;
            return JpsDirections.Count;
        }

        var (pdx, pdy) = JpsDirections.All[parentDir];
        int n = 0;

        if (JpsDirections.IsDiagonal(pdx, pdy))
        {
            _dirBuf[n++] = parentDir;
            _dirBuf[n++] = JpsDirections.IndexOf(pdx, 0);
            _dirBuf[n++] = JpsDirections.IndexOf(0, pdy);

            if (!map.IsWalkable(x - pdx, y))
                _dirBuf[n++] = JpsDirections.IndexOf(-pdx, pdy);
            if (!map.IsWalkable(x, y - pdy))
                _dirBuf[n++] = JpsDirections.IndexOf(pdx, -pdy);

            return n;
        }

        _dirBuf[n++] = parentDir;

        if (pdx != 0)
        {
            if (!map.IsWalkable(x, y + 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, 1);
            if (!map.IsWalkable(x, y - 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, -1);
        }
        else
        {
            if (!map.IsWalkable(x + 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(1, pdy);
            if (!map.IsWalkable(x - 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(-1, pdy);
        }

        return n;
    }

    // ---------------- 目标导向跳跃 ----------------

    private static JumpEntry CardinalJump(int x, int y, int dx, int dy, int gx, int gy, int dist)
    {
        int maxTravel = dist > 0 ? dist : -dist;

        bool goalOnRay =
            (dy == 0 && gy == y && Math.Sign(gx - x) == dx) ||
            (dx == 0 && gx == x && Math.Sign(gy - y) == dy);

        if (goalOnRay)
        {
            int distToGoal = dx != 0 ? Math.Abs(gx - x) : Math.Abs(gy - y);
            if (distToGoal <= maxTravel)
                return new JumpEntry(gx, gy, distToGoal);
        }

        return dist > 0
            ? new JumpEntry(x + dx * dist, y + dy * dist, dist)
            : JumpEntry.None;
    }

    private static JumpEntry DiagonalJump(JumpCache cache, int x, int y, int dx, int dy, int gx, int gy, int dist)
    {
        int maxDiag = dist > 0 ? dist : -dist;

        if (Math.Sign(gx - x) == dx && Math.Sign(gy - y) == dy)
        {
            int absDx = Math.Abs(gx - x);
            int absDy = Math.Abs(gy - y);
            int minDiff = Math.Min(absDx, absDy);

            if (dist > 0 && dist < minDiff)
                return new JumpEntry(x + dx * dist, y + dy * dist, dist);

            if (minDiff <= maxDiag)
            {
                if (absDx == absDy)
                    return new JumpEntry(gx, gy, minDiff);

                int ax = x + dx * minDiff;
                int ay = y + dy * minDiff;
                int remaining = Math.Abs(absDx - absDy);
                int cardDir = absDx > absDy
                    ? JpsDirections.IndexOf(dx, 0)
                    : JpsDirections.IndexOf(0, dy);
                int cardDist = cache.Get(ax, ay, cardDir);
                int cardTravel = cardDist > 0 ? cardDist : -cardDist;

                if (remaining <= cardTravel)
                    return new JumpEntry(ax, ay, minDiff);
            }
        }

        return dist > 0
            ? new JumpEntry(x + dx * dist, y + dy * dist, dist)
            : JumpEntry.None;
    }

    // ---------------- 可视化采集 ----------------

    private void CollectFailedRay(GridMap map, int x, int y, int dx, int dy)
    {
        int cx = x, cy = y;
        while (true)
        {
            cx += dx;
            cy += dy;
            if (!map.IsWalkable(cx, cy))
                return;
            AddScanned(cx, cy);
        }
    }

    private void CollectSkippedRay(int x1, int y1, int x2, int y2)
    {
        int dx = Math.Sign(x2 - x1);
        int dy = Math.Sign(y2 - y1);
        int x = x1, y = y1;
        while (x != x2 || y != y2)
        {
            x += dx;
            y += dy;
            AddScanned(x, y);
        }
    }

    private void AddScanned(int x, int y)
    {
        int id = Id(x, y);
        if (_scanGen[id] == _gen)
            return;
        _scanGen[id] = _gen;
        _scannedIds.Add(id);
    }

    // ---------------- 路径重建 ----------------

    private List<(int X, int Y)> ReconstructPath(int startId, int goalId)
    {
        var nodes = new List<int> { goalId };
        int current = goalId;
        while (current != startId)
        {
            current = _parent[current];
            nodes.Add(current);
        }
        nodes.Reverse();

        var path = new List<(int X, int Y)>();
        for (int i = 0; i < nodes.Count - 1; i++)
        {
            int fromId = nodes[i];
            int toId = nodes[i + 1];
            AppendSegment(path, fromId % _w, fromId / _w, toId % _w, toId / _w);
        }
        return path;
    }

    private static void AppendSegment(List<(int X, int Y)> path, int fx, int fy, int tx, int ty)
    {
        int dx = Math.Sign(tx - fx);
        int dy = Math.Sign(ty - fy);
        int x = fx, y = fy;

        if (path.Count == 0 || path[^1] != (fx, fy))
            path.Add((fx, fy));

        while (x != tx || y != ty)
        {
            x += dx;
            y += dy;
            path.Add((x, y));
        }
    }
}
