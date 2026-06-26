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

    public void RebuildCache(GridMap map) => Cache = _precomputer.Build(map);

    public PathResult FindPath(GridMap map)
    {
        if (!map.HasStart || !map.HasEnd)
        {
            return new PathResult { Message = "请先设置起点和终点。" };
        }

        if (!map.IsWalkable(map.StartX, map.StartY) || !map.IsWalkable(map.EndX, map.EndY))
        {
            return new PathResult { Message = "起点或终点位于阻挡上。" };
        }

        if (!map.IsPrecomputeValid || Cache == null)
            RebuildCache(map);

        var cache = Cache!;
        var open = new PriorityQueue<(int X, int Y), long>();
        var gScore = new Dictionary<(int X, int Y), long>();
        var parent = new Dictionary<(int X, int Y), (int X, int Y)>();
        var parentDir = new Dictionary<(int X, int Y), (int Dx, int Dy)>();
        var expandedSet = new HashSet<(int X, int Y)>();
        var scanned = new HashSet<(int X, int Y)>();
        var closed = new HashSet<(int X, int Y)>();

        var start = (X: map.StartX, Y: map.StartY);
        var goal = (X: map.EndX, Y: map.EndY);

        gScore[start] = 0;
        open.Enqueue(start, JpsDirections.OctileHeuristic(start.X, start.Y, goal.X, goal.Y));

        int expanded = 0;

        while (open.Count > 0)
        {
            var current = open.Dequeue();
            if (closed.Contains(current))
                continue;

            closed.Add(current);
            expandedSet.Add(current);
            expanded++;

            if (current == goal)
            {
                var path = ReconstructPath(parent, start, goal);
                var frontier = Frontier(gScore, expandedSet);
                return BuildResult(true, path, expandedSet, frontier, scanned,
                    $"JPS：扩展 {expandedSet.Count}，入队未扩展 {frontier.Count}，扫描跳过 {scanned.Count} 格，路径 {path.Count} 格。");
            }

            IEnumerable<(int Dx, int Dy)> directions;
            if (parentDir.TryGetValue(current, out var pd))
                directions = GetPrunedDirections(map, current.X, current.Y, pd.Dx, pd.Dy);
            else
                directions = JpsDirections.All;

            foreach (var (dx, dy) in directions)
            {
                var jump = Jump(map, cache, current.X, current.Y, dx, dy, goal.X, goal.Y);

                if (!jump.HasJump)
                {
                    // 该方向没有跳点：射线一路扫到撞墙/越界，扫过的格子未进 open
                    CollectFailedRay(map, scanned, current.X, current.Y, dx, dy);
                    continue;
                }

                // 成功跳跃：起点与跳点之间被射线扫过、未进 open 的格子
                CollectSkippedRay(scanned, current.X, current.Y, jump.X, jump.Y);

                var neighbor = (jump.X, jump.Y);
                if (closed.Contains(neighbor))
                    continue;

                long moveCost = (long)jump.Steps *
                    (JpsDirections.IsDiagonal(dx, dy) ? JpsDirections.DiagonalCost : JpsDirections.CardinalCost);

                long tentative = gScore[current] + moveCost;
                if (gScore.TryGetValue(neighbor, out var known) && tentative >= known)
                    continue;

                gScore[neighbor] = tentative;
                parent[neighbor] = current;
                parentDir[neighbor] = (dx, dy);

                long f = tentative + JpsDirections.OctileHeuristic(neighbor.X, neighbor.Y, goal.X, goal.Y);
                open.Enqueue(neighbor, f);
            }
        }

        var frontierFail = Frontier(gScore, expandedSet);
        return BuildResult(false, [], expandedSet, frontierFail, scanned,
            $"JPS：未找到路径（扩展 {expandedSet.Count}，扫描跳过 {scanned.Count} 格）。");
    }

    // open 列表里"已入队但从未出队展开"的跳点：所有算过 g 的节点减去已扩展的
    private static List<(int X, int Y)> Frontier(
        Dictionary<(int X, int Y), long> gScore,
        HashSet<(int X, int Y)> expandedSet)
    {
        var frontier = new List<(int X, int Y)>();
        foreach (var node in gScore.Keys)
            if (!expandedSet.Contains(node))
                frontier.Add(node);
        return frontier;
    }

    private static PathResult BuildResult(
        bool success,
        List<(int X, int Y)> path,
        HashSet<(int X, int Y)> expandedSet,
        List<(int X, int Y)> frontier,
        HashSet<(int X, int Y)> scanned,
        string message)
    {
        return new PathResult
        {
            Success = success,
            Path = path,
            Expanded = [.. expandedSet],
            Frontier = frontier,
            Scanned = [.. scanned],
            ExpandedNodes = expandedSet.Count,
            Message = message
        };
    }

    private static void CollectFailedRay(GridMap map, HashSet<(int X, int Y)> failed, int x, int y, int dx, int dy)
    {
        int cx = x;
        int cy = y;

        while (true)
        {
            cx += dx;
            cy += dy;

            if (!map.IsWalkable(cx, cy))
                return;

            failed.Add((cx, cy));
        }
    }

    private static void CollectSkippedRay(HashSet<(int X, int Y)> skipped, int x1, int y1, int x2, int y2)
    {
        int dx = Math.Sign(x2 - x1);
        int dy = Math.Sign(y2 - y1);
        int x = x1;
        int y = y1;

        while (x != x2 || y != y2)
        {
            x += dx;
            y += dy;
            skipped.Add((x, y));
        }
    }

    // JPS+ 目标导向跳跃：在查预计算表的同时，把终点当作动态跳点处理，
    // 这样即使地图上没有任何静态跳点（例如完全空白地图）也能找到终点。
    private static JumpEntry Jump(GridMap map, JumpCache cache, int x, int y, int dx, int dy, int gx, int gy)
    {
        int dir = JpsDirections.IndexOf(dx, dy);
        int dist = cache.Get(x, y, dir);

        return JpsDirections.IsDiagonal(dx, dy)
            ? DiagonalJump(cache, x, y, dx, dy, gx, gy, dist)
            : CardinalJump(x, y, dx, dy, gx, gy, dist);
    }

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

        // 终点位于该对角方向所在的象限时，尝试拦截
        if (Math.Sign(gx - x) == dx && Math.Sign(gy - y) == dy)
        {
            int absDx = Math.Abs(gx - x);
            int absDy = Math.Abs(gy - y);
            int minDiff = Math.Min(absDx, absDy);

            // 若在对齐拐点之前已存在真正的对角跳点，则先返回它
            if (dist > 0 && dist < minDiff)
                return new JumpEntry(x + dx * dist, y + dy * dist, dist);

            if (minDiff <= maxDiag)
            {
                // 终点恰好落在纯对角线上
                if (absDx == absDy)
                    return new JumpEntry(gx, gy, minDiff);

                // 走 minDiff 步对角到达“拐点”，从这里终点变成纯正交方向
                int ax = x + dx * minDiff;
                int ay = y + dy * minDiff;
                int remaining = Math.Abs(absDx - absDy);
                int cardDir = absDx > absDy
                    ? JpsDirections.IndexOf(dx, 0)
                    : JpsDirections.IndexOf(0, dy);
                int cardDist = cache.Get(ax, ay, cardDir);
                int cardTravel = cardDist > 0 ? cardDist : -cardDist;

                // 从拐点到终点的正交路径畅通，则把拐点作为中间跳点入队
                if (remaining <= cardTravel)
                    return new JumpEntry(ax, ay, minDiff);
            }
        }

        return dist > 0
            ? new JumpEntry(x + dx * dist, y + dy * dist, dist)
            : JumpEntry.None;
    }

    private static IEnumerable<(int Dx, int Dy)> GetPrunedDirections(GridMap map, int x, int y, int pdx, int pdy)
    {
        if (JpsDirections.IsDiagonal(pdx, pdy))
        {
            yield return (pdx, pdy);
            yield return (pdx, 0);
            yield return (0, pdy);

            if (!map.IsWalkable(x - pdx, y))
                yield return (-pdx, pdy);

            if (!map.IsWalkable(x, y - pdy))
                yield return (pdx, -pdy);

            yield break;
        }

        yield return (pdx, pdy);

        if (pdx != 0)
        {
            if (!map.IsWalkable(x, y + 1))
                yield return (pdx, 1);

            if (!map.IsWalkable(x, y - 1))
                yield return (pdx, -1);
        }
        else
        {
            if (!map.IsWalkable(x + 1, y))
                yield return (1, pdy);

            if (!map.IsWalkable(x - 1, y))
                yield return (-1, pdy);
        }
    }

    private static List<(int X, int Y)> ReconstructPath(
        Dictionary<(int X, int Y), (int X, int Y)> parent,
        (int X, int Y) start,
        (int X, int Y) goal)
    {
        var nodes = new List<(int X, int Y)> { goal };
        var current = goal;

        while (current != start)
        {
            current = parent[current];
            nodes.Add(current);
        }

        nodes.Reverse();

        var path = new List<(int X, int Y)>();
        for (int i = 0; i < nodes.Count - 1; i++)
        {
            var from = nodes[i];
            var to = nodes[i + 1];
            AppendSegment(path, from, to);
        }

        return path;
    }

    private static void AppendSegment(List<(int X, int Y)> path, (int X, int Y) from, (int X, int Y) to)
    {
        int dx = Math.Sign(to.X - from.X);
        int dy = Math.Sign(to.Y - from.Y);
        int x = from.X;
        int y = from.Y;

        if (path.Count == 0 || path[^1] != from)
            path.Add(from);

        while (x != to.X || y != to.Y)
        {
            x += dx;
            y += dy;
            path.Add((x, y));
        }
    }
}
