using JPS.Models;

namespace JPS.Pathfinding;

/// <summary>
/// 标准 8 邻接 A* 寻路，作为 JPS 的对照组。
/// 移动规则与本项目的 JPS 保持一致（对角只要目标格可走即可），以便公平比较。
/// </summary>
public sealed class AStarPathfinder
{
    public PathResult FindPath(GridMap map)
    {
        if (!map.HasStart || !map.HasEnd)
            return new PathResult { Message = "请先设置起点和终点。" };

        if (!map.IsWalkable(map.StartX, map.StartY) || !map.IsWalkable(map.EndX, map.EndY))
            return new PathResult { Message = "起点或终点位于阻挡上。" };

        var start = (X: map.StartX, Y: map.StartY);
        var goal = (X: map.EndX, Y: map.EndY);

        var open = new PriorityQueue<(int X, int Y), long>();
        var gScore = new Dictionary<(int X, int Y), long>();
        var parent = new Dictionary<(int X, int Y), (int X, int Y)>();
        var closed = new HashSet<(int X, int Y)>();

        gScore[start] = 0;
        open.Enqueue(start, JpsDirections.OctileHeuristic(start.X, start.Y, goal.X, goal.Y));

        while (open.Count > 0)
        {
            var current = open.Dequeue();
            if (!closed.Add(current))
                continue;

            if (current == goal)
            {
                var path = Reconstruct(parent, start, goal);
                var frontier = Frontier(gScore, closed);
                return new PathResult
                {
                    Success = true,
                    Path = path,
                    Expanded = [.. closed],
                    Frontier = frontier,
                    ExpandedNodes = closed.Count,
                    Message = $"A*：扩展 {closed.Count}，入队未扩展 {frontier.Count}，搜索合计 {closed.Count + frontier.Count} 格，路径 {path.Count} 格。"
                };
            }

            foreach (var (dx, dy) in JpsDirections.All)
            {
                int nx = current.X + dx;
                int ny = current.Y + dy;

                if (!map.IsWalkable(nx, ny))
                    continue;

                var neighbor = (X: nx, Y: ny);
                if (closed.Contains(neighbor))
                    continue;

                long tentative = gScore[current] + JpsDirections.MoveCost(dx, dy);
                if (gScore.TryGetValue(neighbor, out var known) && tentative >= known)
                    continue;

                gScore[neighbor] = tentative;
                parent[neighbor] = current;
                open.Enqueue(neighbor, tentative + JpsDirections.OctileHeuristic(nx, ny, goal.X, goal.Y));
            }
        }

        return new PathResult
        {
            Expanded = [.. closed],
            Frontier = Frontier(gScore, closed),
            ExpandedNodes = closed.Count,
            Message = $"A*：未找到路径（扩展 {closed.Count} 格）。"
        };
    }

    private static List<(int X, int Y)> Frontier(
        Dictionary<(int X, int Y), long> gScore,
        HashSet<(int X, int Y)> closed)
    {
        var frontier = new List<(int X, int Y)>();
        foreach (var node in gScore.Keys)
            if (!closed.Contains(node))
                frontier.Add(node);
        return frontier;
    }

    private static List<(int X, int Y)> Reconstruct(
        Dictionary<(int X, int Y), (int X, int Y)> parent,
        (int X, int Y) start,
        (int X, int Y) goal)
    {
        var path = new List<(int X, int Y)> { goal };
        var current = goal;

        while (current != start)
        {
            current = parent[current];
            path.Add(current);
        }

        path.Reverse();
        return path;
    }
}
