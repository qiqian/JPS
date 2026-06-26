using JPS.Models;

namespace JPS.Pathfinding;

public sealed class JpsPrecomputer
{
    public JumpCache Build(GridMap map)
    {
        var cache = new JumpCache(map.Width, map.Height);

        // 先算 4 个正交方向（对角方向的预计算依赖正交结果）
        for (int y = 0; y < map.Height; y++)
        {
            for (int x = 0; x < map.Width; x++)
            {
                if (!map.IsWalkable(x, y))
                    continue;

                for (int d = 0; d < 4; d++)
                {
                    var (dx, dy) = JpsDirections.All[d];
                    cache.Set(x, y, d, ScanCardinal(map, x, y, dx, dy));
                }
            }
        }

        // 再算 4 个对角方向
        for (int y = 0; y < map.Height; y++)
        {
            for (int x = 0; x < map.Width; x++)
            {
                if (!map.IsWalkable(x, y))
                    continue;

                for (int d = 4; d < JpsDirections.Count; d++)
                {
                    var (dx, dy) = JpsDirections.All[d];
                    cache.Set(x, y, d, ScanDiagonal(map, cache, x, y, dx, dy));
                }
            }
        }

        map.MarkPrecomputeValid();
        return cache;
    }

    private static int ScanCardinal(GridMap map, int x, int y, int dx, int dy)
    {
        int cx = x;
        int cy = y;
        int steps = 0;

        while (true)
        {
            cx += dx;
            cy += dy;
            steps++;

            if (!map.IsWalkable(cx, cy))
                return -(steps - 1);

            if (IsJumpPoint(map, cx, cy, dx, dy))
                return steps;
        }
    }

    private static int ScanDiagonal(GridMap map, JumpCache cache, int x, int y, int dx, int dy)
    {
        int cx = x;
        int cy = y;
        int steps = 0;

        while (true)
        {
            cx += dx;
            cy += dy;
            steps++;

            if (!map.IsWalkable(cx, cy))
                return -(steps - 1);

            if (IsJumpPoint(map, cx, cy, dx, dy))
                return steps;

            // 对角途中，如果向其两个正交分量上存在跳点，则此格也算对角跳点
            int dirX = JpsDirections.IndexOf(dx, 0);
            int dirY = JpsDirections.IndexOf(0, dy);
            if (cache.Get(cx, cy, dirX) > 0 || cache.Get(cx, cy, dirY) > 0)
                return steps;
        }
    }

    private static bool IsJumpPoint(GridMap map, int x, int y, int dx, int dy)
    {
        if (JpsDirections.IsDiagonal(dx, dy))
            return HasDiagonalForcedNeighbor(map, x, y, dx, dy);

        return HasCardinalForcedNeighbor(map, x, y, dx, dy);
    }

    // 对角移动 (dx,dy) 到达 (x,y) 时的强迫邻居：
    //   (x-dx,y) 被挡 且 (x-dx,y+dy) 可走  → 强迫邻居 (x-dx,y+dy)
    //   (x,y-dy) 被挡 且 (x+dx,y-dy) 可走  → 强迫邻居 (x+dx,y-dy)
    private static bool HasDiagonalForcedNeighbor(GridMap map, int x, int y, int dx, int dy)
    {
        return (!map.IsWalkable(x - dx, y) && map.IsWalkable(x - dx, y + dy)) ||
               (!map.IsWalkable(x, y - dy) && map.IsWalkable(x + dx, y - dy));
    }

    // 直线移动时的强迫邻居：检查“前进方向上(x+dx / y+dy)”的斜前方格子，
    // 必须与 GetPrunedDirections 中探索的方向保持一致，否则会漏掉真正的跳点。
    private static bool HasCardinalForcedNeighbor(GridMap map, int x, int y, int dx, int dy)
    {
        if (dy == 0)
        {
            return (!map.IsWalkable(x, y + 1) && map.IsWalkable(x + dx, y + 1)) ||
                   (!map.IsWalkable(x, y - 1) && map.IsWalkable(x + dx, y - 1));
        }

        return (!map.IsWalkable(x + 1, y) && map.IsWalkable(x + 1, y + dy)) ||
               (!map.IsWalkable(x - 1, y) && map.IsWalkable(x - 1, y + dy));
    }
}
