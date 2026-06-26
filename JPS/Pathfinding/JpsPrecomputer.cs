using JPS.Models;

namespace JPS.Pathfinding;

/// <summary>
/// JPS+ 跳点距离表预计算。每个方向用一次性扫描 DP，复杂度 O(N)（N=格子数），
/// 而非从每个格子各自重新走到墙的 O(N·L)。
/// 方向索引约定（与 JpsDirections.All 一致）：
///   0:E(1,0) 1:W(-1,0) 2:S(0,1) 3:N(0,-1) 4:SE(1,1) 5:NE(-1,1) 6:SW(1,-1) 7:NW(-1,-1)
/// </summary>
public sealed class JpsPrecomputer
{
    private const int E = 0, W = 1, S = 2, N = 3, SE = 4, NE = 5, SWd = 6, NWd = 7;

    public JumpCache Build(GridMap map)
    {
        var cache = new JumpCache(map.Width, map.Height);

        // 正交四方向（对角 DP 依赖它们的结果）
        SweepEast(map, cache);
        SweepWest(map, cache);
        SweepSouth(map, cache);
        SweepNorth(map, cache);

        // 对角四方向
        SweepDiagonal(map, cache, SE, 1, 1, E, S);
        SweepDiagonal(map, cache, NE, -1, 1, W, S);
        SweepDiagonal(map, cache, SWd, 1, -1, E, N);
        SweepDiagonal(map, cache, NWd, -1, -1, W, N);

        map.MarkPrecomputeValid();
        return cache;
    }

    private static void SweepEast(GridMap map, JumpCache cache)
    {
        int wdt = map.Width;
        for (int y = 0; y < map.Height; y++)
        {
            int lastJump = -1;
            int wall = wdt;        // 右边界外视为墙
            for (int x = wdt - 1; x >= 0; x--)
            {
                if (!map.IsWalkable(x, y)) { cache.Set(x, y, E, 0); lastJump = -1; wall = x; continue; }
                int dist = lastJump != -1 ? lastJump - x : -((wall - x) - 1);
                cache.Set(x, y, E, dist);
                if (IsJumpPoint(map, x, y, 1, 0)) lastJump = x;
            }
        }
    }

    private static void SweepWest(GridMap map, JumpCache cache)
    {
        int wdt = map.Width;
        for (int y = 0; y < map.Height; y++)
        {
            int lastJump = -1;
            int wall = -1;
            for (int x = 0; x < wdt; x++)
            {
                if (!map.IsWalkable(x, y)) { cache.Set(x, y, W, 0); lastJump = -1; wall = x; continue; }
                int dist = lastJump != -1 ? x - lastJump : -((x - wall) - 1);
                cache.Set(x, y, W, dist);
                if (IsJumpPoint(map, x, y, -1, 0)) lastJump = x;
            }
        }
    }

    private static void SweepSouth(GridMap map, JumpCache cache)
    {
        int hgt = map.Height;
        for (int x = 0; x < map.Width; x++)
        {
            int lastJump = -1;
            int wall = hgt;
            for (int y = hgt - 1; y >= 0; y--)
            {
                if (!map.IsWalkable(x, y)) { cache.Set(x, y, S, 0); lastJump = -1; wall = y; continue; }
                int dist = lastJump != -1 ? lastJump - y : -((wall - y) - 1);
                cache.Set(x, y, S, dist);
                if (IsJumpPoint(map, x, y, 0, 1)) lastJump = y;
            }
        }
    }

    private static void SweepNorth(GridMap map, JumpCache cache)
    {
        int hgt = map.Height;
        for (int x = 0; x < map.Width; x++)
        {
            int lastJump = -1;
            int wall = -1;
            for (int y = 0; y < hgt; y++)
            {
                if (!map.IsWalkable(x, y)) { cache.Set(x, y, N, 0); lastJump = -1; wall = y; continue; }
                int dist = lastJump != -1 ? y - lastJump : -((y - wall) - 1);
                cache.Set(x, y, N, dist);
                if (IsJumpPoint(map, x, y, 0, -1)) lastJump = y;
            }
        }
    }

    // 对角 DP：按使邻居 (x+dx,y+dy) 先于 (x,y) 被处理的顺序扫描。
    private static void SweepDiagonal(GridMap map, JumpCache cache, int dir, int dx, int dy, int cardX, int cardY)
    {
        int wdt = map.Width;
        int hgt = map.Height;

        int xStart = dx > 0 ? wdt - 1 : 0;
        int xEnd = dx > 0 ? -1 : wdt;
        int xStep = dx > 0 ? -1 : 1;
        int yStart = dy > 0 ? hgt - 1 : 0;
        int yEnd = dy > 0 ? -1 : hgt;
        int yStep = dy > 0 ? -1 : 1;

        for (int y = yStart; y != yEnd; y += yStep)
        {
            for (int x = xStart; x != xEnd; x += xStep)
            {
                if (!map.IsWalkable(x, y)) { cache.Set(x, y, dir, 0); continue; }

                int nx = x + dx;
                int ny = y + dy;
                int dist;

                if (!map.IsWalkable(nx, ny))
                {
                    dist = 0;   // 连一步都迈不过去
                }
                else if (IsJumpPoint(map, nx, ny, dx, dy) ||
                         cache.Get(nx, ny, cardX) > 0 ||
                         cache.Get(nx, ny, cardY) > 0)
                {
                    dist = 1;   // 邻居就是对角跳点
                }
                else
                {
                    int d = cache.Get(nx, ny, dir);
                    dist = d > 0 ? d + 1 : d - 1;
                }

                cache.Set(x, y, dir, dist);
            }
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
    // 必须与 JpsPathfinder.FillDirections 中探索的方向保持一致，否则会漏掉真正的跳点。
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
