namespace JPS.Pathfinding
{
    /// <summary>
    /// JPS 跳点 / 强迫邻居判定规则（角点可斜穿模型）。
    /// 用一个 walkable 委托抽象“可走”判定，规则在预计算/运行时保持一致。
    /// </summary>
    internal static class JpsRules
    {
        public static bool IsJumpPoint(Func<int, int, bool> walkable, int x, int y, int dx, int dy)
        {
            if (JpsDirections.IsDiagonal(dx, dy))
                return HasDiagonalForcedNeighbor(walkable, x, y, dx, dy);

            return HasCardinalForcedNeighbor(walkable, x, y, dx, dy);
        }

        // 对角移动 (dx,dy) 到达 (x,y) 时的强迫邻居：
        //   (x-dx,y) 被挡 且 (x-dx,y+dy) 可走  → 强迫邻居 (x-dx,y+dy)
        //   (x,y-dy) 被挡 且 (x+dx,y-dy) 可走  → 强迫邻居 (x+dx,y-dy)
        public static bool HasDiagonalForcedNeighbor(Func<int, int, bool> w, int x, int y, int dx, int dy)
        {
            return (!w(x - dx, y) && w(x - dx, y + dy)) ||
                   (!w(x, y - dy) && w(x + dx, y - dy));
        }

        // 直线移动时的强迫邻居：检查“前进方向上(x+dx / y+dy)”的斜前方格子，
        // 必须与 JpsPathfinder.FillDirections 中探索的方向保持一致，否则会漏掉真正的跳点。
        public static bool HasCardinalForcedNeighbor(Func<int, int, bool> w, int x, int y, int dx, int dy)
        {
            if (dy == 0)
            {
                return (!w(x, y + 1) && w(x + dx, y + 1)) ||
                       (!w(x, y - 1) && w(x + dx, y - 1));
            }

            return (!w(x + 1, y) && w(x + 1, y + dy)) ||
                   (!w(x - 1, y) && w(x - 1, y + dy));
        }
    }
}
