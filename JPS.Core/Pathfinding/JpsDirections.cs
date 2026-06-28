/*
 * JpsDirections.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;
using JPS.Models;

namespace JPS.Pathfinding
{
    internal static class JpsDirections
    {
        public const int Count = 8;

        public static readonly (int Dx, int Dy)[] All = new (int, int)[]
        {
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (-1, 1),
            (1, -1),
            (-1, -1),
        };

        public static int IndexOf(int dx, int dy) => (dx, dy) switch
        {
            (1, 0) => 0,
            (-1, 0) => 1,
            (0, 1) => 2,
            (0, -1) => 3,
            (1, 1) => 4,
            (-1, 1) => 5,
            (1, -1) => 6,
            (-1, -1) => 7,
            _ => -1,
        };

        // 整数距离：横向 1000，斜向 1414
        public const int CardinalCost = 1000;
        public const int DiagonalCost = 1414;

        public static bool IsDiagonal(int dx, int dy) => dx != 0 && dy != 0;

        public static bool IsDiagonalIndex(int index) => index >= 4;

        // 从 (x,y) 沿对角 (dx,dy) 走一步是否合法。
        // 默认（未定义 JPS_ALLOW_CORNER_CUTTING）：禁止斜穿角——目标格 + 两侧正交格都必须可走。
        // 定义 JPS_ALLOW_CORNER_CUTTING：恢复旧的“允许斜穿拐角”——只要目标格可走。
        public static bool DiagonalAllowed(GridMap map, int x, int y, int dx, int dy) =>
#if JPS_ALLOW_CORNER_CUTTING
            map.IsWalkable(x + dx, y + dy);
#else
            map.IsWalkable(x + dx, y + dy) && map.IsWalkable(x + dx, y) && map.IsWalkable(x, y + dy);
#endif

        public static long OctileHeuristic(int x1, int y1, int x2, int y2)
        {
            int dx = Math.Abs(x1 - x2);
            int dy = Math.Abs(y1 - y2);
            int min = Math.Min(dx, dy);
            int max = Math.Max(dx, dy);
            return (long)(max - min) * CardinalCost + (long)min * DiagonalCost;
        }

        public static int MoveCost(int dx, int dy) =>
            IsDiagonal(dx, dy) ? DiagonalCost : CardinalCost;
    }
}
