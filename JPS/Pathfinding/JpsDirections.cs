namespace JPS.Pathfinding;

internal static class JpsDirections
{
    public const int Count = 8;

    public static readonly (int Dx, int Dy)[] All =
    [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (-1, 1),
        (1, -1),
        (-1, -1)
    ];

    public static int IndexOf(int dx, int dy)
    {
        for (int i = 0; i < Count; i++)
        {
            if (All[i].Dx == dx && All[i].Dy == dy)
                return i;
        }

        return -1;
    }

    // 整数距离：横向 1000，斜向 1414
    public const int CardinalCost = 1000;
    public const int DiagonalCost = 1414;

    public static bool IsDiagonal(int dx, int dy) => dx != 0 && dy != 0;

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
