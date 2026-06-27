namespace JPS.Models;

public enum EditMode
{
    BrushObstacle,
    SetStart,
    SetEnd
}

/// <summary>
/// 纯网格模型：只承载“地图本身”（尺寸、阻挡、起终点、版本号）。
/// 不含任何可视化/搜索叠加状态——那部分由视图层的 SearchOverlay 持有，保持模型与视图分离。
/// </summary>
public sealed class GridMap
{
    public int Width { get; }
    public int Height { get; }
    public int CellSize { get; }

    private readonly bool[] _blocked;

    public int StartX { get; private set; } = -1;
    public int StartY { get; private set; } = -1;
    public int EndX { get; private set; } = -1;
    public int EndY { get; private set; } = -1;

    /// <summary>
    /// 阻挡布局的版本号：任何阻挡增删都自增。寻路器据此判断惰性跳点缓存是否失效。
    /// </summary>
    public int Version { get; private set; }

    public GridMap(int width, int height, int cellSize)
    {
        Width = width;
        Height = height;
        CellSize = cellSize;
        _blocked = new bool[width * height];
    }

    public bool IsBlocked(int x, int y) => InBounds(x, y) && _blocked[Index(x, y)];

    public bool IsWalkable(int x, int y) => InBounds(x, y) && !_blocked[Index(x, y)];

    public bool InBounds(int x, int y) => x >= 0 && x < Width && y >= 0 && y < Height;

    public void SetBlocked(int x, int y, bool blocked)
    {
        if (!InBounds(x, y))
            return;

        int idx = Index(x, y);
        if (_blocked[idx] == blocked)
            return;   // 无变化，不动版本号

        _blocked[idx] = blocked;
        if (blocked)
        {
            if (x == StartX && y == StartY)
                StartX = StartY = -1;
            if (x == EndX && y == EndY)
                EndX = EndY = -1;
        }

        Version++;   // 阻挡变化 → 惰性跳点缓存整体失效
    }

    public void SetStart(int x, int y)
    {
        if (!IsWalkable(x, y))
            return;

        StartX = x;
        StartY = y;
    }

    public void SetEnd(int x, int y)
    {
        if (!IsWalkable(x, y))
            return;

        EndX = x;
        EndY = y;
    }

    public void ClearAll()
    {
        Array.Fill(_blocked, false);
        StartX = StartY = EndX = EndY = -1;
        Version++;
    }

    public bool HasStart => StartX >= 0 && StartY >= 0;

    public bool HasEnd => EndX >= 0 && EndY >= 0;

    private int Index(int x, int y) => y * Width + x;
}
