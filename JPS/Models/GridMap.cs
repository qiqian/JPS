using System.Numerics;

namespace JPS.Models;

public enum EditMode
{
    BrushObstacle,
    SetStart,
    SetEnd,
    DynamicObstacle
}

public sealed class GridMap
{
    public int Width { get; }
    public int Height { get; }
    public int CellSize { get; }

    private readonly bool[] _blocked;   // 静态阻挡：参与 JPS 预计算
    private readonly bool[] _dynamic;   // 动态阻挡：阻挡寻路，但不参与 JPS 预计算
    private int _dynamicCount;
    private readonly HashSet<int> _expanded = [];
    private readonly HashSet<int> _frontier = [];
    private readonly HashSet<int> _scanned = [];
    private readonly List<(int X, int Y)> _path = [];
    private readonly List<Vector2> _smoothPath = [];

    public int StartX { get; private set; } = -1;
    public int StartY { get; private set; } = -1;
    public int EndX { get; private set; } = -1;
    public int EndY { get; private set; } = -1;

    public bool IsPrecomputeValid { get; private set; }

    public GridMap(int width, int height, int cellSize)
    {
        Width = width;
        Height = height;
        CellSize = cellSize;
        _blocked = new bool[width * height];
        _dynamic = new bool[width * height];
    }

    public bool IsBlocked(int x, int y) => InBounds(x, y) && _blocked[Index(x, y)];

    public bool IsDynamic(int x, int y) => InBounds(x, y) && _dynamic[Index(x, y)];

    /// <summary>寻路用的可走判定：静态、动态阻挡都算不可走。</summary>
    public bool IsWalkable(int x, int y)
    {
        if (!InBounds(x, y))
            return false;
        int idx = Index(x, y);
        return !_blocked[idx] && !_dynamic[idx];
    }

    /// <summary>预计算用的可走判定：只看静态阻挡，忽略动态阻挡。</summary>
    public bool IsStaticWalkable(int x, int y) => InBounds(x, y) && !_blocked[Index(x, y)];

    /// <summary>是否存在动态阻挡（决定寻路是否走经典逐格跳跃的回退路径）。</summary>
    public bool HasDynamicObstacles => _dynamicCount > 0;

    public bool InBounds(int x, int y) => x >= 0 && x < Width && y >= 0 && y < Height;

    public void SetBlocked(int x, int y, bool blocked)
    {
        if (!InBounds(x, y))
            return;

        int idx = Index(x, y);
        _blocked[idx] = blocked;
        if (blocked)
        {
            // 同一格不能既是静态又是动态阻挡
            if (_dynamic[idx])
            {
                _dynamic[idx] = false;
                _dynamicCount--;
            }

            if (x == StartX && y == StartY)
                StartX = StartY = -1;

            if (x == EndX && y == EndY)
                EndX = EndY = -1;
        }

        IsPrecomputeValid = false;
        ClearSearchOverlay();
    }

    /// <summary>
    /// 动态阻挡画刷的点击切换：
    ///  - 已是动态阻挡 → 销毁；
    ///  - 是静态阻挡 → 转为动态阻挡（静态布局改变，预计算失效）；
    ///  - 是空地 → 新增动态阻挡（不影响预计算）。
    /// 起点/终点格不可放置。
    /// </summary>
    public void ToggleDynamic(int x, int y)
    {
        if (!InBounds(x, y))
            return;

        int idx = Index(x, y);

        if (_dynamic[idx])
        {
            // 销毁动态阻挡（动态不参与预计算，无需失效）
            _dynamic[idx] = false;
            _dynamicCount--;
            ClearSearchOverlay();
            return;
        }

        if (_blocked[idx])
        {
            // 静态阻挡 → 动态阻挡：静态布局变了，必须清空预计算
            _blocked[idx] = false;
            _dynamic[idx] = true;
            _dynamicCount++;
            IsPrecomputeValid = false;
            ClearSearchOverlay();
            return;
        }

        // 空地：起点/终点上不能放
        if ((x == StartX && y == StartY) || (x == EndX && y == EndY))
            return;

        _dynamic[idx] = true;
        _dynamicCount++;
        ClearSearchOverlay();
    }

    public void SetStart(int x, int y)
    {
        if (!IsWalkable(x, y))
            return;

        StartX = x;
        StartY = y;
        ClearSearchOverlay();
    }

    public void SetEnd(int x, int y)
    {
        if (!IsWalkable(x, y))
            return;

        EndX = x;
        EndY = y;
        ClearSearchOverlay();
    }

    public void ClearAll()
    {
        Array.Fill(_blocked, false);
        Array.Fill(_dynamic, false);
        _dynamicCount = 0;
        StartX = StartY = EndX = EndY = -1;
        IsPrecomputeValid = false;
        ClearSearchOverlay();
    }

    public void MarkPrecomputeValid() => IsPrecomputeValid = true;

    public void SetSearchCells(
        IEnumerable<(int X, int Y)> expanded,
        IEnumerable<(int X, int Y)> frontier,
        IEnumerable<(int X, int Y)> scanned)
    {
        _expanded.Clear();
        _frontier.Clear();
        _scanned.Clear();

        foreach (var (x, y) in scanned)
            _scanned.Add(Index(x, y));

        foreach (var (x, y) in frontier)
            _frontier.Add(Index(x, y));

        foreach (var (x, y) in expanded)
            _expanded.Add(Index(x, y));
    }

    public void SetPath(IEnumerable<(int X, int Y)> cells)
    {
        _path.Clear();
        _path.AddRange(cells);
    }

    public void SetSmoothPath(IEnumerable<Vector2> waypoints)
    {
        _smoothPath.Clear();
        _smoothPath.AddRange(waypoints);
    }

    public IReadOnlyList<Vector2> SmoothPath => _smoothPath;

    public void ClearSearchOverlay()
    {
        _expanded.Clear();
        _frontier.Clear();
        _scanned.Clear();
        _path.Clear();
        _smoothPath.Clear();
    }

    public bool IsExpanded(int x, int y) => _expanded.Contains(Index(x, y));

    public bool IsFrontier(int x, int y) => _frontier.Contains(Index(x, y));

    public bool IsScanned(int x, int y) => _scanned.Contains(Index(x, y));

    public bool IsOnPath(int x, int y) => _path.Contains((x, y));

    public IReadOnlyList<(int X, int Y)> Path => _path;

    public bool HasStart => StartX >= 0 && StartY >= 0;

    public bool HasEnd => EndX >= 0 && EndY >= 0;

    private int Index(int x, int y) => y * Width + x;
}
