namespace JPS.Models;

public enum EditMode
{
    BrushObstacle,
    SetStart,
    SetEnd
}

public sealed class GridMap
{
    public int Width { get; }
    public int Height { get; }
    public int CellSize { get; }

    private readonly bool[] _blocked;
    private readonly HashSet<int> _expanded = [];
    private readonly HashSet<int> _frontier = [];
    private readonly HashSet<int> _scanned = [];
    private readonly List<(int X, int Y)> _path = [];
    private readonly List<(int X, int Y)> _smoothPath = [];

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
    }

    public bool IsBlocked(int x, int y) => InBounds(x, y) && _blocked[Index(x, y)];

    public bool IsWalkable(int x, int y) => InBounds(x, y) && !_blocked[Index(x, y)];

    public bool InBounds(int x, int y) => x >= 0 && x < Width && y >= 0 && y < Height;

    public void SetBlocked(int x, int y, bool blocked)
    {
        if (!InBounds(x, y))
            return;

        _blocked[Index(x, y)] = blocked;
        if (blocked)
        {
            if (x == StartX && y == StartY)
                StartX = StartY = -1;

            if (x == EndX && y == EndY)
                EndX = EndY = -1;
        }

        IsPrecomputeValid = false;
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

    public void SetSmoothPath(IEnumerable<(int X, int Y)> waypoints)
    {
        _smoothPath.Clear();
        _smoothPath.AddRange(waypoints);
    }

    public IReadOnlyList<(int X, int Y)> SmoothPath => _smoothPath;

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
