using System.Numerics;
using JPS.Pathfinding;

namespace JPS.Controls;

/// <summary>
/// 寻路过程的可视化叠加层（纯视图状态，由 <see cref="GridControl"/> 持有）。
///
/// 它实现算法核心定义的 <see cref="ISearchObserver"/>：寻路时作为观察者直接传给
/// FindPath，算法在展开 / 入队 / 扫描时回调，这里负责收集、去重与存储。
/// 算法核心（JpsPathfinder / AStarPathfinder）完全不持有也不读取任何可视化数据。
/// </summary>
public sealed class SearchOverlay : ISearchObserver
{
    private int _width = 1;
    private readonly HashSet<int> _expanded = [];
    private readonly HashSet<int> _generated = [];   // 首次入队的前沿候选；前沿 = 生成 − 已展开
    private readonly HashSet<int> _scanned = [];
    private readonly HashSet<int> _pathSet = [];
    private readonly List<(int X, int Y)> _path = [];
    private readonly List<List<(int X, int Y)>> _pathSegments = [];
    private readonly List<Vector2> _smoothPath = [];

    public void SetWidth(int width) => _width = Math.Max(1, width);

    // ---- ISearchObserver：搜索过程中由算法回调 ----
    public void OnExpand(int x, int y) => _expanded.Add(Index(x, y));
    public void OnFrontier(int x, int y) => _generated.Add(Index(x, y));
    public void OnScan(int x, int y) => _scanned.Add(Index(x, y));

    /// <summary>开始一次新搜索前清空采集状态（路径/平滑路径随后单独设置）。</summary>
    public void BeginCollect()
    {
        _expanded.Clear();
        _generated.Clear();
        _scanned.Clear();
        _pathSet.Clear();
        _path.Clear();
        _pathSegments.Clear();
        _smoothPath.Clear();
    }

    public void SetPath(IEnumerable<(int X, int Y)> cells)
    {
        _path.Clear();
        _pathSet.Clear();
        AddPath(cells);
    }

    public void AddPath(IEnumerable<(int X, int Y)> cells)
    {
        var segment = new List<(int X, int Y)>();
        foreach (var c in cells)
        {
            segment.Add(c);
            _path.Add(c);
            _pathSet.Add(Index(c.X, c.Y));
        }

        if (segment.Count > 0)
            _pathSegments.Add(segment);
    }

    public void SetSmoothPath(IEnumerable<Vector2> waypoints)
    {
        _smoothPath.Clear();
        _smoothPath.AddRange(waypoints);
    }

    public void AddFrom(SearchOverlay other)
    {
        foreach (int id in other._expanded)
            _expanded.Add(id);
        foreach (int id in other._generated)
            _generated.Add(id);
        foreach (int id in other._scanned)
            _scanned.Add(id);
        foreach (var segment in other._pathSegments)
            AddPath(segment);
    }

    public void Clear()
    {
        _expanded.Clear();
        _generated.Clear();
        _scanned.Clear();
        _pathSet.Clear();
        _path.Clear();
        _pathSegments.Clear();
        _smoothPath.Clear();
    }

    public bool IsExpanded(int x, int y) => _expanded.Contains(Index(x, y));
    public bool IsFrontier(int x, int y)
    {
        int id = Index(x, y);
        return _generated.Contains(id) && !_expanded.Contains(id);
    }
    public bool IsScanned(int x, int y) => _scanned.Contains(Index(x, y));
    public bool IsOnPath(int x, int y) => _pathSet.Contains(Index(x, y));

    public int ExpandedCount => _expanded.Count;
    public int FrontierCount
    {
        get
        {
            int n = 0;
            foreach (int id in _generated)
                if (!_expanded.Contains(id)) n++;
            return n;
        }
    }
    public int ScannedCount => _scanned.Count;

    public IReadOnlyList<(int X, int Y)> Path => _path;
    public IReadOnlyList<IReadOnlyList<(int X, int Y)>> PathSegments => _pathSegments;
    public IReadOnlyList<Vector2> SmoothPath => _smoothPath;

    private int Index(int x, int y) => y * _width + x;
}
