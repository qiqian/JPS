using System.Numerics;

namespace JPS.Controls;

/// <summary>
/// 寻路过程的可视化叠加层（纯视图状态，由 <see cref="GridControl"/> 持有）。
/// 与算法/模型(GridMap)解耦：寻路器只产出 PathResult，由控件填入这里供绘制；
/// 寻路算法完全不读取本对象。
/// </summary>
public sealed class SearchOverlay
{
    private int _width = 1;
    private readonly HashSet<int> _expanded = [];
    private readonly HashSet<int> _frontier = [];
    private readonly HashSet<int> _scanned = [];
    private readonly HashSet<int> _pathSet = [];
    private readonly List<(int X, int Y)> _path = [];
    private readonly List<Vector2> _smoothPath = [];

    public void SetWidth(int width) => _width = Math.Max(1, width);

    public void SetSearchCells(
        IEnumerable<(int X, int Y)> expanded,
        IEnumerable<(int X, int Y)> frontier,
        IEnumerable<(int X, int Y)> scanned)
    {
        _expanded.Clear();
        _frontier.Clear();
        _scanned.Clear();

        foreach (var (x, y) in scanned) _scanned.Add(Index(x, y));
        foreach (var (x, y) in frontier) _frontier.Add(Index(x, y));
        foreach (var (x, y) in expanded) _expanded.Add(Index(x, y));
    }

    public void SetPath(IEnumerable<(int X, int Y)> cells)
    {
        _path.Clear();
        _pathSet.Clear();
        foreach (var c in cells)
        {
            _path.Add(c);
            _pathSet.Add(Index(c.X, c.Y));
        }
    }

    public void SetSmoothPath(IEnumerable<Vector2> waypoints)
    {
        _smoothPath.Clear();
        _smoothPath.AddRange(waypoints);
    }

    public void Clear()
    {
        _expanded.Clear();
        _frontier.Clear();
        _scanned.Clear();
        _pathSet.Clear();
        _path.Clear();
        _smoothPath.Clear();
    }

    public bool IsExpanded(int x, int y) => _expanded.Contains(Index(x, y));
    public bool IsFrontier(int x, int y) => _frontier.Contains(Index(x, y));
    public bool IsScanned(int x, int y) => _scanned.Contains(Index(x, y));
    public bool IsOnPath(int x, int y) => _pathSet.Contains(Index(x, y));

    public IReadOnlyList<(int X, int Y)> Path => _path;
    public IReadOnlyList<Vector2> SmoothPath => _smoothPath;

    private int Index(int x, int y) => y * _width + x;
}
