using System.Diagnostics;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS.Controls;

public sealed class GridControl : Control
{
    private const int BrushSize = 2;

    private readonly int _cellSize;
    private readonly JpsPathfinder _jps = new();
    private readonly AStarPathfinder _astar = new();

    private GridMap _map;
    private EditMode _mode = EditMode.BrushObstacle;
    private bool _isPainting;
    private bool _eraseObstacle;

    private static readonly Color GridLineColor = Color.FromArgb(78, 78, 84);

    // 公开的配色，供图例复用，保证图例与网格颜色一致
    public static readonly Color WalkableColor = Color.FromArgb(112, 112, 118);
    public static readonly Color ObstacleColor = Color.FromArgb(32, 32, 36);
    public static readonly Color DynamicObstacleColor = Color.FromArgb(150, 85, 35);
    public static readonly Color ExpandedColor = Color.FromArgb(30, 158, 70);
    public static readonly Color FrontierColor = Color.FromArgb(168, 84, 224);
    public static readonly Color ScannedColor = Color.FromArgb(70, 104, 168);
    public static readonly Color PathColor = Color.FromArgb(255, 196, 0);
    public static readonly Color SmoothPathColor = Color.FromArgb(255, 60, 60);
    public static readonly Color StartColor = Color.FromArgb(0, 224, 224);
    public static readonly Color EndColor = Color.FromArgb(255, 0, 170);

    public event EventHandler<string>? StatusChanged;

    public GridControl(int cellSize)
    {
        _cellSize = cellSize;

        SetStyle(ControlStyles.AllPaintingInWmPaint |
                 ControlStyles.UserPaint |
                 ControlStyles.ResizeRedraw |
                 ControlStyles.OptimizedDoubleBuffer, true);

        BackColor = Color.FromArgb(60, 60, 64);
        _map = new GridMap(80, 50, _cellSize);
    }

    public GridMap Map
    {
        get
        {
            EnsureGrid();
            return _map;
        }
    }

    public void SetMode(EditMode mode) => _mode = mode;

    public void ClearMap()
    {
        EnsureGrid();
        _map.ClearAll();
        Invalidate();
        NotifyStatus("地图已清除。");
    }

    public MapData Export()
    {
        EnsureGrid();
        var data = new MapData { Width = _map.Width, Height = _map.Height };

        if (_map.HasStart)
            data.Start = new PointData { X = _map.StartX, Y = _map.StartY };
        if (_map.HasEnd)
            data.End = new PointData { X = _map.EndX, Y = _map.EndY };

        for (int y = 0; y < _map.Height; y++)
            for (int x = 0; x < _map.Width; x++)
            {
                if (_map.IsBlocked(x, y))
                    data.Obstacles.Add(new PointData { X = x, Y = y });
                else if (_map.IsDynamic(x, y))
                    data.DynamicObstacles.Add(new PointData { X = x, Y = y });
            }

        return data;
    }

    public void Import(MapData data)
    {
        EnsureGrid();
        _map.ClearAll();

        foreach (var o in data.Obstacles)
            _map.SetBlocked(o.X, o.Y, true);   // 越界坐标会被 SetBlocked 自动忽略

        if (data.Start != null)
            _map.SetStart(data.Start.X, data.Start.Y);
        if (data.End != null)
            _map.SetEnd(data.End.X, data.End.Y);

        // 动态阻挡在静态/起终点之后恢复：ToggleDynamic 会自动跳过非法格
        foreach (var d in data.DynamicObstacles)
            _map.ToggleDynamic(d.X, d.Y);

        Invalidate();
    }

    public PathResult RunJps()
    {
        EnsureGrid();
        // 有动态阻挡时走经典逐格跳跃，不需要预计算表；仅静态场景下预先建表（与寻路计时分开）
        if (!_map.HasDynamicObstacles && !_map.IsPrecomputeValid)
            _jps.RebuildCache(_map);

        var sw = Stopwatch.StartNew();
        var result = _jps.FindPath(_map);
        sw.Stop();

        _map.SetSearchCells(result.Expanded, result.Frontier, result.Scanned);
        _map.SetPath(result.Path);
        _map.SetSmoothPath(PathSmoother.Smooth(_map, result.Path));
        Invalidate();
        NotifyStatus($"{result.Message} 用时 {sw.Elapsed.TotalMilliseconds:F2} ms");
        return result;
    }

    public PathResult RunAStar()
    {
        EnsureGrid();
        var sw = Stopwatch.StartNew();
        var result = _astar.FindPath(_map);
        sw.Stop();

        _map.SetSearchCells(result.Expanded, result.Frontier, result.Scanned);
        _map.SetPath(result.Path);
        _map.SetSmoothPath(PathSmoother.Smooth(_map, result.Path));
        Invalidate();
        NotifyStatus($"{result.Message} 用时 {sw.Elapsed.TotalMilliseconds:F2} ms");
        return result;
    }

    protected override void OnResize(EventArgs e)
    {
        base.OnResize(e);
        EnsureGrid();
        Invalidate();
    }

    protected override void OnMouseDown(MouseEventArgs e)
    {
        base.OnMouseDown(e);
        Focus();

        if (!TryGetCell(e.Location, out int x, out int y))
            return;

        _isPainting = true;
        _eraseObstacle = e.Button == MouseButtons.Right;
        ApplyEdit(x, y);
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        base.OnMouseMove(e);

        if (!_isPainting || _mode != EditMode.BrushObstacle)
            return;

        if (!TryGetCell(e.Location, out int x, out int y))
            return;

        ApplyEdit(x, y);
    }

    protected override void OnMouseUp(MouseEventArgs e)
    {
        base.OnMouseUp(e);
        // 预计算已变为内部惰性行为：寻路时若无动态阻挡且表失效，会自动重建。
        _isPainting = false;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        EnsureGrid();

        var g = e.Graphics;
        g.Clear(BackColor);

        int cs = _cellSize;

        for (int y = 0; y < _map.Height; y++)
        {
            for (int x = 0; x < _map.Width; x++)
            {
                var rect = new Rectangle(x * cs, y * cs, cs, cs);
                Brush cellBrush = _map.IsBlocked(x, y) ? ObstacleBrush
                    : _map.IsDynamic(x, y) ? DynamicObstacleBrush
                    : WalkableBrush;
                g.FillRectangle(cellBrush, rect);
            }
        }

        DrawSearchOverlay(g, cs);
        DrawGridLines(g, cs);
        DrawMarkers(g, cs);
    }

    private static readonly SolidBrush WalkableBrush = new(WalkableColor);
    private static readonly SolidBrush ObstacleBrush = new(ObstacleColor);
    private static readonly SolidBrush DynamicObstacleBrush = new(DynamicObstacleColor);

    private void EnsureGrid()
    {
        int w = ClientSize.Width;
        int h = ClientSize.Height;

        int cols = w > 1 ? (w - 1) / _cellSize : _map.Width;
        int rows = h > 1 ? (h - 1) / _cellSize : _map.Height;
        cols = Math.Max(2, cols);
        rows = Math.Max(2, rows);

        if (_map.Width == cols && _map.Height == rows)
            return;

        var next = new GridMap(cols, rows, _cellSize);

        int copyW = Math.Min(cols, _map.Width);
        int copyH = Math.Min(rows, _map.Height);
        for (int y = 0; y < copyH; y++)
            for (int x = 0; x < copyW; x++)
                if (_map.IsBlocked(x, y))
                    next.SetBlocked(x, y, true);

        if (_map.HasStart && next.InBounds(_map.StartX, _map.StartY))
            next.SetStart(_map.StartX, _map.StartY);

        if (_map.HasEnd && next.InBounds(_map.EndX, _map.EndY))
            next.SetEnd(_map.EndX, _map.EndY);

        _map = next;
    }

    private void PaintObstacleBlock(int cx, int cy)
    {
        // 以点击格为中心，按 BrushSize×BrushSize 的块刷阻挡
        int half = BrushSize / 2;
        int x0 = cx - half;
        int y0 = cy - half;

        for (int y = y0; y < y0 + BrushSize; y++)
            for (int x = x0; x < x0 + BrushSize; x++)
                _map.SetBlocked(x, y, !_eraseObstacle);
    }

    private void ApplyEdit(int x, int y)
    {
        switch (_mode)
        {
            case EditMode.BrushObstacle:
                PaintObstacleBlock(x, y);
                Invalidate();
                break;

            case EditMode.SetStart:
                _map.SetStart(x, y);
                Invalidate();
                NotifyStatus($"起点：({x}, {y})");
                break;

            case EditMode.SetEnd:
                _map.SetEnd(x, y);
                Invalidate();
                NotifyStatus($"终点：({x}, {y})");
                break;

            case EditMode.DynamicObstacle:
                // 单格点击切换：有则销毁、无则新增。不重建跳点表（动态阻挡不参与预计算）。
                _map.ToggleDynamic(x, y);
                Invalidate();
                NotifyStatus(_map.IsDynamic(x, y)
                    ? $"动态阻挡 +({x}, {y})"
                    : $"动态阻挡 -({x}, {y})");
                break;
        }
    }

    private void DrawSearchOverlay(Graphics g, int cs)
    {
        // 绿=已扩展，紫=已入队未扩展（前沿），蓝灰=扫描跳过（未进 open）
        using var scannedBrush = new SolidBrush(ScannedColor);
        using var frontierBrush = new SolidBrush(FrontierColor);
        using var expandedBrush = new SolidBrush(ExpandedColor);
        using var pathBrush = new SolidBrush(PathColor);
        using var pathPen = new Pen(Color.FromArgb(255, 255, 140, 0), Math.Max(2f, cs / 3f));

        for (int y = 0; y < _map.Height; y++)
        {
            for (int x = 0; x < _map.Width; x++)
            {
                if (_map.IsOnPath(x, y))
                    continue;

                if (_map.IsExpanded(x, y))
                    g.FillRectangle(expandedBrush, new Rectangle(x * cs, y * cs, cs, cs));
                else if (_map.IsFrontier(x, y))
                    g.FillRectangle(frontierBrush, new Rectangle(x * cs, y * cs, cs, cs));
                else if (_map.IsScanned(x, y))
                    g.FillRectangle(scannedBrush, new Rectangle(x * cs, y * cs, cs, cs));
            }
        }

        foreach (var (x, y) in _map.Path)
            g.FillRectangle(pathBrush, new Rectangle(x * cs, y * cs, cs, cs));

        if (_map.Path.Count >= 2)
        {
            var points = _map.Path
                .Select(p => new Point(p.X * cs + cs / 2, p.Y * cs + cs / 2))
                .ToArray();
            g.DrawLines(pathPen, points);
        }

        // 平滑后的路径（视线拉直）用红色折线叠加显示
        if (_map.SmoothPath.Count >= 2)
        {
            using var smoothPen = new Pen(SmoothPathColor, Math.Max(2f, cs / 4f))
            {
                StartCap = System.Drawing.Drawing2D.LineCap.Round,
                EndCap = System.Drawing.Drawing2D.LineCap.Round,
                LineJoin = System.Drawing.Drawing2D.LineJoin.Round
            };
            using var nodeBrush = new SolidBrush(SmoothPathColor);

            var points = _map.SmoothPath
                .Select(p => new PointF(p.X * cs, p.Y * cs))
                .ToArray();

            var prevMode = g.SmoothingMode;
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            g.DrawLines(smoothPen, points);

            float r = Math.Max(2f, cs / 5f);
            foreach (var p in points)
                g.FillEllipse(nodeBrush, p.X - r, p.Y - r, r * 2, r * 2);

            g.SmoothingMode = prevMode;
        }
    }

    private void DrawGridLines(Graphics g, int cs)
    {
        using var pen = new Pen(GridLineColor);

        for (int x = 0; x <= _map.Width; x++)
            g.DrawLine(pen, x * cs, 0, x * cs, _map.Height * cs);

        for (int y = 0; y <= _map.Height; y++)
            g.DrawLine(pen, 0, y * cs, _map.Width * cs, y * cs);
    }

    private void DrawMarkers(Graphics g, int cs)
    {
        if (_map.HasStart)
            DrawMarker(g, _map.StartX, _map.StartY, cs, StartColor, "S");

        if (_map.HasEnd)
            DrawMarker(g, _map.EndX, _map.EndY, cs, EndColor, "G");
    }

    private static void DrawMarker(Graphics g, int x, int y, int cs, Color color, string label)
    {
        var rect = new Rectangle(x * cs + 1, y * cs + 1, cs - 2, cs - 2);
        using var brush = new SolidBrush(color);
        using var pen = new Pen(Color.White, Math.Max(1.5f, cs / 8f));

        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        g.FillEllipse(brush, rect);
        g.DrawEllipse(pen, rect);
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.Default;

        if (cs >= 12)
        {
            using var font = new Font("Segoe UI", Math.Max(6f, cs * 0.45f), FontStyle.Bold);
            using var textBrush = new SolidBrush(Color.Black);
            var size = g.MeasureString(label, font);
            g.DrawString(label, font, textBrush,
                x * cs + (cs - size.Width) / 2f,
                y * cs + (cs - size.Height) / 2f);
        }
    }

    private bool TryGetCell(Point location, out int x, out int y)
    {
        x = location.X / _cellSize;
        y = location.Y / _cellSize;
        return _map.InBounds(x, y);
    }

    private void NotifyStatus(string message) => StatusChanged?.Invoke(this, message);
}
