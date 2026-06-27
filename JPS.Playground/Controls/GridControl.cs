using System.Diagnostics;
using JPS.Data;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS.Controls;

public sealed class GridControl : ScrollableControl
{
    private const int BrushSize = 2;

    private int _cellSize;               // 当前格像素尺寸
    private readonly int _baseCellSize;  // 沙盒模式的默认格尺寸
    private bool _fixedSize;             // true=载入了定尺地图（如 MovingAI），网格不再随窗口自适应

    private const int MinCell = 2;       // Ctrl+滚轮缩放下限
    private const int MaxCell = 64;      // Ctrl+滚轮缩放上限
    private readonly JpsPathfinder _jps = new();
    private readonly AStarPathfinder _astar = new();
    private readonly SearchOverlay _overlay = new();   // 视图层的可视化叠加，与模型分离

    private JpsSystem _system;   // 地图 + 共享跳点缓存
    private GridMap _map;        // = _system.Map（保留以简化既有引用）
    private EditMode _mode = EditMode.BrushObstacle;
    private bool _isPainting;
    private bool _eraseObstacle;

    // 当前选中的起点/终点（视图/编辑状态，作为寻路查询参数；不属于地图模型）
    private int _startX = -1, _startY = -1, _endX = -1, _endY = -1;
    private bool HasStart => _startX >= 0 && _startY >= 0;
    private bool HasEnd => _endX >= 0 && _endY >= 0;

    // 纯 UI 层快照：寻路前各方向 clean 状态，用于区分“本次寻路新更新”的跳点方向
    private bool[] _cleanBefore = [];
    private int _snapW, _snapH;

    private static readonly Color GridLineColor = Color.FromArgb(78, 78, 84);

    // 公开的配色，供图例复用，保证图例与网格颜色一致
    public static readonly Color WalkableColor = Color.FromArgb(112, 112, 118);
    public static readonly Color ObstacleColor = Color.FromArgb(32, 32, 36);
    public static readonly Color ExpandedColor = Color.FromArgb(30, 158, 70);
    public static readonly Color FrontierColor = Color.FromArgb(168, 84, 224);
    public static readonly Color ScannedColor = Color.FromArgb(70, 104, 168);
    public static readonly Color PathColor = Color.FromArgb(255, 196, 0);
    public static readonly Color SmoothPathColor = Color.FromArgb(255, 60, 60);
    public static readonly Color StartColor = Color.FromArgb(0, 224, 224);
    public static readonly Color EndColor = Color.FromArgb(255, 0, 170);
    public static readonly Color JumpCleanColor = Color.FromArgb(240, 240, 240);   // 之前已缓存的跳点方向（白）
    public static readonly Color JumpFreshColor = Color.FromArgb(255, 140, 0);      // 本次寻路新更新的跳点方向（橙）

    public event EventHandler<string>? StatusChanged;

    public GridControl(int cellSize)
    {
        _cellSize = cellSize;
        _baseCellSize = cellSize;

        SetStyle(ControlStyles.AllPaintingInWmPaint |
                 ControlStyles.UserPaint |
                 ControlStyles.ResizeRedraw |
                 ControlStyles.OptimizedDoubleBuffer, true);

        BackColor = Color.FromArgb(60, 60, 64);
        _system = new JpsSystem(new GridMap(80, 50));
        _map = _system.Map;
        _overlay.SetWidth(_map.Width);
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
        _startX = _startY = _endX = _endY = -1;
        _overlay.Clear();
        Invalidate();
        NotifyStatus(Loc.T("地图已清除。", "Map cleared."));
    }

    public MapData Export()
    {
        EnsureGrid();
        var data = new MapData { Width = _map.Width, Height = _map.Height };

        if (HasStart)
            data.Start = new PointData { X = _startX, Y = _startY };
        if (HasEnd)
            data.End = new PointData { X = _endX, Y = _endY };

        for (int y = 0; y < _map.Height; y++)
            for (int x = 0; x < _map.Width; x++)
                if (_map.IsBlocked(x, y))
                    data.Obstacles.Add(new PointData { X = x, Y = y });

        return data;
    }

    /// <summary>
    /// 载入一张“定尺”地图（如 MovingAI .map）：网格设为地图的精确宽高，**保持原始格子尺寸不缩小**，
    /// 通过控件自身的滚动条查看超出视口的部分。此后网格不再随窗口大小自动重排。
    /// </summary>
    public void LoadFixedMap(GridMap map)
    {
        _fixedSize = true;
        _system = new JpsSystem(map);
        _map = map;
        _startX = _startY = _endX = _endY = -1;
        _overlay.SetWidth(_map.Width);
        _overlay.Clear();

        _cellSize = _baseCellSize;   // 固定格子尺寸，不缩小
        AutoScroll = true;
        AutoScrollMinSize = new Size(_map.Width * _cellSize, _map.Height * _cellSize);
        AutoScrollPosition = new Point(0, 0);
        Invalidate();
    }

    public void Import(MapData data)
    {
        // JSON 走沙盒模式：随窗口自适应（关闭滚动，恢复默认格尺寸）
        _fixedSize = false;
        _cellSize = _baseCellSize;
        AutoScroll = false;
        AutoScrollMinSize = Size.Empty;

        EnsureGrid();
        _map.ClearAll();

        foreach (var o in data.Obstacles)
            _map.SetBlocked(o.X, o.Y, true);   // 越界坐标会被 SetBlocked 自动忽略

        _startX = _startY = _endX = _endY = -1;
        if (data.Start != null && _map.IsWalkable(data.Start.X, data.Start.Y))
        {
            _startX = data.Start.X;
            _startY = data.Start.Y;
        }
        if (data.End != null && _map.IsWalkable(data.End.X, data.End.Y))
        {
            _endX = data.End.X;
            _endY = data.End.Y;
        }

        _overlay.Clear();
        Invalidate();
    }

    public PathResult RunJps()
    {
        EnsureGrid();
        _system.Sync();         // 单线程把共享缓存同步到当前地图（按版本置脏）
        SnapshotCleanState();   // 记录寻路前的 clean 状态，供 UI 区分本次新更新的方向
        var sw = Stopwatch.StartNew();
        var result = _jps.FindPath(_system, (_startX, _startY), (_endX, _endY));
        sw.Stop();

        _overlay.SetSearchCells(result.Expanded, result.Frontier, result.Scanned);
        _overlay.SetPath(result.Path);
        _overlay.SetSmoothPath(PathSmoother.Smooth(_map, result.Path));
        Invalidate();
        NotifyStatus(DescribeResult("JPS", result, sw));
        return result;
    }

    public PathResult RunAStar()
    {
        EnsureGrid();
        var sw = Stopwatch.StartNew();
        var result = _astar.FindPath(_map, (_startX, _startY), (_endX, _endY));
        sw.Stop();

        _overlay.SetSearchCells(result.Expanded, result.Frontier, result.Scanned);
        _overlay.SetPath(result.Path);
        _overlay.SetSmoothPath(PathSmoother.Smooth(_map, result.Path));
        Invalidate();
        NotifyStatus(DescribeResult("A*", result, sw));
        return result;
    }

    // 按系统语言把寻路结果格式化为状态栏文案（中/英）。表现层负责本地化，
    // 算法层（PathResult）只提供数据（扩展数、前沿、扫描、路径等），保持 UI 无关。
    private string DescribeResult(string algo, PathResult r, Stopwatch sw)
    {
        if (!HasStart || !HasEnd)
            return Loc.T("请先设置起点和终点。", "Set a start and a goal first.");
        if (!_map.IsWalkable(_startX, _startY) || !_map.IsWalkable(_endX, _endY))
            return Loc.T("起点或终点位于阻挡上。", "Start or goal is on an obstacle.");

        bool isAStar = algo == "A*";
        string body;
        if (r.Success)
        {
            string mid = isAStar
                ? Loc.Zh ? $"搜索合计 {r.Expanded.Count + r.Frontier.Count} 格，" : $"searched {r.Expanded.Count + r.Frontier.Count} cells, "
                : Loc.Zh ? $"扫描跳过 {r.Scanned.Count} 格，" : $"scanned-skipped {r.Scanned.Count} cells, ";
            body = Loc.Zh
                ? $"{algo}：扩展 {r.ExpandedNodes}，入队未扩展 {r.Frontier.Count}，{mid}路径 {r.Path.Count} 格。"
                : $"{algo}: expanded {r.ExpandedNodes}, frontier {r.Frontier.Count}, {mid}path {r.Path.Count} cells.";
        }
        else
        {
            string tail = isAStar
                ? Loc.Zh ? " 格" : " cells"
                : Loc.Zh ? $"，扫描跳过 {r.Scanned.Count} 格" : $", scanned-skipped {r.Scanned.Count} cells";
            body = Loc.Zh
                ? $"{algo}：未找到路径（扩展 {r.ExpandedNodes}{tail}）。"
                : $"{algo}: no path found (expanded {r.ExpandedNodes}{tail}).";
        }

        return $"{body} {Loc.T("用时", "in")} {sw.Elapsed.TotalMilliseconds:F2} ms";
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
        // 按下时的格子决定整笔笔触：点在阻挡上 → 擦除（单格）；点在空地 → 绘制（2×2）
        _eraseObstacle = _map.IsBlocked(x, y);
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

    protected override void OnMouseEnter(EventArgs e)
    {
        base.OnMouseEnter(e);
        if (!Focused)
            Focus();   // 让滚轮（滚动 / Ctrl 缩放）悬停即生效，无需先点击
    }

    protected override void OnMouseWheel(MouseEventArgs e)
    {
        if ((ModifierKeys & Keys.Control) == Keys.Control)
        {
            int dir = Math.Sign(e.Delta);
            if (dir != 0)
                ZoomAt(e.Location, dir);   // Ctrl+滚轮：缩放格子
            return;                        // 吞掉滚轮事件，不再滚动
        }

        base.OnMouseWheel(e);              // 普通滚轮：定尺地图下滚动查看
    }

    /// <summary>以鼠标位置为锚点缩放格子像素尺寸（Ctrl+滚轮）。</summary>
    private void ZoomAt(Point mouse, int dir)
    {
        int old = _cellSize;
        int next = dir > 0 ? (int)Math.Ceiling(old * 1.25) : (int)Math.Floor(old / 1.25);
        if (next == old) next = old + dir;                       // 保证至少变化 1px
        next = Math.Min(MaxCell, Math.Max(MinCell, next));
        if (next == old) return;

        if (_fixedSize)
        {
            // 记录鼠标处对应的“浮点格坐标”，缩放后把它重新对回鼠标位置，做到锚点缩放
            var ap = AutoScrollPosition;                         // <= 0
            double fx = (mouse.X - ap.X) / (double)old;
            double fy = (mouse.Y - ap.Y) / (double)old;

            _cellSize = next;
            AutoScrollMinSize = new Size(_map.Width * next, _map.Height * next);

            int targetX = (int)Math.Round(fx * next) - mouse.X;
            int targetY = (int)Math.Round(fy * next) - mouse.Y;
            AutoScrollPosition = new Point(Math.Max(0, targetX), Math.Max(0, targetY));
        }
        else
        {
            _cellSize = next;
            EnsureGrid();   // 沙盒模式：按新格尺寸重排网格密度
        }

        Invalidate();
        NotifyStatus(Loc.Zh ? $"缩放：{next}px / 格" : $"Zoom: {next}px/cell");
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);
        EnsureGrid();

        var g = e.Graphics;
        g.Clear(BackColor);

        int cs = _cellSize;

        // 按滚动位置平移，之后所有绘制都用“世界坐标”(x*cs)。沙盒模式 AutoScroll=false 时偏移为 0。
        var ap = AutoScrollPosition;            // x,y <= 0
        g.TranslateTransform(ap.X, ap.Y);

        // 仅绘制当前视口覆盖的格子（大图必须裁剪，否则遍历百万格会卡）
        int viewX = -ap.X, viewY = -ap.Y;
        int sx = Math.Max(0, viewX / cs);
        int sy = Math.Max(0, viewY / cs);
        int ex = Math.Min(_map.Width, (viewX + ClientSize.Width) / cs + 1);
        int ey = Math.Min(_map.Height, (viewY + ClientSize.Height) / cs + 1);

        for (int y = sy; y < ey; y++)
        {
            for (int x = sx; x < ex; x++)
            {
                var rect = new Rectangle(x * cs, y * cs, cs, cs);
                g.FillRectangle(_map.IsBlocked(x, y) ? ObstacleBrush : WalkableBrush, rect);
            }
        }

        DrawSearchOverlay(g, cs, sx, sy, ex, ey);
        DrawGridLines(g, cs, sx, sy, ex, ey);
        DrawMarkers(g, cs);
        DrawDirtyDots(g, cs, sx, sy, ex, ey);
    }

    private static readonly SolidBrush WalkableBrush = new(WalkableColor);
    private static readonly SolidBrush ObstacleBrush = new(ObstacleColor);

    private void EnsureGrid()
    {
        if (_fixedSize)
            return;   // 定尺地图：网格固定为地图尺寸，不随窗口重排

        int w = ClientSize.Width;
        int h = ClientSize.Height;

        int cols = w > 1 ? (w - 1) / _cellSize : _map.Width;
        int rows = h > 1 ? (h - 1) / _cellSize : _map.Height;
        cols = Math.Max(2, cols);
        rows = Math.Max(2, rows);

        if (_map.Width == cols && _map.Height == rows)
            return;

        var next = new GridMap(cols, rows);

        int copyW = Math.Min(cols, _map.Width);
        int copyH = Math.Min(rows, _map.Height);
        for (int y = 0; y < copyH; y++)
            for (int x = 0; x < copyW; x++)
                if (_map.IsBlocked(x, y))
                    next.SetBlocked(x, y, true);

        _system = new JpsSystem(next);   // 新地图 → 新的共享缓存
        _map = next;
        _overlay.SetWidth(_map.Width);
        _overlay.Clear();   // 网格尺寸变化，旧的搜索结果失效

        // 起终点若超出新网格或落到阻挡上则清除
        if (HasStart && !_map.IsWalkable(_startX, _startY))
            _startX = _startY = -1;
        if (HasEnd && !_map.IsWalkable(_endX, _endY))
            _endX = _endY = -1;
    }

    private void PaintObstacleBlock(int cx, int cy)
    {
        // 绘制：以点击格为中心，刷 BrushSize×BrushSize 的阻挡块
        int half = BrushSize / 2;
        int x0 = cx - half;
        int y0 = cy - half;

        for (int y = y0; y < y0 + BrushSize; y++)
            for (int x = x0; x < x0 + BrushSize; x++)
                _map.SetBlocked(x, y, true);
    }

    private void ClearMarkersOnObstacles()
    {
        if (HasStart && !_map.IsWalkable(_startX, _startY))
            _startX = _startY = -1;
        if (HasEnd && !_map.IsWalkable(_endX, _endY))
            _endX = _endY = -1;
    }

    private void ApplyEdit(int x, int y)
    {
        // 任何编辑都会让上一次的搜索结果失效，清掉可视化叠加
        _overlay.Clear();

        switch (_mode)
        {
            case EditMode.BrushObstacle:
                if (_eraseObstacle)
                {
                    _map.SetBlocked(x, y, false);   // 点在阻挡上：只清 1 格
                }
                else
                {
                    PaintObstacleBlock(x, y);        // 点在空地：刷 2×2 阻挡
                    ClearMarkersOnObstacles();       // 起终点被刷成阻挡则清除
                }
                Invalidate();
                break;

            case EditMode.SetStart:
                if (_map.IsWalkable(x, y))
                {
                    _startX = x; _startY = y;
                    Invalidate();
                    NotifyStatus(Loc.Zh ? $"起点：({x}, {y})" : $"Start: ({x}, {y})");
                }
                break;

            case EditMode.SetEnd:
                if (_map.IsWalkable(x, y))
                {
                    _endX = x; _endY = y;
                    Invalidate();
                    NotifyStatus(Loc.Zh ? $"终点：({x}, {y})" : $"Goal: ({x}, {y})");
                }
                break;
        }
    }

    private void DrawSearchOverlay(Graphics g, int cs, int sx, int sy, int ex, int ey)
    {
        // 绿=已扩展，紫=已入队未扩展（前沿），蓝灰=扫描跳过（未进 open）
        using var scannedBrush = new SolidBrush(ScannedColor);
        using var frontierBrush = new SolidBrush(FrontierColor);
        using var expandedBrush = new SolidBrush(ExpandedColor);
        using var pathBrush = new SolidBrush(PathColor);
        using var pathPen = new Pen(Color.FromArgb(255, 255, 140, 0), Math.Max(2f, cs / 3f));

        for (int y = sy; y < ey; y++)
        {
            for (int x = sx; x < ex; x++)
            {
                if (_overlay.IsOnPath(x, y))
                    continue;

                if (_overlay.IsExpanded(x, y))
                    g.FillRectangle(expandedBrush, new Rectangle(x * cs, y * cs, cs, cs));
                else if (_overlay.IsFrontier(x, y))
                    g.FillRectangle(frontierBrush, new Rectangle(x * cs, y * cs, cs, cs));
                else if (_overlay.IsScanned(x, y))
                    g.FillRectangle(scannedBrush, new Rectangle(x * cs, y * cs, cs, cs));
            }
        }

        foreach (var (x, y) in _overlay.Path)
            g.FillRectangle(pathBrush, new Rectangle(x * cs, y * cs, cs, cs));

        if (_overlay.Path.Count >= 2)
        {
            var points = _overlay.Path
                .Select(p => new Point(p.X * cs + cs / 2, p.Y * cs + cs / 2))
                .ToArray();
            g.DrawLines(pathPen, points);
        }

        // 平滑后的路径（视线拉直）用红色折线叠加显示
        if (_overlay.SmoothPath.Count >= 2)
        {
            using var smoothPen = new Pen(SmoothPathColor, Math.Max(2f, cs / 4f))
            {
                StartCap = System.Drawing.Drawing2D.LineCap.Round,
                EndCap = System.Drawing.Drawing2D.LineCap.Round,
                LineJoin = System.Drawing.Drawing2D.LineJoin.Round
            };
            using var nodeBrush = new SolidBrush(SmoothPathColor);

            var points = _overlay.SmoothPath
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

    // 在每个可走格内按方位摆成十字的 4 个点，表示该格 4 个正交方向跳点缓存的状态：
    // 实心=clean（已缓存），空心=dirty（待计算）。位置即方向：上=N、下=S、左=W、右=E。
    private static readonly (int Dir, float Ox, float Oy)[] DotLayout =
    [
        (0,  1f,  0f),   // E → 右
        (1, -1f,  0f),   // W → 左
        (2,  0f,  1f),   // S → 下
        (3,  0f, -1f),   // N → 上
    ];

    // 寻路前为每格每方向记录当前 clean 状态（纯 UI，不触碰算法/数据结构）
    private void SnapshotCleanState()
    {
        int need = _map.Width * _map.Height * 4;
        if (_cleanBefore.Length != need)
            _cleanBefore = new bool[need];
        _snapW = _map.Width;
        _snapH = _map.Height;

        for (int y = 0; y < _map.Height; y++)
            for (int x = 0; x < _map.Width; x++)
                for (int dir = 0; dir < 4; dir++)
                    _cleanBefore[((y * _map.Width + x) * 4) + dir] = _system.Cache.IsClean(_map, x, y, dir);
    }

    private void DrawDirtyDots(Graphics g, int cs, int sx, int sy, int ex, int ey)
    {
        if (cs < 14)
            return;

        float r = Math.Max(2f, cs * 0.1f);
        float off = cs * 0.30f;
        bool snapOk = _snapW == _map.Width && _snapH == _map.Height;

        using var cleanBrush = new SolidBrush(JumpCleanColor);
        using var freshBrush = new SolidBrush(JumpFreshColor);
        using var ringPen = new Pen(Color.FromArgb(200, 240, 240, 240), Math.Max(1.2f, cs / 22f));

        var prev = g.SmoothingMode;
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

        for (int y = sy; y < ey; y++)
        {
            for (int x = sx; x < ex; x++)
            {
                if (!_map.IsWalkable(x, y))
                    continue;
                if ((x == _startX && y == _startY) || (x == _endX && y == _endY))
                    continue;   // 起终点格让位给 S/G 标记

                float cx = x * cs + cs / 2f;
                float cy = y * cs + cs / 2f;

                foreach (var (dir, ox, oy) in DotLayout)
                {
                    float dx = cx + ox * off;
                    float dy = cy + oy * off;

                    if (!_system.Cache.IsClean(_map, x, y, dir))
                    {
                        g.DrawEllipse(ringPen, dx - r, dy - r, r * 2, r * 2);   // dirty：空心
                        continue;
                    }

                    // clean：本次寻路新更新（之前 dirty）用橙色，之前已缓存用白色
                    bool wasClean = snapOk && _cleanBefore[((y * _map.Width + x) * 4) + dir];
                    g.FillEllipse(wasClean ? cleanBrush : freshBrush, dx - r, dy - r, r * 2, r * 2);
                }
            }
        }

        g.SmoothingMode = prev;
    }

    private void DrawGridLines(Graphics g, int cs, int sx, int sy, int ex, int ey)
    {
        if (cs < 4)
            return;   // 格太小：画网格线会糊成一片且拖慢大图渲染

        using var pen = new Pen(GridLineColor);

        int y0 = sy * cs, y1 = ey * cs;
        for (int x = sx; x <= ex; x++)
            g.DrawLine(pen, x * cs, y0, x * cs, y1);

        int x0 = sx * cs, x1 = ex * cs;
        for (int y = sy; y <= ey; y++)
            g.DrawLine(pen, x0, y * cs, x1, y * cs);
    }

    private void DrawMarkers(Graphics g, int cs)
    {
        if (HasStart)
            DrawMarker(g, _startX, _startY, cs, StartColor, "S");

        if (HasEnd)
            DrawMarker(g, _endX, _endY, cs, EndColor, "G");
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
        // 视口坐标 → 世界坐标（AutoScrollPosition 为 <=0，减去等于加回滚动量）
        var ap = AutoScrollPosition;
        x = (location.X - ap.X) / _cellSize;
        y = (location.Y - ap.Y) / _cellSize;
        return _map.InBounds(x, y);
    }

    private void NotifyStatus(string message) => StatusChanged?.Invoke(this, message);
}
