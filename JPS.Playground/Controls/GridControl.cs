/*
 * GridControl.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System.Diagnostics;
using JPS.Data;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS.Controls;

public sealed class GridControl : ScrollableControl
{
    private const int BrushSize = 2;
    private const int DynamicWidth = 137;
    private const int DynamicHeight = 68;
    private const int DynamicMonsterCount = 14;
    private const int DynamicBlockW = 10;
    private const int DynamicBlockH = 7;
    private const int DynamicRandomObstacleCount = 6;
    private const int DynamicMaxBlockStepsPerTick = 1;
    private const int DynamicMonsterMoveInterval = 10;
    private const float DynamicMonsterVisualLerp = 0.18f;
    private const int DynamicObstacleMoveInterval = 30;
    private const int DynamicObstacleMaxDrift = 3;
    private const double DynamicObstacleMoveChance = 0.18;
    private const double DynamicRandomRepathChance = 0.06;
    private const int DynamicPathfinderPoolInitialSize = 2;

    private sealed class DynamicMonster
    {
        public int X;
        public int Y;
        public float VisualX;
        public float VisualY;
        public int TargetX;
        public int TargetY;
        public readonly List<(int X, int Y)> Path = new();
        public int PathIndex;
        public readonly List<System.Numerics.Vector2> SmoothPath = new();
        public int SmoothPathIndex;

        public DynamicMonster(int x, int y, int targetX, int targetY)
        {
            X = x;
            Y = y;
            VisualX = x;
            VisualY = y;
            TargetX = targetX;
            TargetY = targetY;
        }
    }

    private sealed class DynamicObstacle
    {
        public readonly int HomeX;
        public readonly int HomeY;
        public int X;
        public int Y;
        public readonly List<(int X, int Y)> Cells = new();
        public readonly HashSet<(int X, int Y)> CellSet = new();

        public DynamicObstacle(int x, int y)
        {
            HomeX = x;
            HomeY = y;
            X = x;
            Y = y;
        }

        public void AddCell(int x, int y)
        {
            if (CellSet.Add((x, y)))
                Cells.Add((x, y));
        }

        public bool ContainsWorldCell(int x, int y) => CellSet.Contains((x - X, y - Y));
    }

    private readonly record struct DynamicMonsterSnapshot(int Index, int X, int Y, int TargetX, int TargetY);
    private readonly record struct DynamicPlan(
        int Index,
        bool Success,
        List<(int X, int Y)> Path,
        List<System.Numerics.Vector2> SmoothPath,
        SearchOverlay Overlay);

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

    private readonly System.Windows.Forms.Timer _dynamicTimer = new();
    private readonly Random _dynamicRng = new(20260628);
    private DynamicMonster[] _monsters = [];
    private bool[,]? _dynamicStaticBlocked;
    private readonly List<DynamicObstacle> _dynamicObstacles = new();
    private readonly List<JpsPathfinder> _dynamicPathfinderPool = new();
    private bool _dynamicMode;
    private bool _dynamicBusy;
    private int _dynamicBlockX;
    private int _dynamicBlockY;
    private long _dynamicFrames;
    private long _dynamicPathFrames;
    private double _dynamicPathTotalMs;
    private double _dynamicLastPathMs;
    private int _dynamicLastPathRequests;
    private long _dynamicPathFailures;
    private int _dynamicLastPathFailures;
    private int _pendingBlockDx;
    private int _pendingBlockDy;
    private readonly Bitmap[] _monsterSprites = CreateMonsterSprites();

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
    public static readonly Color DynamicBlockColor = Color.FromArgb(34, 34, 38);
    public static readonly Color MonsterColor = Color.FromArgb(255, 74, 58);
    private static readonly Color[] DynamicPathColors =
    [
        Color.FromArgb(255, 214, 76),
        Color.FromArgb(76, 220, 255),
        Color.FromArgb(117, 255, 134),
        Color.FromArgb(255, 112, 210),
        Color.FromArgb(255, 143, 82),
        Color.FromArgb(176, 132, 255),
        Color.FromArgb(95, 255, 211),
        Color.FromArgb(255, 235, 122),
        Color.FromArgb(120, 170, 255),
        Color.FromArgb(255, 107, 107),
    ];

    public event EventHandler<string>? StatusChanged;

    public GridControl(int cellSize)
    {
        _cellSize = cellSize;
        _baseCellSize = cellSize;

        SetStyle(ControlStyles.AllPaintingInWmPaint |
                 ControlStyles.UserPaint |
                 ControlStyles.ResizeRedraw |
                 ControlStyles.OptimizedDoubleBuffer, true);

        AutoScroll = true;   // 所有模式都启用滚动：内容超出视口（放大后/大图）即出现滚动条

        BackColor = Color.FromArgb(60, 60, 64);
        _system = new JpsSystem(new GridMap(80, 50));
        _map = _system.Map;
        _overlay.SetWidth(_map.Width);

        _dynamicTimer.Interval = 33;
        _dynamicTimer.Tick += DynamicTimer_Tick;
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

    public bool DynamicMode => _dynamicMode;

    public void ToggleDynamicDemo()
    {
        if (_dynamicMode)
            StopDynamicDemo();
        else
            StartDynamicDemo();
    }

    private void StartDynamicDemo()
    {
        _dynamicMode = true;
        _dynamicBusy = false;
        _dynamicFrames = 0;
        _dynamicPathFrames = 0;
        _dynamicPathTotalMs = 0;
        _dynamicLastPathMs = 0;
        _dynamicLastPathRequests = 0;
        _dynamicPathFailures = 0;
        _dynamicLastPathFailures = 0;
        _pendingBlockDx = 0;
        _pendingBlockDy = 0;
        _fixedSize = true;
        _cellSize = _baseCellSize;
        AutoScroll = true;
        AutoScrollMinSize = new Size(DynamicWidth * _cellSize, DynamicHeight * _cellSize);
        AutoScrollPosition = new Point(0, 0);
        _overlay.Clear();
        _startX = _startY = _endX = _endY = -1;
        _dynamicBlockX = DynamicWidth / 2 - DynamicBlockW / 2;
        _dynamicBlockY = DynamicHeight / 2 - DynamicBlockH / 2;
        BuildDynamicStaticObstacles();
        EnsureDynamicPathfinderPool();

        _monsters = new DynamicMonster[DynamicMonsterCount];
        for (int i = 0; i < _monsters.Length; i++)
        {
            var pos = RandomDynamicFreeCell(exceptMonster: -1);
            _monsters[i] = new DynamicMonster(pos.X, pos.Y, pos.X, pos.Y);
            PickMonsterTarget(i);
        }

        RebuildDynamicDisplayMap();
        Focus();
        _dynamicTimer.Start();
        Invalidate();
        NotifyStatus(Loc.Zh
            ? "动态障碍测试：方向键移动大障碍，小怪共享同一 JPS 缓存并用占位表避让。"
            : "Dynamic obstacle test: arrow keys move the block; monsters share one JPS cache and avoid via reservations.");
    }

    private void StopDynamicDemo()
    {
        if (!_dynamicMode)
            return;

        _dynamicTimer.Stop();
        _dynamicMode = false;
        _dynamicBusy = false;
        _monsters = [];
        _dynamicStaticBlocked = null;
        _dynamicObstacles.Clear();
        _pendingBlockDx = 0;
        _pendingBlockDy = 0;
        _overlay.Clear();
        NotifyStatus(Loc.T("动态障碍测试已停止。", "Dynamic obstacle test stopped."));
    }

    private void BuildDynamicStaticObstacles()
    {
        _dynamicStaticBlocked = new bool[DynamicWidth, DynamicHeight];
        _dynamicObstacles.Clear();

        for (int x = 0; x < DynamicWidth; x++)
        {
            _dynamicStaticBlocked[x, 0] = true;
            _dynamicStaticBlocked[x, DynamicHeight - 1] = true;
        }
        for (int y = 0; y < DynamicHeight; y++)
        {
            _dynamicStaticBlocked[0, y] = true;
            _dynamicStaticBlocked[DynamicWidth - 1, y] = true;
        }

        AddDynamicBlob(16, 14, 8, 5);
        AddDynamicBlob(36, 18, 12, 6);
        AddDynamicBlob(23, 54, 10, 8);
        AddDynamicBlob(84, 16, 11, 7);
        AddDynamicBlob(103, 48, 13, 8);
        AddDynamicBlob(76, 66, 15, 5);
        AddDynamicWall(48, 8, 4, 19);
        AddDynamicWall(34, 35, 20, 4);
        AddDynamicWall(94, 24, 4, 18);

        for (int i = 0; i < DynamicRandomObstacleCount; i++)
        {
            int x = _dynamicRng.Next(3, DynamicWidth - 3);
            int y = _dynamicRng.Next(3, DynamicHeight - 3);
            AddDynamicPebble(x, y);
        }
    }

    private void AddDynamicWall(int x0, int y0, int w, int h)
    {
        if (_dynamicStaticBlocked == null)
            return;

        var obstacle = new DynamicObstacle(x0, y0);
        for (int y = y0; y < y0 + h; y++)
            for (int x = x0; x < x0 + w; x++)
                obstacle.AddCell(x - x0, y - y0);

        AddDynamicObstacle(obstacle);
    }

    private void AddDynamicBlob(int cx, int cy, int rx, int ry)
    {
        if (_dynamicStaticBlocked == null)
            return;

        var obstacle = new DynamicObstacle(cx, cy);
        for (int y = cy - ry; y <= cy + ry; y++)
            for (int x = cx - rx; x <= cx + rx; x++)
            {
                double nx = (double)(x - cx) / rx;
                double ny = (double)(y - cy) / ry;
                double edgeNoise = _dynamicRng.NextDouble() * 0.28 - 0.10;
                if (nx * nx + ny * ny <= 1.0 + edgeNoise)
                    obstacle.AddCell(x - cx, y - cy);
            }

        int chunks = Math.Max(3, (rx + ry) / 3);
        for (int i = 0; i < chunks; i++)
        {
            int ox = _dynamicRng.Next(-rx, rx + 1);
            int oy = _dynamicRng.Next(-ry, ry + 1);
            int radius = _dynamicRng.Next(1, 4);
            for (int y = cy + oy - radius; y <= cy + oy + radius; y++)
                for (int x = cx + ox - radius; x <= cx + ox + radius; x++)
                    if (Math.Abs(x - (cx + ox)) + Math.Abs(y - (cy + oy)) <= radius + 1)
                        obstacle.AddCell(x - cx, y - cy);
        }

        AddDynamicObstacle(obstacle);
    }

    private void AddDynamicPebble(int x, int y)
    {
        var obstacle = new DynamicObstacle(x, y);
        obstacle.AddCell(0, 0);
        if (_dynamicRng.Next(2) == 0)
            obstacle.AddCell(1, 0);
        else
            obstacle.AddCell(0, 1);

        AddDynamicObstacle(obstacle);
    }

    private void AddDynamicObstacle(DynamicObstacle obstacle)
    {
        if (obstacle.Cells.Count == 0 || !CanPlaceDynamicObstacle(obstacle, 0, 0))
            return;

        _dynamicObstacles.Add(obstacle);
        SetDynamicObstacleCells(obstacle, blocked: true, updateMap: false);
    }

    private void AddDynamicStaticCell(int x, int y)
    {
        if (_dynamicStaticBlocked == null)
            return;
        if ((uint)x >= DynamicWidth || (uint)y >= DynamicHeight)
            return;
        if (IsInDynamicBlock(x, y))
            return;

        _dynamicStaticBlocked[x, y] = true;
    }

    private void StepDynamicObstacles()
    {
        if (_dynamicObstacles.Count == 0 || _dynamicFrames % DynamicObstacleMoveInterval != 0)
            return;

        bool movedAny = false;
        for (int i = 0; i < _dynamicObstacles.Count; i++)
        {
            if (_dynamicRng.NextDouble() > DynamicObstacleMoveChance)
                continue;

            var obstacle = _dynamicObstacles[i];
            for (int attempt = 0; attempt < 4; attempt++)
            {
                int dx = _dynamicRng.Next(-1, 2);
                int dy = _dynamicRng.Next(-1, 2);
                if (dx == 0 && dy == 0)
                    continue;
                if (TryMoveDynamicObstacle(obstacle, dx, dy))
                {
                    movedAny = true;
                    break;
                }
            }
        }

        if (movedAny)
            _overlay.Clear();
    }

    private bool TryMoveDynamicObstacle(DynamicObstacle obstacle, int dx, int dy)
    {
        int nextX = obstacle.X + dx;
        int nextY = obstacle.Y + dy;
        if (Math.Abs(nextX - obstacle.HomeX) > DynamicObstacleMaxDrift ||
            Math.Abs(nextY - obstacle.HomeY) > DynamicObstacleMaxDrift)
            return false;
        if (!CanPlaceDynamicObstacle(obstacle, dx, dy))
            return false;

        SetDynamicObstacleCells(obstacle, blocked: false, updateMap: true);
        obstacle.X = nextX;
        obstacle.Y = nextY;
        SetDynamicObstacleCells(obstacle, blocked: true, updateMap: true);
        return true;
    }

    private bool CanPlaceDynamicObstacle(DynamicObstacle obstacle, int dx, int dy)
    {
        if (_dynamicStaticBlocked == null)
            return false;

        foreach (var cell in obstacle.Cells)
        {
            int x = obstacle.X + dx + cell.X;
            int y = obstacle.Y + dy + cell.Y;
            if ((uint)x >= DynamicWidth || (uint)y >= DynamicHeight)
                return false;
            if (IsInDynamicBlock(x, y))
                return false;
            if (DynamicCellHasMonster(x, y))
                return false;
            if (_dynamicStaticBlocked[x, y] && !obstacle.ContainsWorldCell(x, y))
                return false;
        }

        return true;
    }

    private bool DynamicCellHasMonster(int x, int y)
    {
        for (int i = 0; i < _monsters.Length; i++)
        {
            var m = _monsters[i];
            if (m.X == x && m.Y == y)
                return true;
        }

        return false;
    }

    private void SetDynamicObstacleCells(DynamicObstacle obstacle, bool blocked, bool updateMap)
    {
        if (_dynamicStaticBlocked == null)
            return;

        foreach (var cell in obstacle.Cells)
        {
            int x = obstacle.X + cell.X;
            int y = obstacle.Y + cell.Y;
            if ((uint)x >= DynamicWidth || (uint)y >= DynamicHeight)
                continue;

            _dynamicStaticBlocked[x, y] = blocked;
            if (updateMap && _map.Width == DynamicWidth && _map.Height == DynamicHeight)
                _map.SetBlocked(x, y, blocked);
        }
    }

    private async void DynamicTimer_Tick(object? sender, EventArgs e)
    {
        if (!_dynamicMode || _dynamicBusy)
            return;

        _dynamicBusy = true;
        try
        {
            ApplyPendingDynamicBlockMove();
            StepDynamicObstacles();
            await StepDynamicMonstersAsync();
            UpdateDynamicMonsterVisuals();
            if (!_dynamicMode)
                return;

            _dynamicFrames++;
            Invalidate();
            if (_dynamicFrames % 15 != 0)
                return;

            NotifyStatus(DescribeDynamicStatus() + DescribeDynamicFailureStatus());
        }
        finally
        {
            _dynamicBusy = false;
        }
    }

    private string DescribeDynamicStatus()
    {
        if (_dynamicPathFrames == 0)
        {
            return Loc.Zh
                ? $"动态障碍：帧 {_dynamicFrames}，小怪 {_monsters.Length}，共享 1 个 JpsSystem，寻路均耗 --。"
                : $"Dynamic: frame {_dynamicFrames}, monsters {_monsters.Length}, sharing 1 JpsSystem, path avg --.";
        }

        double avg = _dynamicPathTotalMs / _dynamicPathFrames;
        return Loc.Zh
            ? $"动态障碍：帧 {_dynamicFrames}，小怪 {_monsters.Length}，寻路均耗 {avg:F2} ms（{_dynamicPathFrames} 帧，最近 {_dynamicLastPathMs:F2} ms / {_dynamicLastPathRequests} 次）。"
            : $"Dynamic: frame {_dynamicFrames}, monsters {_monsters.Length}, path avg {avg:F2} ms over {_dynamicPathFrames} path frames, last {_dynamicLastPathMs:F2} ms / {_dynamicLastPathRequests} requests.";
    }

    private string DescribeDynamicFailureStatus() =>
        Loc.Zh
            ? $" 寻路失败 {_dynamicPathFailures} 次，最近 {_dynamicLastPathFailures} 次。"
            : $" Path failures {_dynamicPathFailures}, last {_dynamicLastPathFailures}.";

    private async Task StepDynamicMonstersAsync()
    {
        if (_dynamicFrames % DynamicMonsterMoveInterval != 0)
            return;

        _system.Sync();
        var requests = new List<DynamicMonsterSnapshot>();

        for (int i = 0; i < _monsters.Length; i++)
        {
            var monster = _monsters[i];
            if ((monster.X == monster.TargetX && monster.Y == monster.TargetY) ||
                !IsDynamicFreeForMonster(monster.TargetX, monster.TargetY, exceptMonster: i))
                PickMonsterTarget(i);

            bool noPath = monster.Path.Count == 0 || monster.PathIndex >= monster.Path.Count - 1;
            bool nextBlocked = !noPath && !IsMonsterPathNextUsable(monster, i);
            bool randomRepath = !noPath && !nextBlocked && _dynamicRng.NextDouble() < DynamicRandomRepathChance;
            if (noPath || nextBlocked || randomRepath)
            {
                requests.Add(new DynamicMonsterSnapshot(i, monster.X, monster.Y, monster.TargetX, monster.TargetY));
            }
        }

        var sw = requests.Count > 0 ? Stopwatch.StartNew() : null;
        var plans = JpsBuildInfo.ConcurrentCache
            ? await RunDynamicPlansParallelAsync(requests)
            : RunDynamicPlansSingleThread(requests);

        if (sw != null)
        {
            sw.Stop();
            _dynamicPathFrames++;
            _dynamicLastPathMs = sw.Elapsed.TotalMilliseconds;
            _dynamicLastPathRequests = requests.Count;
            _dynamicPathTotalMs += _dynamicLastPathMs;
        }

        var reserved = new HashSet<int>();
        var merged = new SearchOverlay();
        merged.SetWidth(DynamicWidth);
        merged.BeginCollect();
        int failedPlans = 0;

        foreach (var plan in plans)
        {
            merged.AddFrom(plan.Overlay);
            var monster = _monsters[plan.Index];
            if (!plan.Success)
            {
                failedPlans++;
                PickMonsterTarget(plan.Index);
                continue;
            }

            monster.Path.Clear();
            monster.Path.AddRange(plan.Path);
            monster.PathIndex = 0;
            monster.SmoothPath.Clear();
            monster.SmoothPath.AddRange(plan.SmoothPath);
            monster.SmoothPathIndex = 0;
        }

        if (failedPlans > 0)
        {
            _dynamicLastPathFailures = failedPlans;
            _dynamicPathFailures += failedPlans;
        }
        else if (plans.Length > 0)
        {
            _dynamicLastPathFailures = 0;
        }

        for (int i = 0; i < _monsters.Length; i++)
        {
            var monster = _monsters[i];
            var next = GetMonsterNextStep(monster);
            if (!IsDynamicFreeForMonster(next.X, next.Y, exceptMonster: i) || !reserved.Add(next.Y * DynamicWidth + next.X))
            {
                PickMonsterTarget(i);
                next = (monster.X, monster.Y);
            }

            if (next.X != monster.X || next.Y != monster.Y)
            {
                monster.X = next.X;
                monster.Y = next.Y;
                AdvanceMonsterPathIndex(monster);
            }

            if (monster.X == monster.TargetX && monster.Y == monster.TargetY)
                PickMonsterTarget(i);

        }

        _overlay.BeginCollect();
        _overlay.AddFrom(merged);
    }

    private DynamicPlan PlanMonsterStep(JpsSystem system, DynamicMonsterSnapshot monster, JpsPathfinder finder)
    {
        var overlay = new SearchOverlay();
        overlay.SetWidth(DynamicWidth);
        overlay.BeginCollect();
        var result = finder.FindPath(system, (monster.X, monster.Y), (monster.TargetX, monster.TargetY), overlay);

        // 小怪跟随**平滑后**的路线：把视线拉直的折线栅格化成 8 连通逐格序列。
        // 段已通过 LOS，故栅格化经过的每格都可走且不斜穿角，可安全逐格移动。
        var smoothPath = result.Success
            ? new List<System.Numerics.Vector2>(result.SmoothedPath)
            : new List<System.Numerics.Vector2>();

        return new DynamicPlan(
            monster.Index,
            result.Success,
            result.Success ? RasterizeSmoothPath(smoothPath) : new List<(int X, int Y)>(),
            smoothPath,
            overlay);
    }

    private async Task<DynamicPlan[]> RunDynamicPlansParallelAsync(List<DynamicMonsterSnapshot> requests)
    {
        var rentedFinders = new List<JpsPathfinder>(requests.Count);
        var tasks = new Task<DynamicPlan>[requests.Count];
        try
        {
            for (int i = 0; i < requests.Count; i++)
            {
                var request = requests[i];
                var finder = RentDynamicPathfinder();
                rentedFinders.Add(finder);
                tasks[i] = Task.Run(() => PlanMonsterStep(_system, request, finder));
            }

            return await Task.WhenAll(tasks);
        }
        finally
        {
            for (int i = 0; i < rentedFinders.Count; i++)
                ReturnDynamicPathfinder(rentedFinders[i]);
        }
    }

    private DynamicPlan[] RunDynamicPlansSingleThread(List<DynamicMonsterSnapshot> requests)
    {
        var plans = new DynamicPlan[requests.Count];
        if (requests.Count == 0)
            return plans;

        var finder = RentDynamicPathfinder();
        try
        {
            for (int i = 0; i < requests.Count; i++)
                plans[i] = PlanMonsterStep(_system, requests[i], finder);
        }
        finally
        {
            ReturnDynamicPathfinder(finder);
        }

        return plans;
    }

    private void EnsureDynamicPathfinderPool()
    {
        while (_dynamicPathfinderPool.Count < DynamicPathfinderPoolInitialSize)
            _dynamicPathfinderPool.Add(new JpsPathfinder());
    }

    private JpsPathfinder RentDynamicPathfinder()
    {
        int last = _dynamicPathfinderPool.Count - 1;
        if (last < 0)
            return new JpsPathfinder();

        var finder = _dynamicPathfinderPool[last];
        _dynamicPathfinderPool.RemoveAt(last);
        return finder;
    }

    private void ReturnDynamicPathfinder(JpsPathfinder finder)
    {
        _dynamicPathfinderPool.Add(finder);
    }

    private void UpdateDynamicMonsterVisuals()
    {
        for (int i = 0; i < _monsters.Length; i++)
        {
            var monster = _monsters[i];
            var target = GetMonsterVisualTarget(monster);
            monster.VisualX = Approach(monster.VisualX, target.X, DynamicMonsterVisualLerp);
            monster.VisualY = Approach(monster.VisualY, target.Y, DynamicMonsterVisualLerp);
        }
    }

    private static (float X, float Y) GetMonsterVisualTarget(DynamicMonster monster)
    {
        if (monster.SmoothPath.Count < 2)
            return (monster.X, monster.Y);

        AdvanceMonsterSmoothPathIndex(monster);
        if (monster.SmoothPathIndex >= monster.SmoothPath.Count - 1)
            return SmoothCell(monster.SmoothPath[^1]);

        var a = SmoothCell(monster.SmoothPath[monster.SmoothPathIndex]);
        var b = SmoothCell(monster.SmoothPath[monster.SmoothPathIndex + 1]);
        float vx = b.X - a.X;
        float vy = b.Y - a.Y;
        float len2 = vx * vx + vy * vy;
        if (len2 <= float.Epsilon)
            return b;

        float t = ((monster.X - a.X) * vx + (monster.Y - a.Y) * vy) / len2;
        t = Math.Clamp(t, 0f, 1f);
        return (a.X + vx * t, a.Y + vy * t);
    }

    private static float Approach(float current, float target, float factor)
    {
        float delta = target - current;
        if (Math.Abs(delta) < 0.01f)
            return target;

        return current + delta * factor;
    }

    private GridMap BuildDynamicMap()
    {
        var map = new GridMap(DynamicWidth, DynamicHeight);
        if (_dynamicStaticBlocked != null)
        {
            for (int y = 0; y < DynamicHeight; y++)
                for (int x = 0; x < DynamicWidth; x++)
                    if (_dynamicStaticBlocked[x, y])
                        map.SetBlocked(x, y, true);
        }

        for (int y = _dynamicBlockY; y < _dynamicBlockY + DynamicBlockH; y++)
            for (int x = _dynamicBlockX; x < _dynamicBlockX + DynamicBlockW; x++)
                map.SetBlocked(x, y, true);

        return map;
    }

    private void RebuildDynamicDisplayMap()
    {
        var map = BuildDynamicMap();
        _system = new JpsSystem(map);
        _map = map;
        _overlay.SetWidth(_map.Width);
        if (!_dynamicMode)
            _overlay.Clear();
    }

    private void PickMonsterTarget(int index)
    {
        var p = RandomDynamicFreeCell(exceptMonster: index);
        if (!IsDynamicFreeForMonster(p.X, p.Y, exceptMonster: index))
            p = (_monsters[index].X, _monsters[index].Y);

        _monsters[index].TargetX = p.X;
        _monsters[index].TargetY = p.Y;
        ClearMonsterPath(_monsters[index]);
    }

    private static void ClearMonsterPath(DynamicMonster monster)
    {
        monster.Path.Clear();
        monster.PathIndex = 0;
        monster.SmoothPath.Clear();
        monster.SmoothPathIndex = 0;
    }

    private bool IsMonsterPathNextUsable(DynamicMonster monster, int index)
    {
        var next = GetMonsterNextStep(monster);
        if (next.X == monster.X && next.Y == monster.Y)
            return false;

        return IsDynamicFreeForMonster(next.X, next.Y, exceptMonster: index);
    }

    private static (int X, int Y) GetMonsterNextStep(DynamicMonster monster)
    {
        if (monster.Path.Count == 0 || monster.PathIndex >= monster.Path.Count - 1)
            return (monster.X, monster.Y);

        NormalizeMonsterPathIndex(monster);
        if (monster.Path.Count == 0 || monster.PathIndex >= monster.Path.Count - 1)
            return (monster.X, monster.Y);

        var target = monster.Path[monster.PathIndex + 1];
        return (
            monster.X + Math.Sign(target.X - monster.X),
            monster.Y + Math.Sign(target.Y - monster.Y));
    }

    private static void AdvanceMonsterPathIndex(DynamicMonster monster)
    {
        while (monster.PathIndex < monster.Path.Count - 1 &&
               monster.Path[monster.PathIndex + 1] == (monster.X, monster.Y))
            monster.PathIndex++;

        if (monster.PathIndex < monster.Path.Count - 1 &&
            !PointOnSegment((monster.X, monster.Y), monster.Path[monster.PathIndex], monster.Path[monster.PathIndex + 1]))
            NormalizeMonsterPathIndex(monster);

        AdvanceMonsterSmoothPathIndex(monster);
    }

    private static void AdvanceMonsterSmoothPathIndex(DynamicMonster monster)
    {
        if (monster.SmoothPathIndex < 0)
            monster.SmoothPathIndex = 0;

        while (monster.SmoothPathIndex < monster.SmoothPath.Count - 1 &&
               SmoothCellInt(monster.SmoothPath[monster.SmoothPathIndex + 1]) == (monster.X, monster.Y))
            monster.SmoothPathIndex++;
    }

    private static (float X, float Y) SmoothCell(System.Numerics.Vector2 p) => (p.X - 0.5f, p.Y - 0.5f);

    private static (int X, int Y) SmoothCellInt(System.Numerics.Vector2 p) => ((int)p.X, (int)p.Y);

    private static void NormalizeMonsterPathIndex(DynamicMonster monster)
    {
        if (monster.PathIndex < 0)
            monster.PathIndex = 0;

        if (monster.PathIndex < monster.Path.Count - 1 &&
            PointOnSegment((monster.X, monster.Y), monster.Path[monster.PathIndex], monster.Path[monster.PathIndex + 1]))
            return;

        for (int i = 0; i < monster.Path.Count - 1; i++)
        {
            if (PointOnSegment((monster.X, monster.Y), monster.Path[i], monster.Path[i + 1]))
            {
                monster.PathIndex = i;
                return;
            }
        }

        ClearMonsterPath(monster);
    }

    private static bool PointOnSegment((int X, int Y) p, (int X, int Y) a, (int X, int Y) b)
    {
        int sx = Math.Sign(b.X - a.X);
        int sy = Math.Sign(b.Y - a.Y);
        int dx = Math.Abs(b.X - a.X);
        int dy = Math.Abs(b.Y - a.Y);
        if (dx == 0 && dy == 0)
            return p == a;
        if (dx != 0 && dy != 0 && dx != dy)
            return false;

        int apx = p.X - a.X;
        int apy = p.Y - a.Y;
        if ((sx == 0 && apx != 0) || (sy == 0 && apy != 0))
            return false;
        if (sx != 0 && apx * sx < 0) return false;
        if (sy != 0 && apy * sy < 0) return false;
        if (sx != 0 && sy != 0 && Math.Abs(apx) != Math.Abs(apy))
            return false;

        return Math.Abs(apx) <= dx && Math.Abs(apy) <= dy;
    }

    // 把平滑路径（连续格中心顶点 cx+0.5）栅格化成 8 连通逐格整数序列，供小怪按拉直后的路线逐格移动。
    // 顶点取整即格坐标；相邻顶点间用与 PathSmoother 视线检测同款的超覆盖增量遍历，
    // 因此经过的格与 LOS 检查过的格一致——段既已通视，逐格皆可走且不斜穿角。
    private static List<(int X, int Y)> RasterizeSmoothPath(IReadOnlyList<System.Numerics.Vector2> smooth)
    {
        var cells = new List<(int X, int Y)>();
        if (smooth.Count == 0)
            return cells;

        cells.Add(((int)smooth[0].X, (int)smooth[0].Y));
        for (int i = 1; i < smooth.Count; i++)
            AppendSupercover(cells, (int)smooth[i - 1].X, (int)smooth[i - 1].Y, (int)smooth[i].X, (int)smooth[i].Y);
        return cells;
    }

    // 超覆盖直线：把 (x0,y0)->(x1,y1) 经过的整数格依次追加到 cells（跳过起点，它已是 cells 末尾）。
    // decision==0 时对角推进（与移动规则一致，不产生斜穿角），与 PathSmoother.LineOfSight 的遍历完全对应。
    private static void AppendSupercover(List<(int X, int Y)> cells, int x0, int y0, int x1, int y1)
    {
        int nx = Math.Abs(x1 - x0), ny = Math.Abs(y1 - y0);
        int sx = Math.Sign(x1 - x0), sy = Math.Sign(y1 - y0);
        int x = x0, y = y0, ix = 0, iy = 0;
        while (ix < nx || iy < ny)
        {
            long decision = (1L + 2 * ix) * ny - (1L + 2 * iy) * nx;
            if (decision == 0) { x += sx; y += sy; ix++; iy++; }
            else if (decision < 0) { x += sx; ix++; }
            else { y += sy; iy++; }
            cells.Add((x, y));
        }
    }

    private (int X, int Y) RandomDynamicFreeCell(int exceptMonster)
    {
        for (int i = 0; i < 5000; i++)
        {
            int x = _dynamicRng.Next(1, DynamicWidth - 1);
            int y = _dynamicRng.Next(1, DynamicHeight - 1);
            if (IsDynamicFreeForMonster(x, y, exceptMonster))
                return (x, y);
        }

        for (int y = 1; y < DynamicHeight - 1; y++)
            for (int x = 1; x < DynamicWidth - 1; x++)
                if (IsDynamicFreeForMonster(x, y, exceptMonster))
                    return (x, y);

        return (1, 1);
    }

    private bool IsDynamicFreeForMonster(int x, int y, int exceptMonster)
    {
        if ((uint)x >= DynamicWidth || (uint)y >= DynamicHeight)
            return false;
        if (_dynamicStaticBlocked != null && _dynamicStaticBlocked[x, y])
            return false;
        if (IsInDynamicBlock(x, y))
            return false;

        for (int i = 0; i < _monsters.Length; i++)
        {
            var m = _monsters[i];
            if (m == null)
                continue;
            if (i != exceptMonster && m.X == x && m.Y == y)
                return false;
        }

        return true;
    }

    private bool IsInDynamicBlock(int x, int y) =>
        x >= _dynamicBlockX && x < _dynamicBlockX + DynamicBlockW &&
        y >= _dynamicBlockY && y < _dynamicBlockY + DynamicBlockH;

    private void MoveDynamicBlock(int dx, int dy)
    {
        if (!_dynamicMode)
            return;

        _pendingBlockDx += dx;
        _pendingBlockDy += dy;
    }

    private void ApplyPendingDynamicBlockMove()
    {
        for (int i = 0; i < DynamicMaxBlockStepsPerTick; i++)
        {
            int dx = Math.Sign(_pendingBlockDx);
            int dy = Math.Sign(_pendingBlockDy);
            if (dx == 0 && dy == 0)
                return;

            if (!TryMoveDynamicBlock(dx, dy))
            {
                if (dx != 0)
                    _pendingBlockDx = 0;
                if (dy != 0)
                    _pendingBlockDy = 0;
                return;
            }

            _pendingBlockDx -= dx;
            _pendingBlockDy -= dy;
        }
    }

    private bool TryMoveDynamicBlock(int dx, int dy)
    {
        if (!_dynamicMode)
            return false;

        int nextX = Math.Max(1, Math.Min(DynamicWidth - DynamicBlockW - 1, _dynamicBlockX + dx));
        int nextY = Math.Max(1, Math.Min(DynamicHeight - DynamicBlockH - 1, _dynamicBlockY + dy));
        if (nextX == _dynamicBlockX && nextY == _dynamicBlockY)
            return false;

        for (int y = nextY; y < nextY + DynamicBlockH; y++)
            for (int x = nextX; x < nextX + DynamicBlockW; x++)
                if (_dynamicStaticBlocked != null && _dynamicStaticBlocked[x, y])
                    return false;

        if (DynamicBlockOverlapsMonster(nextX, nextY))
            return false;

        int oldX = _dynamicBlockX;
        int oldY = _dynamicBlockY;
        _dynamicBlockX = nextX;
        _dynamicBlockY = nextY;
        ApplyDynamicBlockToCurrentMap(oldX, oldY, nextX, nextY);

        _overlay.Clear();
        return true;
    }

    private bool DynamicBlockOverlapsMonster(int blockX, int blockY)
    {
        for (int i = 0; i < _monsters.Length; i++)
        {
            var m = _monsters[i];
            if (m.X >= blockX && m.X < blockX + DynamicBlockW &&
                m.Y >= blockY && m.Y < blockY + DynamicBlockH)
                return true;
        }

        return false;
    }

    private void ApplyDynamicBlockToCurrentMap(int oldX, int oldY, int newX, int newY)
    {
        for (int y = oldY; y < oldY + DynamicBlockH; y++)
            for (int x = oldX; x < oldX + DynamicBlockW; x++)
                if (_dynamicStaticBlocked == null || !_dynamicStaticBlocked[x, y])
                    SetDynamicAwareBlocked(x, y, false);

        for (int y = newY; y < newY + DynamicBlockH; y++)
            for (int x = newX; x < newX + DynamicBlockW; x++)
                _map.SetBlocked(x, y, true);
    }

    public void ClearMap()
    {
        StopDynamicDemo();
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
        StopDynamicDemo();
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
        // 按**存档里的真实尺寸**建图，再走“定尺 + 可滚动”模式（同打开 .map）。
        // 之前用窗口自适应的沙盒网格载入，超出窗口的障碍会被 SetBlocked 当越界丢掉——
        // 大图（如 sc1 512²+）只会载入屏幕内那一角。这里按 Width×Height 建图，配合滚动条查看，不再裁剪。
        int w = data.Width, h = data.Height;
        if (w <= 0 || h <= 0)
        {
            // 兼容缺少尺寸的旧/手写存档：由障碍与起终点的最大坐标推断尺寸
            w = h = 1;
            foreach (var o in data.Obstacles) { w = Math.Max(w, o.X + 1); h = Math.Max(h, o.Y + 1); }
            if (data.Start != null) { w = Math.Max(w, data.Start.X + 1); h = Math.Max(h, data.Start.Y + 1); }
            if (data.End != null) { w = Math.Max(w, data.End.X + 1); h = Math.Max(h, data.End.Y + 1); }
        }

        var map = new GridMap(w, h);
        foreach (var o in data.Obstacles)
            map.SetBlocked(o.X, o.Y, true);   // 网格 = 存档尺寸，障碍不再被裁

        LoadFixedMap(map);   // 定尺、保持原始格子大小、AutoScroll 可滚动；会把起终点重置为 -1

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

        Invalidate();
    }

    public PathResult RunJps()
    {
        EnsureGrid();
        _system.Sync();         // 单线程把共享缓存同步到当前地图（按版本置脏）
        SnapshotCleanState();   // 记录寻路前的 clean 状态，供 UI 区分本次新更新的方向
        _overlay.SetWidth(_map.Width);
        _overlay.BeginCollect();
        var sw = Stopwatch.StartNew();
        var result = _jps.FindPath(_system, (_startX, _startY), (_endX, _endY), _overlay);
        sw.Stop();

        _overlay.SetPath(result.Path);
        _overlay.SetSmoothPath(result.SmoothedPath);
        Invalidate();
        NotifyStatus(DescribeResult("JPS", result, sw));
        return result;
    }

    public PathResult RunAStar()
    {
        EnsureGrid();
        _overlay.SetWidth(_map.Width);
        _overlay.BeginCollect();
        var sw = Stopwatch.StartNew();
        var result = _astar.FindPath(_map, (_startX, _startY), (_endX, _endY), _overlay);
        sw.Stop();

        _overlay.SetPath(result.Path);
        _overlay.SetSmoothPath(result.SmoothedPath);
        Invalidate();
        NotifyStatus(DescribeResult("A*", result, sw));
        return result;
    }

    // 按系统语言把寻路结果格式化为状态栏文案（中/英）。表现层负责本地化：
    // 算法层（PathResult）只提供纯数据（扩展数、路径等），可视化计数从 _overlay（采集器）读取。
    private string DescribeResult(string algo, PathResult r, Stopwatch sw)
    {
        if (!HasStart || !HasEnd)
            return Loc.T("请先设置起点和终点。", "Set a start and a goal first.");
        if (!_map.IsWalkable(_startX, _startY) || !_map.IsWalkable(_endX, _endY))
            return Loc.T("起点或终点位于阻挡上。", "Start or goal is on an obstacle.");

        bool isAStar = algo == "A*";
        int frontier = _overlay.FrontierCount;
        int scanned = _overlay.ScannedCount;
        string body;
        if (r.Success)
        {
            string mid = isAStar
                ? Loc.Zh ? $"搜索合计 {_overlay.ExpandedCount + frontier} 格，" : $"searched {_overlay.ExpandedCount + frontier} cells, "
                : Loc.Zh ? $"扫描跳过 {scanned} 格，" : $"scanned-skipped {scanned} cells, ";
            body = Loc.Zh
                ? $"{algo}：扩展 {r.ExpandedNodes}，入队未扩展 {frontier}，{mid}路径 {r.Path.Count} 点。"
                : $"{algo}: expanded {r.ExpandedNodes}, frontier {frontier}, {mid}path {r.Path.Count} points.";
        }
        else
        {
            string tail = isAStar
                ? Loc.Zh ? " 格" : " cells"
                : Loc.Zh ? $"，扫描跳过 {scanned} 格" : $", scanned-skipped {scanned} cells";
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

    protected override bool IsInputKey(Keys keyData)
    {
        if (_dynamicMode)
        {
            Keys key = keyData & Keys.KeyCode;
            if (key == Keys.Left || key == Keys.Right || key == Keys.Up || key == Keys.Down)
                return true;
        }

        return base.IsInputKey(keyData);
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);

        if (!_dynamicMode)
            return;

        switch (e.KeyCode)
        {
            case Keys.Left:
                MoveDynamicBlock(-1, 0);
                e.Handled = true;
                break;
            case Keys.Right:
                MoveDynamicBlock(1, 0);
                e.Handled = true;
                break;
            case Keys.Up:
                MoveDynamicBlock(0, -1);
                e.Handled = true;
                break;
            case Keys.Down:
                MoveDynamicBlock(0, 1);
                e.Handled = true;
                break;
        }
    }

    /// <summary>以鼠标位置为锚点缩放格子像素尺寸（Ctrl+滚轮）。</summary>
    private void ZoomAt(Point mouse, int dir)
    {
        int old = _cellSize;
        int next = dir > 0 ? (int)Math.Ceiling(old * 1.25) : (int)Math.Floor(old / 1.25);
        if (next == old) next = old + dir;                       // 保证至少变化 1px
        next = Math.Min(MaxCell, Math.Max(MinCell, next));
        if (next == old) return;

        // 锚点缩放（所有模式统一）：记录鼠标处的“浮点格坐标”，缩放后把它对回鼠标位置。
        // 放大后网格像素超出视口即由 AutoScroll 出滚动条；沙盒模式此时维度被 EnsureGrid 冻结（见其注释），
        // 故不会被重排回去。
        var ap = AutoScrollPosition;                         // <= 0
        double fx = (mouse.X - ap.X) / (double)old;
        double fy = (mouse.Y - ap.Y) / (double)old;

        _cellSize = next;
        AutoScrollMinSize = new Size(_map.Width * next, _map.Height * next);

        int targetX = (int)Math.Round(fx * next) - mouse.X;
        int targetY = (int)Math.Round(fy * next) - mouse.Y;
        AutoScrollPosition = new Point(Math.Max(0, targetX), Math.Max(0, targetY));

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

        // 按滚动位置平移，之后所有绘制都用“世界坐标”(x*cs)。未滚动（基准缩放/小图）时偏移为 0。
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
        DrawDynamicBlock(g, cs, sx, sy, ex, ey);
        DrawGridLines(g, cs, sx, sy, ex, ey);
        DrawMarkers(g, cs);
        DrawDirtyDots(g, cs, sx, sy, ex, ey);
        DrawDynamicMonsterPaths(g, cs);
        DrawDynamicMonsters(g, cs, sx, sy, ex, ey);
    }

    private static readonly SolidBrush WalkableBrush = new(WalkableColor);
    private static readonly SolidBrush ObstacleBrush = new(ObstacleColor);

    private void DrawDynamicBlock(Graphics g, int cs, int sx, int sy, int ex, int ey)
    {
        if (!_dynamicMode)
            return;

        using var blockBrush = new SolidBrush(DynamicBlockColor);
        using var blockPen = new Pen(Color.FromArgb(210, 255, 255, 255), Math.Max(1f, cs / 9f));
        var blockRect = new Rectangle(_dynamicBlockX * cs, _dynamicBlockY * cs, DynamicBlockW * cs, DynamicBlockH * cs);
        if (blockRect.IntersectsWith(new Rectangle(sx * cs, sy * cs, (ex - sx) * cs, (ey - sy) * cs)))
        {
            g.FillRectangle(blockBrush, blockRect);
            g.DrawRectangle(blockPen, blockRect);
        }
    }

    private void DrawDynamicMonsters(Graphics g, int cs, int sx, int sy, int ex, int ey)
    {
        if (!_dynamicMode)
            return;

        using var targetPen = new Pen(Color.FromArgb(230, MonsterColor), Math.Max(1.5f, cs / 10f));
        var prevMode = g.SmoothingMode;
        var prevInterpolation = g.InterpolationMode;
        var prevPixelOffset = g.PixelOffsetMode;
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
        g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.NearestNeighbor;
        g.PixelOffsetMode = System.Drawing.Drawing2D.PixelOffsetMode.Half;

        foreach (var m in _monsters)
        {
            if (m.TargetX >= sx && m.TargetX < ex && m.TargetY >= sy && m.TargetY < ey && cs >= 7)
            {
                float tx = m.TargetX * cs + cs / 2f;
                float ty = m.TargetY * cs + cs / 2f;
                float tr = Math.Max(2f, cs * 0.22f);
                g.DrawEllipse(targetPen, tx - tr, ty - tr, tr * 2, tr * 2);
            }

            if (m.VisualX < sx - 1 || m.VisualX >= ex + 1 || m.VisualY < sy - 1 || m.VisualY >= ey + 1)
                continue;

            float cx = m.VisualX * cs + cs / 2f;
            float cy = m.VisualY * cs + cs / 2f;
            int size = Math.Max(12, Math.Min(cs + 6, (int)Math.Round(cs * 1.12)));
            var dest = new Rectangle(
                (int)Math.Round(cx - size / 2f),
                (int)Math.Round(cy - size / 2f),
                size,
                size);
            var sprite = _monsterSprites[(int)((_dynamicFrames + m.X + m.Y) % _monsterSprites.Length)];
            g.DrawImage(sprite, dest);
        }

        g.PixelOffsetMode = prevPixelOffset;
        g.InterpolationMode = prevInterpolation;
        g.SmoothingMode = prevMode;
    }

    private void DrawDynamicMonsterPaths(Graphics g, int cs)
    {
        if (!_dynamicMode)
            return;

        var prevMode = g.SmoothingMode;
        g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

        for (int i = 0; i < _monsters.Length; i++)
        {
            var monster = _monsters[i];
            bool hasSmoothPath = monster.SmoothPath.Count - monster.SmoothPathIndex >= 2;
            if (!hasSmoothPath && monster.Path.Count - monster.PathIndex < 2)
                continue;

            var color = DynamicPathColors[i % DynamicPathColors.Length];
            using var pen = new Pen(Color.FromArgb(235, color), Math.Max(2.5f, cs / 3.2f))
            {
                StartCap = System.Drawing.Drawing2D.LineCap.Round,
                EndCap = System.Drawing.Drawing2D.LineCap.Round,
                LineJoin = System.Drawing.Drawing2D.LineJoin.Round
            };
            using var dotBrush = new SolidBrush(Color.FromArgb(220, color));

            var points = new List<PointF>
            {
                new PointF(monster.VisualX * cs + cs / 2f, monster.VisualY * cs + cs / 2f)
            };
            if (hasSmoothPath)
            {
                AdvanceMonsterSmoothPathIndex(monster);
                for (int p = monster.SmoothPathIndex + 1; p < monster.SmoothPath.Count; p++)
                    points.Add(new PointF(monster.SmoothPath[p].X * cs, monster.SmoothPath[p].Y * cs));
            }
            else
            {
                for (int p = monster.PathIndex + 1; p < monster.Path.Count; p++)
                    points.Add(new PointF(monster.Path[p].X * cs + cs / 2f, monster.Path[p].Y * cs + cs / 2f));
            }
            if (points.Count < 2)
                continue;

            g.DrawLines(pen, points.ToArray());

            float r = Math.Max(1.8f, cs / 7f);
            for (int p = 1; p < points.Count; p += 3)
                g.FillEllipse(dotBrush, points[p].X - r, points[p].Y - r, r * 2, r * 2);
        }

        g.SmoothingMode = prevMode;
    }

    private static Bitmap[] CreateMonsterSprites()
    {
        var sprites = new Bitmap[8];
        for (int frame = 0; frame < sprites.Length; frame++)
        {
            var bmp = new Bitmap(24, 24, System.Drawing.Imaging.PixelFormat.Format32bppPArgb);
            using var g = Graphics.FromImage(bmp);
            g.Clear(Color.Transparent);
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            int[] bobFrames = [0, -1, -2, -1, 0, 1, 0, -1];
            int[] strideFrames = [0, 1, 2, 1, 0, -1, -2, -1];
            int bob = bobFrames[frame];
            int stride = strideFrames[frame];
            int squash = frame is 4 or 5 ? 1 : 0;
            int hornWave = frame is 1 or 2 ? 1 : frame is 5 or 6 ? -1 : 0;
            using var shadow = new SolidBrush(Color.FromArgb(145, 0, 0, 0));
            using var outline = new SolidBrush(Color.FromArgb(255, 12, 13, 18));
            using var body = new SolidBrush(Color.FromArgb(255, 28, 30, 40));
            using var face = new SolidBrush(Color.FromArgb(255, 44, 47, 60));
            using var shine = new SolidBrush(Color.FromArgb(110, 120, 132, 160));
            using var horn = new SolidBrush(Color.FromArgb(255, 74, 78, 96));
            using var eye = new SolidBrush(Color.FromArgb(255, 255, 78, 68));
            using var glint = new SolidBrush(Color.FromArgb(255, 255, 228, 112));
            using var mouth = new Pen(Color.FromArgb(230, 255, 132, 120), 1.4f);

            g.FillEllipse(shadow, 3, 19, 18, 4);

            Point[] leftHorn =
            [
                new(6, 7 + bob),
                new(4 - hornWave, 2 + bob),
                new(10, 6 + bob)
            ];
            Point[] rightHorn =
            [
                new(14, 6 + bob),
                new(20 + hornWave, 2 + bob),
                new(18, 8 + bob)
            ];
            g.FillPolygon(outline, leftHorn);
            g.FillPolygon(outline, rightHorn);
            g.FillPolygon(horn, [new(6, 6 + bob), new(5 - hornWave, 3 + bob), new(9, 6 + bob)]);
            g.FillPolygon(horn, [new(15, 6 + bob), new(19 + hornWave, 3 + bob), new(18, 7 + bob)]);

            g.FillEllipse(outline, 2, 5 + bob + squash, 20, 16 - squash);
            g.FillEllipse(body, 4, 5 + bob + squash, 16, 15 - squash);
            g.FillEllipse(face, 6, 9 + bob + squash, 12, 8);
            g.FillEllipse(shine, 7, 7 + bob + squash, 5, 3);

            g.FillEllipse(outline, 2 + stride, 14 + bob, 6, 5);
            g.FillEllipse(outline, 16 - stride, 14 + bob, 6, 5);
            g.FillEllipse(body, 3 + stride, 14 + bob, 4, 4);
            g.FillEllipse(body, 17 - stride, 14 + bob, 4, 4);
            g.FillEllipse(outline, 4 + stride, 18 + bob, 6, 3);
            g.FillEllipse(outline, 14 - stride, 18 + bob, 6, 3);

            g.FillEllipse(eye, 7, 10 + bob + squash, 4, 3);
            g.FillEllipse(eye, 14, 10 + bob + squash, 4, 3);
            g.FillEllipse(glint, 8, 10 + bob + squash, 1.4f, 1.4f);
            g.FillEllipse(glint, 15, 10 + bob + squash, 1.4f, 1.4f);
            g.DrawArc(mouth, 9, 13 + bob + squash, 6, 3, 15, 150);

            sprites[frame] = bmp;
        }

        return sprites;
    }

    private void EnsureGrid()
    {
        if (_fixedSize)
            return;   // 定尺地图：网格固定为地图尺寸，不随窗口重排

        // 放大（格尺寸 > 基准）时不重排维度：冻结当前维度、只长像素，由 AutoScroll 出滚动条（见 ZoomAt），
        // 否则会被重排回去。基准缩放及“缩小”时仍随窗口自适应重排（改密度、铺满窗口、不出滚动条）——
        // 保留原有“缩小看更多”的手感。
        if (_cellSize > _baseCellSize)
        {
            UpdateSandboxScrollSize();
            return;
        }

        int w = ClientSize.Width;
        int h = ClientSize.Height;

        // 基准/缩小：按当前格尺寸铺满视口（cols*_cellSize ≤ 视口 → AutoScrollMinSize ≤ 视口 → 不出滚动条）。
        int cols = w > 1 ? (w - 1) / _cellSize : _map.Width;
        int rows = h > 1 ? (h - 1) / _cellSize : _map.Height;
        cols = Math.Max(2, cols);
        rows = Math.Max(2, rows);

        if (_map.Width == cols && _map.Height == rows)
        {
            UpdateSandboxScrollSize();
            return;
        }

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

        UpdateSandboxScrollSize();
    }

    /// <summary>沙盒模式下把滚动尺寸同步为“网格维度 × 当前格尺寸”。基准缩放时 ≤ 视口（无滚动条），放大后超出即出条。</summary>
    private void UpdateSandboxScrollSize()
    {
        var want = new Size(_map.Width * _cellSize, _map.Height * _cellSize);
        if (AutoScrollMinSize != want)
            AutoScrollMinSize = want;
    }

    private void PaintObstacleBlock(int cx, int cy)
    {
        // 绘制：以点击格为中心，刷 BrushSize×BrushSize 的阻挡块
        int half = BrushSize / 2;
        int x0 = cx - half;
        int y0 = cy - half;

        for (int y = y0; y < y0 + BrushSize; y++)
            for (int x = x0; x < x0 + BrushSize; x++)
                SetDynamicAwareBlocked(x, y, true);
    }

    private void ClearMarkersOnObstacles()
    {
        if (HasStart && !_map.IsWalkable(_startX, _startY))
            _startX = _startY = -1;
        if (HasEnd && !_map.IsWalkable(_endX, _endY))
            _endX = _endY = -1;
    }

    private void SetDynamicAwareBlocked(int x, int y, bool blocked)
    {
        if (!_dynamicMode)
        {
            _map.SetBlocked(x, y, blocked);
            return;
        }

        if ((uint)x >= DynamicWidth || (uint)y >= DynamicHeight)
            return;
        if (IsInDynamicBlock(x, y))
            return;
        if (blocked && DynamicCellHasMonster(x, y))
            return;

        _dynamicStaticBlocked ??= new bool[DynamicWidth, DynamicHeight];
        _dynamicStaticBlocked[x, y] = blocked;
        _map.SetBlocked(x, y, blocked);
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
                    SetDynamicAwareBlocked(x, y, false);   // 点在阻挡上：只清 1 格
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

        foreach (var segment in _overlay.PathSegments)
        {
            if (segment.Count < 2)
                continue;

            var points = segment
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

        var cacheSystem = _system;
        var cacheMap = cacheSystem.Map;
        float r = Math.Max(2f, cs * 0.1f);
        float off = cs * 0.30f;
        bool snapOk = !_dynamicMode && _snapW == _map.Width && _snapH == _map.Height;

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

                    if (!cacheSystem.Cache.IsClean(cacheMap, x, y, dir))
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
