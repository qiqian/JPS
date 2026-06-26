using JPS.Models;

namespace JPS.Pathfinding;

/// <summary>
/// 一次“跳跃”的结果：从某格沿某方向跳到的目标格。
/// HasJump=false 表示该方向跳不到任何跳点/终点（撞墙）。
/// Steps 是跳跃跨越的格数（用于按 步数×单格代价 累计 g 值）。
/// </summary>
public readonly struct JumpEntry
{
    public readonly bool HasJump;
    public readonly int X;
    public readonly int Y;
    public readonly int Steps;

    public static JumpEntry None => default;

    public JumpEntry(int x, int y, int steps)
    {
        HasJump = true;
        X = x;
        Y = y;
        Steps = steps;
    }
}

/// <summary>
/// 寻路结果。除最终路径外，还带有用于可视化的三类格子集合：
/// Expanded=已出队展开的节点；Frontier=已入队但未展开的前沿；Scanned=跳跃扫描经过但未进 open 的格子。
/// </summary>
public sealed class PathResult
{
    public bool Success { get; init; }

    /// <summary>最终路径（逐格，含起点与终点）。</summary>
    public List<(int X, int Y)> Path { get; init; } = [];

    /// <summary>已扩展（出队处理过）的跳点。</summary>
    public List<(int X, int Y)> Expanded { get; init; } = [];

    /// <summary>已入队、但搜索结束时仍未出队的前沿跳点。</summary>
    public List<(int X, int Y)> Frontier { get; init; } = [];

    /// <summary>跳跃射线扫描经过、但从未进入 open 列表的格子。</summary>
    public List<(int X, int Y)> Scanned { get; init; } = [];

    public int ExpandedNodes { get; init; }
    public string Message { get; init; } = string.Empty;
}

/// <summary>
/// JPS+（Jump Point Search Plus）寻路器。
///
/// 核心思想：A* 每步要看 8 个邻居，而 JPS 利用网格的对称性，沿某方向“一路跳过”
/// 没有分支意义的格子，只在“跳点”（出现强迫邻居、即必须在此拐弯的格子）处停下入队，
/// 从而把开放列表里的节点数从“每个格子”压缩到“少量跳点”，大幅减少扩展次数。
///
/// 本实现是 JPS+ 变体：先用 <see cref="JpsPrecomputer"/> 把“每格每方向到下一个跳点/墙的距离”
/// 预计算成表（<see cref="JumpCache"/>），运行时查表 O(1) 跳跃，无需逐格扫描。
/// 同时用 JPS+ 的“目标导向”把终点当作动态跳点，保证空白地图也能找到终点。
///
/// 性能要点：
///  - 所有逐节点数据用按 id=y*W+x 索引的扁平数组（无哈希）；
///  - 用“代次戳(generation stamp)”代替每次清零数组；
///  - 缓冲区按地图尺寸只分配一次，跨多次查询复用；
///  - 寻路全程整数运算（代价 横 1000 / 斜 1414）。
/// </summary>
public sealed class JpsPathfinder
{
    private readonly JpsPrecomputer _precomputer = new();

    /// <summary>预计算的跳点距离表，阻挡变化后失效需重建。</summary>
    public JumpCache? Cache { get; private set; }

    // ---- 按地图尺寸一次性分配、跨多次查询复用的缓冲区 ----
    private int _w, _h, _size;
    private long[] _g = [];           // 各节点的已知最短代价 g
    private int[] _parent = [];       // 各节点在最短路径树中的父节点 id（-1 表示无）
    private sbyte[] _parentDir = [];  // 到达该节点的方向索引（0~7，-1 表示起点）
    private int[] _seenGen = [];      // 节点是否已被生成（算过 g）——值等于当前代次即“是”
    private int[] _closedGen = [];    // 节点是否已出队展开
    private int[] _scanGen = [];      // 可视化：是否已记录为“扫描跳过”
    private int _gen;                 // 当前查询的代次号；每次 FindPath 自增以使旧标记自动失效
    private readonly PriorityQueue<int, long> _open = new();   // 开放列表，按 f 值排序，元素为节点 id
    private readonly int[] _dirBuf = new int[JpsDirections.Count];  // 剪枝方向的复用缓冲（避免每次分配）

    // 可视化用的收集列表（仅在 collectDebug 时填充）
    private readonly List<int> _expandedIds = [];
    private readonly List<int> _generatedIds = [];
    private readonly List<int> _scannedIds = [];

    /// <summary>根据当前阻挡布局重建跳点距离表。阻挡改变后必须调用（或由 FindPath 自动触发）。</summary>
    public void RebuildCache(GridMap map) => Cache = _precomputer.Build(map);

    /// <summary>
    /// 在给定地图上从起点搜索到终点。
    /// </summary>
    /// <param name="map">网格地图（含起点、终点、阻挡）。</param>
    /// <param name="collectDebug">是否采集可视化数据（扩展/前沿/扫描格子）。
    /// 关闭时跳过这些逐格采集，得到 JPS 的纯寻路速度。</param>
    public PathResult FindPath(GridMap map, bool collectDebug = true)
    {
        // 前置校验：起终点必须存在且不在阻挡上
        if (!map.HasStart || !map.HasEnd)
            return new PathResult { Message = "请先设置起点和终点。" };

        if (!map.IsWalkable(map.StartX, map.StartY) || !map.IsWalkable(map.EndX, map.EndY))
            return new PathResult { Message = "起点或终点位于阻挡上。" };

        // 存在动态阻挡时改走经典逐格跳跃，不需要（也不能用）只含静态阻挡的跳点表，
        // 因此此时跳过预计算重建，避免浪费。
        bool useClassic = map.HasDynamicObstacles;

        // 仅在使用查表快路径时，才在跳点表失效/未建时重建
        if (!useClassic && (!map.IsPrecomputeValid || Cache == null))
            RebuildCache(map);

        var cache = Cache;   // 经典模式下可能为 null（未用到）
        EnsureBuffers(map);   // 按地图尺寸准备/复用缓冲区
        NextGeneration();     // 代次自增，使上一次查询的所有标记自动失效（无需清零）

        if (collectDebug)
        {
            _expandedIds.Clear();
            _generatedIds.Clear();
            _scannedIds.Clear();
        }

        int gx = map.EndX, gy = map.EndY;
        int startId = Id(map.StartX, map.StartY);
        int goalId = Id(gx, gy);

        // 经典逐格跳跃用的完整可走判定（含动态阻挡）
        Func<int, int, bool> walk = map.IsWalkable;

        // 起点入队：g=0，优先级 f = g + 启发值
        _open.Clear();
        _g[startId] = 0;
        _seenGen[startId] = _gen;
        _parent[startId] = -1;
        _parentDir[startId] = -1;   // 起点没有“来向”，展开时探索全部 8 个方向
        _open.Enqueue(startId, JpsDirections.OctileHeuristic(map.StartX, map.StartY, gx, gy));

        int expandedCount = 0;

        // A* 主循环：每次取出 f 最小的节点展开
        while (_open.TryDequeue(out int current, out _))
        {
            // 惰性删除：同一节点可能以不同 f 多次入队，已展开过的直接跳过
            if (_closedGen[current] == _gen)
                continue;

            _closedGen[current] = _gen;   // 标记为已展开（关闭）
            expandedCount++;
            if (collectDebug)
                _expandedIds.Add(current);

            // 到达终点 → 回溯路径并返回
            if (current == goalId)
                return Success(map, startId, goalId, expandedCount, collectDebug);

            int cx = current % _w;   // 由 id 还原坐标
            int cy = current / _w;
            int baseIdx = useClassic ? 0 : cache!.Base(cx, cy);   // 该格在跳点表中的基址（经典模式不用）

            // 按 JPS 剪枝规则，只在“必要的方向”上尝试跳跃（来向不同则方向集不同）
            int dirCount = FillDirections(map, cx, cy, _parentDir[current]);

            for (int i = 0; i < dirCount; i++)
            {
                int idx = _dirBuf[i];
                var (dx, dy) = JpsDirections.All[idx];

                // 沿该方向跳跃。无动态阻挡时查预计算表 O(1)；有动态阻挡时逐格扫描。
                JumpEntry jump;
                if (useClassic)
                {
                    jump = ClassicJump(walk, cx, cy, dx, dy, gx, gy);
                }
                else
                {
                    int dist = cache!.GetAt(baseIdx, idx);   // 该方向到下一个跳点(正)/到墙(负)的预计算距离
                    jump = JpsDirections.IsDiagonal(dx, dy)
                        ? DiagonalJump(cache!, cx, cy, dx, dy, gx, gy, dist)
                        : CardinalJump(cx, cy, dx, dy, gx, gy, dist);
                }

                if (!jump.HasJump)
                {
                    // 该方向跳不到任何跳点（撞墙）：仅记录可视化的“失败扫描”
                    if (collectDebug)
                        CollectFailedRay(map, cx, cy, dx, dy);
                    continue;
                }

                // 记录起点到跳点之间被跨过的格子（可视化“扫描跳过”）
                if (collectDebug)
                    CollectSkippedRay(cx, cy, jump.X, jump.Y);

                int nbId = Id(jump.X, jump.Y);
                if (_closedGen[nbId] == _gen)
                    continue;   // 跳点已展开过，忽略

                // 跳跃代价 = 跨越格数 × 单格代价（对角 1414 / 正交 1000）
                long moveCost = (long)jump.Steps *
                    (JpsDirections.IsDiagonal(dx, dy) ? JpsDirections.DiagonalCost : JpsDirections.CardinalCost);
                long tentative = _g[current] + moveCost;

                // 松弛：只有当这是首次到达，或找到更短的 g 时才更新并入队
                bool firstSeen = _seenGen[nbId] != _gen;
                if (!firstSeen && tentative >= _g[nbId])
                    continue;

                _g[nbId] = tentative;
                _seenGen[nbId] = _gen;
                _parent[nbId] = current;
                _parentDir[nbId] = (sbyte)idx;

                if (collectDebug && firstSeen)
                    _generatedIds.Add(nbId);   // 首次生成的节点记入前沿候选

                // f = g + 启发值（八方向 octile 距离，可采纳 → 保证最优）
                long f = tentative + JpsDirections.OctileHeuristic(jump.X, jump.Y, gx, gy);
                _open.Enqueue(nbId, f);
            }
        }

        // 开放列表耗尽仍未到终点 → 无解
        return Failure(expandedCount, collectDebug);
    }

    // ---------------- 结果构造 ----------------

    /// <summary>找到终点：回溯出逐格路径，并按需附带可视化的扩展/前沿/扫描集合。</summary>
    private PathResult Success(GridMap map, int startId, int goalId, int expandedCount, bool collectDebug)
    {
        var path = ReconstructPath(startId, goalId);

        if (!collectDebug)
        {
            return new PathResult
            {
                Success = true,
                Path = path,
                ExpandedNodes = expandedCount,
                Message = $"JPS：扩展 {expandedCount}，路径 {path.Count} 格。"
            };
        }

        var expanded = ToPoints(_expandedIds);
        var frontier = FrontierPoints();
        var scanned = ToPoints(_scannedIds);
        return new PathResult
        {
            Success = true,
            Path = path,
            Expanded = expanded,
            Frontier = frontier,
            Scanned = scanned,
            ExpandedNodes = expandedCount,
            Message = $"JPS：扩展 {expanded.Count}，入队未扩展 {frontier.Count}，扫描跳过 {scanned.Count} 格，路径 {path.Count} 格。"
        };
    }

    /// <summary>开放列表耗尽仍未到终点：无解。</summary>
    private PathResult Failure(int expandedCount, bool collectDebug)
    {
        if (!collectDebug)
            return new PathResult { ExpandedNodes = expandedCount, Message = $"JPS：未找到路径（扩展 {expandedCount}）。" };

        var expanded = ToPoints(_expandedIds);
        var frontier = FrontierPoints();
        var scanned = ToPoints(_scannedIds);
        return new PathResult
        {
            Expanded = expanded,
            Frontier = frontier,
            Scanned = scanned,
            ExpandedNodes = expandedCount,
            Message = $"JPS：未找到路径（扩展 {expanded.Count}，扫描跳过 {scanned.Count} 格）。"
        };
    }

    /// <summary>前沿 = 已生成但搜索结束时仍未展开（仍在 open 里）的跳点。</summary>
    private List<(int X, int Y)> FrontierPoints()
    {
        var frontier = new List<(int X, int Y)>();
        foreach (int id in _generatedIds)
            if (_closedGen[id] != _gen)
                frontier.Add((id % _w, id / _w));
        return frontier;
    }

    /// <summary>把节点 id 列表还原为坐标列表。</summary>
    private List<(int X, int Y)> ToPoints(List<int> ids)
    {
        var pts = new List<(int X, int Y)>(ids.Count);
        foreach (int id in ids)
            pts.Add((id % _w, id / _w));
        return pts;
    }

    // ---------------- 缓冲区与代次 ----------------

    /// <summary>地图尺寸变化时才重新分配所有逐节点数组；否则复用，避免每次查询的 GC。</summary>
    private void EnsureBuffers(GridMap map)
    {
        if (_w == map.Width && _size == map.Width * map.Height)
            return;

        _w = map.Width;
        _h = map.Height;
        _size = _w * _h;
        _g = new long[_size];
        _parent = new int[_size];
        _parentDir = new sbyte[_size];
        _seenGen = new int[_size];
        _closedGen = new int[_size];
        _scanGen = new int[_size];
        _gen = 0;
    }

    /// <summary>
    /// 代次戳：每次查询把 _gen 加 1。判断“某节点本次是否访问过”只需比较其戳是否等于 _gen，
    /// 于是无需在每次查询前清零这些数组（O(N) → O(1)）。仅在 _gen 溢出时才真正清零。
    /// </summary>
    private void NextGeneration()
    {
        _gen++;
        if (_gen != int.MaxValue)
            return;

        Array.Clear(_seenGen);
        Array.Clear(_closedGen);
        Array.Clear(_scanGen);
        _gen = 1;
    }

    /// <summary>坐标 → 一维节点 id。</summary>
    private int Id(int x, int y) => y * _w + x;

    // ---------------- 方向剪枝（零分配，写入 _dirBuf，返回数量）----------------

    /// <summary>
    /// 按 JPS 剪枝规则，把当前节点需要尝试跳跃的方向（索引）写入 _dirBuf 并返回数量。
    /// 规则来自“来向”(parentDir)：
    ///  - 起点（parentDir&lt;0）：探索全部 8 个方向。
    ///  - 对角来向：自然方向 = 该对角 + 其两个正交分量；再加由阻挡产生的“强迫方向”。
    ///  - 正交来向：自然方向 = 该正交本身；若侧方被挡，则加对应的对角强迫方向。
    /// 这里的强迫方向判定必须与 <see cref="JpsPrecomputer"/> 的跳点判定一致，否则会漏跳点。
    /// </summary>
    private int FillDirections(GridMap map, int x, int y, sbyte parentDir)
    {
        if (parentDir < 0)
        {
            for (int i = 0; i < JpsDirections.Count; i++)
                _dirBuf[i] = i;
            return JpsDirections.Count;
        }

        var (pdx, pdy) = JpsDirections.All[parentDir];
        int n = 0;

        if (JpsDirections.IsDiagonal(pdx, pdy))
        {
            // 对角的自然后继：继续对角 + 两个分量正交
            _dirBuf[n++] = parentDir;
            _dirBuf[n++] = JpsDirections.IndexOf(pdx, 0);
            _dirBuf[n++] = JpsDirections.IndexOf(0, pdy);

            // 侧后方被挡 → 该侧产生强迫的对角方向
            if (!map.IsWalkable(x - pdx, y))
                _dirBuf[n++] = JpsDirections.IndexOf(-pdx, pdy);
            if (!map.IsWalkable(x, y - pdy))
                _dirBuf[n++] = JpsDirections.IndexOf(pdx, -pdy);

            return n;
        }

        // 正交的自然后继：继续沿原方向
        _dirBuf[n++] = parentDir;

        if (pdx != 0)   // 水平移动：检查上下是否被挡，被挡则产生斜前方强迫方向
        {
            if (!map.IsWalkable(x, y + 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, 1);
            if (!map.IsWalkable(x, y - 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, -1);
        }
        else            // 垂直移动：检查左右
        {
            if (!map.IsWalkable(x + 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(1, pdy);
            if (!map.IsWalkable(x - 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(-1, pdy);
        }

        return n;
    }

    // ---------------- 目标导向跳跃 ----------------

    /// <summary>
    /// 正交方向跳跃。dist 为预计算的带符号距离：&gt;0 表示 dist 步处有跳点，&lt;=0 表示 |dist| 步后撞墙。
    /// JPS+ 目标导向：若终点正好落在这条射线上且在可走范围内，则直接把终点当作跳点返回
    /// （否则空白地图上没有静态跳点就找不到终点）。
    /// </summary>
    private static JumpEntry CardinalJump(int x, int y, int dx, int dy, int gx, int gy, int dist)
    {
        int maxTravel = dist > 0 ? dist : -dist;   // 该方向最多能走多少格

        // 终点是否在本条射线方向上、且同行/同列
        bool goalOnRay =
            (dy == 0 && gy == y && Math.Sign(gx - x) == dx) ||
            (dx == 0 && gx == x && Math.Sign(gy - y) == dy);

        if (goalOnRay)
        {
            int distToGoal = dx != 0 ? Math.Abs(gx - x) : Math.Abs(gy - y);
            if (distToGoal <= maxTravel)   // 终点在撞墙前可达 → 直接拦截
                return new JumpEntry(gx, gy, distToGoal);
        }

        // 否则返回预计算的静态跳点（若有）
        return dist > 0
            ? new JumpEntry(x + dx * dist, y + dy * dist, dist)
            : JumpEntry.None;
    }

    /// <summary>
    /// 对角方向跳跃。除返回静态对角跳点外，做 JPS+ 目标导向：当终点位于该对角象限时，
    /// 沿对角走到与终点对齐的“拐点”，把拐点作为中间跳点入队；下一步从拐点的正交方向即可拦截终点。
    /// 这样每段父子连线仍是单一方向直线，便于路径重建。
    /// </summary>
    private static JumpEntry DiagonalJump(JumpCache cache, int x, int y, int dx, int dy, int gx, int gy, int dist)
    {
        int maxDiag = dist > 0 ? dist : -dist;   // 对角可走格数

        // 终点必须落在该对角所在象限（两轴方向都一致）才尝试拦截
        if (Math.Sign(gx - x) == dx && Math.Sign(gy - y) == dy)
        {
            int absDx = Math.Abs(gx - x);
            int absDy = Math.Abs(gy - y);
            int minDiff = Math.Min(absDx, absDy);   // 对齐到终点某一轴所需的对角步数

            // 若在到达拐点之前就有真正的对角跳点，则先返回它
            if (dist > 0 && dist < minDiff)
                return new JumpEntry(x + dx * dist, y + dy * dist, dist);

            if (minDiff <= maxDiag)   // 能沿对角到达拐点（畅通）
            {
                if (absDx == absDy)   // 终点正好在纯对角线上，直接到终点
                    return new JumpEntry(gx, gy, minDiff);

                // 拐点：对角走 minDiff 步后的格子，此后终点变成纯正交方向
                int ax = x + dx * minDiff;
                int ay = y + dy * minDiff;
                int remaining = Math.Abs(absDx - absDy);    // 拐点到终点的正交距离
                int cardDir = absDx > absDy
                    ? JpsDirections.IndexOf(dx, 0)
                    : JpsDirections.IndexOf(0, dy);
                int cardDist = cache.Get(ax, ay, cardDir);
                int cardTravel = cardDist > 0 ? cardDist : -cardDist;

                // 拐点到终点的正交路径畅通 → 把拐点作为中间跳点
                if (remaining <= cardTravel)
                    return new JumpEntry(ax, ay, minDiff);
            }
        }

        // 否则返回预计算的静态对角跳点（若有）
        return dist > 0
            ? new JumpEntry(x + dx * dist, y + dy * dist, dist)
            : JumpEntry.None;
    }

    // ---------------- 经典逐格跳跃（动态阻挡回退路径）----------------
    // 不用预计算表，逐格扫描并实时检测跳点；walkable 含动态阻挡。
    // 终点在此处通过“到达即返回”自然处理，无需额外的目标导向。

    private static JumpEntry ClassicJump(Func<int, int, bool> w, int x, int y, int dx, int dy, int gx, int gy)
    {
        return JpsDirections.IsDiagonal(dx, dy)
            ? ClassicJumpDiagonal(w, x, y, dx, dy, gx, gy)
            : ClassicJumpCardinal(w, x, y, dx, dy, gx, gy);
    }

    private static JumpEntry ClassicJumpCardinal(Func<int, int, bool> w, int x, int y, int dx, int dy, int gx, int gy)
    {
        int cx = x, cy = y, steps = 0;
        while (true)
        {
            cx += dx;
            cy += dy;
            steps++;

            if (!w(cx, cy))
                return JumpEntry.None;                       // 撞墙/动态阻挡
            if (cx == gx && cy == gy)
                return new JumpEntry(cx, cy, steps);         // 到达终点
            if (JpsRules.HasCardinalForcedNeighbor(w, cx, cy, dx, dy))
                return new JumpEntry(cx, cy, steps);         // 出现强迫邻居 → 跳点
        }
    }

    private static JumpEntry ClassicJumpDiagonal(Func<int, int, bool> w, int x, int y, int dx, int dy, int gx, int gy)
    {
        int cx = x, cy = y, steps = 0;
        while (true)
        {
            cx += dx;
            cy += dy;
            steps++;

            if (!w(cx, cy))
                return JumpEntry.None;
            if (cx == gx && cy == gy)
                return new JumpEntry(cx, cy, steps);
            if (JpsRules.HasDiagonalForcedNeighbor(w, cx, cy, dx, dy))
                return new JumpEntry(cx, cy, steps);

            // 对角途中：若沿两个正交分量能找到跳点，则此格也是对角跳点
            if (ClassicJumpCardinal(w, cx, cy, dx, 0, gx, gy).HasJump ||
                ClassicJumpCardinal(w, cx, cy, 0, dy, gx, gy).HasJump)
                return new JumpEntry(cx, cy, steps);
        }
    }

    // ---------------- 可视化采集（不影响算法结果，仅供界面展示）----------------

    /// <summary>记录某方向“撞墙失败”的射线扫过的格子。</summary>
    private void CollectFailedRay(GridMap map, int x, int y, int dx, int dy)
    {
        int cx = x, cy = y;
        while (true)
        {
            cx += dx;
            cy += dy;
            if (!map.IsWalkable(cx, cy))
                return;
            AddScanned(cx, cy);
        }
    }

    /// <summary>记录从当前格到跳点之间被跨过（未进 open）的格子。</summary>
    private void CollectSkippedRay(int x1, int y1, int x2, int y2)
    {
        int dx = Math.Sign(x2 - x1);
        int dy = Math.Sign(y2 - y1);
        int x = x1, y = y1;
        while (x != x2 || y != y2)
        {
            x += dx;
            y += dy;
            AddScanned(x, y);
        }
    }

    /// <summary>用代次戳去重后把格子记入扫描列表。</summary>
    private void AddScanned(int x, int y)
    {
        int id = Id(x, y);
        if (_scanGen[id] == _gen)
            return;
        _scanGen[id] = _gen;
        _scannedIds.Add(id);
    }

    // ---------------- 路径重建 ----------------

    /// <summary>
    /// 沿 _parent 从终点回溯到起点得到跳点序列，再用 <see cref="AppendSegment"/> 把相邻跳点之间
    /// 补成逐格路径（所以返回的是完整逐格路径，而非稀疏跳点）。
    /// </summary>
    private List<(int X, int Y)> ReconstructPath(int startId, int goalId)
    {
        var nodes = new List<int> { goalId };
        int current = goalId;
        while (current != startId)
        {
            current = _parent[current];
            nodes.Add(current);
        }
        nodes.Reverse();   // 回溯得到的是终点→起点，反转为起点→终点

        var path = new List<(int X, int Y)>();
        for (int i = 0; i < nodes.Count - 1; i++)
        {
            int fromId = nodes[i];
            int toId = nodes[i + 1];
            AppendSegment(path, fromId % _w, fromId / _w, toId % _w, toId / _w);
        }
        return path;
    }

    /// <summary>把一段直线（单一方向，正交或对角）逐格追加到路径，避免重复加入连接点。</summary>
    private static void AppendSegment(List<(int X, int Y)> path, int fx, int fy, int tx, int ty)
    {
        int dx = Math.Sign(tx - fx);
        int dy = Math.Sign(ty - fy);
        int x = fx, y = fy;

        if (path.Count == 0 || path[^1] != (fx, fy))
            path.Add((fx, fy));

        while (x != tx || y != ty)
        {
            x += dx;
            y += dy;
            path.Add((x, y));
        }
    }
}
