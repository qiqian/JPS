using JPS.Models;

namespace JPS.Pathfinding
{
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
        public bool Success { get; set; }
        public List<(int X, int Y)> Path { get; set; } = new List<(int X, int Y)>();
        public List<(int X, int Y)> Expanded { get; set; } = new List<(int X, int Y)>();
        public List<(int X, int Y)> Frontier { get; set; } = new List<(int X, int Y)>();
        public List<(int X, int Y)> Scanned { get; set; } = new List<(int X, int Y)>();
        public int ExpandedNodes { get; set; }
        public string Message { get; set; } = string.Empty;
    }

    /// <summary>
    /// JPS 寻路器（惰性跳点缓存版）。
    ///
    /// 设计：没有任何 eager 预计算，也不区分静态/动态阻挡。
    ///  - 正交方向用“惰性跳点 memo”：每格每正交方向缓存“到下一个跳点(正)/到墙(负)的带符号距离”。
    ///    命中（clean）则 O(1) 直接读；未命中则沿该方向扫描一次（O(L)），并把整段 run 一起洗成 clean。
    ///    阻挡一旦变化（map.Version 改变），用世代计数器 O(1) 整体置脏。
    ///  - 对角方向永远经典逐格扫描，但其内部的正交分量子检测复用上面的正交 memo，
    ///    因此对角跳跃从 O(L^2) 降到接近 O(L)。
    ///
    /// 性能手法：逐节点数据用扁平数组（无哈希）+ 代次戳免清零 + 缓冲区复用，全程整数运算。
    /// </summary>
    public sealed class JpsPathfinder
    {
        // ---- 按地图尺寸一次性分配、跨多次查询复用的缓冲区 ----
        private int _w, _h, _size;
        private long[] _g = new long[0];           // 各节点已知最短代价 g
        private int[] _parent = new int[0];        // 父跳点 id（JPS 跨格跳跃，回溯需要完整父 id）
        private sbyte[] _parentDir = new sbyte[0]; // 到达该节点的方向索引（剪枝用）
        private int[] _mark = new int[0];          // 访问状态：== 2·gen → open(已生成)，== 2·gen+1 → closed(已展开)
        private int _gen;
        private readonly MinHeap _open = new MinHeap();
        private readonly int[] _dirBuf = new int[JpsDirections.Count];

        // ---- 仅可视化用，按需惰性分配（collectDebug=false 时完全不占用）----
        private int[] _scanGen = new int[0];

        // ---- 惰性正交跳点缓存（独立结构，按 map.Version 失效）----
        private readonly JumpPointCache _jumpCache = new JumpPointCache();

        // 当前查询用的“可走”委托（= map.IsWalkable），供对角强迫邻居判定复用
        private Func<int, int, bool> _walk = static (_, _) => false;

        // 可视化收集
        private readonly List<int> _expandedIds = new List<int>();
        private readonly List<int> _generatedIds = new List<int>();
        private readonly List<int> _scannedIds = new List<int>();

        /// <summary>
        /// 【只读内省 / 仅可视化使用，不在寻路算法路径上】
        /// 查询某格某正交方向的惰性跳点缓存当前是否为 clean。方向索引：0=E,1=W,2=S,3=N。
        /// 缓冲未就绪或地图版本已变则视为 dirty。算法本身用的是 CardinalDist，与此方法无关。
        /// </summary>
        public bool IsCardinalClean(GridMap map, int x, int y, int dir) =>
            _jumpCache.IsClean(map, x, y, dir);

        public PathResult FindPath(GridMap map, (int X, int Y) start, (int X, int Y) goal, bool collectDebug = true)
        {
            if (start.X < 0 || start.Y < 0 || goal.X < 0 || goal.Y < 0)
                return new PathResult { Message = "请先设置起点和终点。" };

            if (!map.IsWalkable(start.X, start.Y) || !map.IsWalkable(goal.X, goal.Y))
                return new PathResult { Message = "起点或终点位于阻挡上。" };

            EnsureBuffers(map);
            NextGeneration();
            _jumpCache.Sync(map);   // 按尺寸准备跳点缓存，并在地图版本变化时 O(1) 整体置脏
            _walk = map.IsWalkable;

            int openMark = _gen * 2;          // 本代“已生成/在 open”标记
            int closedMark = openMark + 1;     // 本代“已展开/closed”标记

            if (collectDebug)
            {
                if (_scanGen.Length != _size)   // 首次需要可视化时才分配
                    _scanGen = new int[_size];
                _expandedIds.Clear();
                _generatedIds.Clear();
                _scannedIds.Clear();
            }

            int gx = goal.X, gy = goal.Y;
            int startId = Id(start.X, start.Y);
            int goalId = Id(gx, gy);

            _open.Clear();
            _g[startId] = 0;
            _mark[startId] = openMark;
            _parent[startId] = -1;
            _parentDir[startId] = -1;
            _open.Enqueue(startId, JpsDirections.OctileHeuristic(start.X, start.Y, gx, gy));

            int expandedCount = 0;

            while (_open.TryDequeue(out int current, out _))
            {
                if (_mark[current] == closedMark)
                    continue;

                _mark[current] = closedMark;
                expandedCount++;
                if (collectDebug)
                    _expandedIds.Add(current);

                if (current == goalId)
                    return Success(startId, goalId, expandedCount, collectDebug, openMark);

                int cx = current % _w;
                int cy = current / _w;

                int dirCount = FillDirections(map, cx, cy, _parentDir[current]);

                for (int i = 0; i < dirCount; i++)
                {
                    int idx = _dirBuf[i];
                    var (dx, dy) = JpsDirections.All[idx];

                    JumpEntry jump = JpsDirections.IsDiagonal(dx, dy)
                        ? DiagonalJump(map, cx, cy, dx, dy, gx, gy)
                        : CardinalJump(map, cx, cy, dx, dy, gx, gy);

                    if (!jump.HasJump)
                    {
                        if (collectDebug)
                            CollectFailedRay(map, cx, cy, dx, dy);
                        continue;
                    }

                    if (collectDebug)
                        CollectSkippedRay(cx, cy, jump.X, jump.Y);

                    int nbId = Id(jump.X, jump.Y);
                    if (_mark[nbId] == closedMark)
                        continue;

                    long moveCost = (long)jump.Steps *
                        (JpsDirections.IsDiagonal(dx, dy) ? JpsDirections.DiagonalCost : JpsDirections.CardinalCost);
                    long tentative = _g[current] + moveCost;

                    bool firstSeen = _mark[nbId] < openMark;
                    if (!firstSeen && tentative >= _g[nbId])
                        continue;

                    _g[nbId] = tentative;
                    _mark[nbId] = openMark;
                    _parent[nbId] = current;
                    _parentDir[nbId] = (sbyte)idx;

                    if (collectDebug && firstSeen)
                        _generatedIds.Add(nbId);

                    long f = tentative + JpsDirections.OctileHeuristic(jump.X, jump.Y, gx, gy);
                    _open.Enqueue(nbId, f);
                }
            }

            return Failure(expandedCount, collectDebug, openMark);
        }

        // ---------------- 正交跳跃（查/更新惰性跳点缓存）----------------

        private JumpEntry CardinalJump(GridMap map, int x, int y, int dx, int dy, int gx, int gy)
        {
            int dir = JpsDirections.IndexOf(dx, dy);   // 正交方向 → 0..3
            int dist = _jumpCache.CardinalDist(map, x, y, dx, dy, dir);
            int maxTravel = dist > 0 ? dist : -dist;

            // 终点正好在这条射线上且可达 → 直接拦截
            bool goalOnRay =
                (dy == 0 && gy == y && Math.Sign(gx - x) == dx) ||
                (dx == 0 && gx == x && Math.Sign(gy - y) == dy);
            if (goalOnRay)
            {
                int distToGoal = dx != 0 ? Math.Abs(gx - x) : Math.Abs(gy - y);
                if (distToGoal <= maxTravel)
                    return new JumpEntry(gx, gy, distToGoal);
            }

            return dist > 0
                ? new JumpEntry(x + dx * dist, y + dy * dist, dist)
                : JumpEntry.None;
        }

        // ---------------- 对角：经典逐格扫描，复用正交 memo ----------------

        private JumpEntry DiagonalJump(GridMap map, int x, int y, int dx, int dy, int gx, int gy)
        {
            int cx = x, cy = y, steps = 0;
            while (true)
            {
                cx += dx;
                cy += dy;
                steps++;

                if (!map.IsWalkable(cx, cy))
                    return JumpEntry.None;
                if (cx == gx && cy == gy)
                    return new JumpEntry(cx, cy, steps);
                if (JpsRules.HasDiagonalForcedNeighbor(_walk, cx, cy, dx, dy))
                    return new JumpEntry(cx, cy, steps);

                // 正交分量子检测（含终点拦截），命中正交 memo 时为 O(1)
                if (CardinalJump(map, cx, cy, dx, 0, gx, gy).HasJump ||
                    CardinalJump(map, cx, cy, 0, dy, gx, gy).HasJump)
                    return new JumpEntry(cx, cy, steps);
            }
        }

        // ---------------- 结果构造 ----------------

        private PathResult Success(int startId, int goalId, int expandedCount, bool collectDebug, int openMark)
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
            var frontier = FrontierPoints(openMark);
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

        private PathResult Failure(int expandedCount, bool collectDebug, int openMark)
        {
            if (!collectDebug)
                return new PathResult { ExpandedNodes = expandedCount, Message = $"JPS：未找到路径（扩展 {expandedCount}）。" };

            var expanded = ToPoints(_expandedIds);
            var frontier = FrontierPoints(openMark);
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

        private List<(int X, int Y)> FrontierPoints(int openMark)
        {
            var frontier = new List<(int X, int Y)>();
            foreach (int id in _generatedIds)
                if (_mark[id] == openMark)   // 已生成但未被关闭
                    frontier.Add((id % _w, id / _w));
            return frontier;
        }

        private List<(int X, int Y)> ToPoints(List<int> ids)
        {
            var pts = new List<(int X, int Y)>(ids.Count);
            foreach (int id in ids)
                pts.Add((id % _w, id / _w));
            return pts;
        }

        // ---------------- 缓冲区与代次 ----------------

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
            _mark = new int[_size];
            _scanGen = new int[0];   // 可视化用，按需在 collectDebug 时分配
            _gen = 0;
            // 跳点缓存由 _jumpCache.Sync 自行按尺寸/版本管理
        }

        private void NextGeneration()
        {
            // 状态用 2·gen / 2·gen+1 编码，接近溢出时清零重来（实践几乎不触发）
            _gen++;
            if (_gen <= (int.MaxValue / 2) - 1)
                return;

            Array.Clear(_mark, 0, _mark.Length);
            Array.Clear(_scanGen, 0, _scanGen.Length);
            _gen = 1;
        }

        private int Id(int x, int y) => y * _w + x;

        // ---------------- 方向剪枝（零分配，写入 _dirBuf，返回数量）----------------

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
                _dirBuf[n++] = parentDir;
                _dirBuf[n++] = JpsDirections.IndexOf(pdx, 0);
                _dirBuf[n++] = JpsDirections.IndexOf(0, pdy);

                if (!map.IsWalkable(x - pdx, y))
                    _dirBuf[n++] = JpsDirections.IndexOf(-pdx, pdy);
                if (!map.IsWalkable(x, y - pdy))
                    _dirBuf[n++] = JpsDirections.IndexOf(pdx, -pdy);

                return n;
            }

            _dirBuf[n++] = parentDir;

            if (pdx != 0)
            {
                if (!map.IsWalkable(x, y + 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, 1);
                if (!map.IsWalkable(x, y - 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, -1);
            }
            else
            {
                if (!map.IsWalkable(x + 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(1, pdy);
                if (!map.IsWalkable(x - 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(-1, pdy);
            }

            return n;
        }

        // ---------------- 可视化采集（不影响算法结果，仅供界面展示）----------------

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

        private void AddScanned(int x, int y)
        {
            int id = Id(x, y);
            if (_scanGen[id] == _gen)
                return;
            _scanGen[id] = _gen;
            _scannedIds.Add(id);
        }

        // ---------------- 路径重建 ----------------

        private List<(int X, int Y)> ReconstructPath(int startId, int goalId)
        {
            var nodes = new List<int> { goalId };
            int current = goalId;
            while (current != startId)
            {
                current = _parent[current];
                nodes.Add(current);
            }
            nodes.Reverse();

            var path = new List<(int X, int Y)>();
            for (int i = 0; i < nodes.Count - 1; i++)
            {
                int fromId = nodes[i];
                int toId = nodes[i + 1];
                AppendSegment(path, fromId % _w, fromId / _w, toId % _w, toId / _w);
            }
            return path;
        }

        private static void AppendSegment(List<(int X, int Y)> path, int fx, int fy, int tx, int ty)
        {
            int dx = Math.Sign(tx - fx);
            int dy = Math.Sign(ty - fy);
            int x = fx, y = fy;

            if (path.Count == 0 || path[path.Count - 1] != (fx, fy))
                path.Add((fx, fy));

            while (x != tx || y != ty)
            {
                x += dx;
                y += dy;
                path.Add((x, y));
            }
        }
    }
}
