/*
 * JpsPathfinder.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;
using System.Collections.Generic;
using JPS.Models;
#if UNITY_2022_1_OR_NEWER
using Vector2 = UnityEngine.Vector2;
#else
using Vector2 = System.Numerics.Vector2;
#endif

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
    /// 寻路结果（纯算法产物，不含任何可视化数据）。
    /// JPS 返回 compact path（起点、跳点/拐点、终点）；A* baseline 返回相邻格路径。
    /// 可视化所需的展开 / 前沿 / 扫描格子通过 <see cref="ISearchObserver"/> 在搜索过程中获取。
    /// </summary>
    public sealed class PathResult
    {
        public bool Success { get; set; }
        /// <summary>true=到达有效目标（goal，或 FindPathNearest 对被挡 goal 做 snap 后的接触格）；
        /// false=返回的是离目标最近的已展开点（FindPathNearest 未达时）。</summary>
        public bool ReachedGoal { get; set; }
        /// <summary>
        /// JPS exposes only compact path points. It does not expose an expanded per-cell path.
        /// A* keeps its adjacent-cell baseline path for cost/accuracy comparison.
        /// </summary>
        public List<(int X, int Y)> Path { get; set; } = new List<(int X, int Y)>();
        /// <summary>Smoothed path, computed by FindPath together with Path.</summary>
        public List<Vector2> SmoothedPath { get; set; } = new List<Vector2>();
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
        // g / steps / 来向索引打包进同一个 ulong（与 native pathfinder.cpp 的 g_dir 同构），
        // 取代原先独立的 _g / _parentSteps / _parentDir 三个数组：一次索引访问同取三者，
        // relax 一次 store 写全，逐节点搜索态 15→10 B/格。布局：
        //   位[0,44)  = g 值（迷宫最优路径 g ≤ ~1.5e12 < 2^44）
        //   位[44,60) = steps 到达该节点的跳跃步数（≤ max(W,H) ≤ 32767）；父节点 = 当前 − dir×steps
        //   位[60,64) = 来向索引+1（0 = 无父/起点哨兵；1..8 = 方向索引 0..7）
        private ulong[] _gDir = new ulong[0];
        private ushort[] _mark = new ushort[0];    // 访问状态：== 2·gen → open(已生成)，== 2·gen+1 → closed(已展开)
        private int _gen;

        private const int GStepsShift = 44;
        private const int GDirShift = 60;
        private const ulong GMask = (1UL << GStepsShift) - 1;   // g 的低 44 位
        private const ulong StepsMask = 0xFFFF;                 // steps 的 16 位

        private static long GdG(ulong gd) => (long)(gd & GMask);
        private static int GdSteps(ulong gd) => (int)((gd >> GStepsShift) & StepsMask);
        private static int GdDir(ulong gd) => (int)((gd >> GDirShift) & 0xF) - 1;   // -1 = 无父（起点）
        private static ulong PackGdir(long g, int steps, int dir) =>
            ((ulong)g & GMask)
            | (((ulong)steps & StepsMask) << GStepsShift)
            | ((ulong)(dir + 1) << GDirShift);
        private readonly MinHeap _open = new MinHeap();
        private readonly int[] _dirBuf = new int[JpsDirections.Count];

        // 当前查询用的共享跳点缓存（每次 FindPath 入口绑定到传入的 JpsSystem）
        private JumpPointCache _cache = null!;

        /// <summary>
        /// 在共享的 <see cref="JpsSystem"/>（地图 + 跳点缓存）上寻路。
        /// 调用前需确保 system 已 <see cref="JpsSystem.Sync"/> 到当前地图（单线程同步缓存版本）。
        /// 每个 JpsPathfinder 只持有自己的逐节点搜索状态，故不同实例可在各自线程上共用同一个 system。
        /// </summary>
        public PathResult FindPath(JpsSystem system, (int X, int Y) start, (int X, int Y) goal, ISearchObserver? obs = null)
            => FindPathCore(system, start, goal, obs, allowNearest: false);

        /// <summary>
        /// 不可达兜底版寻路：与 <see cref="FindPath"/> 相同，但 (1) 允许 goal 落在阻挡上（膨胀后 goal 进障碍的
        /// 大体型场景）；(2) 到不了 goal（含 goal 被挡/被围）时不失败，而是返回搜索展开过的、离 goal 最近
        /// (octile 启发最小) 的节点路径（起点亦在候选内，至少 1 点）。用 <see cref="PathResult.ReachedGoal"/>
        /// 区分“真到达”与“尽力最近”。与 C 版 jps_pathfinder_find_path_nearest 语义与定序严格一致。
        /// </summary>
        public PathResult FindPathNearest(JpsSystem system, (int X, int Y) start, (int X, int Y) goal, ISearchObserver? obs = null)
            => FindPathCore(system, start, goal, obs, allowNearest: true);

        private PathResult FindPathCore(JpsSystem system, (int X, int Y) start, (int X, int Y) goal, ISearchObserver? obs, bool allowNearest)
        {
            var map = system.Map;
            _cache = system.Cache;

            if (start.X < 0 || start.Y < 0 || start.X >= map.Width || start.Y >= map.Height ||
                goal.X < 0 || goal.Y < 0 || goal.X >= map.Width || goal.Y >= map.Height)
                return new PathResult { Message = "起点或终点越界。" };
            if (!map.IsWalkable(start.X, start.Y))
                return new PathResult { Message = "起点位于阻挡上。" };
            // 严格模式要求 goal 可走；nearest 模式允许 goal 落在阻挡上。
            if (!allowNearest && !map.IsWalkable(goal.X, goal.Y))
                return new PathResult { Message = "终点位于阻挡上。" };

            // nearest 模式 + goal 被挡：先 goal-snapping 到最近、朝 start 一侧的可走格（接近侧接触格）再寻路；
            // 够得到 → reached=true 停在接触格，够不到 → 退化为最近已展开点。snap 不到则维持原 goal。
            // 于是 ReachedGoal 表示“到达了这个（可能已 snap 的）有效目标”。与 C 版 jps__snap_goal 逐位一致。
            if (allowNearest && !map.IsWalkable(goal.X, goal.Y)
                && SnapGoal(map, start.X, start.Y, goal.X, goal.Y, out int sgx, out int sgy))
            {
                goal = (sgx, sgy);
            }

            EnsureBuffers(map);
            NextGeneration();   // 缓存同步由 JpsSystem.Sync 负责（调用方在寻路前完成）

            int openMark = _gen * 2;          // 本代“已生成/在 open”标记
            int closedMark = openMark + 1;     // 本代“已展开/closed”标记

            int gx = goal.X, gy = goal.Y;
            int startId = Id(start.X, start.Y);
            int goalId = Id(gx, gy);

            int bestNode = startId;                                              // 起点必是首个展开点 → best 恒有效
            long bestH = JpsDirections.OctileHeuristic(start.X, start.Y, gx, gy);

            _open.Clear();
            _gDir[startId] = PackGdir(0, 0, -1);   // g=0、steps=0、无来向（剪枝时探索全部方向；回溯到此即停）
            _mark[startId] = (ushort)openMark;
            _open.Enqueue(startId, bestH);

            int expandedCount = 0;

            while (_open.TryDequeue(out int current, out _))
            {
                if (_mark[current] == closedMark)
                    continue;

                _mark[current] = (ushort)closedMark;
                expandedCount++;

                int cx = current % _w;
                int cy = current / _w;
                obs?.OnExpand(cx, cy);

                if (current == goalId)
                    return Success(map, startId, goalId, expandedCount, reached: true, gx, gy);

                // nearest 兜底：记录离 goal 最近(octile 最小)的已展开节点。严格 tie-break（h 相等保留先展开者），
                // 展开序确定 → C≡C# 一致。仅 nearest 模式计。
                if (allowNearest)
                {
                    long h = JpsDirections.OctileHeuristic(cx, cy, gx, gy);
                    if (h < bestH) { bestH = h; bestNode = current; }
                }

                int dirCount = FillDirections(map, cx, cy, GdDir(_gDir[current]));

                for (int i = 0; i < dirCount; i++)
                {
                    int idx = _dirBuf[i];
                    var (dx, dy) = JpsDirections.All[idx];

                    bool diagonal = JpsDirections.IsDiagonalIndex(idx);
                    JumpEntry jump = diagonal
                        ? DiagonalJump(map, cx, cy, dx, dy, gx, gy)
                        : CardinalJump(map, cx, cy, dx, dy, idx, gx, gy);

                    if (!jump.HasJump)
                    {
                        if (obs != null) ScanFailedRay(map, cx, cy, dx, dy, obs);
                        continue;
                    }

                    if (obs != null) ScanSkippedRay(cx, cy, jump.X, jump.Y, obs);

                    int nbId = Id(jump.X, jump.Y);
                    if (_mark[nbId] == closedMark)
                        continue;

                    long moveCost = (long)jump.Steps * 
                        (diagonal ? JpsDirections.DiagonalCost : JpsDirections.CardinalCost);
                    long tentative = GdG(_gDir[current]) + moveCost;

                    bool firstSeen = _mark[nbId] < openMark;
                    if (!firstSeen && tentative >= GdG(_gDir[nbId]))
                        continue;

                    // g、steps、来向同字：一次 store 写入三者（步数 ≤ 边长 ≤ 32767）
                    _gDir[nbId] = PackGdir(tentative, jump.Steps, idx);
                    _mark[nbId] = (ushort)openMark;

                    if (firstSeen) obs?.OnFrontier(jump.X, jump.Y);

                    _open.Enqueue(nbId, tentative + JpsDirections.OctileHeuristic(jump.X, jump.Y, gx, gy));
                }
            }

            // 搜索耗尽未达 goal。返回离 goal 最近的已展开节点，再朝 goal 贪心下降贴近（在 Success 内）。
            if (allowNearest)
                return Success(map, startId, bestNode, expandedCount, reached: false, gx, gy);
            return Failure(expandedCount);
        }

        // ---------------- 正交跳跃（查/更新惰性跳点缓存）----------------
        //
        // jump（直线跳跃，论文 Algorithm 2 的直线分支）：沿单一正交方向一路推进——
        //   撞墙/越界 → 该方向无跳点；到终点或当前格出现强迫邻居 → 该格即跳点并返回。
        // 中间格不入队、不展开，只“扫一眼”。这里用惰性正交缓存 CardinalDist 把
        // “到下一个跳点/墙的带符号距离”O(1) 复用，避免每次重复逐格扫描。

        private JumpEntry CardinalJump(GridMap map, int x, int y, int dx, int dy, int dir, int gx, int gy)
        {
            int dist = _cache.CardinalDist(map, x, y, dx, dy, dir);
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
        //
        // jump（对角跳跃，论文 Algorithm 2 的对角分支）：沿对角一步步走 c——
        //   撞墙/越界（含默认禁止切角时两侧未全开）→ 无跳点；到终点 → 返回；
        //   当前格有强迫邻居 → 返回（跳点）；
        //   关键（论文 Definition 2 条件 3）：每个对角格先沿它的两个正交分量各做一次直线跳跃，
        //   只要任一分量找到跳点，当前对角格就也算跳点并返回——否则会漏掉“该转直线”的拐点。

        private JumpEntry DiagonalJump(GridMap map, int x, int y, int dx, int dy, int gx, int gy)
        {
            int cx = x, cy = y, steps = 0;
            int horizontalDir = JpsDirections.IndexOf(dx, 0);
            int verticalDir = JpsDirections.IndexOf(0, dy);
            while (true)
            {
                // 默认禁止斜穿角：从当前格斜走一步需目标格 + 两侧正交格都可走
                if (!JpsDirections.DiagonalAllowed(map, cx, cy, dx, dy))
                    return JumpEntry.None;

                cx += dx;
                cy += dy;
                steps++;

                if (cx == gx && cy == gy)
                    return new JumpEntry(cx, cy, steps);
#if JPS_ALLOW_CORNER_CUTTING
                // 只有切角模型下对角才可能产生强迫邻居；默认禁止切角时 HasDiagonalForcedNeighbor 恒为 false
                // （见 JpsRules），故编译期剔除这次调用——每个对角步省一次空判定。
                if (JpsRules.HasDiagonalForcedNeighbor(map, cx, cy, dx, dy))
                    return new JumpEntry(cx, cy, steps);
#endif

                // 正交分量子检测：直接读正交 memo（命中为 O(1)），等价于 CardinalJump(...).HasJump，
                // 但避免每步构造 JumpEntry。短路顺序与原来一致（横向先于纵向，任一成立即返回）。
                //
                // 终点拦截只可能发生在“对角恰好走到终点所在行(cy==gy → 横向射线)或列(cx==gx → 纵向射线)”的
                // 那唯一一步——因为 cy/cx 每步只变 1，最多各跨过 gy/gx 一次。故把 goalOnRay 的 Math.Sign/Abs
                // 从“每步、每个子检测都算”降为“仅在跨到该行/列时算一次”，长对角上省下成片判定。
                // 可达性沿用 CardinalJump 的口径：maxTravel = |memo dist|，此处 memo ≤ 0（>0 已先返回）故 = -dist。
                int hd = _cache.CardinalDist(map, cx, cy, dx, 0, horizontalDir);
                if (hd > 0) return new JumpEntry(cx, cy, steps);                       // 横向有正交跳点
                if (cy == gy && Math.Sign(gx - cx) == dx && Math.Abs(gx - cx) <= -hd) // 终点在横向射线上且可达
                    return new JumpEntry(cx, cy, steps);

                int vd = _cache.CardinalDist(map, cx, cy, 0, dy, verticalDir);
                if (vd > 0) return new JumpEntry(cx, cy, steps);                       // 纵向有正交跳点
                if (cx == gx && Math.Sign(gy - cy) == dy && Math.Abs(gy - cy) <= -vd) // 终点在纵向射线上且可达
                    return new JumpEntry(cx, cy, steps);
            }
        }

        // ---------------- 结果构造 ----------------

        private PathResult Success(GridMap map, int startId, int endId, int expandedCount, bool reached, int gx, int gy)
        {
            var path = ReconstructPath(startId, endId);
            if (!reached)
                NearestRefine(map, path[^1].X, path[^1].Y, gx, gy, path);   // 跳点粒度补偿：从最近跳点有界 GBFS 找真最近可达格
            return new PathResult
            {
                Success = true,
                ReachedGoal = reached,
                Path = path,
                SmoothedPath = PathSmoother.Smooth(map, path),
                ExpandedNodes = expandedCount,
                Message = reached
                    ? $"JPS：扩展 {expandedCount}，路径 {path.Count} 点。"
                    : $"JPS：未达终点，返回最近点（扩展 {expandedCount}，路径 {path.Count} 点）。"
            };
        }

        private static PathResult Failure(int expandedCount) =>
            new PathResult { ExpandedNodes = expandedCount, Message = $"JPS：未找到路径（扩展 {expandedCount}）。" };

        /// <summary>
        /// goal-snapping：goal 落在阻挡上时按 Chebyshev 环由近及远扫，返回最近的可走格——同环内取离 start
        /// 最近(octile 最小)者（接近侧接触格），严格 tie-break（同 h 保留扫描序先者）。周围全被挡返回 false。
        /// 环由近及远 → 首个含可走格的环即最近；OOB 环格由 IsWalkable 的界内判定天然跳过。
        /// 与 C 版 jps__snap_goal 的环迭代序与选取严格一致，保证 C≡C#。
        /// </summary>
        private static bool SnapGoal(GridMap map, int sx, int sy, int gx, int gy, out int ox, out int oy)
        {
            ox = 0; oy = 0;
            int maxR = Math.Max(map.Width, map.Height);
            for (int r = 1; r <= maxR; r++)
            {
                long bestH = -1;   // 哨兵：本环尚未找到
                int bx = 0, by = 0;
                for (int yy = gy - r; yy <= gy + r; yy++)
                {
                    bool onYBorder = (yy == gy - r || yy == gy + r);   // 上/下边整行；中间行只取左右两端
                    for (int xx = gx - r; xx <= gx + r; xx++)
                    {
                        if (!onYBorder && xx != gx - r && xx != gx + r)
                            continue;                                  // 跳过环内部，只取边界
                        if (!map.IsWalkable(xx, yy))
                            continue;
                        long h = JpsDirections.OctileHeuristic(xx, yy, sx, sy);
                        if (bestH < 0 || h < bestH)                    // 严格 <：同 h 保留先者
                        {
                            bestH = h; bx = xx; by = yy;
                        }
                    }
                }
                if (bestH >= 0) { ox = bx; oy = by; return true; }
            }
            return false;
        }

        /// <summary>
        /// 跳点粒度补偿：从最近跳点 (bx,by) 做有界 greedy-best-first flood（按 octile-to-goal 排序），在连通
        /// 可达域内找 octile 最小的可达格——GBFS 遇死胡同会回探其他分支，故不像纯贪心那样卡局部最小（最近
        /// 可达格常在需先“绕远”才能到的方向）。上限 Cap 格封顶开销。复用 _open 堆 + _gDir(存 BFS 父，主搜索
        /// 数据已被 ReconstructPath 提走) + _mark(fresh 世代作 visited)。沿父链回溯把 (bx,by) 之后到最近格逐格
        /// 追加进 path（平滑器后续拉直）。堆只按 octile 排序、载荷(id)不参与比较 → 出队序与 C 版一致，输出相同。
        /// </summary>
        private void NearestRefine(GridMap map, int bx, int by, int gx, int gy, List<(int X, int Y)> path)
        {
            const int Cap = 4096;
            int startNode = Id(bx, by);
            int bestNode = startNode;
            long bestH = JpsDirections.OctileHeuristic(bx, by, gx, gy);
            int visited = 0;

            NextGeneration();                          // fresh 世代 → 全新 visited 标记，不与主搜索冲突
            int visitedMark = _gen * 2;

            _open.Clear();
            _mark[startNode] = (ushort)visitedMark;
            _gDir[startNode] = (ulong)(uint)startNode;   // 自指 = 无父哨兵
            _open.Enqueue(startNode, bestH);

            while (visited < Cap && _open.TryDequeue(out int cur, out long prio))
            {
                int cx = cur % _w, cy = cur / _w;
                visited++;
                if (prio < bestH) { bestH = prio; bestNode = cur; }   // prio 即该格 octile（入队即标记，无重复）
                for (int i = 0; i < JpsDirections.Count; i++)
                {
                    var (dx, dy) = JpsDirections.All[i];
                    int nx = cx + dx, ny = cy + dy;
                    bool ok = JpsDirections.IsDiagonalIndex(i)
                        ? JpsDirections.DiagonalAllowed(map, cx, cy, dx, dy)
                        : map.IsWalkable(nx, ny);
                    if (!ok)
                        continue;
                    int nid = Id(nx, ny);
                    if (_mark[nid] == (ushort)visitedMark)
                        continue;
                    _mark[nid] = (ushort)visitedMark;
                    _gDir[nid] = (ulong)(uint)cur;                    // BFS 父 = cur
                    _open.Enqueue(nid, JpsDirections.OctileHeuristic(nx, ny, gx, gy));
                }
            }

            if (bestNode == startNode)
                return;   // 起点即最近，无需追加

            // 沿 _gDir 父链收集 best→…→start（含 best、不含 start），逆序追加为 start 之后→best 的正向段
            var chain = new List<int>();
            int c = bestNode;
            while (c != startNode && chain.Count < Cap)
            {
                chain.Add(c);
                c = (int)(uint)_gDir[c];
            }
            for (int i = chain.Count - 1; i >= 0; i--)
                path.Add((chain[i] % _w, chain[i] / _w));
        }

        // ---------------- 缓冲区与代次 ----------------

        private void EnsureBuffers(GridMap map)
        {
            if (_w == map.Width && _size == map.Width * map.Height)
                return;

            _w = map.Width;
            _h = map.Height;
            _size = _w * _h;
            _gDir = new ulong[_size];
            _mark = new ushort[_size];
            _gen = 0;
            // 跳点缓存由 JpsSystem/JumpPointCache 自行按尺寸/版本管理
        }

        private void NextGeneration()
        {
            // 状态用 2·gen / 2·gen+1 编码；mark 为 ushort → 需 2·gen+1 ≤ 65535，
            // 故 gen 在 1..32767 循环，回绕时清零 mark（每 ~3.3 万次查询一次，摊薄可忽略）
            _gen++;
            if (_gen <= 32767)
                return;

            Array.Clear(_mark, 0, _mark.Length);
            _gen = 1;
        }

        private int Id(int x, int y) => y * _w + x;

        // ---------------- 方向剪枝（零分配，写入 _dirBuf，返回数量）----------------
        //
        // 剪枝 → 自然邻居（natural neighbor）+ 强迫邻居方向。对每个邻居 n，比较
        // “经过 x 的路 π=〈p,x,n〉”和“不经过 x 的路 π'”：π' 不更差就把 n 剪掉（交给别的路径走）。
        //   · 直线移动：π' ≤ π 即剪 → 只剩 1 个自然邻居（正前方）：
        //         · · ·
        //         p x o        （向右；其余 7 个邻居都能不经 x 等长到达 → 全剪）
        //         · · ·
        //   · 对角移动：π' < π 才剪（等长保留 = “对角优先”，借此消除等长对称）→ 剩 3 个：
        //         p · ·
        //         · x o        正右 o
        //         · o o        正下 o、右下 o（对角本身）
        // 旁边有障碍时，再额外把“强迫邻居”方向加入探索（强迫邻居规则见 JpsRules）。

        private int FillDirections(GridMap map, int x, int y, int parentDir)
        {
            // 作用：把“从 x 出发、剪枝后仍需要探索的方向索引”写进复用缓冲 _dirBuf，返回数量 n。
            // 主循环随后对这 n 个方向逐个做 CardinalJump / DiagonalJump。

            // 起点没有父（parentDir<0）：没有“来向”可供剪枝，必须探索全部 8 个方向。
            if (parentDir < 0)
            {
                int startCount = 0;
                for (int i = 0; i < JpsDirections.Count; i++)
                {
                    var (dx, dy) = JpsDirections.All[i];
                    bool allowed = JpsDirections.IsDiagonalIndex(i)
                        ? JpsDirections.DiagonalAllowed(map, x, y, dx, dy)
                        : map.IsWalkable(x + dx, y + dy);
                    if (allowed)
                        _dirBuf[startCount++] = i;
                }
                return startCount;
            }

            // pdx,pdy = 父→x 的移动方向（“来向”，也即 x 当前的前进方向）。
            // n 累计已写入的方向数。下面所有 _dirBuf[n++]=... 都是“保留一个待探索方向”。
            var (pdx, pdy) = JpsDirections.All[parentDir];
            int n = 0;

#if JPS_ALLOW_CORNER_CUTTING
            // ============ 允许斜穿角 ============
            if (JpsDirections.IsDiagonal(pdx, pdy))
            {
                // 对角来向 → 3 个自然邻居：继续对角 + 它的两个正交分量。
                _dirBuf[n++] = parentDir;                       // 继续沿对角 (pdx,pdy)
                _dirBuf[n++] = JpsDirections.IndexOf(pdx, 0);   // 水平分量 (pdx,0)
                _dirBuf[n++] = JpsDirections.IndexOf(0, pdy);   // 垂直分量 (0,pdy)

                // 强迫邻居：身后某个正交格被挡时，要绕到它斜对面只能经过 x → 把该斜向也加入探索。
                if (!map.IsWalkable(x - pdx, y))                       // 身后水平格 (x-pdx,y) 被挡
                    _dirBuf[n++] = JpsDirections.IndexOf(-pdx, pdy);   // → 强制探索斜向 (-pdx,pdy)
                if (!map.IsWalkable(x, y - pdy))                       // 身后垂直格 (x,y-pdy) 被挡
                    _dirBuf[n++] = JpsDirections.IndexOf(pdx, -pdy);   // → 强制探索斜向 (pdx,-pdy)

                return n;
            }

            // 直线来向 → 唯一自然邻居就是“继续直走”。
            _dirBuf[n++] = parentDir;

            if (pdx != 0)   // 水平移动：看 (x,y+1)、(x,y-1) 两侧
            {
                // 某侧紧贴的格被挡 → 它的斜前方是强迫邻居（切角斜穿过去）。
                if (!map.IsWalkable(x, y + 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, 1);
                if (!map.IsWalkable(x, y - 1)) _dirBuf[n++] = JpsDirections.IndexOf(pdx, -1);
            }
            else            // 垂直移动：看 (x+1,y)、(x-1,y) 两侧（与上面对称）
            {
                if (!map.IsWalkable(x + 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(1, pdy);
                if (!map.IsWalkable(x - 1, y)) _dirBuf[n++] = JpsDirections.IndexOf(-1, pdy);
            }

            return n;
#else
            // ============ 禁止斜穿角（默认，SoCS'12）============
            if (JpsDirections.IsDiagonal(pdx, pdy))
            {
                // 对角来向 → 只有 3 个自然邻居；并且禁止切角时“对角不产生强迫邻居”
                // （到达 x 已保证两侧正交格可走，见 JpsRules 说明）→ 不再追加任何方向。
                _dirBuf[n++] = parentDir;                       // 继续沿对角
                _dirBuf[n++] = JpsDirections.IndexOf(pdx, 0);   // 水平分量
                _dirBuf[n++] = JpsDirections.IndexOf(0, pdy);   // 垂直分量
                return n;
            }

            // 直线来向 → 唯一自然邻居就是“继续直走”。
            _dirBuf[n++] = parentDir;

            if (pdx != 0)   // 水平移动：在 (x,y+1)、(x,y-1) 两侧找“墙刚到头”的位置
            {
                // 判据 =“当前侧 (x,y+1) 可走” 且 “身后侧 (x-pdx,y+1) 被挡”：
                // 一堵沿行走方向延伸的墙到 x 这一列正好结束。墙后那片只能经过 x 进入，
                // 于是产生两个强迫邻居，两个方向都要探索：
                //   (0,+1)   从 x 竖直拐进正侧那格 (x,y+1)
                //   (pdx,+1) 从 x 斜前拐进 (x+pdx,y+1)
                if (map.IsWalkable(x, y + 1) && !map.IsWalkable(x - pdx, y + 1))
                {
                    _dirBuf[n++] = JpsDirections.IndexOf(0, 1);
                    _dirBuf[n++] = JpsDirections.IndexOf(pdx, 1);
                }
                // (x,y-1) 侧同理
                if (map.IsWalkable(x, y - 1) && !map.IsWalkable(x - pdx, y - 1))
                {
                    _dirBuf[n++] = JpsDirections.IndexOf(0, -1);
                    _dirBuf[n++] = JpsDirections.IndexOf(pdx, -1);
                }
            }
            else            // 垂直移动：把上面的“y±1 两侧”换成“x±1 两侧”，逻辑对称
            {
                if (map.IsWalkable(x + 1, y) && !map.IsWalkable(x + 1, y - pdy))
                {
                    _dirBuf[n++] = JpsDirections.IndexOf(1, 0);     // 水平拐进 (x+1,y)
                    _dirBuf[n++] = JpsDirections.IndexOf(1, pdy);   // 斜前拐进 (x+1,y+pdy)
                }
                if (map.IsWalkable(x - 1, y) && !map.IsWalkable(x - 1, y - pdy))
                {
                    _dirBuf[n++] = JpsDirections.IndexOf(-1, 0);
                    _dirBuf[n++] = JpsDirections.IndexOf(-1, pdy);
                }
            }

            return n;
#endif
        }

        // ---------------- 搜索可观测钩子（仅向 observer 发事件，自身不存储/不去重）----------------

        // 失败方向：从 (x,y) 沿 (dx,dy) 一路扫到墙，沿途的可走格都是“扫过但被抛弃”的格子。
        private static void ScanFailedRay(GridMap map, int x, int y, int dx, int dy, ISearchObserver obs)
        {
            int cx = x, cy = y;
            bool diagonal = JpsDirections.IsDiagonal(dx, dy);
            while (true)
            {
                if (diagonal)
                {
                    if (!JpsDirections.DiagonalAllowed(map, cx, cy, dx, dy))
                        return;
                }
                else if (!map.IsWalkable(cx + dx, cy + dy))
                {
                    return;
                }

                cx += dx;
                cy += dy;
                obs.OnScan(cx, cy);
            }
        }

        // 成功跳跃：从 (x1,y1) 到跳点 (x2,y2) 之间被一笔跳过的中间格。
        private static void ScanSkippedRay(int x1, int y1, int x2, int y2, ISearchObserver obs)
        {
            int dx = Math.Sign(x2 - x1);
            int dy = Math.Sign(y2 - y1);
            int x = x1, y = y1;
            while (x != x2 || y != y2)
            {
                x += dx;
                y += dy;
                obs.OnScan(x, y);
            }
        }

        // ---------------- 路径重建 ----------------

        private List<(int X, int Y)> ReconstructPath(int startId, int goalId)
        {
            var nodes = new List<int> { goalId };
            int current = goalId;
            while (current != startId)
            {
                // 父节点 = 当前 − 来向 × 跳跃步数（g_dir 一次读取同取来向与步数）
                ulong gd = _gDir[current];
                var (dx, dy) = JpsDirections.All[GdDir(gd)];
                int steps = GdSteps(gd);
                int cx = current % _w, cy = current / _w;
                current = (cy - dy * steps) * _w + (cx - dx * steps);
                nodes.Add(current);
            }
            nodes.Reverse();

            // compact path = JPS 原始跳点序列（起点、跳点/拐点、终点），不做共线合并、不展开中间格。
            var path = new List<(int X, int Y)>(nodes.Count);
            for (int i = 0; i < nodes.Count; i++)
                path.Add((nodes[i] % _w, nodes[i] / _w));
            return path;
        }
    }
}
