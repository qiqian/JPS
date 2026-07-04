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
        private long[] _g = new long[0];           // 各节点已知最短代价 g
        private sbyte[] _parentDir = new sbyte[0]; // 到达该节点的方向索引（剪枝用 + 回溯反推父节点）
        private short[] _parentSteps = new short[0]; // 到达该节点的跳跃步数；父节点 = 当前 − dir×steps（省去绝对父 id）
        private int[] _mark = new int[0];          // 访问状态：== 2·gen → open(已生成)，== 2·gen+1 → closed(已展开)
        private int _gen;
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
        {
            var map = system.Map;
            _cache = system.Cache;

            if (start.X < 0 || start.Y < 0 || goal.X < 0 || goal.Y < 0)
                return new PathResult { Message = "请先设置起点和终点。" };

            if (!map.IsWalkable(start.X, start.Y) || !map.IsWalkable(goal.X, goal.Y))
                return new PathResult { Message = "起点或终点位于阻挡上。" };

            EnsureBuffers(map);
            NextGeneration();   // 缓存同步由 JpsSystem.Sync 负责（调用方在寻路前完成）

            int openMark = _gen * 2;          // 本代“已生成/在 open”标记
            int closedMark = openMark + 1;     // 本代“已展开/closed”标记

            int gx = goal.X, gy = goal.Y;
            int startId = Id(start.X, start.Y);
            int goalId = Id(gx, gy);

            _open.Clear();
            _g[startId] = 0;
            _mark[startId] = openMark;
            _parentDir[startId] = -1;   // 起点无来向（剪枝时探索全部方向；回溯到此即停）
            _open.Enqueue(startId, JpsDirections.OctileHeuristic(start.X, start.Y, gx, gy));

            int expandedCount = 0;

            while (_open.TryDequeue(out int current, out _))
            {
                if (_mark[current] == closedMark)
                    continue;

                _mark[current] = closedMark;
                expandedCount++;

                int cx = current % _w;
                int cy = current / _w;
                obs?.OnExpand(cx, cy);

                if (current == goalId)
                    return Success(map, startId, goalId, expandedCount);

                int dirCount = FillDirections(map, cx, cy, _parentDir[current]);

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
                    long tentative = _g[current] + moveCost;

                    bool firstSeen = _mark[nbId] < openMark;
                    if (!firstSeen && tentative >= _g[nbId])
                        continue;

                    _g[nbId] = tentative;
                    _mark[nbId] = openMark;
                    _parentDir[nbId] = (sbyte)idx;
                    _parentSteps[nbId] = (short)jump.Steps;   // 步数 ≤ 边长 ≤ short.MaxValue

                    if (firstSeen) obs?.OnFrontier(jump.X, jump.Y);

                    _open.Enqueue(nbId, tentative + JpsDirections.OctileHeuristic(jump.X, jump.Y, gx, gy));
                }
            }

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

        private PathResult Success(GridMap map, int startId, int goalId, int expandedCount)
        {
            var path = ReconstructPath(startId, goalId);
            return new PathResult
            {
                Success = true,
                Path = path,
                SmoothedPath = PathSmoother.Smooth(map, path),
                ExpandedNodes = expandedCount,
                Message = $"JPS：扩展 {expandedCount}，路径 {path.Count} 点。"
            };
        }

        private static PathResult Failure(int expandedCount) =>
            new PathResult { ExpandedNodes = expandedCount, Message = $"JPS：未找到路径（扩展 {expandedCount}）。" };

        // ---------------- 缓冲区与代次 ----------------

        private void EnsureBuffers(GridMap map)
        {
            if (_w == map.Width && _size == map.Width * map.Height)
                return;

            _w = map.Width;
            _h = map.Height;
            _size = _w * _h;
            _g = new long[_size];
            _parentDir = new sbyte[_size];
            _parentSteps = new short[_size];
            _mark = new int[_size];
            _gen = 0;
            // 跳点缓存由 JpsSystem/JumpPointCache 自行按尺寸/版本管理
        }

        private void NextGeneration()
        {
            // 状态用 2·gen / 2·gen+1 编码，接近溢出时清零重来（实践几乎不触发）
            _gen++;
            if (_gen <= (int.MaxValue / 2) - 1)
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

        private int FillDirections(GridMap map, int x, int y, sbyte parentDir)
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
                // 父节点 = 当前 − 来向 × 跳跃步数
                var (dx, dy) = JpsDirections.All[_parentDir[current]];
                int steps = _parentSteps[current];
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
