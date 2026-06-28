/*
 * AStarPathfinder.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;
using System.Collections.Generic;
using JPS.Models;

namespace JPS.Pathfinding
{
    /// <summary>
    /// 标准 8 邻接 A* 寻路，作为 JPS 的对照组。
    ///
    /// 与 JPS 的区别：A* 每展开一个节点都要实时检查 8 个邻居并逐个入队，不做跳跃，
    /// 扩展节点远多于 JPS；不需要预计算。移动规则/启发式与本项目 JPS 一致以便公平对比。
    ///
    /// 内存：逐节点数据用 O(N) 扁平数组（一次分配、跨查询复用）。为压缩单次寻路的内存占用，
    /// 做了两处不损性能的优化：
    ///   ① seen/closed 两个状态合并进一个 int 数组（用 2·gen / 2·gen+1 编码）；
    ///   ② 父信息只存“来向” sbyte（A* 每步一格，回溯时反推父格即可），而非完整父 id。
    /// 于是每格仅 g(8) + 来向(1) + 状态(4) = 13 字节（原先 20 字节）。
    /// </summary>
    public sealed class AStarPathfinder
    {
        /// <summary>每个方向的移动代价（正交 1000 / 对角 1414），按方向索引预存。</summary>
        private static readonly long[] DirCost = BuildDirCost();

        private static long[] BuildDirCost()
        {
            var costs = new long[JpsDirections.Count];
            for (int i = 0; i < JpsDirections.Count; i++)
            {
                var (dx, dy) = JpsDirections.All[i];
                costs[i] = JpsDirections.MoveCost(dx, dy);
            }
            return costs;
        }

        // ---- 按地图尺寸一次性分配、跨多次查询复用的缓冲区 ----
        private int _w, _size;
        private long[] _g = new long[0];           // 各节点已知最短代价 g
        private sbyte[] _cameDir = new sbyte[0];   // 到达该节点的方向索引（0..7，-1=起点），用于回溯
        private int[] _mark = new int[0];          // 访问状态：== 2·gen → open(已生成)，== 2·gen+1 → closed(已展开)
        private int _gen;
        private readonly MinHeap _open = new MinHeap();

        public PathResult FindPath(GridMap map, (int X, int Y) start, (int X, int Y) goal, ISearchObserver? obs = null)
        {
            if (start.X < 0 || start.Y < 0 || goal.X < 0 || goal.Y < 0)
                return new PathResult { Message = "请先设置起点和终点。" };

            if (!map.IsWalkable(start.X, start.Y) || !map.IsWalkable(goal.X, goal.Y))
                return new PathResult { Message = "起点或终点位于阻挡上。" };

            EnsureBuffers(map);
            NextGeneration();

            int openMark = _gen * 2;          // 本代“已生成/在 open”标记
            int closedMark = openMark + 1;     // 本代“已展开/closed”标记

            int gx = goal.X, gy = goal.Y;
            int startId = Id(start.X, start.Y);
            int goalId = Id(gx, gy);

            _open.Clear();
            _g[startId] = 0;
            _mark[startId] = openMark;
            _cameDir[startId] = -1;
            _open.Enqueue(startId, JpsDirections.OctileHeuristic(start.X, start.Y, gx, gy));

            int expandedCount = 0;

            while (_open.TryDequeue(out int current, out _))
            {
                if (_mark[current] == closedMark)
                    continue;   // 惰性删除：已展开的重复入队项跳过

                _mark[current] = closedMark;
                expandedCount++;

                int cx = current % _w;
                int cy = current / _w;
                obs?.OnExpand(cx, cy);

                if (current == goalId)
                    return Success(startId, goalId, expandedCount);

                long gCur = _g[current];

                for (int i = 0; i < JpsDirections.Count; i++)
                {
                    var (dx, dy) = JpsDirections.All[i];
                    int nx = cx + dx;
                    int ny = cy + dy;

                    if (!map.IsWalkable(nx, ny))
                        continue;
                    if (JpsDirections.IsDiagonal(dx, dy) && !JpsDirections.DiagonalAllowed(map, cx, cy, dx, dy))
                        continue;   // 禁止斜穿角（默认；定义 JPS_ALLOW_CORNER_CUTTING 可放开）

                    int nbId = ny * _w + nx;
                    if (_mark[nbId] == closedMark)
                        continue;

                    long tentative = gCur + DirCost[i];
                    bool firstSeen = _mark[nbId] < openMark;
                    if (!firstSeen && tentative >= _g[nbId])
                        continue;

                    _g[nbId] = tentative;
                    _mark[nbId] = openMark;
                    _cameDir[nbId] = (sbyte)i;

                    if (firstSeen) obs?.OnFrontier(nx, ny);

                    _open.Enqueue(nbId, tentative + JpsDirections.OctileHeuristic(nx, ny, gx, gy));
                }
            }

            return Failure(expandedCount);
        }

        private PathResult Success(int startId, int goalId, int expandedCount)
        {
            var path = ReconstructPath(startId, goalId);
            return new PathResult
            {
                Success = true,
                Path = path,
                ExpandedNodes = expandedCount,
                Message = $"A*：扩展 {expandedCount}，路径 {path.Count} 格。"
            };
        }

        private static PathResult Failure(int expandedCount) =>
            new PathResult { ExpandedNodes = expandedCount, Message = $"A*：未找到路径（扩展 {expandedCount}）。" };

        private void EnsureBuffers(GridMap map)
        {
            if (_w == map.Width && _size == map.Width * map.Height)
                return;

            _w = map.Width;
            _size = map.Width * map.Height;
            _g = new long[_size];
            _cameDir = new sbyte[_size];
            _mark = new int[_size];
            _gen = 0;
        }

        /// <summary>
        /// 代次自增。状态用 2·gen / 2·gen+1 编码，故每次自增 1 但有效区间是 2 个值；
        /// 接近溢出时清零重来（实践中几乎不会触发）。
        /// </summary>
        private void NextGeneration()
        {
            _gen++;
            if (_gen <= (int.MaxValue / 2) - 1)
                return;

            Array.Clear(_mark, 0, _mark.Length);
            _gen = 1;
        }

        private int Id(int x, int y) => y * _w + x;

        /// <summary>沿“来向”从终点反推父格回溯到起点（A* 每步一格，无需补段）。</summary>
        private List<(int X, int Y)> ReconstructPath(int startId, int goalId)
        {
            var ids = new List<int> { goalId };
            int current = goalId;
            while (current != startId)
            {
                int dir = _cameDir[current];
                var (dx, dy) = JpsDirections.All[dir];
                int cx = current % _w;
                int cy = current / _w;
                current = (cy - dy) * _w + (cx - dx);   // 父格 = 当前格 - 来向位移
                ids.Add(current);
            }
            ids.Reverse();

            var path = new List<(int X, int Y)>(ids.Count);
            foreach (int id in ids)
                path.Add((id % _w, id / _w));
            return path;
        }
    }
}
