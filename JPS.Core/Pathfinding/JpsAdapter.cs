/*
 * JpsAdapter.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;
using System.Collections.Generic;
using JPS.Models;

namespace JPS.Pathfinding
{
    /// <summary>由 <see cref="JpsAdapter"/> 跟踪的一块动态矩形阻挡。</summary>
    public readonly struct DynamicObstacle
    {
        public int Id { get; }
        public int X { get; }
        public int Y { get; }
        public int Width { get; }
        public int Height { get; }

        public DynamicObstacle(int id, int x, int y, int width, int height)
        {
            Id = id;
            X = x;
            Y = y;
            Width = width;
            Height = height;
        }
    }

    /// <summary>
    /// 面向游戏逻辑的 JPS 适配器：封装一份 <see cref="JpsSystem"/>，在原始静态阻挡之上提供
    /// 障碍阔边以及按 id 更新的动态矩形阻挡。
    ///
    /// <para><see cref="ObstaclePadding"/> 使用 Chebyshev/矩形阔边：padding=p 时，每块阻挡向
    /// 左、右、上、下各扩张 p 格。地图外也视为阻挡，因此有效地图四周会同时保留 p 格安全边界。
    /// 传给构造函数的静态地图会被快照；之后请用 <see cref="SetStaticBlocked"/> 修改静态阻挡，
    /// 不要直接修改 <see cref="Map"/>，否则覆盖计数会失去同步。</para>
    ///
    /// <para>动态阻挡坐标采用左上角 + 半开尺寸 [x,x+w)×[y,y+h)，允许矩形部分或全部位于地图外。
    /// 同一个 id 可逐帧更新位置/尺寸；w=h=0 删除。静态阻挡、地图安全边界和多个动态 id 的阔边
    /// 可以任意重叠，删除其中一个不会错误放开仍被其他来源覆盖的格子。</para>
    ///
    /// <para>本类的阻挡更新不是并发写安全的。批量更新完一帧的动态阻挡后，先在单线程调用
    /// <see cref="Sync"/>，再让每个线程持有自己的 <see cref="JpsPathfinder"/> 并共享
    /// <see cref="System"/>。adapter 不持有搜索状态。</para>
    /// </summary>
    public sealed class JpsAdapter
    {
        private readonly bool[] _staticBlocked;
        private readonly int[] _coverage;
        private readonly Dictionary<int, DynamicObstacle> _dynamicObstacles =
            new Dictionary<int, DynamicObstacle>();

        public GridMap Map { get; }
        public JpsSystem System { get; }
        public int ObstaclePadding { get; private set; }
        public int DynamicObstacleCount => _dynamicObstacles.Count;

        /// <summary>创建一张空的静态地图并应用阔边。</summary>
        public JpsAdapter(int width, int height, int obstaclePadding = 0)
            : this(new GridMap(width, height), obstaclePadding)
        {
        }

        /// <summary>快照 <paramref name="staticMap"/>，生成供 JPS 使用的阔边后有效地图。</summary>
        public JpsAdapter(GridMap staticMap, int obstaclePadding = 0)
        {
            if (staticMap == null)
                throw new ArgumentNullException(nameof(staticMap));
            ValidatePadding(obstaclePadding);

            ObstaclePadding = obstaclePadding;
            Map = new GridMap(staticMap.Width, staticMap.Height);
            System = new JpsSystem(Map);
            _staticBlocked = new bool[staticMap.Width * staticMap.Height];
            _coverage = new int[_staticBlocked.Length];

            for (int y = 0; y < staticMap.Height; y++)
                for (int x = 0; x < staticMap.Width; x++)
                    _staticBlocked[y * staticMap.Width + x] = staticMap.IsBlocked(x, y);

            RebuildEffectiveMap(clearMap: false);
            System.Sync();
        }

        /// <summary>
        /// 改变静态与动态阻挡共用的阔边大小，并重建有效地图。返回 false 表示值未变化。
        /// </summary>
        public bool SetObstaclePadding(int obstaclePadding)
        {
            ValidatePadding(obstaclePadding);
            if (ObstaclePadding == obstaclePadding)
                return false;

            ObstaclePadding = obstaclePadding;
            RebuildEffectiveMap(clearMap: true);
            return true;
        }

        /// <summary>增删一个原始静态阻挡格；越界或值未变化时返回 false。</summary>
        public bool SetStaticBlocked(int x, int y, bool blocked)
        {
            if (!Map.InBounds(x, y))
                return false;

            int index = y * Map.Width + x;
            if (_staticBlocked[index] == blocked)
                return false;

            _staticBlocked[index] = blocked;
            ApplyExpandedRectangle(x, y, 1, 1, blocked ? 1 : -1);
            return true;
        }

        /// <summary>查询未阔边前的静态阻挡；越界返回 false。</summary>
        public bool IsStaticBlocked(int x, int y) =>
            Map.InBounds(x, y) && _staticBlocked[y * Map.Width + x];

        /// <summary>
        /// 新增或更新一块动态阻挡。同 id 更新会保留新旧阔边重叠区的覆盖，不会反复开关这些格子。
        /// 仅当 width==0 且 height==0 时表示删除；其余尺寸必须同时为正。
        /// 返回 false 表示删除了不存在的 id，或矩形完全未变化。
        /// </summary>
        public bool UpdateDynamicObstacle(int id, int x, int y, int width, int height)
        {
            bool remove = width == 0 && height == 0;
            if (!remove && width <= 0)
                throw new ArgumentOutOfRangeException(nameof(width),
                    "动态阻挡尺寸必须同时为正；仅 width=height=0 表示删除。");
            if (!remove && height <= 0)
                throw new ArgumentOutOfRangeException(nameof(height),
                    "动态阻挡尺寸必须同时为正；仅 width=height=0 表示删除。");

            DynamicObstacle oldObstacle;
            bool existed = _dynamicObstacles.TryGetValue(id, out oldObstacle);

            if (remove)
            {
                if (!existed)
                    return false;
                ApplyExpandedRectangle(oldObstacle.X, oldObstacle.Y, oldObstacle.Width, oldObstacle.Height, -1);
                _dynamicObstacles.Remove(id);
                return true;
            }

            if (existed && oldObstacle.X == x && oldObstacle.Y == y &&
                oldObstacle.Width == width && oldObstacle.Height == height)
                return false;

            var newObstacle = new DynamicObstacle(id, x, y, width, height);

            if (existed)
            {
                ReplaceExpandedRectangle(oldObstacle, newObstacle);
            }
            else
            {
                ApplyExpandedRectangle(x, y, width, height, 1);
            }
            _dynamicObstacles[id] = newObstacle;
            return true;
        }

        public bool TryGetDynamicObstacle(int id, out DynamicObstacle obstacle) =>
            _dynamicObstacles.TryGetValue(id, out obstacle);

        /// <summary>删除全部动态阻挡；没有动态阻挡时返回 false。</summary>
        public bool ClearDynamicObstacles()
        {
            if (_dynamicObstacles.Count == 0)
                return false;

            foreach (DynamicObstacle obstacle in _dynamicObstacles.Values)
                ApplyExpandedRectangle(obstacle.X, obstacle.Y, obstacle.Width, obstacle.Height, -1);
            _dynamicObstacles.Clear();
            return true;
        }

        /// <summary>把 JPS 跳点缓存同步到当前有效地图；一帧批量更新后调用一次即可。</summary>
        public void Sync() => System.Sync();

        private static void ValidatePadding(int obstaclePadding)
        {
            if (obstaclePadding < 0)
                throw new ArgumentOutOfRangeException(nameof(obstaclePadding), "阻挡阔边不能为负数。");
        }

        private void RebuildEffectiveMap(bool clearMap)
        {
            Array.Clear(_coverage, 0, _coverage.Length);
            if (clearMap)
                Map.ClearAll();

            ApplyMapBoundary();

            for (int y = 0; y < Map.Height; y++)
                for (int x = 0; x < Map.Width; x++)
                    if (_staticBlocked[y * Map.Width + x])
                        ApplyExpandedRectangle(x, y, 1, 1, 1);

            foreach (DynamicObstacle obstacle in _dynamicObstacles.Values)
                ApplyExpandedRectangle(obstacle.X, obstacle.Y, obstacle.Width, obstacle.Height, 1);
        }

        /// <summary>地图外是墙；给大体积物体的中心点留下 padding 格边界安全区。</summary>
        private void ApplyMapBoundary()
        {
            int padding = ObstaclePadding;
            if (padding == 0)
                return;

            for (int y = 0; y < Map.Height; y++)
            {
                for (int x = 0; x < Map.Width; x++)
                {
                    if (x < padding || x >= Map.Width - padding ||
                        y < padding || y >= Map.Height - padding)
                        ChangeCoverage(x, y, 1);
                }
            }
        }

        private void ApplyExpandedRectangle(int x, int y, int width, int height, int delta)
        {
            CoverageRectangle rectangle = GetExpandedRectangle(x, y, width, height);
            ApplyCoverageRectangle(rectangle, delta);
        }

        /// <summary>
        /// 只更新新旧 footprint 的差集。动态矩形逐帧小步移动时，复杂度与进入/离开的边条面积相关，
        /// 而不是每帧重新遍历整个矩形；重叠部分仍保持原有那一份 coverage。
        /// </summary>
        private void ReplaceExpandedRectangle(DynamicObstacle oldObstacle, DynamicObstacle newObstacle)
        {
            CoverageRectangle oldRectangle = GetExpandedRectangle(
                oldObstacle.X, oldObstacle.Y, oldObstacle.Width, oldObstacle.Height);
            CoverageRectangle newRectangle = GetExpandedRectangle(
                newObstacle.X, newObstacle.Y, newObstacle.Width, newObstacle.Height);

            ApplyRectangleDifference(newRectangle, oldRectangle, 1);
            ApplyRectangleDifference(oldRectangle, newRectangle, -1);
        }

        private CoverageRectangle GetExpandedRectangle(int x, int y, int width, int height)
        {
            long padding = ObstaclePadding;
            long left = (long)x - padding;
            long top = (long)y - padding;
            long right = (long)x + width - 1L + padding;
            long bottom = (long)y + height - 1L + padding;

            int x0 = (int)Math.Max(0L, left);
            int y0 = (int)Math.Max(0L, top);
            int x1 = (int)Math.Min(Map.Width - 1L, right);
            int y1 = (int)Math.Min(Map.Height - 1L, bottom);
            if (x0 > x1 || y0 > y1)
                return CoverageRectangle.Empty;

            return new CoverageRectangle(x0, y0, x1, y1);
        }

        private void ApplyRectangleDifference(CoverageRectangle source, CoverageRectangle subtract, int delta)
        {
            if (source.IsEmpty)
                return;

            CoverageRectangle intersection = CoverageRectangle.Intersect(source, subtract);
            if (intersection.IsEmpty)
            {
                ApplyCoverageRectangle(source, delta);
                return;
            }

            // source - intersection，拆成互不重叠的上、下、左、右四条。
            ApplyCoverageRectangle(new CoverageRectangle(
                source.Left, source.Top, source.Right, intersection.Top - 1), delta);
            ApplyCoverageRectangle(new CoverageRectangle(
                source.Left, intersection.Bottom + 1, source.Right, source.Bottom), delta);
            ApplyCoverageRectangle(new CoverageRectangle(
                source.Left, intersection.Top, intersection.Left - 1, intersection.Bottom), delta);
            ApplyCoverageRectangle(new CoverageRectangle(
                intersection.Right + 1, intersection.Top, source.Right, intersection.Bottom), delta);
        }

        private void ApplyCoverageRectangle(CoverageRectangle rectangle, int delta)
        {
            if (rectangle.IsEmpty)
                return;

            for (int yy = rectangle.Top; yy <= rectangle.Bottom; yy++)
                for (int xx = rectangle.Left; xx <= rectangle.Right; xx++)
                    ChangeCoverage(xx, yy, delta);
        }

        private void ChangeCoverage(int x, int y, int delta)
        {
            int index = y * Map.Width + x;
            int before = _coverage[index];
            int after = before + delta;
            if (after < 0)
                throw new InvalidOperationException("JpsAdapter 阻挡覆盖计数失衡。");

            _coverage[index] = after;
            if (before == 0 && after != 0)
                Map.SetBlocked(x, y, true);
            else if (before != 0 && after == 0)
                Map.SetBlocked(x, y, false);
        }

        private readonly struct CoverageRectangle
        {
            public static CoverageRectangle Empty => new CoverageRectangle(0, 0, -1, -1);

            public readonly int Left;
            public readonly int Top;
            public readonly int Right;
            public readonly int Bottom;
            public bool IsEmpty => Left > Right || Top > Bottom;

            public CoverageRectangle(int left, int top, int right, int bottom)
            {
                Left = left;
                Top = top;
                Right = right;
                Bottom = bottom;
            }

            public static CoverageRectangle Intersect(CoverageRectangle a, CoverageRectangle b)
            {
                if (a.IsEmpty || b.IsEmpty)
                    return Empty;
                return new CoverageRectangle(
                    Math.Max(a.Left, b.Left),
                    Math.Max(a.Top, b.Top),
                    Math.Min(a.Right, b.Right),
                    Math.Min(a.Bottom, b.Bottom));
            }
        }
    }
}
