/*
 * JpsAdapter.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;
using System.Runtime.InteropServices;
using JPS.Models;

namespace JPS.Pathfinding
{
    /// <summary>由 <see cref="JpsAdapter"/> 跟踪的一块动态矩形阻挡。</summary>
    [StructLayout(LayoutKind.Sequential)]
    public readonly struct DynamicObstacle
    {
        private readonly int _id;
        private readonly uint _xy;
        private readonly uint _size;

        public int Id => _id;
        public int X => (short)(_xy & ushort.MaxValue);
        public int Y => (short)(_xy >> 16);
        public int Width => (ushort)(_size & ushort.MaxValue);
        public int Height => (ushort)(_size >> 16);

        public DynamicObstacle(int id, int x, int y, int width, int height)
        {
            if (x < short.MinValue || x > short.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(x));
            if (y < short.MinValue || y > short.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(y));
            if (width <= 0 || width > ushort.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(width));
            if (height <= 0 || height > ushort.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(height));

            _id = id;
            _xy = (uint)(ushort)(short)x | ((uint)(ushort)(short)y << 16);
            _size = (uint)(ushort)width | ((uint)(ushort)height << 16);
        }

        internal bool SameRectangle(DynamicObstacle other) =>
            _xy == other._xy && _size == other._size;
    }

    /// <summary>
    /// 面向游戏逻辑的 JPS 适配器。静态地图在构造时快照并保持不可变；运行时变化全部使用
    /// 按 id 跟踪的动态矩形。adapter 持有共享 <see cref="JpsSystem"/>，但不持有 pathfinder。
    /// </summary>
    /// <remarks>
    /// 静态原图和静态阔边分别使用 1 bit/格。动态覆盖只保存 refcount&gt;0 的格子，按线性格索引
    /// 排序并使用 ushort 引用计数；更新动态矩形时把旧位置 -1、新位置 +1 与现有覆盖数组线性归并，
    /// 所以新旧 footprint 的交集不会产生临时的 1→0→1 地图抖动。动态障碍使用紧凑数组保存，
    /// 线性查找 id，并以末项覆盖删除位置。不要直接修改 <see cref="Map"/>；
    /// 一帧更新完成后先调用 <see cref="Sync"/>，再用独立的 <see cref="JpsPathfinder"/> 查询。
    /// </remarks>
    public sealed class JpsAdapter
    {
        private struct DynamicCoverage
        {
            public ushort X;
            public ushort Y;
            public ushort RefCount;

            public DynamicCoverage(int index, int mapWidth, ushort refCount)
            {
                X = (ushort)(index % mapWidth);
                Y = (ushort)(index / mapWidth);
                RefCount = refCount;
            }
        }

        private readonly ulong[] _staticBlockedBits;
        private readonly ulong[] _expandedStaticBits;
        private DynamicObstacle[] _dynamicObstacles = Array.Empty<DynamicObstacle>();
        private int _dynamicObstacleCount;
        private DynamicCoverage[] _dynamicCoverage = Array.Empty<DynamicCoverage>();
        private DynamicCoverage[] _coverageScratch = Array.Empty<DynamicCoverage>();
        private int _dynamicCoverageCount;

        public GridMap Map { get; }
        public JpsSystem System { get; }
        public int ObstaclePadding { get; private set; }
        public int DynamicObstacleCount => _dynamicObstacleCount;
        public int DynamicCoveredCellCount => _dynamicCoverageCount;

        /// <summary>创建一张不可变的空静态地图。</summary>
        public JpsAdapter(int width, int height, int obstaclePadding = 0)
            : this(new GridMap(width, height), obstaclePadding)
        {
        }

        /// <summary>快照静态地图；之后如需改变静态阻挡，请创建新的 adapter。</summary>
        public JpsAdapter(GridMap staticMap, int obstaclePadding = 0)
        {
            if (staticMap == null)
                throw new ArgumentNullException(nameof(staticMap));
            ValidatePadding(obstaclePadding);

            ObstaclePadding = obstaclePadding;
            Map = new GridMap(staticMap.Width, staticMap.Height);
            System = new JpsSystem(Map);
            int wordCount = (staticMap.Width * staticMap.Height + 63) >> 6;
            _staticBlockedBits = new ulong[wordCount];
            _expandedStaticBits = new ulong[wordCount];

            for (int y = 0; y < staticMap.Height; y++)
                for (int x = 0; x < staticMap.Width; x++)
                    if (staticMap.IsBlocked(x, y))
                        SetBit(_staticBlockedBits, y * staticMap.Width + x);

            RebuildEffectiveMap();
            System.Sync();
        }

        /// <summary>改变静态和动态阻挡共用的阔边大小并重建有效地图。</summary>
        public bool SetObstaclePadding(int obstaclePadding)
        {
            ValidatePadding(obstaclePadding);
            if (ObstaclePadding == obstaclePadding)
                return false;

            int previousPadding = ObstaclePadding;
            ObstaclePadding = obstaclePadding;
            try
            {
                RebuildEffectiveMap();
            }
            catch
            {
                ObstaclePadding = previousPadding;
                RebuildEffectiveMap();
                throw;
            }
            return true;
        }

        /// <summary>查询构造时快照的、未经阔边的静态阻挡。</summary>
        public bool IsStaticBlocked(int x, int y) =>
            Map.InBounds(x, y) && GetBit(_staticBlockedBits, y * Map.Width + x);

        /// <summary>查询静态、阔边和动态矩形合成后的有效阻挡；地图外视为阻挡。</summary>
        public bool IsBlocked(int x, int y) =>
            !Map.InBounds(x, y) || Map.IsBlocked(x, y);

        /// <summary>
        /// 新增或更新动态矩形。同 id 可跨帧移动；仅 width=height=0 表示删除。
        /// 矩形以 int16 x/y + uint16 width/height 压缩存储。
        /// 动态障碍总数不能超过 ushort.MaxValue，以保证每格引用计数不会溢出。
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

            if (!remove && (x < short.MinValue || x > short.MaxValue))
                throw new ArgumentOutOfRangeException(nameof(x),
                    "动态阻挡 x 必须在 Int16 范围内。");
            if (!remove && (y < short.MinValue || y > short.MaxValue))
                throw new ArgumentOutOfRangeException(nameof(y),
                    "动态阻挡 y 必须在 Int16 范围内。");
            if (!remove && width > ushort.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(width),
                    "动态阻挡 width 必须在 UInt16 范围内。");
            if (!remove && height > ushort.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(height),
                    "动态阻挡 height 必须在 UInt16 范围内。");

            int obstacleIndex = FindDynamicObstacle(id);
            if (remove)
            {
                if (obstacleIndex < 0)
                    return false;

                DynamicObstacle oldObstacle = _dynamicObstacles[obstacleIndex];
                ApplyDynamicChange(oldObstacle, null);
                int lastIndex = --_dynamicObstacleCount;
                if (obstacleIndex != lastIndex)
                    _dynamicObstacles[obstacleIndex] = _dynamicObstacles[lastIndex];
                _dynamicObstacles[lastIndex] = default;
                return true;
            }

            var newObstacle = new DynamicObstacle(id, x, y, width, height);
            if (obstacleIndex >= 0)
            {
                DynamicObstacle oldObstacle = _dynamicObstacles[obstacleIndex];
                if (oldObstacle.SameRectangle(newObstacle))
                    return false;

                ApplyDynamicChange(oldObstacle, newObstacle);
                _dynamicObstacles[obstacleIndex] = newObstacle;
                return true;
            }

            if (_dynamicObstacleCount >= ushort.MaxValue)
                throw new InvalidOperationException("动态障碍数量不能超过 65535。");
            EnsureObstacleCapacity(_dynamicObstacleCount + 1);
            ApplyDynamicChange(null, newObstacle);
            _dynamicObstacles[_dynamicObstacleCount++] = newObstacle;
            return true;
        }

        public bool TryGetDynamicObstacle(int id, out DynamicObstacle obstacle)
        {
            int index = FindDynamicObstacle(id);
            if (index >= 0)
            {
                obstacle = _dynamicObstacles[index];
                return true;
            }

            obstacle = default;
            return false;
        }

        public bool ClearDynamicObstacles()
        {
            if (_dynamicObstacleCount == 0)
                return false;

            for (int i = 0; i < _dynamicCoverageCount; i++)
            {
                int index = CoverageIndex(_dynamicCoverage[i]);
                if (!GetBit(_expandedStaticBits, index))
                    Map.SetBlocked(index % Map.Width, index / Map.Width, false);
            }

            _dynamicCoverageCount = 0;
            Array.Clear(_dynamicObstacles, 0, _dynamicObstacleCount);
            _dynamicObstacleCount = 0;
            return true;
        }

        public void Sync() => System.Sync();

        private int FindDynamicObstacle(int id)
        {
            for (int i = 0; i < _dynamicObstacleCount; i++)
                if (_dynamicObstacles[i].Id == id)
                    return i;
            return -1;
        }

        private void EnsureObstacleCapacity(int required)
        {
            if (_dynamicObstacles.Length >= required)
                return;

            int capacity = _dynamicObstacles.Length == 0 ? 8 : _dynamicObstacles.Length;
            while (capacity < required)
            {
                int next = capacity <= ushort.MaxValue / 2
                    ? capacity * 2
                    : ushort.MaxValue;
                if (next == capacity)
                    throw new OutOfMemoryException();
                capacity = next;
            }
            Array.Resize(ref _dynamicObstacles, capacity);
        }

        private static void ValidatePadding(int obstaclePadding)
        {
            if (obstaclePadding < 0)
                throw new ArgumentOutOfRangeException(nameof(obstaclePadding), "阻挡阔边不能为负数。");
        }

        private void RebuildEffectiveMap()
        {
            Array.Clear(_expandedStaticBits, 0, _expandedStaticBits.Length);
            Map.ClearAll();
            ApplyMapBoundary();

            int cellCount = Map.Width * Map.Height;
            for (int index = 0; index < cellCount; index++)
            {
                if (GetBit(_staticBlockedBits, index))
                    ApplyExpandedStatic(index % Map.Width, index / Map.Width);
            }

            _dynamicCoverageCount = 0;
            for (int i = 0; i < _dynamicObstacleCount; i++)
                ApplyDynamicChange(null, _dynamicObstacles[i]);
        }

        private void ApplyMapBoundary()
        {
            int p = ObstaclePadding;
            if (p == 0)
                return;

            for (int y = 0; y < Map.Height; y++)
            {
                for (int x = 0; x < Map.Width; x++)
                {
                    if (x < p || x >= Map.Width - p || y < p || y >= Map.Height - p)
                        SetExpandedStatic(x, y);
                }
            }
        }

        private void ApplyExpandedStatic(int x, int y)
        {
            CoverageRectangle rectangle = GetExpandedRectangle(x, y, 1, 1);
            for (int yy = rectangle.Top; yy <= rectangle.Bottom; yy++)
                for (int xx = rectangle.Left; xx <= rectangle.Right; xx++)
                    SetExpandedStatic(xx, yy);
        }

        private void SetExpandedStatic(int x, int y)
        {
            int index = y * Map.Width + x;
            if (GetBit(_expandedStaticBits, index))
                return;
            SetBit(_expandedStaticBits, index);
            Map.SetBlocked(x, y, true);
        }

        private void ApplyDynamicChange(DynamicObstacle? oldObstacle, DynamicObstacle? newObstacle)
        {
            CoverageRectangle oldRectangle = oldObstacle.HasValue
                ? GetExpandedRectangle(oldObstacle.Value.X, oldObstacle.Value.Y,
                    oldObstacle.Value.Width, oldObstacle.Value.Height)
                : CoverageRectangle.Empty;
            CoverageRectangle newRectangle = newObstacle.HasValue
                ? GetExpandedRectangle(newObstacle.Value.X, newObstacle.Value.Y,
                    newObstacle.Value.Width, newObstacle.Value.Height)
                : CoverageRectangle.Empty;

            long maximumOutput = (long)_dynamicCoverageCount + newRectangle.Area;
            if (maximumOutput > int.MaxValue)
                throw new InvalidOperationException("动态覆盖格数量过大。");
            EnsureScratchCapacity((int)maximumOutput);

            var oldCells = new RectangleCellCursor(oldRectangle, Map.Width);
            var newCells = new RectangleCellCursor(newRectangle, Map.Width);
            bool hasOld = oldCells.MoveNext();
            bool hasNew = newCells.MoveNext();
            int coveragePosition = 0;
            int outputCount = 0;

            while (coveragePosition < _dynamicCoverageCount || hasOld || hasNew)
            {
                int key = coveragePosition < _dynamicCoverageCount
                    ? CoverageIndex(_dynamicCoverage[coveragePosition])
                    : int.MaxValue;
                if (hasOld && oldCells.Current < key) key = oldCells.Current;
                if (hasNew && newCells.Current < key) key = newCells.Current;

                int before = 0;
                if (coveragePosition < _dynamicCoverageCount &&
                    CoverageIndex(_dynamicCoverage[coveragePosition]) == key)
                {
                    before = _dynamicCoverage[coveragePosition].RefCount;
                    coveragePosition++;
                }

                int delta = 0;
                if (hasOld && oldCells.Current == key)
                {
                    delta--;
                    hasOld = oldCells.MoveNext();
                }
                if (hasNew && newCells.Current == key)
                {
                    delta++;
                    hasNew = newCells.MoveNext();
                }

                int after = before + delta;
                if (after < 0 || after > ushort.MaxValue)
                    throw new InvalidOperationException("动态阻挡引用计数失衡或溢出。");

                if (before == 0 && after != 0)
                    Map.SetBlocked(key % Map.Width, key / Map.Width, true);
                else if (before != 0 && after == 0 && !GetBit(_expandedStaticBits, key))
                    Map.SetBlocked(key % Map.Width, key / Map.Width, false);

                if (after != 0)
                    _coverageScratch[outputCount++] = new DynamicCoverage(key, Map.Width, (ushort)after);
            }

            DynamicCoverage[] previous = _dynamicCoverage;
            _dynamicCoverage = _coverageScratch;
            _coverageScratch = previous;
            _dynamicCoverageCount = outputCount;
        }

        private void EnsureScratchCapacity(int required)
        {
            if (_coverageScratch.Length >= required)
                return;

            int capacity = _coverageScratch.Length == 0 ? 16 : _coverageScratch.Length;
            while (capacity < required)
            {
                int next = capacity <= int.MaxValue / 2 ? capacity * 2 : int.MaxValue;
                if (next == capacity)
                    throw new OutOfMemoryException();
                capacity = next;
            }
            _coverageScratch = new DynamicCoverage[capacity];
        }

        private int CoverageIndex(DynamicCoverage coverage) => coverage.Y * Map.Width + coverage.X;

        private CoverageRectangle GetExpandedRectangle(int x, int y, int width, int height)
        {
            long p = ObstaclePadding;
            int left = (int)Math.Max(0L, (long)x - p);
            int top = (int)Math.Max(0L, (long)y - p);
            int right = (int)Math.Min(Map.Width - 1L, (long)x + width - 1L + p);
            int bottom = (int)Math.Min(Map.Height - 1L, (long)y + height - 1L + p);
            return left > right || top > bottom
                ? CoverageRectangle.Empty
                : new CoverageRectangle(left, top, right, bottom);
        }

        private static bool GetBit(ulong[] bits, int index) =>
            (bits[index >> 6] & (1UL << (index & 63))) != 0;

        private static void SetBit(ulong[] bits, int index) =>
            bits[index >> 6] |= 1UL << (index & 63);

        private readonly struct CoverageRectangle
        {
            public static CoverageRectangle Empty => new CoverageRectangle(0, 0, -1, -1);
            public readonly int Left;
            public readonly int Top;
            public readonly int Right;
            public readonly int Bottom;
            public bool IsEmpty => Left > Right || Top > Bottom;
            public long Area => IsEmpty ? 0L : (long)(Right - Left + 1) * (Bottom - Top + 1);

            public CoverageRectangle(int left, int top, int right, int bottom)
            {
                Left = left;
                Top = top;
                Right = right;
                Bottom = bottom;
            }
        }

        private struct RectangleCellCursor
        {
            private readonly CoverageRectangle _rectangle;
            private readonly int _mapWidth;
            private int _x;
            private int _y;
            private bool _started;
            public int Current { get; private set; }

            public RectangleCellCursor(CoverageRectangle rectangle, int mapWidth)
            {
                _rectangle = rectangle;
                _mapWidth = mapWidth;
                _x = rectangle.Left;
                _y = rectangle.Top;
                _started = false;
                Current = -1;
            }

            public bool MoveNext()
            {
                if (_rectangle.IsEmpty)
                    return false;

                if (!_started)
                {
                    _started = true;
                }
                else if (_x < _rectangle.Right)
                {
                    _x++;
                }
                else
                {
                    _x = _rectangle.Left;
                    _y++;
                    if (_y > _rectangle.Bottom)
                        return false;
                }

                Current = _y * _mapWidth + _x;
                return true;
            }
        }
    }
}
