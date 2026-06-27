using System;

namespace JPS.Models
{
    /// <summary>
    /// 纯网格模型：只承载“地图本身”（尺寸、阻挡、版本号）。
    /// 不含起终点、可视化等——起终点是寻路查询参数，叠加状态由视图层持有，保持模型纯净。
    /// </summary>
    public sealed class GridMap
    {
        public int Width { get; }
        public int Height { get; }

        // 阻挡用位压缩：每个 ulong 存 64 格，1 bit/格（比 bool[] 省 8 倍，缓存更友好）。
        private readonly ulong[] _blocked;

        /// <summary>
        /// 阻挡布局的版本号：任何阻挡增删都自增。寻路器据此判断惰性跳点缓存是否失效。
        /// </summary>
        public int Version { get; private set; }

        public GridMap(int width, int height)
        {
            if (width <= 0 || height <= 0)
                throw new ArgumentOutOfRangeException(nameof(width), $"地图尺寸必须为正：{width}×{height}。");

            // 跳点缓存用 short 存距离，距离 ≤ max(宽,高)，故边长不能超过 short.MaxValue。
            if (width > short.MaxValue || height > short.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(width),
                    $"地图边长不能超过 {short.MaxValue}（当前 {width}×{height}）：跳点缓存以 short 存距离。");

            Width = width;
            Height = height;
            _blocked = new ulong[(width * height + 63) / 64];   // 向上取整到 64 位字
        }

        private bool GetBit(int idx) => (_blocked[idx >> 6] & (1UL << (idx & 63))) != 0UL;

        public bool IsBlocked(int x, int y) => InBounds(x, y) && GetBit(Index(x, y));

        public bool IsWalkable(int x, int y) => InBounds(x, y) && !GetBit(Index(x, y));

        public bool InBounds(int x, int y) => x >= 0 && x < Width && y >= 0 && y < Height;

        public void SetBlocked(int x, int y, bool blocked)
        {
            if (!InBounds(x, y))
                return;

            int idx = Index(x, y);
            if (GetBit(idx) == blocked)
                return;   // 无变化，不动版本号

            ulong mask = 1UL << (idx & 63);
            if (blocked)
                _blocked[idx >> 6] |= mask;
            else
                _blocked[idx >> 6] &= ~mask;

            Version++;   // 阻挡变化 → 惰性跳点缓存整体失效
        }

        public void ClearAll()
        {
            Array.Clear(_blocked, 0, _blocked.Length);
            Version++;
        }

        private int Index(int x, int y) => y * Width + x;
    }
}
