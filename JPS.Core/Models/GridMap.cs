using System;

namespace JPS.Models
{
    /// <summary>
    /// 纯网格模型：只承载“地图本身”（尺寸、阻挡、版本号）。
    /// 不含起终点、可视化等——起终点是寻路查询参数，叠加状态由视图层持有，保持模型纯净。
    ///
    /// 存储：阻挡用位压缩，1 bit/格。**按行对齐到 ulong**——每行独占 <see cref="Stride"/> 个 ulong，
    /// 行首恒落在 ulong 边界（行尾不足 64 格的部分为 padding，逻辑上视为阻挡）。
    /// 这样同一行连续 64 格落在同一个 ulong 内、不跨行，水平方向的跳点扫描可一次按字（64 格）批处理。
    /// </summary>
    public sealed class GridMap
    {
        public int Width { get; }
        public int Height { get; }

        // 阻挡位图，按行对齐：第 y 行的第 (x>>6) 个字 = _blocked[y*_stride + (x>>6)]，位 = x&63。
        // 行尾 padding（列 ≥ Width 的位）始终保持 0；读取整字时由 _lastColMask 屏蔽为“不可走”。
        private readonly ulong[] _blocked;
        private readonly int _stride;        // 每行的 ulong 数 = ceil(Width/64)
        private readonly ulong _lastColMask; // 行尾字中“有效列”的掩码（其余为 padding）

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
            _stride = (width + 63) >> 6;                 // 每行向上取整到整数个 64 位字
            _blocked = new ulong[_stride * height];

            int validInLastWord = width - (_stride - 1) * 64;   // 行尾字里的有效列数 1..64
            _lastColMask = validInLastWord == 64 ? ~0UL : ((1UL << validInLastWord) - 1);
        }

        /// <summary>每行占用的 ulong 数（行首对齐用）。</summary>
        internal int Stride => _stride;

        private int Word(int x, int y) => y * _stride + (x >> 6);
        private static ulong Bit(int x) => 1UL << (x & 63);
        private bool GetBit(int x, int y) => (_blocked[Word(x, y)] & Bit(x)) != 0UL;

        public bool IsBlocked(int x, int y) => InBounds(x, y) && GetBit(x, y);

        public bool IsWalkable(int x, int y) => InBounds(x, y) && !GetBit(x, y);

        public bool InBounds(int x, int y) => x >= 0 && x < Width && y >= 0 && y < Height;

        /// <summary>
        /// 取第 <paramref name="y"/> 行第 <paramref name="wordCol"/> 个字的“可走位”（1=可走，0=阻挡）。
        /// 越界行 / 越界字 / 行尾 padding 一律返回 0（视为阻挡），与 <see cref="IsWalkable"/> 对越界的判定一致，
        /// 供 JumpPointCache 的水平按字扫描使用（边界天然成墙，无需额外判越界）。
        /// </summary>
        internal ulong WalkableWord(int wordCol, int y)
        {
            if ((uint)y >= (uint)Height) return 0UL;
            if ((uint)wordCol >= (uint)_stride) return 0UL;
            ulong w = ~_blocked[y * _stride + wordCol];          // 取反：阻挡=0，可走=1
            if (wordCol == _stride - 1) w &= _lastColMask;       // 屏蔽行尾 padding（→ 不可走）
            return w;
        }

        public void SetBlocked(int x, int y, bool blocked)
        {
            if (!InBounds(x, y))
                return;

            int word = Word(x, y);
            ulong mask = Bit(x);
            if (((_blocked[word] & mask) != 0UL) == blocked)
                return;   // 无变化，不动版本号

            if (blocked)
                _blocked[word] |= mask;
            else
                _blocked[word] &= ~mask;

            Version++;   // 阻挡变化 → 惰性跳点缓存整体失效
        }

        public void ClearAll()
        {
            Array.Clear(_blocked, 0, _blocked.Length);   // padding 本就为 0，清零不影响其语义
            Version++;
        }
    }
}
