/*
 * GridMap.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

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
        // 行尾 padding（列 ≥ Width 的位）在构造/ClearAll 时即置 1（视为阻挡），故整字直接取反即得可走位，无需运行时屏蔽。
        private readonly ulong[] _blocked;
        private readonly int _stride;        // 每行的 ulong 数 = ceil(Width/64)
        private readonly int[] _rowVersion;
        private readonly int[] _colVersion;

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
            _rowVersion = new int[height];
            _colVersion = new int[width];
            MarkPaddingBlocked();                        // 行尾 padding 位预置为阻挡
        }

        /// <summary>每行占用的 ulong 数（行首对齐用）。</summary>
        internal int Stride => _stride;
        internal int[] RowVersion => _rowVersion;
        internal int[] ColVersion => _colVersion;

        /// <summary>
        /// 把每行行尾字中的 padding 位（列 ≥ Width）置 1（阻挡）。
        /// 构造时调用一次；ClearAll 用 Array.Clear 抹平后需再调一次恢复。
        /// 这样 <see cref="WalkableWord"/> 取反即得正确可走位，省去逐字屏蔽。
        /// </summary>
        private void MarkPaddingBlocked()
        {
            int validInLastWord = Width - (_stride - 1) * 64;   // 行尾字里的有效列数 1..64
            if (validInLastWord == 64)
                return;                                          // 整除 64，无 padding

            ulong paddingMask = ~((1UL << validInLastWord) - 1); // 行尾字里的 padding 位
            int last = _stride - 1;
            for (int y = 0; y < Height; y++)
                _blocked[y * _stride + last] |= paddingMask;
        }

        // 行对齐定位：第 y 行第 (x>>6) 个字、位 (x&63)。注意 (x >> 6) 必须加括号——
        // 移位优先级低于加法，写成 y*_stride + x >> 6 会被解析成 (y*_stride + x) >> 6（错位读到别行）。
        private bool GetBit(int x, int y) => (_blocked[y * _stride + (x >> 6)] & (1UL << (x & 63))) != 0UL;

        public bool IsBlocked(int x, int y) => InBounds(x, y) && GetBit(x, y);

        public bool IsWalkable(int x, int y) => InBounds(x, y) && !GetBit(x, y);

        public bool InBounds(int x, int y) => x >= 0 && x < Width && y >= 0 && y < Height;

        /// <summary>
        /// 取第 <paramref name="y"/> 行第 <paramref name="wordCol"/> 个字（64 格）的**可走位图**，供
        /// JumpPointCache 的水平按字扫描使用。⚠️ 三个易误解点：
        ///
        /// <para>1) **位含义与 <c>_blocked</c> 相反**：返回值里 <c>1=可走，0=阻挡</c>（是 <c>_blocked</c> 取反）。
        /// 位 i 对应列 <c>wordCol*64 + i</c>（最低位 = 最小列号）。</para>
        ///
        /// <para>2) **越界是调用方的有意约定，不是要防御的错误**：调用方会故意传入越界的 <paramref name="wordCol"/>/
        /// <paramref name="y"/>（例如 <c>y-1=-1</c>、<c>wordCol-1=-1</c>、<c>wordCol=_stride</c>，用于取上/下行与跨字进位、
        /// 以及向右扫描的终止哨兵），含义是“此处在地图外 = 墙”。故越界一律返回 0（整字全阻挡），
        /// 让边界天然成墙——这正是扫描不必单独判越界、且循环必然终止的原因。**不要把这两行当冗余检查删掉**
        /// （删掉会越界访问 + 死循环；详见调用方 HorizontalScan/ForcedRight/ForcedLeft）。</para>
        ///
        /// <para>3) **<c>(uint)</c> 转换是关键技巧**：把可能为负的下标按无符号比较，使 <c>(uint)(-1)=4294967295 ≥ 上界</c>，
        /// 于是**一次比较同时挡住“负数下越界”和“≥上界的上越界”**。这也是参数用 <c>int</c> 而非 <c>uint</c> 的原因：
        /// 让调用方能直接写 <c>y-1</c>/<c>wordCol-1</c>，把这唯一一次强转集中在这里。</para>
        ///
        /// <para>行尾 padding 列无需在此屏蔽：它们在构造/ClearAll 时已被置 1（阻挡），取反后自然为 0（不可走）。</para>
        /// </summary>
        internal ulong WalkableWord(int wordCol, int y)
        {
            if ((uint)y >= (uint)Height) return 0UL;            // 行越界（含 y<0）→ 全阻挡
            if ((uint)wordCol >= (uint)_stride) return 0UL;     // 字越界（含 wordCol<0）→ 全阻挡
            return ~_blocked[y * _stride + wordCol];            // 取反：阻挡=0，可走=1（padding 已置 1 → 0 不可走）
        }

        private void BumpLineVersions(int x, int y)
        {
            for (int yy = y - 1; yy <= y + 1; yy++)
                if ((uint)yy < (uint)Height)
                    _rowVersion[yy]++;
            for (int xx = x - 1; xx <= x + 1; xx++)
                if ((uint)xx < (uint)Width)
                    _colVersion[xx]++;
        }

        private void BumpAllLineVersions()
        {
            for (int y = 0; y < Height; y++)
                _rowVersion[y]++;
            for (int x = 0; x < Width; x++)
                _colVersion[x]++;
        }

        public void SetBlocked(int x, int y, bool blocked)
        {
            if (!InBounds(x, y))
                return;

            if (GetBit(x, y) == blocked)
                return;   // 无变化，不动版本号

            int word = y * _stride + (x >> 6);
            ulong mask = 1UL << (x & 63);
            if (blocked)
                _blocked[word] |= mask;
            else
                _blocked[word] &= ~mask;

            Version++;
            BumpLineVersions(x, y);
        }

        public void ClearAll()
        {
            Array.Clear(_blocked, 0, _blocked.Length);
            MarkPaddingBlocked();   // Array.Clear 把 padding 也抹成 0，需重新置 1
            Version++;
            BumpAllLineVersions();
        }
    }
}
