/*
 * JumpPointCache.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System;
using JPS.Models;

namespace JPS.Pathfinding
{
    /// <summary>
    /// 定长 4（short），按方向索引（0=E,1=W,2=S,3=N）。值索引器读 + Set 写，
    /// 等价内联数组但只需 C# 8/9（兼容 Unity 2022）；数组元素是可寻址左值，Set 为就地变更。
    /// </summary>
    public struct Dir4Short
    {
        private short _0;
        private short _1;
        private short _2;
        private short _3;

        public readonly short this[int index] => index switch
        {
            0 => _0,
            1 => _1,
            2 => _2,
            _ => _3,
        };

        public void Set(int index, short value)
        {
            switch (index)
            {
                case 0: _0 = value; break;
                case 1: _1 = value; break;
                case 2: _2 = value; break;
                default: _3 = value; break;
            }
        }
    }

    /// <summary>
    /// 定长 4（byte），同 <see cref="Dir4Short"/>，用于世代戳。
    /// 并发模式（JPS_CONCURRENT_CACHE）下世代戳是“发布标志”，用 <see cref="Slot"/> 取字段引用
    /// 配合 <c>Volatile.Read/Write</c> 做 acquire/release 发布。
    /// ⚠️ 本结构体及其字段必须保持**可变、非 readonly**，否则 ref 会退化为对副本取引用、丢写/读旧值。
    /// </summary>
    public struct Dir4Byte
    {
        private byte _0;
        private byte _1;
        private byte _2;
        private byte _3;

        public readonly byte this[int index] => index switch
        {
            0 => _0,
            1 => _1,
            2 => _2,
            _ => _3,
        };

        public void Set(int index, byte value)
        {
            switch (index)
            {
                case 0: _0 = value; break;
                case 1: _1 = value; break;
                case 2: _2 = value; break;
                default: _3 = value; break;
            }
        }

        /// <summary>
        /// 取第 <paramref name="index"/> 个字节的可写引用（指向调用方存储，例如数组元素内部）。
        /// 必须按 ref 接收（静态方法）——结构体实例方法不能 ref 返回自身字段（CS8170）。
        /// </summary>
        public static ref byte Slot(ref Dir4Byte g, int index) =>
            ref (index == 0 ? ref g._0
               : ref (index == 1 ? ref g._1
                    : ref (index == 2 ? ref g._2
                         : ref g._3)));
    }

    /// <summary>
    /// 单个格子的跳点缓存：4 方向带符号距离(short) + 4 方向世代戳(byte) = 12 字节/格。
    /// gen 与 dist 同结构相邻（AoS），同一格读 gen+dist 在同一缓存行，缓存更友好。
    /// 距离 ≤ max(Width,Height) ≤ short.MaxValue（由 GridMap 构造时校验）。
    /// </summary>
    public struct CellJump
    {
        public Dir4Short Dist;   // >0 跳点距离，<=0 到墙距离
        public Dir4Byte Gen;     // 世代戳，等于当前有效世代即 clean
    }

    /// <summary>
    /// 惰性正交跳点缓存（JPS 加速结构）。
    ///
    /// 每格每正交方向（E=0,W=1,S=2,N=3）缓存一个带符号距离（&gt;0 跳点、&lt;=0 到墙）+ 一个世代戳。
    /// 生命周期上从属于一张地图：按 <see cref="GridMap.Version"/> 失效；但它是 JPS 专属的加速结构，
    /// 刻意独立于纯模型 <see cref="GridMap"/> 之外，避免让模型依赖具体算法（A* 并不需要它）。
    ///
    ///  - 障碍变化（Version 改变）→ 全局世代 +1，O(1) 整体置脏。
    ///  - clean 命中 → O(1) 读。
    ///  - dirty → 沿该方向扫一次到跳点/墙，并把整段 run 一起洗白。
    ///
    /// 多线程：定义条件编译符号 <c>JPS_CONCURRENT_CACHE</c> 时，用 <c>Volatile.Write/Read</c> 对世代戳做
    /// acquire/release 发布（写 dist → Volatile.Write 发布该格 gen；读 gen 用 Volatile.Read，再普通读 dist），
    /// 使多个 JpsPathfinder 可在并行寻路中**只读/惰性补写同一份缓存**（前提：并行前单线程 Sync 一次、
    /// 并行期间地图不变；因补写值是固定地图的纯函数，重复计算结果一致）。读热路径为半屏障（acquire），
    /// 比全屏障更省；不定义该符号时退回单线程极速模式（普通读写，零屏障）。
    /// </summary>
    public sealed class JumpPointCache
    {
        private int _w;
        private int _size;
        private CellJump[] _cells = new CellJump[0];
        private byte _validGen;
        private int _mapVersion = -1;

        /// <summary>
        /// 每次搜索开始时调用：按尺寸准备缓冲，并在地图版本变化时 O(1) 整体置脏。
        /// 世代戳用 byte：到 byte.MaxValue(255) 时整体清零（清零=全 dirty）并从 1 重来——
        /// 即每 255 次障碍变化做一次 O(N) 清零，省内存、清零频率仍远低于"每次清"。
        /// </summary>
        public void Sync(GridMap map)
        {
            if (_w != map.Width || _size != map.Width * map.Height)
            {
                _w = map.Width;
                _size = map.Width * map.Height;
                _cells = new CellJump[_size];
                _validGen = 0;
                _mapVersion = -1;
            }

            if (_mapVersion != map.Version)
            {
                if (_validGen >= byte.MaxValue)
                {
                    Array.Clear(_cells, 0, _cells.Length);   // 世代回绕：整体清零→全 dirty
                    _validGen = 1;
                }
                else
                {
                    _validGen++;
                }
                _mapVersion = map.Version;
            }
        }

        /// <summary>某格某正交方向当前是否 clean（可视化用）。尺寸/版本不符则视为 dirty。</summary>
        public bool IsClean(GridMap map, int x, int y, int dir)
        {
            if (_w != map.Width || _size != map.Width * map.Height)
                return false;
            if (_mapVersion != map.Version)
                return false;
#if JPS_CONCURRENT_CACHE
            return System.Threading.Volatile.Read(ref Dir4Byte.Slot(ref _cells[y * _w + x].Gen, dir)) == _validGen;
#else
            return _cells[y * _w + x].Gen[dir] == _validGen;
#endif
        }

        /// <summary>
        /// 取 (x,y) 沿正交方向 (dx,dy) 的带符号跳点距离。
        /// 命中 clean 直接读；未命中沿射线扫一次并把整段 run 一起洗白。
        /// </summary>
        public int CardinalDist(GridMap map, int x, int y, int dx, int dy, int dir)
        {
            int idx0 = y * _w + x;
#if JPS_CONCURRENT_CACHE
            // acquire 读世代戳：若看到 clean，则发布它的那次 release 写之前的 dist 写均已可见，普通读 dist 即安全。
            if (System.Threading.Volatile.Read(ref Dir4Byte.Slot(ref _cells[idx0].Gen, dir)) == _validGen)
                return _cells[idx0].Dist[dir];
#else
            if (_cells[idx0].Gen[dir] == _validGen)
                return _cells[idx0].Dist[dir];
#endif

            // 扫描：从 (x,y) 沿方向找最近跳点或墙。
            // 水平（dy==0）走按字（ulong）批扫描：一次处理 64 格的“撞墙/强迫邻居”判定（HorizontalScan）。
            // 垂直（dx==0）仍逐格扫描——行主序下垂直相邻格不同字，无法按字批处理，也非本次优化目标。
            int s;
            bool jumpFound;
            if (dy == 0)
            {
                (s, jumpFound) = HorizontalScan(map, x, y, dx);
            }
            else
            {
                s = 0;
                int ry = y;
                jumpFound = false;
                while (true)
                {
                    ry += dy;
                    s++;
                    if (!map.IsWalkable(x, ry)) { jumpFound = false; break; }
                    if (JpsRules.IsJumpPoint(map, x, ry, dx, dy)) { jumpFound = true; break; }
                }
            }

            // 回填整段 run（步 k=0..s-1 的可走格）。距离量级 ≤ max(W,H) ≤ short.MaxValue，安全转 short。
            // 并发模式：每格先普通写 dist，再用 Volatile.Write(release) 发布该格 gen——
            // 每格 gen 只守护本格 dist，故按元素发布即可，无需额外全屏障，也无需分两遍。
            int fx = x, fy = y;
            for (int k = 0; k <= s - 1; k++)
            {
                int ci = fy * _w + fx;
                _cells[ci].Dist.Set(dir, (short)(jumpFound ? (s - k) : -((s - 1) - k)));
#if JPS_CONCURRENT_CACHE
                System.Threading.Volatile.Write(ref Dir4Byte.Slot(ref _cells[ci].Gen, dir), _validGen);
#else
                _cells[ci].Gen.Set(dir, _validGen);
#endif
                fx += dx;
                fy += dy;
            }

            return _cells[idx0].Dist[dir];   // 本线程自己刚写的值，程序序可见，无需屏障
        }

        // ---------------- 水平按字（ulong）跳点扫描 ----------------
        //
        // 从 (x,y) 沿水平方向 dx∈{+1,-1} 找最近的“停点”，语义与逐格扫描完全一致：
        //   · 撞墙（含越界/padding）→ 返回 (到墙步数, jumpFound=false)
        //   · 出现强迫邻居（该格是跳点）→ 返回 (到跳点步数, jumpFound=true)
        // 取两者中“先到”的那个（逐格循环里墙判定在前，故同列同时成立时按墙处理；本实现两掩码天然互斥）。
        //
        // 一次处理一个 64 位字：在“当前行 y / 上行 y-1 / 下行 y+1”的可走位图上做位运算，
        // 用 de Bruijn 找字内最近的置位，从而把“逐格 IsWalkable + IsJumpPoint”压成按字批处理。
        // 行按 64 对齐（见 GridMap）保证同一行的 64 格落在同一个字内、不跨行，扫描才能纯按字推进。
        // 每个字需要“当前行 y / 上行 y-1 / 下行 y+1”三行的可走位，外加强迫邻居判定中 block(c∓1) 的跨字进位
        // ——而进位来源正是“相邻字”的 y±1 两行。沿扫描方向推进时，本字的 walkUp/walkDn 恰好就是下一字的进位源，
        // 故跨迭代缓存它（prevUp/prevDn 或 nextUp/nextDn），把每字 5 次 WalkableWord 取字降到 3 次（仅取本字 y/y-1/y+1）。
        private static (int s, bool jumpFound) HorizontalScan(GridMap map, int x, int y, int dx)
        {
            if (dx > 0)
            {
                int startCol = x + 1;                  // 逐格循环先 +dx 再判，故从 x+1 起
                int w = startCol >> 6;
                ulong sub = ~0UL << (startCol & 63);   // 首字内只看 ≥ 起始位的列
                // 进位源 = 前一字（w-1）的 y±1 两行；首次取一遍，之后由上一轮的 walkUp/walkDn 滚动复用。
                ulong prevUp = map.WalkableWord(w - 1, y - 1);
                ulong prevDn = map.WalkableWord(w - 1, y + 1);
                while (true)
                {
                    ulong walkY = map.WalkableWord(w, y);
                    ulong walkUp = map.WalkableWord(w, y - 1);
                    ulong walkDn = map.WalkableWord(w, y + 1);
                    ulong jump = ForcedRight(walkY, walkUp, walkDn, prevUp, prevDn);
                    ulong m = (~walkY | jump) & sub;   // ~walkY = 行 y 阻挡位（含越界/padding）；与 jump 天然互斥
                    if (m != 0UL)
                    {
                        int b = Bits.LowestSet(m);
                        int col = (w << 6) + b;
                        bool isJump = ((jump >> b) & 1UL) != 0UL;
                        return (col - x, isJump);      // 越界字会令 ~walkY 命中，循环必然终止
                    }
                    prevUp = walkUp; prevDn = walkDn;  // 本字 y±1 行即下一字的进位源
                    w++;
                    sub = ~0UL;                        // 后续字看满 64 列
                }
            }
            else
            {
                int startCol = x - 1;
                if (startCol < 0) return (1, false);   // 左邻即越界 → 墙，步数 1
                int w = startCol >> 6;
                int sb = startCol & 63;
                ulong sub = sb == 63 ? ~0UL : ((1UL << (sb + 1)) - 1);   // 首字内只看 ≤ 起始位的列
                // 进位源 = 后一字（w+1）的 y±1 两行；首次取一遍，之后由上一轮的 walkUp/walkDn 滚动复用。
                ulong nextUp = map.WalkableWord(w + 1, y - 1);
                ulong nextDn = map.WalkableWord(w + 1, y + 1);
                while (true)
                {
                    ulong walkY = map.WalkableWord(w, y);
                    ulong walkUp = map.WalkableWord(w, y - 1);
                    ulong walkDn = map.WalkableWord(w, y + 1);
                    ulong jump = ForcedLeft(walkY, walkUp, walkDn, nextUp, nextDn);
                    ulong m = (~walkY | jump) & sub;
                    if (m != 0UL)
                    {
                        int b = Bits.HighestSet(m);
                        int col = (w << 6) + b;
                        bool isJump = ((jump >> b) & 1UL) != 0UL;
                        return (x - col, isJump);
                    }
                    nextUp = walkUp; nextDn = walkDn;  // 本字 y±1 行即下一字（更小列）的进位源
                    w--;
                    if (w < 0) return (x + 1, false);  // 越过第 0 列 → 第 -1 列是墙，步数 x+1
                    sub = ~0UL;
                }
            }
        }

        // 向右（dx=+1）时，行 y 的字内每列 c 的“强迫邻居跳点”掩码：
        //   jump(c) = walk(c,y) ∧ [ (walk(c,y-1) ∧ block(c-1,y-1)) ∨ (walk(c,y+1) ∧ block(c-1,y+1)) ]
        // block(c-1) = 该行阻挡位左移 1（c-1→c）；跨字时低位从前一字（prevUp/prevDn）的最高位补入。
        private static ulong ForcedRight(ulong walkY, ulong walkUp, ulong walkDn, ulong prevUp, ulong prevDn)
        {
            ulong blkUpShift = (~walkUp << 1) | ((~prevUp) >> 63);   // block(c-1, y-1)
            ulong blkDnShift = (~walkDn << 1) | ((~prevDn) >> 63);   // block(c-1, y+1)
            return ((walkUp & blkUpShift) | (walkDn & blkDnShift)) & walkY;
        }

        // 向左（dx=-1）时对称：邻居取 c+1 → 阻挡位右移 1（c+1→c）；跨字时高位从后一字（nextUp/nextDn）的最低位补入。
        private static ulong ForcedLeft(ulong walkY, ulong walkUp, ulong walkDn, ulong nextUp, ulong nextDn)
        {
            ulong blkUpShift = (~walkUp >> 1) | ((~nextUp & 1UL) << 63);   // block(c+1, y-1)
            ulong blkDnShift = (~walkDn >> 1) | ((~nextDn & 1UL) << 63);   // block(c+1, y+1)
            return ((walkUp & blkUpShift) | (walkDn & blkDnShift)) & walkY;
        }
    }

    /// <summary>
    /// 64 位位扫描（de Bruijn 序列实现）。手写而非用 <c>System.Numerics.BitOperations</c>，
    /// 以兼容 netstandard2.1 / Unity 2022（避免 .NET Core 专属 API）。
    /// </summary>
    internal static class Bits
    {
        private const ulong DeBruijn = 0x03f79d71b4cb0a89UL;
        private static readonly int[] Index =
        {
             0,  1, 48,  2, 57, 49, 28,  3,
            61, 58, 50, 42, 38, 29, 17,  4,
            62, 55, 59, 36, 53, 51, 43, 22,
            45, 39, 33, 30, 24, 18, 12,  5,
            63, 47, 56, 27, 60, 41, 37, 16,
            54, 35, 52, 21, 44, 32, 23, 11,
            46, 26, 40, 15, 34, 20, 31, 10,
            25, 14, 19,  9, 13,  8,  7,  6,
        };

        /// <summary>最低置位的下标（要求 x != 0）。</summary>
        public static int LowestSet(ulong x) => Index[((x & (~x + 1)) * DeBruijn) >> 58];

        /// <summary>最高置位的下标（要求 x != 0）。</summary>
        public static int HighestSet(ulong x)
        {
            x |= x >> 1; x |= x >> 2; x |= x >> 4; x |= x >> 8; x |= x >> 16; x |= x >> 32;
            x &= ~(x >> 1);   // 仅保留最高位
            return Index[(x * DeBruijn) >> 58];
        }
    }
}
