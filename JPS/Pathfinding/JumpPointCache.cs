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

    /// <summary>定长 4（byte），同 <see cref="Dir4Short"/>，用于世代戳。</summary>
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
    /// </summary>
    public sealed class JumpPointCache
    {
        private int _w;
        private int _size;
        private CellJump[] _cells = new CellJump[0];
        private byte _validGen;
        private int _mapVersion = -1;
        private Func<int, int, bool> _walk = static (_, _) => false;

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

            _walk = map.IsWalkable;
        }

        /// <summary>某格某正交方向当前是否 clean（可视化用）。尺寸/版本不符则视为 dirty。</summary>
        public bool IsClean(GridMap map, int x, int y, int dir)
        {
            if (_w != map.Width || _size != map.Width * map.Height)
                return false;
            if (_mapVersion != map.Version)
                return false;
            return _cells[y * _w + x].Gen[dir] == _validGen;
        }

        /// <summary>
        /// 取 (x,y) 沿正交方向 (dx,dy) 的带符号跳点距离。
        /// 命中 clean 直接读；未命中沿射线扫一次并把整段 run 一起洗白。
        /// </summary>
        public int CardinalDist(GridMap map, int x, int y, int dx, int dy, int dir)
        {
            int idx0 = y * _w + x;
            if (_cells[idx0].Gen[dir] == _validGen)
                return _cells[idx0].Dist[dir];

            // 扫描：从 (x,y) 沿方向找最近跳点或墙
            int s = 0, rx = x, ry = y;
            bool jumpFound = false;
            while (true)
            {
                rx += dx;
                ry += dy;
                s++;
                if (!map.IsWalkable(rx, ry)) { jumpFound = false; break; }
                if (JpsRules.IsJumpPoint(_walk, rx, ry, dx, dy)) { jumpFound = true; break; }
            }

            // 回填整段 run（步 k=0..s-1 的可走格）。距离量级 ≤ max(W,H) ≤ short.MaxValue，安全转 short。
            int fx = x, fy = y;
            for (int k = 0; k <= s - 1; k++)
            {
                int ci = fy * _w + fx;
                _cells[ci].Dist.Set(dir, (short)(jumpFound ? (s - k) : -((s - 1) - k)));
                _cells[ci].Gen.Set(dir, _validGen);   // _validGen 为 byte
                fx += dx;
                fy += dy;
            }

            return _cells[idx0].Dist[dir];
        }
    }
}
