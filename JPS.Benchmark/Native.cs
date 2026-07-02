/*
 * Native.cs
 * JPS Pathfinding — JPS.Native（C DLL）的 P/Invoke 封装，供基准测试把 C 版 JPS
 * 作为与 C# 版 JPS / A* 并列的第三个被测对象（性能对比 + 结果强一致抽检）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System.Runtime.InteropServices;
using JPS.Models;

namespace JPS.Benchmark
{
    /// <summary>
    /// JPS.Native.dll 的扁平 C ABI 绑定（与 jps.h 一一对应）。
    /// 静态构造时注册 DllImportResolver，从仓库的 x64/{Release,Debug} 等位置定位 DLL。
    /// </summary>
    internal static class NativeJps
    {
        private const string Dll = "JPS.Native";

        public static string? ResolvedPath { get; private set; }

        static NativeJps()
        {
            NativeLibrary.SetDllImportResolver(typeof(NativeJps).Assembly, (name, asm, search) =>
            {
                if (name != Dll) return IntPtr.Zero;
                foreach (var cand in Candidates())
                    if (File.Exists(cand) && NativeLibrary.TryLoad(cand, out var h))
                    {
                        ResolvedPath = cand;
                        return h;
                    }
                return IntPtr.Zero;
            });
        }

        private static IEnumerable<string> Candidates()
        {
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 12 && dir != null; i++)
            {
                yield return Path.Combine(dir, Dll + ".dll");
                yield return Path.Combine(dir, "x64", "Release", Dll + ".dll");
                yield return Path.Combine(dir, "x64", "Debug", Dll + ".dll");
                dir = Directory.GetParent(dir)?.FullName;
            }
        }

        /// <summary>探测原生库是否可用（顺带触发加载并记录路径）。失败返回 false，不抛异常。</summary>
        public static bool TryInit(out string info)
        {
            try
            {
                IntPtr s = jps_system_create(1, 1);
                if (s == IntPtr.Zero) { info = "jps_system_create 返回 NULL"; return false; }
                jps_system_destroy(s);
                info = ResolvedPath ?? "(默认搜索路径)";
                return true;
            }
            catch (Exception ex) { info = ex.Message; return false; }
        }

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr jps_system_create(int width, int height);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_destroy(IntPtr s);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_set_blocked(IntPtr s, int x, int y, int blocked);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_set_blocked_buffer(IntPtr s, byte[] cells, int count);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_set_blocked_batch(IntPtr s, int[] xyv, int editCount);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_sync(IntPtr s);

        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr jps_pathfinder_create();
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_pathfinder_destroy(IntPtr pf);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jps_pathfinder_find_path(IntPtr pf, IntPtr system, int sx, int sy, int gx, int gy);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jps_pathfinder_expanded_nodes(IntPtr pf);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jps_pathfinder_copy_path(IntPtr pf, int[] outXy, int capacityPoints);
    }

    /// <summary>
    /// 单张地图的原生 JPS 运行器：从 <see cref="GridMap"/> 构建一份原生 jps_system（阻挡灌入 + 同步）
    /// 与一个 jps_pathfinder，之后可重复寻路（缓存热复用，语义与 C# 版 JpsSystem.Sync 后一致）。
    /// 基准为单线程逐图测，故一张图一个运行器即可；用完 Dispose 释放原生资源。
    /// </summary>
    internal sealed class NativeMap : IDisposable
    {
        private IntPtr _sys;
        private IntPtr _pf;
        private int[] _editBuf = Array.Empty<int>();   // 批量增量的 (x,y,blocked) 三元组暂存，跨调用复用

        public NativeMap(GridMap map)
        {
            _sys = NativeJps.jps_system_create(map.Width, map.Height);
            if (_sys == IntPtr.Zero) throw new InvalidOperationException("jps_system_create returned NULL.");
            try
            {
            var cells = new byte[map.Width * map.Height];   // 行主序，与 set_blocked_buffer 约定一致
            int i = 0;
            for (int y = 0; y < map.Height; y++)
                for (int x = 0; x < map.Width; x++)
                    cells[i++] = map.IsBlocked(x, y) ? (byte)1 : (byte)0;
            NativeJps.jps_system_set_blocked_buffer(_sys, cells, cells.Length);
            NativeJps.jps_system_sync(_sys);                // 灌入阻挡后同步缓存（之后地图只读）
            _pf = NativeJps.jps_pathfinder_create();
            if (_pf == IntPtr.Zero) throw new InvalidOperationException("jps_pathfinder_create returned NULL.");
            }
            catch
            {
                Dispose();
                throw;
            }
        }

        /// <summary>寻路；返回 find_path 的原始返回值（&gt;=0 路径格数，&lt;0 错误）。仅供计时调用。</summary>
        public int Find(int sx, int sy, int gx, int gy) =>
            NativeJps.jps_pathfinder_find_path(_pf, _sys, sx, sy, gx, gy);

        /// <summary>
        /// 翻转一个可走格再翻回（Version 跳变两次）并重新 Sync，使整表 dirty——
        /// 与 C# 基准的 ForceDirty 等价，用于测“缓存无复用”的冷开销。要求 (x,y) 当前可走。
        /// </summary>
        public void ForceDirty(int x, int y)
        {
            NativeJps.jps_system_set_blocked(_sys, x, y, 1);
            NativeJps.jps_system_set_blocked(_sys, x, y, 0);
            NativeJps.jps_system_sync(_sys);
        }

        /// <summary>重新把缓存同步到当前（未改动的）地图：热缓存计时前调用一次。</summary>
        public void SetBlocked(int x, int y, bool blocked) =>
            NativeJps.jps_system_set_blocked(_sys, x, y, blocked ? 1 : 0);

        /// <summary>
        /// 一次 P/Invoke 应用一批稀疏阻挡增量。<paramref name="apply"/>=true 翻到 <c>!oldBlocked</c>（施加改动），
        /// false 翻回 <c>oldBlocked</c>（还原）。用于冷路径计时里模拟"每次查询前一小簇格子变化"，
        /// 避免逐格 P/Invoke 的开销污染计时。
        /// </summary>
        public void SetBlockedBatch(IReadOnlyList<(int x, int y, bool oldBlocked)> edits, bool apply)
        {
            int n = edits.Count;
            if (n == 0) return;
            if (_editBuf.Length < n * 3) _editBuf = new int[n * 3];
            for (int i = 0; i < n; i++)
            {
                var e = edits[i];
                _editBuf[i * 3] = e.x;
                _editBuf[i * 3 + 1] = e.y;
                _editBuf[i * 3 + 2] = (apply ? !e.oldBlocked : e.oldBlocked) ? 1 : 0;
            }
            NativeJps.jps_system_set_blocked_batch(_sys, _editBuf, n);
        }

        public void Sync() => NativeJps.jps_system_sync(_sys);

        /// <summary>最近一次寻路展开的节点数。</summary>
        public int Expanded => NativeJps.jps_pathfinder_expanded_nodes(_pf);

        /// <summary>寻路并返回 (是否有解, 路径)；供与 C# 版逐格一致性抽检。</summary>
        public (bool ok, List<(int X, int Y)>? path) FindAndCopy(int sx, int sy, int gx, int gy)
        {
            int n = Find(sx, sy, gx, gy);
            if (n < 0) return (false, null);
            var buf = new int[(long)n * 2];
            int got = NativeJps.jps_pathfinder_copy_path(_pf, buf, n);
            var path = new List<(int X, int Y)>(got);
            for (int k = 0; k < got; k++) path.Add((buf[k * 2], buf[k * 2 + 1]));
            return (true, path);
        }

        public void Dispose()
        {
            if (_pf != IntPtr.Zero)
            {
                NativeJps.jps_pathfinder_destroy(_pf);
                _pf = IntPtr.Zero;
            }
            if (_sys != IntPtr.Zero)
            {
                NativeJps.jps_system_destroy(_sys);
                _sys = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }
    }
}
