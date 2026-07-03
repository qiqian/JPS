/*
 * Native.cs
 * JPS Pathfinding — JPS.Native（C DLL）的 P/Invoke 封装与每线程运行器，
 * 供正确性测试把 C 版 JPS 的寻路结果与 C# 版逐格比对（强一致校验）。
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System.Runtime.InteropServices;
using JPS.Models;

namespace JPS.Accuracy
{
    /// <summary>
    /// JPS.Native.dll 的扁平 C ABI 绑定（与 jps.h 一一对应）。
    /// 静态构造时注册 DllImportResolver，从仓库的 x64/{Release,Debug} 等位置定位 DLL，
    /// 无需把原生 DLL 复制进托管输出目录。
    /// </summary>
    internal static class NativeJps
    {
        private const string Dll = "JPS.Native";

        /// <summary>实际加载到的 DLL 路径（探测成功后填充，用于报告横幅）。</summary>
        public static string? ResolvedPath { get; private set; }

        static NativeJps()
        {
            NativeLibrary.SetDllImportResolver(typeof(NativeJps).Assembly, (name, asm, search) =>
            {
                if (name != Dll) return IntPtr.Zero;
                foreach (var cand in Candidates())
                {
                    if (File.Exists(cand) && NativeLibrary.TryLoad(cand, out var h))
                    {
                        ResolvedPath = cand;
                        return h;
                    }
                }
                return IntPtr.Zero;   // 交回默认搜索（找不到则首次 P/Invoke 抛 DllNotFoundException）
            });
        }

        // 从应用基目录逐级向上，在每一层探测原生 DLL 的常见输出位置。
        private static IEnumerable<string> Candidates()
        {
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 12 && dir != null; i++)
            {
                yield return Path.Combine(dir, Dll + ".dll");                       // 与托管输出同目录（若已复制）
                yield return Path.Combine(dir, "x64", "Release", Dll + ".dll");     // 解决方案级 x64 输出
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
            catch (Exception ex)
            {
                info = ex.Message;
                return false;
            }
        }

        // ---- jps_system：建图 / 改阻挡 / 同步 ----
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr jps_system_create(int width, int height);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_destroy(IntPtr s);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_set_blocked_buffer(IntPtr s, byte[] cells, int count);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_set_blocked(IntPtr s, int x, int y, int blocked);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_system_sync(IntPtr s);

        // ---- jps_pathfinder：寻路 / 取结果 ----
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr jps_pathfinder_create();
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern void jps_pathfinder_destroy(IntPtr pf);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jps_pathfinder_find_path(IntPtr pf, IntPtr system, int sx, int sy, int gx, int gy);
        [DllImport(Dll, CallingConvention = CallingConvention.Cdecl)]
        public static extern int jps_pathfinder_copy_path(IntPtr pf, int[] outXy, int capacityPoints);
    }

    /// <summary>
    /// 一张地图对应的原生 jps_system（拥有地图 + 跳点缓存）。
    /// **单线程**构建：灌入阻挡 + Sync 一次；之后地图只读。与 C# 的 JpsSystem 用法严格对应：
    /// 一张图建一个 system，随后多个线程（各持自己的 NativePathfinder）共享它并行寻路，
    /// 这张图的用例跑完即 Dispose。
    /// </summary>
    internal sealed class NativeSystem : IDisposable
    {
        public IntPtr Handle { get; private set; }

        public NativeSystem(GridMap map)
        {
            Handle = NativeJps.jps_system_create(map.Width, map.Height);
            var cells = new byte[map.Width * map.Height];   // 行主序：与 jps_system_set_blocked_buffer 约定一致
            int i = 0;
            for (int y = 0; y < map.Height; y++)
                for (int x = 0; x < map.Width; x++)
                    cells[i++] = map.IsBlocked(x, y) ? (byte)1 : (byte)0;
            NativeJps.jps_system_set_blocked_buffer(Handle, cells, cells.Length);
            NativeJps.jps_system_sync(Handle);              // 灌入阻挡后同步缓存（并行寻路前由单线程做）
        }

        /// <summary>改单格阻挡（供冷缓存测试改图/还原用；改后需 Sync 才生效）。仅在单线程阶段调用。</summary>
        public void SetBlocked(int x, int y, bool blocked) =>
            NativeJps.jps_system_set_blocked(Handle, x, y, blocked ? 1 : 0);

        /// <summary>把缓存同步到当前地图（改图后调用一次，使受影响行/列失效）。</summary>
        public void Sync() => NativeJps.jps_system_sync(Handle);

        public void Dispose()
        {
            if (Handle != IntPtr.Zero) { NativeJps.jps_system_destroy(Handle); Handle = IntPtr.Zero; }
        }
    }

    /// <summary>
    /// 线程私有的原生 jps_pathfinder（搜索态线程私有，对应 C# 的 JpsPathfinder）。
    /// 寻路时绑定到某个共享的 NativeSystem——多个 NativePathfinder 可并行复用同一个 system。
    /// </summary>
    internal sealed class NativePathfinder : IDisposable
    {
        private IntPtr _pf;

        public NativePathfinder() => _pf = NativeJps.jps_pathfinder_create();

        /// <summary>在给定 system 上寻路。返回 (是否有解, 路径)；无解时路径为 null。</summary>
        public (bool ok, List<(int X, int Y)>? path) Find(NativeSystem sys, int sx, int sy, int gx, int gy)
        {
            int n = NativeJps.jps_pathfinder_find_path(_pf, sys.Handle, sx, sy, gx, gy);
            if (n < 0) return (false, null);

            var buf = new int[(long)n * 2];
            int got = NativeJps.jps_pathfinder_copy_path(_pf, buf, n);
            var path = new List<(int X, int Y)>(got);
            for (int k = 0; k < got; k++)
                path.Add((buf[k * 2], buf[k * 2 + 1]));
            return (true, path);
        }

        public void Dispose()
        {
            if (_pf != IntPtr.Zero) { NativeJps.jps_pathfinder_destroy(_pf); _pf = IntPtr.Zero; }
        }
    }
}
