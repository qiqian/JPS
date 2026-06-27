using System.Diagnostics;
using System.Text;
using System.Text.Json;
using JPS.Data;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS.Benchmark
{
    /// <summary>
    /// 命令行工具：
    ///   dotnet run -- bench         单线程 JPS / A* 性能基准（test2.json）
    ///   dotnet run -- mt            多线程共享缓存正确性压测（JPS.Core 默认已开启 JPS_CONCURRENT_CACHE）
    ///   dotnet run -- map &lt;path&gt;    加载一张 MovingAI .map 并做一次 JPS 寻路（解析/寻路自检）
    /// </summary>
    internal static class Program
    {
        static int Main(string[] args)
        {
            string cmd = args.Length >= 1 ? args[0] : "bench";
            switch (cmd)
            {
                case "bench": Bench(); return 0;
                case "mt": MtTest(); return 0;
                case "map": MapTest(args.Length >= 2 ? args[1] : FindFile("movingai/maze-32-32-2.map")); return 0;
                default:
                    Console.WriteLine("用法: dotnet run -- [bench|mt|map <path.map>]");
                    return 1;
            }
        }

        private static long PathCost(List<(int X, int Y)> p)
        {
            long c = 0;
            for (int i = 1; i < p.Count; i++)
            {
                int dx = Math.Abs(p[i].X - p[i - 1].X), dy = Math.Abs(p[i].Y - p[i - 1].Y);
                c += (dx != 0 && dy != 0) ? JpsDirections.DiagonalCost : JpsDirections.CardinalCost;
            }
            return c;
        }

        private static string FindFile(string name)
        {
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 10 && dir != null; i++)
            {
                string p = Path.Combine(dir, name);
                if (File.Exists(p)) return p;
                dir = Directory.GetParent(dir)?.FullName;
            }
            if (File.Exists(name)) return name;
            throw new FileNotFoundException(name);
        }

        private static void Bench()
        {
            var sb = new StringBuilder();
            string path = FindFile("test2.json");
            var data = JsonSerializer.Deserialize<MapData>(File.ReadAllText(path))!;
            int w = data.Width, h = data.Height;
            var map = new GridMap(w, h);
            foreach (var o in data.Obstacles) map.SetBlocked(o.X, o.Y, true);

            var system = new JpsSystem(map);
            var jps = new JpsPathfinder();
            var astar = new AStarPathfinder();
            var rng = new Random(123);
            (int X, int Y) Free() { for (int k = 0; k < 9999; k++) { int x = rng.Next(w), y = rng.Next(h); if (map.IsWalkable(x, y)) return (x, y); } return (-1, -1); }

            const int Q = 8000;
            var qs = new ((int X, int Y) s, (int X, int Y) g)[Q];
            for (int i = 0; i < Q; i++) qs[i] = (Free(), Free());

            // 节点数（与计时分离）：JIT 预热后统计
            system.Sync();
            for (int i = 0; i < Q; i++) jps.FindPath(system, qs[i].s, qs[i].g, false);
            long jExp = 0, aExp = 0; int solved = 0;
            for (int i = 0; i < Q; i++) { var r = jps.FindPath(system, qs[i].s, qs[i].g, false); jExp += r.ExpandedNodes; if (r.Success) solved++; }
            for (int i = 0; i < Q; i++) { var r = astar.FindPath(map, qs[i].s, qs[i].g, false); aExp += r.ExpandedNodes; }

            // 无复用基线：每次查询前翻转一个可走格(改回原状)使 Version 跳变→整表 dirty，
            // 模拟“每个 finder 各持私有冷缓存、彼此不预热”的总开销。
            var (tx, ty) = qs[0].s;   // 一个已知可走格
            void ForceDirty() { map.SetBlocked(tx, ty, true); map.SetBlocked(tx, ty, false); system.Sync(); }

            double jColdMs = double.MaxValue, jWarmMs = double.MaxValue, aMs = double.MaxValue;
            var sw = new Stopwatch();
            for (int rep = 0; rep < 6; rep++)
            {
                GC.Collect(); GC.WaitForPendingFinalizers();
                sw.Restart();
                for (int i = 0; i < Q; i++) { ForceDirty(); jps.FindPath(system, qs[i].s, qs[i].g, false); }
                sw.Stop(); jColdMs = Math.Min(jColdMs, sw.Elapsed.TotalMilliseconds);

                // 热缓存（共享/复用）：Sync 一次，整批复用已洗白的跳点
                GC.Collect(); GC.WaitForPendingFinalizers();
                system.Sync();
                sw.Restart();
                for (int i = 0; i < Q; i++) jps.FindPath(system, qs[i].s, qs[i].g, false);
                sw.Stop(); jWarmMs = Math.Min(jWarmMs, sw.Elapsed.TotalMilliseconds);

                GC.Collect(); GC.WaitForPendingFinalizers();
                sw.Restart();
                for (int i = 0; i < Q; i++) astar.FindPath(map, qs[i].s, qs[i].g, false);
                sw.Stop(); aMs = Math.Min(aMs, sw.Elapsed.TotalMilliseconds);
            }

            sb.AppendLine($"[test2.json] {w}x{h}（结构化地图）, {Q} 组随机起终点, 可解 {solved}");
            sb.AppendLine($"  扩展节点 平均/次: JPS={jExp / (double)Q:F0}  A*={aExp / (double)Q:F0}  (A*/JPS={aExp / (double)Math.Max(1, jExp):F1}x)");
            sb.AppendLine($"  耗时 平均/次(热): JPS={jWarmMs / Q * 1000:F1}us  A*={aMs / Q * 1000:F1}us  (A*/JPS={aMs / Math.Max(0.001, jWarmMs):F1}x)");
            sb.AppendLine($"  缓存无复用/复用:  无复用={jColdMs / Q * 1000:F1}us  复用={jWarmMs / Q * 1000:F1}us  (加速={jColdMs / Math.Max(0.001, jWarmMs):F2}x)");
            Console.WriteLine(sb.ToString());
        }

        // 加载一张 MovingAI .map，做一次 JPS 寻路，验证解析与寻路是否正常。
        private static void MapTest(string path)
        {
            var map = MovingAiMap.Parse(File.ReadAllText(path));
            int walk = 0;
            (int X, int Y) first = (-1, -1), last = (-1, -1);
            for (int y = 0; y < map.Height; y++)
                for (int x = 0; x < map.Width; x++)
                    if (map.IsWalkable(x, y)) { walk++; if (first.X < 0) first = (x, y); last = (x, y); }

            var system = new JpsSystem(map);
            system.Sync();
            var jps = new JpsPathfinder();
            var r = jps.FindPath(system, first, last, false);

            Console.WriteLine($"{Path.GetFileName(path)}  {map.Width}x{map.Height}, 可走 {walk} 格");
            Console.WriteLine($"  JPS {first} -> {last}: success={r.Success}, expanded={r.ExpandedNodes}, path={r.Path.Count}");
        }

        private static void MtTest()
        {
            var rng = new Random(7);
            int w = 200, h = 200;
            var map = new GridMap(w, h);
            for (int i = 0; i < w * h * 0.2; i++) map.SetBlocked(rng.Next(w), rng.Next(h), true);
            var system = new JpsSystem(map);
            system.Sync();   // 并行前单线程同步一次

            (int X, int Y) Free() { for (int k = 0; k < 500; k++) { int x = rng.Next(w), y = rng.Next(h); if (map.IsWalkable(x, y)) return (x, y); } return (-1, -1); }

            // 单线程用 A* 算 ground truth
            var astar = new AStarPathfinder();
            const int Q = 3000;
            var pairs = new (int sx, int sy, int gx, int gy, long cost, bool ok)[Q];
            for (int i = 0; i < Q; i++)
            {
                var s = Free(); var g = Free();
                var r = astar.FindPath(map, s, g, false);
                pairs[i] = (s.X, s.Y, g.X, g.Y, r.Success ? PathCost(r.Path) : -1, r.Success);
            }

            // 多线程：每线程一个 JpsPathfinder，共享同一个 system/cache，并行跑同一批查询
            int threads = 8, mismatches = 0;
            System.Threading.Tasks.Parallel.For(0, threads, _ =>
            {
                var jps = new JpsPathfinder();
                int local = 0;
                for (int i = 0; i < Q; i++)
                {
                    var p = pairs[i];
                    var r = jps.FindPath(system, (p.sx, p.sy), (p.gx, p.gy), false);
                    long cost = r.Success ? PathCost(r.Path) : -1;
                    if (r.Success != p.ok || cost != p.cost) local++;
                }
                System.Threading.Interlocked.Add(ref mismatches, local);
            });

            Console.WriteLine($"多线程共享 cache：{threads} 线程 × {Q} 查询并行，与 A* 不一致 {mismatches}。");
        }
    }
}
