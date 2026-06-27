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
    ///   dotnet run -- mapbench [q] [子目录]  递归遍历 movingai/（或其指定子目录）下所有 .map，每图随机取 q 组可解起终点对比 JPS/A*
    ///                                        结果同时打印到控制台并写入 benchmark-results/ 下的报告文件
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
                case "map": MapTest(args.Length >= 2 ? args[1] : FindFile("movingai/mapf-map/maze-32-32-2.map")); return 0;
                case "mapbench": MapBench(args.Length >= 2 && int.TryParse(args[1], out int q) ? q : 30, args.Length >= 3 ? args[2] : null); return 0;
                default:
                    Console.WriteLine("用法: dotnet run -- [bench | mt | map <path.map> | mapbench [每图样本数] [子目录]]");
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

        private static string FindDir(string name)
        {
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 10 && dir != null; i++)
            {
                string p = Path.Combine(dir, name);
                if (Directory.Exists(p)) return p;
                dir = Directory.GetParent(dir)?.FullName;
            }
            if (Directory.Exists(name)) return name;
            throw new DirectoryNotFoundException(name);
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

        // 遍历 movingai/ 下所有 .map，每图随机取 q 组“可解”起终点，对比 JPS / A*。
        // 同时把输出写到控制台与报告文件（接管 Console.Out，无需改动任何 WriteLine）。
        private sealed class TeeTextWriter : TextWriter
        {
            private readonly TextWriter _a, _b;
            public TeeTextWriter(TextWriter a, TextWriter b) { _a = a; _b = b; }
            public override Encoding Encoding => _a.Encoding;
            public override void Write(char value) { _a.Write(value); _b.Write(value); }
            public override void Write(string? value) { _a.Write(value); _b.Write(value); }
            public override void WriteLine(string? value) { _a.WriteLine(value); _b.WriteLine(value); }
            public override void Flush() { _a.Flush(); _b.Flush(); }
        }

        private static void MapBench(int q, string? sub = null)
        {
            string root = FindDir("movingai");
            string dir = string.IsNullOrEmpty(sub) ? root : Path.Combine(root, sub!);
            if (!Directory.Exists(dir)) { Console.WriteLine($"目录不存在：{dir}"); return; }

            var files = Directory.GetFiles(dir, "*.map", SearchOption.AllDirectories)
                                 .OrderBy(f => Path.GetRelativePath(root, f), StringComparer.OrdinalIgnoreCase)
                                 .ToArray();

            string scope = string.IsNullOrEmpty(sub) ? "递归遍历 movingai/ 全部子目录" : $"子集 movingai/{sub}";

            // 结果同时落盘到 benchmark-results/ 下的报告文件，方便日后查看
            string repoRoot = Directory.GetParent(root)?.FullName ?? root;
            string reportsDir = Path.Combine(repoRoot, "benchmark-results");
            Directory.CreateDirectory(reportsDir);
            string scopeTag = string.IsNullOrEmpty(sub) ? "all" : sub!.Replace('/', '-').Replace('\\', '-');
            string reportPath = Path.Combine(reportsDir, $"mapbench-{scopeTag}-q{q}-{DateTime.Now:yyyyMMdd-HHmmss}.txt");

            var consoleOut = Console.Out;
            var fileOut = new StreamWriter(reportPath) { AutoFlush = true };
            Console.SetOut(new TeeTextWriter(consoleOut, fileOut));

            Console.WriteLine($"# JPS vs A* · MovingAI 基准报告   {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            Console.WriteLine($"MovingAI 批量基准：{files.Length} 张图（{scope}），每图目标 {q} 组随机可解起终点（耗时取多轮最小，逐图输出）");
            Console.WriteLine();
            Console.WriteLine("列说明：");
            Console.WriteLine("  map     地图名");
            Console.WriteLine("  size    地图尺寸（宽 x 高）");
            Console.WriteLine("  walk%   可走格占全部格子的比例");
            Console.WriteLine("  pairs   实际采样到的“可解”起终点对数（达不到目标值时即该图能凑到的数量）");
            Console.WriteLine("  JPSexp  JPS 平均每次寻路的扩展节点数");
            Console.WriteLine("  A*exp   A* 平均每次寻路的扩展节点数");
            Console.WriteLine("  ratio   扩展节点比 = A*exp / JPSexp（越大表示 JPS 越省）");
            Console.WriteLine("  JPSus   JPS 平均每次寻路耗时（微秒，热缓存）");
            Console.WriteLine("  A*us    A* 平均每次寻路耗时（微秒）");
            Console.WriteLine("  speed   墙钟加速比 = A*us / JPSus（越大表示 JPS 越快）");
            Console.WriteLine("  mism    与 A* 结果不一致的组数（成败不同或路径代价不等；正确应为 0）");
            Console.WriteLine();
            Console.WriteLine($"{"map",-34}{"size",11}{"walk%",7}{"pairs",7}{"JPSexp",8}{"A*exp",8}{"ratio",7}{"JPSus",9}{"A*us",9}{"speed",7}{"mism",6}");
            Console.WriteLine(new string('-', 113));

            long sumJexp = 0, sumAexp = 0;
            double sumJms = 0, sumAms = 0;
            int sumPairs = 0, sumMism = 0;

            foreach (var f in files)
            {
                var map = MovingAiMap.Parse(File.ReadAllText(f));
                string rel = Path.GetRelativePath(root, f);
                string name = (rel.EndsWith(".map", StringComparison.OrdinalIgnoreCase)
                    ? rel.Substring(0, rel.Length - 4) : rel).Replace('\\', '/');
                string size = $"{map.Width}x{map.Height}";
                long tot = (long)map.Width * map.Height;

                var walk = new List<(int X, int Y)>();
                for (int y = 0; y < map.Height; y++)
                    for (int x = 0; x < map.Width; x++)
                        if (map.IsWalkable(x, y)) walk.Add((x, y));
                if (walk.Count < 2) { Console.WriteLine($"{name,-34}{size,11}   (可走格不足)"); continue; }

                var system = new JpsSystem(map);
                system.Sync();
                var jps = new JpsPathfinder();
                var astar = new AStarPathfinder();
                var rng = new Random(12345);

                // 用 JPS 随机采样“可解”起终点（顺便预热共享缓存）
                var qs = new List<((int X, int Y) s, (int X, int Y) g)>(q);
                int maxAtt = q * 40 + 500;
                for (int a = 0; a < maxAtt && qs.Count < q; a++)
                {
                    var s = walk[rng.Next(walk.Count)];
                    var g = walk[rng.Next(walk.Count)];
                    if (s.Equals(g)) continue;
                    if (jps.FindPath(system, s, g, false).Success) qs.Add((s, g));
                }
                int n = qs.Count;
                if (n == 0) { Console.WriteLine($"{name,-34}{size,11}   (无可解样本)"); continue; }

                // 扩展节点 + 正确性校验（与耗时分离统计）：
                // 逐样本对比 JPS 与 A* 的成败与“路径代价”是否一致（代价相等即最优；两者可走不同的等价最优路）。
                long jExp = 0, aExp = 0;
                int mism = 0;
                foreach (var p in qs)
                {
                    var rj = jps.FindPath(system, p.s, p.g, false);
                    var ra = astar.FindPath(map, p.s, p.g, false);
                    jExp += rj.ExpandedNodes;
                    aExp += ra.ExpandedNodes;
                    if (rj.Success != ra.Success || (rj.Success && PathCost(rj.Path) != PathCost(ra.Path)))
                        mism++;
                }

                // 耗时：多轮取最小，抑制 GC/调度噪声
                double jMs = double.MaxValue, aMs = double.MaxValue;
                var sw = new Stopwatch();
                for (int rep = 0; rep < 3; rep++)
                {
                    GC.Collect(); GC.WaitForPendingFinalizers();
                    sw.Restart();
                    foreach (var p in qs) jps.FindPath(system, p.s, p.g, false);
                    sw.Stop(); jMs = Math.Min(jMs, sw.Elapsed.TotalMilliseconds);

                    GC.Collect(); GC.WaitForPendingFinalizers();
                    sw.Restart();
                    foreach (var p in qs) astar.FindPath(map, p.s, p.g, false);
                    sw.Stop(); aMs = Math.Min(aMs, sw.Elapsed.TotalMilliseconds);
                }

                double jus = jMs / n * 1000, aus = aMs / n * 1000;
                Console.WriteLine(
                    $"{name,-34}{size,11}{walk.Count * 100.0 / tot,7:F1}{n,7}{jExp / n,8}{aExp / n,8}" +
                    $"{(double)aExp / Math.Max(1, jExp),7:F1}{jus,9:F1}{aus,9:F1}{aus / Math.Max(0.001, jus),7:F1}{mism,6}");

                sumJexp += jExp; sumAexp += aExp; sumJms += jMs; sumAms += aMs; sumPairs += n; sumMism += mism;
            }

            Console.WriteLine(new string('-', 113));
            if (sumPairs > 0)
            {
                Console.WriteLine(
                    $"合计 {sumPairs} 组：扩展节点 A*/JPS = {(double)sumAexp / Math.Max(1, sumJexp):F1}x，" +
                    $"耗时 A*/JPS = {sumAms / Math.Max(0.001, sumJms):F1}x（JPS 总 {sumJms:F0}ms，A* 总 {sumAms:F0}ms）");
                Console.WriteLine(sumMism == 0
                    ? $"正确性：{sumPairs} 组 JPS 与 A* 结果完全一致（成败 + 路径代价），✓ 通过"
                    : $"正确性：⚠ 有 {sumMism} 组与 A* 不一致！");
            }

            Console.Out.Flush();
            Console.SetOut(consoleOut);   // 还原标准输出
            fileOut.Dispose();
            Console.WriteLine($"报告已保存：{reportPath}");
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
