namespace JPS
{
    internal static class Program
    {
        /// <summary>
        ///  The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main(string[] args)
        {
            if (args.Length >= 1 && args[0] == "mt") { MtTest(); return; }
            if (args.Length >= 1 && args[0] == "bench") { Bench(); return; }

            ApplicationConfiguration.Initialize();
            Application.Run(new Form1());
        }

        private static long PathCost(List<(int X, int Y)> p)
        {
            long c = 0;
            for (int i = 1; i < p.Count; i++)
            {
                int dx = Math.Abs(p[i].X - p[i - 1].X), dy = Math.Abs(p[i].Y - p[i - 1].Y);
                c += (dx != 0 && dy != 0) ? JPS.Pathfinding.JpsDirections.DiagonalCost : JPS.Pathfinding.JpsDirections.CardinalCost;
            }
            return c;
        }

        private static string FindFile(string name)
        {
            string? dir = AppContext.BaseDirectory;
            for (int i = 0; i < 8 && dir != null; i++)
            {
                string p = System.IO.Path.Combine(dir, name);
                if (System.IO.File.Exists(p)) return p;
                dir = System.IO.Directory.GetParent(dir)?.FullName;
            }
            if (System.IO.File.Exists(name)) return name;
            throw new System.IO.FileNotFoundException(name);
        }

        private static void Bench()
        {
            var sb = new System.Text.StringBuilder();
            string path = FindFile("test2.json");
            var data = System.Text.Json.JsonSerializer.Deserialize<JPS.Models.MapData>(System.IO.File.ReadAllText(path))!;
            int w = data.Width, h = data.Height;
            var map = new JPS.Models.GridMap(w, h);
            foreach (var o in data.Obstacles) map.SetBlocked(o.X, o.Y, true);

            var system = new JPS.Pathfinding.JpsSystem(map);
            var jps = new JPS.Pathfinding.JpsPathfinder();
            var astar = new JPS.Pathfinding.AStarPathfinder();
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
            // 模拟"每个 finder 各持私有冷缓存、彼此不预热"的总开销。
            var (tx, ty) = qs[0].s;   // 一个已知可走格
            void ForceDirty() { map.SetBlocked(tx, ty, true); map.SetBlocked(tx, ty, false); system.Sync(); }

            double jColdMs = double.MaxValue, jWarmMs = double.MaxValue, aMs = double.MaxValue;
            var sw = new System.Diagnostics.Stopwatch();
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
            System.IO.File.WriteAllText("bench_result.txt", sb.ToString());
        }

        private static void MtTest()
        {
            var rng = new Random(7);
            int w = 200, h = 200;
            var map = new JPS.Models.GridMap(w, h);
            for (int i = 0; i < w * h * 0.2; i++) map.SetBlocked(rng.Next(w), rng.Next(h), true);
            var system = new JPS.Pathfinding.JpsSystem(map);
            system.Sync();   // 并行前单线程同步一次

            (int X, int Y) Free() { for (int k = 0; k < 500; k++) { int x = rng.Next(w), y = rng.Next(h); if (map.IsWalkable(x, y)) return (x, y); } return (-1, -1); }

            // 单线程用 A* 算 ground truth
            var astar = new JPS.Pathfinding.AStarPathfinder();
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
                var jps = new JPS.Pathfinding.JpsPathfinder();
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

            string msg = $"多线程共享 cache：{threads} 线程 × {Q} 查询并行，与 A* 不一致 {mismatches}。";
            Console.WriteLine(msg);
            System.IO.File.WriteAllText("mt_result.txt", msg);
        }
    }
}