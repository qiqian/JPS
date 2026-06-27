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