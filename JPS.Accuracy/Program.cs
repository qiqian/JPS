using System.Diagnostics;
using System.Globalization;
using System.Text;
using JPS.Data;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS.Accuracy
{
    /// <summary>
    /// MovingAI .scen 批量正确性测试。
    ///
    /// 用法：
    ///   dotnet run -c Release -- [子目录] [每个scen最多用例数]
    ///     子目录            限定在 movingai/&lt;子目录&gt; 下递归找 .scen（缺省=movingai/ 全部）
    ///     每个scen最多用例数  0 或缺省 = 该 scen 的全部用例
    ///
    /// 每条场景给出官方最优解长度（octile：直 1、斜 √2，且不允许斜穿角）。本测试对每条用例：
    ///   1) JPS vs A*：整数代价（直 1000 / 斜 1414）必须**完全相等** → JPS 最优性的硬校验；
    ///   2) JPS 路径合法性：首尾正确、逐格相邻、格子可走、（按当前构建）不斜穿角；
    ///   3) JPS vs 官方最优：把 JPS 路径折算成真实 octile 长度（直 1 + 斜 √2），与 optimal 比对，
    ///      校验“本项目的移动模型/最优性”是否与 MovingAI 基准一致。
    ///
    /// 注：算法内部用整数 1414 近似 √2(=1.41421356…)，故第 3 项可能有 ~1e-4·斜步 的舍入偏差，
    /// 属正常现象；真正的 bug（次优 / 斜穿角 / 不可达）会产生远大于此的偏差，会被单独计入。
    /// </summary>
    internal static class Program
    {
        private readonly record struct Entry(string Map, int Sx, int Sy, int Gx, int Gy, double Optimal);

        private const double Sqrt2 = 1.4142135623730951;
        private const double RefTol = 1e-2;   // 与官方最优的容差：远大于整数度量舍入、远小于任何真实次优

        static int Main(string[] args)
        {
            string? sub = args.Length >= 1 && !string.IsNullOrWhiteSpace(args[0]) ? args[0] : null;
            int maxPerScen = args.Length >= 2 && int.TryParse(args[1], out int m) && m > 0 ? m : 0;
            return Run(sub, maxPerScen);
        }

        // 同时把输出写到控制台与报告文件（接管 Console.Out）。
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

        private static int Run(string? sub, int maxPerScen)
        {
            string root = FindDir("movingai");
            string dir = string.IsNullOrEmpty(sub) ? root : Path.Combine(root, sub!);
            if (!Directory.Exists(dir)) { Console.WriteLine($"目录不存在：{dir}"); return 1; }

            var files = Directory.GetFiles(dir, "*.scen", SearchOption.AllDirectories)
                                 .OrderBy(f => Path.GetRelativePath(root, f), StringComparer.OrdinalIgnoreCase)
                                 .ToArray();
            if (files.Length == 0) { Console.WriteLine($"未找到 .scen 文件：{dir}"); return 1; }

            string repoRoot = Directory.GetParent(root)?.FullName ?? root;
            string reportsDir = Path.Combine(repoRoot, "accuracy-results");
            Directory.CreateDirectory(reportsDir);
            string scopeTag = string.IsNullOrEmpty(sub) ? "all" : sub!.Replace('/', '-').Replace('\\', '-');
            string reportPath = Path.Combine(reportsDir, $"scen-{scopeTag}-{DateTime.Now:yyyyMMdd-HHmmss}.txt");

            var consoleOut = Console.Out;
            var fileOut = new StreamWriter(reportPath) { AutoFlush = true };
            Console.SetOut(new TeeTextWriter(consoleOut, fileOut));

            Console.WriteLine($"# JPS / A* · MovingAI .scen 正确性报告   {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            Console.WriteLine($"构建配置（JPS.Core）：斜穿角={(JpsBuildInfo.CornerCutting ? "允许" : "禁止")}");
            string scope = string.IsNullOrEmpty(sub) ? "movingai/ 全部" : $"movingai/{sub}";
            Console.WriteLine($"范围：{scope}，共 {files.Length} 个 .scen 文件，每个 {(maxPerScen == 0 ? "全部用例" : $"最多 {maxPerScen} 例")}");
            Console.WriteLine();
            Console.WriteLine("校验项：① JPS整数代价==A*整数代价（最优性）  ② JPS路径合法（相邻/可走/不切角）  ③ JPS真实长度≈官方最优");
            Console.WriteLine("列说明：n=测试用例数  pass=三项全过  jFail=JPS无解(A*有解)  subopt=JPS≠A*  inval=路径非法  refL=比官方长  refS=比官方短");
            Console.WriteLine();
            Console.WriteLine($"{"scen",-44}{"n",8}{"pass",8}{"jFail",7}{"subopt",8}{"inval",7}{"refL",7}{"refS",7}");
            Console.WriteLine(new string('-', 94));

            // 跨 scen 复用已加载的地图与共享缓存（同一张图被多个 scen 引用时直接命中、且缓存已预热）
            var mapCache = new Dictionary<string, (GridMap map, JpsSystem sys)>(StringComparer.OrdinalIgnoreCase);
            var jps = new JpsPathfinder();
            var astar = new AStarPathfinder();
            bool allowCorner = JpsBuildInfo.CornerCutting;

            long tN = 0, tPass = 0, tJFail = 0, tSubopt = 0, tInval = 0, tRefL = 0, tRefS = 0, tTrivial = 0, tBadCell = 0;
            long exactCnt = 0, artifactCnt = 0;
            double maxDevOk = 0;     // 通过项里的最大 |真实长度-官方|（应为舍入量级）
            double worstBad = 0;     // 失败项里的最大 |偏差|
            var sw = Stopwatch.StartNew();

            foreach (var f in files)
            {
                string rel = Path.GetRelativePath(root, f).Replace('\\', '/');
                string name = rel.EndsWith(".scen", StringComparison.OrdinalIgnoreCase) ? rel[..^5] : rel;
                string scenDir = Path.GetDirectoryName(f)!;

                List<Entry> entries;
                try { entries = ParseScen(f); }
                catch (Exception ex) { Console.WriteLine($"{name,-44}  解析失败：{ex.Message}"); continue; }
                if (maxPerScen > 0 && entries.Count > maxPerScen) entries = entries.GetRange(0, maxPerScen);

                int n = 0, pass = 0, jFail = 0, subopt = 0, inval = 0, refL = 0, refS = 0;

                // 一个 .scen 内所有用例几乎总是引用同一张图：只在 map 字段变化时才解析/解析路径，
                // 其余用例直接复用上次的 GridMap + 预热好的 JpsSystem（跨文件再由 mapCache 复用）。
                GridMap? map = null;
                JpsSystem? sys = null;
                string? curMapField = null;

                foreach (var e in entries)
                {
                    if (!string.Equals(curMapField, e.Map, StringComparison.Ordinal))
                    {
                        curMapField = e.Map;
                        try { (map, sys) = GetMap(scenDir, e.Map, root, mapCache); }
                        catch { map = null; sys = null; }   // 地图缺失：置空，引用它的用例随后被跳过
                    }
                    if (map is null || sys is null) continue;   // 该图缺失，跳过（不影响其余；下方 map/sys 已收窄为非空）

                    var s = (e.Sx, e.Sy);
                    var g = (e.Gx, e.Gy);

                    // 越界 / 起终点在阻挡上：场景/地图不匹配，单独计数，不进入算法校验
                    if (!InBounds(map, s) || !InBounds(map, g) || !map.IsWalkable(s.Item1, s.Item2) || !map.IsWalkable(g.Item1, g.Item2))
                    { tBadCell++; continue; }

                    if (s == g) { tTrivial++; continue; }   // 平凡用例（首尾同格）跳过

                    n++;
                    var rj = jps.FindPath(sys, s, g);
                    var ra = astar.FindPath(map, s, g);

                    // ① 最优性：与 A*（同度量 ground truth）整数代价必须一致
                    if (rj.Success != ra.Success)
                    {
                        if (!rj.Success) jFail++;       // A* 有解而 JPS 无解 → JPS 漏解
                        else subopt++;                  // JPS 有解而 A* 无解 → 异常（计入 subopt）
                        continue;
                    }
                    if (!rj.Success) { jFail++; continue; }   // 两者都无解（可解场景里属异常）

                    var (jCard, jDiag) = CountSteps(rj.Path);
                    var (aCard, aDiag) = CountSteps(ra.Path);
                    long jInt = jCard * 1000L + jDiag * 1414L;
                    long aInt = aCard * 1000L + aDiag * 1414L;
                    if (jInt != aInt) { subopt++; worstBad = Math.Max(worstBad, Math.Abs(jInt - aInt) / 1000.0); continue; }

                    // ② 路径合法性
                    if (!ValidPath(map, rj.Path, s, g, allowCorner, out _)) { inval++; continue; }

                    // ③ 与官方最优比对（真实 octile 长度）
                    double jLen = jCard + jDiag * Sqrt2;
                    double dev = jLen - e.Optimal;
                    if (dev > RefTol) { refL++; worstBad = Math.Max(worstBad, dev); continue; }
                    if (dev < -RefTol) { refS++; worstBad = Math.Max(worstBad, -dev); continue; }

                    double ad = Math.Abs(dev);
                    if (ad <= 1e-6) exactCnt++; else artifactCnt++;
                    maxDevOk = Math.Max(maxDevOk, ad);
                    pass++;
                }

                if (n > 0)
                    Console.WriteLine($"{Trunc(name, 44),-44}{n,8}{pass,8}{jFail,7}{subopt,8}{inval,7}{refL,7}{refS,7}");

                tN += n; tPass += pass; tJFail += jFail; tSubopt += subopt; tInval += inval; tRefL += refL; tRefS += refS;
            }

            sw.Stop();
            Console.WriteLine(new string('-', 94));
            Console.WriteLine($"{"合计",-44}{tN,8}{tPass,8}{tJFail,7}{tSubopt,8}{tInval,7}{tRefL,7}{tRefS,7}");
            Console.WriteLine();
            long fails = tJFail + tSubopt + tInval + tRefL + tRefS;
            Console.WriteLine($"用例 {tN}（平凡 {tTrivial}，无效起终点 {tBadCell} 已跳过），用时 {sw.Elapsed.TotalSeconds:F1}s");
            Console.WriteLine($"通过项中：精确匹配 {exactCnt}，整数度量舍入 {artifactCnt}（最大偏差 {maxDevOk:E2}）");
            Console.WriteLine(fails == 0
                ? $"正确性：全部 {tN} 例三项校验通过，✓ 通过"
                : $"正确性：⚠ {fails} 例未通过（jFail={tJFail} subopt={tSubopt} inval={tInval} refL={tRefL} refS={tRefS}，最大偏差 {worstBad:F4}）");

            Console.Out.Flush();
            Console.SetOut(consoleOut);
            fileOut.Dispose();
            Console.WriteLine($"报告已保存：{reportPath}");
            return fails == 0 ? 0 : 2;
        }

        // ---------------- 解析 / 工具 ----------------

        private static List<Entry> ParseScen(string path)
        {
            var list = new List<Entry>();
            foreach (var raw in File.ReadLines(path))
            {
                string line = raw.Trim();
                if (line.Length == 0) continue;
                if (line.StartsWith("version", StringComparison.OrdinalIgnoreCase)) continue;
                var t = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                if (t.Length < 9) continue;
                // 格式：bucket  map  width  height  startX  startY  goalX  goalY  optimal
                list.Add(new Entry(
                    t[1],
                    int.Parse(t[4], CultureInfo.InvariantCulture),
                    int.Parse(t[5], CultureInfo.InvariantCulture),
                    int.Parse(t[6], CultureInfo.InvariantCulture),
                    int.Parse(t[7], CultureInfo.InvariantCulture),
                    double.Parse(t[8], CultureInfo.InvariantCulture)));
            }
            return list;
        }

        private static (GridMap map, JpsSystem sys) GetMap(
            string scenDir, string mapField, string root, Dictionary<string, (GridMap, JpsSystem)> cache)
        {
            string p = Path.Combine(scenDir, mapField);
            if (!File.Exists(p))
            {
                // 兜底：按文件名在 movingai/ 下递归找
                var found = Directory.GetFiles(root, Path.GetFileName(mapField), SearchOption.AllDirectories);
                if (found.Length == 0) throw new FileNotFoundException(mapField);
                p = found[0];
            }
            string key = Path.GetFullPath(p);
            if (cache.TryGetValue(key, out var v)) return v;

            var map = MovingAiMap.Parse(File.ReadAllText(key));
            var sys = new JpsSystem(map);
            sys.Sync();
            var tup = (map, sys);
            cache[key] = tup;
            return tup;
        }

        private static bool InBounds(GridMap map, (int X, int Y) p) =>
            p.X >= 0 && p.Y >= 0 && p.X < map.Width && p.Y < map.Height;

        private static (int card, int diag) CountSteps(List<(int X, int Y)> p)
        {
            int card = 0, diag = 0;
            for (int i = 1; i < p.Count; i++)
            {
                int dx = Math.Abs(p[i].X - p[i - 1].X), dy = Math.Abs(p[i].Y - p[i - 1].Y);
                if (dx != 0 && dy != 0) diag++; else card++;
            }
            return (card, diag);
        }

        private static bool ValidPath(
            GridMap map, List<(int X, int Y)> p, (int X, int Y) s, (int X, int Y) g, bool allowCorner, out string why)
        {
            why = "";
            if (p.Count == 0) { why = "empty"; return false; }
            if (p[0] != s) { why = "start"; return false; }
            if (p[^1] != g) { why = "goal"; return false; }

            for (int i = 1; i < p.Count; i++)
            {
                int dx = p[i].X - p[i - 1].X, dy = p[i].Y - p[i - 1].Y;
                if (Math.Abs(dx) > 1 || Math.Abs(dy) > 1 || (dx == 0 && dy == 0)) { why = $"step@{i}"; return false; }
                if (!map.IsWalkable(p[i].X, p[i].Y)) { why = $"blocked@{i}"; return false; }
                if (dx != 0 && dy != 0 && !allowCorner &&
                    (!map.IsWalkable(p[i - 1].X + dx, p[i - 1].Y) || !map.IsWalkable(p[i - 1].X, p[i - 1].Y + dy)))
                { why = $"corner@{i}"; return false; }
            }
            return true;
        }

        private static string Trunc(string s, int max) => s.Length <= max ? s : s[..(max - 1)] + "…";

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
    }
}
