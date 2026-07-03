/*
 * Accuracy.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
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
    internal static class Accuracy
    {
        private readonly record struct Entry(string Map, int Sx, int Sy, int Gx, int Gy, double Optimal);

        private const double RefTol = 1e-2;   // 与官方最优的容差：远大于整数度量舍入、远小于任何真实次优

        // 失败用例自动存盘（地图+起终点 → Playground 可载入的 MapData JSON）。限量，避免回归把每条都判失败时写爆磁盘。
        private const int MaxFailDumps = 100;
        private static int _failDumpCount;

        // 冷缓存测试：每张图最多抽测的用例数（单线程跑，抽样以控住串行开销；改动可调）。
        private const int ColdSampleCap = 64;

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

        // 进度行：只用 \r 在真实终端底部刷新（直接写真实 console，不经 TeeTextWriter，
        // 因此绝不会进入报告文件）。输出被重定向（管道/文件）时自动关闭，避免污染。
        private sealed class ConsoleProgress
        {
            private readonly TextWriter _console;
            private readonly bool _enabled;
            private int _lastLen;
            private long _lastShownMs = -100000;          // 上次实际刷新时刻（Environment.TickCount64）
            private const long MinIntervalMs = 5000;       // 限频：两次刷新至少间隔 5s，避免刷进度拖慢测试

            public ConsoleProgress(TextWriter console)
            {
                _console = console;
                _enabled = !Console.IsOutputRedirected;
            }

            public void Show(string text)
            {
                if (!_enabled) return;
                long now = Environment.TickCount64;
                if (now - _lastShownMs < MinIntervalMs) return;   // 距上次刷新不足 5s 直接跳过
                _lastShownMs = now;
                int max = SafeWidth();
                if (text.Length > max) text = text.Substring(0, max);
                _console.Write('\r');
                _console.Write(text);
                if (_lastLen > text.Length) _console.Write(new string(' ', _lastLen - text.Length));
                _console.Write('\r');
                _console.Flush();
                _lastLen = text.Length;
            }

            public void Clear()
            {
                if (!_enabled || _lastLen == 0) return;
                _console.Write('\r');
                _console.Write(new string(' ', _lastLen));
                _console.Write('\r');
                _console.Flush();
                _lastLen = 0;
            }

            private static int SafeWidth()
            {
                try { int w = Console.WindowWidth; return w > 1 ? w - 1 : 120; }
                catch { return 120; }
            }
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
            string stamp = DateTime.Now.ToString("yyyyMMdd-HHmmss");
            string reportPath = Path.Combine(reportsDir, $"scen-{scopeTag}-{stamp}.txt");
            string failDir = Path.Combine(reportsDir, $"failures-{scopeTag}-{stamp}");   // 失败用例 JSON（惰性创建：全通过则不留目录）
            _failDumpCount = 0;

            var consoleOut = Console.Out;
            var fileOut = new StreamWriter(reportPath) { AutoFlush = true };
            Console.SetOut(new TeeTextWriter(consoleOut, fileOut));
            var progress = new ConsoleProgress(consoleOut);   // 仅刷新到真实终端，不写报告
            void Emit(string s) { progress.Clear(); Console.WriteLine(s); }

            // JPS.Core 未定义 JPS_CONCURRENT_CACHE 时共享缓存非线程安全 → 强制单线程，避免数据竞争。
            int threadCount = JpsBuildInfo.ConcurrentCache ? Math.Max(1, Environment.ProcessorCount / 2) : 1;

            // 原生 C 库（JPS.Native.dll）：可用则对每条用例额外做“C 版 JPS 路径 == C# 版 JPS 路径”强一致校验。
            bool nativeEnabled = NativeJps.TryInit(out string nativeInfo);

            Console.WriteLine($"# JPS / A* · MovingAI .scen 正确性报告   {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            Console.WriteLine($"构建配置（JPS.Core）：斜穿角={(JpsBuildInfo.CornerCutting ? "允许" : "禁止")}");
            string scope = string.IsNullOrEmpty(sub) ? "movingai/ 全部" : $"movingai/{sub}";
            Console.WriteLine($"范围：{scope}，共 {files.Length} 个 .scen 文件，每个 {(maxPerScen == 0 ? "全部用例" : $"最多 {maxPerScen} 例")}");
            Console.WriteLine(JpsBuildInfo.ConcurrentCache
                ? $"并行：每张图加载后用 {threadCount} 线程（CPU {Environment.ProcessorCount} 的 1/2）共享同一 JpsSystem 跑该图的用例（兼测 JPS 多线程安全）"
                : "并行：JPS.Core 未定义 JPS_CONCURRENT_CACHE（共享缓存非线程安全）→ 强制单线程");
            Console.WriteLine(nativeEnabled
                ? $"原生库：JPS.Native.dll 已加载（{nativeInfo}）→ 每条用例校验「C 版 JPS 路径 == C# 版 JPS 路径」"
                : $"原生库：未启用（{nativeInfo}）→ 跳过 C/C# 强一致校验（先构建 JPS.Native 的 x64 DLL）");
            Console.WriteLine();
            Console.WriteLine("校验项：① JPS整数代价==A*整数代价（最优性）  ② JPS路径合法（相邻/可走/不切角）  ③ JPS真实长度≈官方最优"
                + (nativeEnabled ? "  ④ C版JPS路径==C#版JPS路径（强一致）" : "")
                + "  ⑤ 随机改图+还原致缓存失效后，冷重扫==热结果");
            Console.WriteLine("列说明：n=用例数  pass=三项全过  jFail=JPS无解(A*有解)  subopt=JPS≠A*  inval=路径非法  refL=比官方长  refS=比官方短  mism=C≠C#  cold=失效重扫≠热结果");
            Console.WriteLine();
            Console.WriteLine($"{"scen",-44}{"n",8}{"pass",8}{"jFail",7}{"subopt",8}{"inval",7}{"refL",7}{"refS",7}{"mism",7}{"cold",7}");
            Console.WriteLine(new string('-', 108));

            bool allowCorner = JpsBuildInfo.CornerCutting;

            Stats total = default;
            var sw = Stopwatch.StartNew();

            int totalFiles = files.Length;
            var parOpt = new ParallelOptions { MaxDegreeOfParallelism = threadCount };
            for (int fi = 0; fi < files.Length; fi++)
            {
                var f = files[fi];
                string rel = Path.GetRelativePath(root, f).Replace('\\', '/');
                string name = rel.EndsWith(".scen", StringComparison.OrdinalIgnoreCase) ? rel[..^5] : rel;
                string scenDir = Path.GetDirectoryName(f)!;
                progress.Show($"[{fi + 1}/{totalFiles}] {Trunc(name, 44)}");

                List<Entry> entries;
                try { entries = ParseScen(f); }
                catch (Exception ex) { Emit($"{name,-44}  解析失败：{ex.Message}"); continue; }
                if (maxPerScen > 0 && entries.Count > maxPerScen) entries = entries.GetRange(0, maxPerScen);

                // 按 Map 字段分组（MovingAI 每个 .scen 几乎总是单张图，但稳妥起见按图分组）。
                // 对每张图：单线程加载并 Sync → 并行跑该图全部用例 → 用完即销毁其 system，**不缓存**任何 system。
                Stats scen = default;
                foreach (var grp in entries.GroupBy(e => e.Map, StringComparer.Ordinal))
                {
                    // 单线程加载该图并 Sync（满足“并行寻路前必须单线程 Sync 一次”的前提）。缺失/失败则跳过该图用例。
                    (GridMap map, JpsSystem sys) loaded;
                    try { loaded = LoadMap(scenDir, grp.Key, root); }
                    catch { continue; }
                    var mapEntries = grp.ToList();

                    // 原生侧与 C# 侧严格同构：单线程为这张图建一个原生 jps_system（灌阻挡 + Sync），
                    // 各线程的 NativePathfinder 共享它并行寻路；这张图测完即销毁（异常路径也销毁）。
                    NativeSystem? nsys = nativeEnabled ? new NativeSystem(loaded.map) : null;
                    var partial = new Stats[threadCount];
                    Stats coldStats = default;
                    try
                    {
                        // 把该图用例按步长切给 threadCount 个线程**并行**验证；所有线程共享同一 JpsSystem / 原生 system
                        // （地图 + 跳点缓存）——既跑正确性，也压测多线程安全。每线程持有自己的 Jps/AStar/NativePathfinder
                        //（搜索态线程私有）；并行期间地图只读、不再 Sync。
                        Parallel.For(0, threadCount, parOpt, t =>
                        {
                            var jps = new JpsPathfinder();
                            var astar = new AStarPathfinder();
                            var npf = nativeEnabled ? new NativePathfinder() : null;   // 每线程一个原生 pathfinder
                            try
                            {
                                Stats st = default;
                                // 步长划分：线程 t 取下标 t, t+threadCount, …（entry i 归线程 i%threadCount），不重不漏。
                                for (int i = t; i < mapEntries.Count; i += threadCount)
                                    ClassifyEntry(loaded.map, loaded.sys, jps, astar, nsys, npf, mapEntries[i], allowCorner, ref st, failDir, name);
                                partial[t] = st;
                            }
                            finally { npf?.Dispose(); }   // 本线程原生 pathfinder 用完即销毁（异常路径也走到）
                        });

                        // 冷缓存正确性（并行阶段之后**单线程**跑，改图会破坏“地图只读”前提，故不能并行）：
                        // 抽样若干用例，每例随机改若干格再还原 → Sync 使跳点缓存受影响行/列失效，
                        // 冷重扫结果须与热参考逐格一致（复用该图共享 system，用完即随本图销毁）。
                        coldStats = ColdCachePass(loaded.map, loaded.sys, nsys, mapEntries, failDir, name);
                    }
                    finally { nsys?.Dispose(); }   // 这张图测完即销毁原生 system（用完即销毁，不缓存）

                    for (int t = 0; t < threadCount; t++) scen.Merge(partial[t]);
                    scen.Merge(coldStats);
                }

                if (scen.N > 0)
                    Emit($"{Trunc(name, 44),-44}{scen.N,8}{scen.Pass,8}{scen.JFail,7}{scen.Subopt,8}{scen.Inval,7}{scen.RefL,7}{scen.RefS,7}{scen.Mism,7}{scen.Cold,7}");

                total.Merge(scen);
            }

            sw.Stop();
            progress.Clear();
            Console.WriteLine(new string('-', 108));
            Console.WriteLine($"{"合计",-44}{total.N,8}{total.Pass,8}{total.JFail,7}{total.Subopt,8}{total.Inval,7}{total.RefL,7}{total.RefS,7}{total.Mism,7}{total.Cold,7}");
            Console.WriteLine();
            long fails = total.JFail + total.Subopt + total.Inval + total.RefL + total.RefS;
            Console.WriteLine($"用例 {total.N}（平凡 {total.Trivial}，无效起终点 {total.BadCell} 已跳过），用时 {sw.Elapsed.TotalSeconds:F1}s");
            Console.WriteLine($"通过项中：精确匹配 {total.Exact}，整数度量舍入 {total.Artifact}（最大偏差 {total.MaxDevOk:E2}）");
            Console.WriteLine(fails == 0
                ? $"正确性：全部 {total.N} 例三项校验通过，✓ 通过"
                : $"正确性：⚠ {fails} 例未通过（jFail={total.JFail} subopt={total.Subopt} inval={total.Inval} refL={total.RefL} refS={total.RefS}，最大偏差 {total.WorstBad:F4}）");
            if (nativeEnabled)
                Console.WriteLine(total.Mism == 0
                    ? $"C/C# 强一致：全部 {total.N} 例 C 版 JPS 路径与 C# 版逐格一致，✓ 通过"
                    : $"C/C# 强一致：⚠ {total.Mism} 例 C 版与 C# 版 JPS 结果不一致（mism）");
            if (total.ColdN > 0)
                Console.WriteLine(total.Cold == 0
                    ? $"冷缓存正确性：抽测 {total.ColdN} 例，随机改图+还原致缓存失效后冷重扫与热结果逐格一致，✓ 通过"
                    : $"冷缓存正确性：⚠ {total.Cold} 例失效重扫结果与热结果不一致（cold）");

            long allFails = fails + total.Mism + total.Cold;   // 退出码同时反映正确性失败、C/C# 不一致与冷缓存失效 bug
            if (allFails > 0)
            {
                int saved = Math.Min(_failDumpCount, MaxFailDumps);
                Console.WriteLine($"失败用例地图已存为 JSON：{saved} 个 → {failDir}" +
                    (_failDumpCount > MaxFailDumps ? $"（达上限 {MaxFailDumps}，其余未存）" : "") +
                    "；可在 Playground 用「载入」打开复现。");
            }

            Console.Out.Flush();
            Console.SetOut(consoleOut);
            fileOut.Dispose();
            Console.WriteLine($"报告已保存：{reportPath}");
            return allFails == 0 ? 0 : 2;
        }

        // ---------------- 校验（线程私有 jps/astar，可并行）----------------

        // 每线程累计的统计量；并行后各线程结果通过 Merge 汇总（计数取和、偏差取最大，与顺序无关）。
        private struct Stats
        {
            public long N, Pass, JFail, Subopt, Inval, RefL, RefS, Trivial, BadCell, Exact, Artifact;
            public long Mism;        // C 版 JPS 与 C# 版 JPS 结果不一致的用例数（强一致校验）
            public long Cold, ColdN;  // 冷缓存：失效后寻路≠热结果的用例数 Cold（失败）/ 已抽测数 ColdN
            public double MaxDevOk;   // 通过项里的最大 |真实长度-官方|（应为舍入量级）
            public double WorstBad;   // 失败项里的最大 |偏差|

            public void Merge(in Stats o)
            {
                N += o.N; Pass += o.Pass; JFail += o.JFail; Subopt += o.Subopt; Inval += o.Inval;
                RefL += o.RefL; RefS += o.RefS; Trivial += o.Trivial; BadCell += o.BadCell;
                Exact += o.Exact; Artifact += o.Artifact; Mism += o.Mism; Cold += o.Cold; ColdN += o.ColdN;
                if (o.MaxDevOk > MaxDevOk) MaxDevOk = o.MaxDevOk;
                if (o.WorstBad > WorstBad) WorstBad = o.WorstBad;
            }
        }

        // 对单条用例做三项校验，结果累加进调用线程私有的 <paramref name="st"/>（无共享写，故并行安全）。
        // jps/astar 为线程私有实例；map/sys 为线程间共享只读（sys 已在并行前单线程 Sync 过）。
        private static void ClassifyEntry(
            GridMap map, JpsSystem sys, JpsPathfinder jps, AStarPathfinder astar,
            NativeSystem? nsys, NativePathfinder? npf,
            in Entry e, bool allowCorner, ref Stats st, string failDir, string scenName)
        {
            var s = (e.Sx, e.Sy);
            var g = (e.Gx, e.Gy);

            // 失败时把"地图+起终点"存成 JSON（局部函数：只捕获非 ref 的 map/s/g/failDir/scenName，不碰 ref st）。
            void Dump(string reason) => SaveFailureJson(map, s, g, reason, failDir, scenName);

            // 越界 / 起终点在阻挡上：场景/地图不匹配，单独计数，不进入算法校验
            if (!InBounds(map, s) || !InBounds(map, g) || !map.IsWalkable(s.Item1, s.Item2) || !map.IsWalkable(g.Item1, g.Item2))
            { st.BadCell++; return; }

            if (s == g) { st.Trivial++; return; }   // 平凡用例（首尾同格）跳过

            st.N++;
            var rj = jps.FindPath(sys, s, g);

            // ④ 强一致：C 版 JPS（DLL）必须与 C# 版 JPS 结果完全一致（有解性 + 逐格路径）。
            // 这是独立维度，与下面 A*/官方 校验互不影响；不一致单独计入 mism 并存盘。
            // 原生侧与 C# 侧同构：本线程私有的 npf 在本图共享的 nsys 上寻路。
            if (npf != null && nsys != null)
            {
                var (nok, npath) = npf.Find(nsys, e.Sx, e.Sy, e.Gx, e.Gy);
                if (nok != rj.Success || (rj.Success && !PathEqual(npath, rj.Path)))
                { st.Mism++; Dump("native"); }
            }

            var ra = astar.FindPath(map, s, g);

            // ① 最优性：与 A*（同度量 ground truth）整数代价必须一致
            if (rj.Success != ra.Success)
            {
                if (!rj.Success) { st.JFail++; Dump("jFail"); }    // A* 有解而 JPS 无解 → JPS 漏解
                else { st.Subopt++; Dump("subopt"); }              // JPS 有解而 A* 无解 → 异常（计入 subopt）
                return;
            }
            if (!rj.Success) { st.JFail++; Dump("jFail"); return; }   // 两者都无解（可解场景里属异常）

            var (jCard, jDiag) = CountSteps(rj.Path);
            var (aCard, aDiag) = CountSteps(ra.Path);
            long jInt = jCard * 1000L + jDiag * 1414L;
            long aInt = aCard * 1000L + aDiag * 1414L;
            if (jInt != aInt) { st.Subopt++; st.WorstBad = Math.Max(st.WorstBad, Math.Abs(jInt - aInt) / 1000.0); Dump("subopt"); return; }

            // ② 路径合法性
            if (!ValidPath(map, rj.Path, s, g, allowCorner, out _)) { st.Inval++; Dump("inval"); return; }

            // ③ 与官方最优比对（真实 octile 长度）
            double jLen = jCard + Math.Sqrt(2) * jDiag;
            double dev = jLen - e.Optimal;
            if (dev > RefTol) { st.RefL++; st.WorstBad = Math.Max(st.WorstBad, dev); Dump("refL"); return; }
            if (dev < -RefTol) { st.RefS++; st.WorstBad = Math.Max(st.WorstBad, -dev); Dump("refS"); return; }

            double ad = Math.Abs(dev);
            if (ad <= 1e-6) st.Exact++; else st.Artifact++;
            st.MaxDevOk = Math.Max(st.MaxDevOk, ad);
            st.Pass++;
        }

        // ---------------- 冷缓存正确性（单线程；改图会破坏“地图只读”，不能与并行阶段同跑）----------------
        //
        // 对抽样用例：随机改若干格阻挡再**还原**（地图净不变，但 Version/行列版本跳变）→ Sync 令跳点缓存
        // 受影响行/列失效 → 冷重扫。重扫结果必须与“热参考”逐格一致（热参考即并行阶段已对过 A*/官方最优的那次搜索，
        // 同图同询问确定性相同）。这样能抓住失效/重扫机制自身的 bug（例如重扫回写越界污染邻格、行列失效漏标等），
        // 而这些在“缓存一直热”的主校验里不会触发。C# 与 C 两版各自比对自己的热参考。
        private static Stats ColdCachePass(
            GridMap map, JpsSystem sys, NativeSystem? nsys, List<Entry> entries, string failDir, string scenName)
        {
            Stats st = default;
            var jps = new JpsPathfinder();
            NativePathfinder? npf = nsys != null ? new NativePathfinder() : null;
            var rng = new Random(0x00C01D + entries.Count);   // 与图规模挂钩、可复现
            var seen = new int[map.Width * map.Height];        // 去重戳：seen[id]==stamp 表示本次已选过该格
            int stamp = 0;
            var edits = new List<(int x, int y, bool old)>();
            // 每次改动的格数：随机撒到 ~W+H 个，令多数行/列被触及（冷重扫覆盖搜索用到的线）。
            int editCount = Math.Min(map.Width * map.Height - 2, map.Width + map.Height);

            try
            {
                int step = Math.Max(1, entries.Count / ColdSampleCap);
                for (int i = 0; i < entries.Count; i += step)
                {
                    var e = entries[i];
                    var s = (e.Sx, e.Sy);
                    var g = (e.Gx, e.Gy);
                    if (!InBounds(map, s) || !InBounds(map, g) ||
                        !map.IsWalkable(s.Item1, s.Item2) || !map.IsWalkable(g.Item1, g.Item2) || s == g)
                        continue;

                    // 热参考（当前缓存；等价于并行阶段已验证过的结果，同图同询问确定性一致）。
                    var warm = jps.FindPath(sys, s, g);
                    (bool ok, List<(int X, int Y)>? path) nwarm = (false, null);
                    if (npf != null) nwarm = npf.Find(nsys!, e.Sx, e.Sy, e.Gx, e.Gy);

                    // 选 editCount 个互异格（避开起终点），先改到相反态、再还原 → 地图净不变，缓存受影响行/列失效。
                    edits.Clear();
                    stamp++;
                    int attempts = 0, maxAttempts = editCount * 8 + 64;
                    while (edits.Count < editCount && attempts++ < maxAttempts)
                    {
                        int x = rng.Next(map.Width), y = rng.Next(map.Height);
                        if ((x == s.Item1 && y == s.Item2) || (x == g.Item1 && y == g.Item2)) continue;
                        int id = y * map.Width + x;
                        if (seen[id] == stamp) continue;
                        seen[id] = stamp;
                        edits.Add((x, y, map.IsBlocked(x, y)));
                    }
                    foreach (var ed in edits) { map.SetBlocked(ed.x, ed.y, !ed.old); nsys?.SetBlocked(ed.x, ed.y, !ed.old); }
                    foreach (var ed in edits) { map.SetBlocked(ed.x, ed.y, ed.old); nsys?.SetBlocked(ed.x, ed.y, ed.old); }
                    sys.Sync();
                    nsys?.Sync();

                    st.ColdN++;

                    // C# 冷重扫 == C# 热参考（逐格）
                    var cold = jps.FindPath(sys, s, g);
                    if (cold.Success != warm.Success || (warm.Success && !PathEqual(cold.Path, warm.Path)))
                    { st.Cold++; SaveFailureJson(map, s, g, "cold-cs", failDir, scenName); }

                    // C 冷重扫 == C 热参考（逐格）——直接验证原生 SoA/SIMD 回写在失效重扫下是否正确
                    if (npf != null)
                    {
                        var (cok, cpath) = npf.Find(nsys!, e.Sx, e.Sy, e.Gx, e.Gy);
                        if (cok != nwarm.ok || (nwarm.ok && !PathEqual(cpath, nwarm.path!)))
                        { st.Cold++; SaveFailureJson(map, s, g, "cold-c", failDir, scenName); }
                    }
                }
            }
            finally { npf?.Dispose(); }

            return st;
        }

        // 把失败用例的地图（位图→阻挡点列表）+ 起终点存成 Playground 可载入的 MapData JSON。
        // 线程安全：唯一文件名（无共享文件）、Interlocked 限量、目录惰性创建；保存异常被吞掉，绝不影响测试。
        private static void SaveFailureJson(
            GridMap map, (int X, int Y) s, (int X, int Y) g, string reason, string failDir, string scenName)
        {
            int n = Interlocked.Increment(ref _failDumpCount);
            if (n > MaxFailDumps) return;   // 超上限只计数、不再写盘

            var data = new MapData
            {
                Width = map.Width,
                Height = map.Height,
                Start = new PointData { X = s.X, Y = s.Y },
                End = new PointData { X = g.X, Y = g.Y },
            };
            for (int y = 0; y < map.Height; y++)
                for (int x = 0; x < map.Width; x++)
                    if (map.IsBlocked(x, y))
                        data.Obstacles.Add(new PointData { X = x, Y = y });

            try
            {
                Directory.CreateDirectory(failDir);   // 惰性：首个失败时才建目录
                string tag = scenName.Replace('/', '-').Replace('\\', '-');
                string file = Path.Combine(failDir, $"{n:D4}-{tag}-{s.X}_{s.Y}-to-{g.X}_{g.Y}-{reason}.json");
                File.WriteAllText(file, JsonSerializer.Serialize(data));
            }
            catch { /* 存盘失败不影响正确性测试本身 */ }
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

        // 加载一张图并 Sync（**不缓存**：每张图用完即由调用方丢弃/销毁其 system）。
        private static (GridMap map, JpsSystem sys) LoadMap(string scenDir, string mapField, string root)
        {
            string p = Path.Combine(scenDir, mapField);
            if (!File.Exists(p))
            {
                // 兜底：按文件名在 movingai/ 下递归找
                var found = Directory.GetFiles(root, Path.GetFileName(mapField), SearchOption.AllDirectories);
                if (found.Length == 0) throw new FileNotFoundException(mapField);
                p = found[0];
            }

            var map = MovingAiMap.Parse(File.ReadAllText(Path.GetFullPath(p)));
            var sys = new JpsSystem(map);
            sys.Sync();
            return (map, sys);
        }

        private static bool InBounds(GridMap map, (int X, int Y) p) =>
            p.X >= 0 && p.Y >= 0 && p.X < map.Width && p.Y < map.Height;

        // C 版与 C# 版 JPS 路径逐格相等判定（强一致校验用）。
        private static bool PathEqual(List<(int X, int Y)>? a, List<(int X, int Y)> b)
        {
            if (a is null || a.Count != b.Count) return false;
            for (int i = 0; i < a.Count; i++)
                if (a[i].X != b[i].X || a[i].Y != b[i].Y) return false;
            return true;
        }

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
