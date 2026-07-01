/*
 * Benchmark.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
using JPS.Data;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS.Benchmark
{
    /// <summary>
    /// 命令行合并基准（唯一模式）：
    ///   dotnet run -- [每图随机样本数=30] [子目录]
    ///
    /// 先归并去重所有 .scen；再对每张被引用的地图（每张只解析一次）依次做「随机投点(rand)」与「scen」两段。
    /// 每段都测：JPS(C#) 与 JPS(C) 的**冷/热**路径耗时、A* 耗时，并给出 A*/C 的冷、热加速比。
    ///   冷路径 = 每次寻路前翻转一个可走格并 Sync，使跳点缓存整表失效（强制每次重新跑跳点扫描）；
    ///   热路径 = 缓存复用（Sync 一次后连续寻路）。
    /// 不校验路径正确性（由 JPS.Accuracy 负责）。
    /// </summary>
    internal static class Benchmark
    {
        static int Main(string[] args)
        {
            // 唯一模式 combo：容忍首参写成 "combo"；其余参数为 [每图随机样本数] [子目录]。
            var a = args;
            if (a.Length > 0 && string.Equals(a[0], "combo", StringComparison.OrdinalIgnoreCase)) a = a[1..];
            int q = a.Length >= 1 && int.TryParse(a[0], out int v) ? v : 30;
            string? sub = a.Length >= 2 ? a[1] : null;
            ComboBench(q, sub);
            return 0;
        }

        // 由 JPS.Core 的编译期常量组织出一行构建配置摘要。
        private static string BuildConfig() =>
            $"斜穿角={(JpsBuildInfo.CornerCutting ? "允许" : "禁止")}，" +
            $"多线程共享缓存={(JpsBuildInfo.ConcurrentCache ? "开启" : "关闭")}";

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

        // 进度行：只用 \r 在真实终端底部刷新（直接写真实 console，不经 TeeTextWriter，
        // 因此绝不会进入报告文件）。输出被重定向（管道/文件）时自动关闭，避免污染。
        private sealed class ConsoleProgress
        {
            private readonly TextWriter _console;
            private readonly bool _enabled;
            private int _lastLen;
            private long _lastShownMs = -100000;
            private const long MinIntervalMs = 5000;

            public ConsoleProgress(TextWriter console)
            {
                _console = console;
                _enabled = !Console.IsOutputRedirected;
            }

            public void Show(string text)
            {
                if (!_enabled) return;
                long now = Environment.TickCount64;
                if (now - _lastShownMs < MinIntervalMs) return;
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

        // 按“地图”归并所有 .scen 用例：每张 .map 只被 Parse 一次；重复起终点对去重（每对只跑一次）。
        private sealed class ScenGroup
        {
            public readonly string MapPath;
            public int ScenCount;
            public readonly HashSet<(int sx, int sy, int gx, int gy)> Pairs = new HashSet<(int, int, int, int)>();
            public ScenGroup(string mapPath) { MapPath = mapPath; }
        }

        // 只读所有 .scen（不 Parse 任何 .map），按“地图实际路径”归并用例并去重。
        private static (Dictionary<string, ScenGroup> groups, long totalEntries, int scenFileCount) CollectScenGroups(string dir, string root)
        {
            var groups = new Dictionary<string, ScenGroup>(StringComparer.OrdinalIgnoreCase);
            long totalEntries = 0;
            int scenFileCount = 0;
            foreach (var f in Directory.GetFiles(dir, "*.scen", SearchOption.AllDirectories))
            {
                string scenDir = Path.GetDirectoryName(f)!;
                string? mapField = null;
                var pairs = new List<(int sx, int sy, int gx, int gy)>();
                foreach (var raw in File.ReadLines(f))
                {
                    string line = raw.Trim();
                    if (line.Length == 0 || line.StartsWith("version", StringComparison.OrdinalIgnoreCase)) continue;
                    var t = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                    if (t.Length < 9) continue;
                    mapField ??= t[1];
                    pairs.Add((int.Parse(t[4]), int.Parse(t[5]), int.Parse(t[6]), int.Parse(t[7])));
                }
                if (mapField == null || pairs.Count == 0) continue;

                string? mapPath = ResolveMapPath(scenDir, mapField, root);
                if (mapPath == null) continue;

                if (!groups.TryGetValue(mapPath, out var g)) { g = new ScenGroup(mapPath); groups[mapPath] = g; }
                g.ScenCount++;
                foreach (var p in pairs) g.Pairs.Add(p);
                totalEntries += pairs.Count;
                scenFileCount++;
            }
            return (groups, totalEntries, scenFileCount);
        }

        private static string? ResolveMapPath(string scenDir, string mapField, string root)
        {
            string p = Path.Combine(scenDir, mapField);
            if (!File.Exists(p))
            {
                var found = Directory.GetFiles(root, Path.GetFileName(mapField), SearchOption.AllDirectories);
                if (found.Length == 0) return null;
                p = found[0];
            }
            return Path.GetFullPath(p);
        }

        private static string Trunc(string s, int max) => s.Length <= max ? s : s.Substring(0, max - 1) + "…";

        // ANSI 前景色包裹（绿=更快，红=更慢），仅用于控制台着色。
        private static string Green(string s) => "\x1b[32m" + s + "\x1b[0m";
        private static string Red(string s) => "\x1b[31m" + s + "\x1b[0m";

#pragma warning disable SYSLIB1054   // 一处控制台着色，用经典 DllImport，不为此把整类改成 partial
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern IntPtr GetStdHandle(int nStdHandle);
        [DllImport("kernel32.dll")]
        private static extern bool GetConsoleMode(IntPtr hConsoleHandle, out uint lpMode);
        [DllImport("kernel32.dll")]
        private static extern bool SetConsoleMode(IntPtr hConsoleHandle, uint dwMode);
#pragma warning restore SYSLIB1054

        // 在 Windows 旧版 conhost 上启用 ANSI 转义（Windows Terminal 默认已启用）；失败则忽略。
        private static void EnableVirtualTerminal()
        {
            try
            {
                IntPtr h = GetStdHandle(-11);          // STD_OUTPUT_HANDLE
                if (GetConsoleMode(h, out uint mode))
                    SetConsoleMode(h, mode | 0x0004);  // ENABLE_VIRTUAL_TERMINAL_PROCESSING
            }
            catch { /* 非 Windows 或调用失败：忽略，退化为无色 */ }
        }

        // 用一张小合成图跑少量查询，触发 JIT 编译（不计入正式计时，避免首图被编译开销污染）。
        private static void WarmupJit()
        {
            var m = new GridMap(64, 64);
            var rng = new Random(1);
            for (int i = 0; i < 300; i++) m.SetBlocked(rng.Next(64), rng.Next(64), true);
            var sys = new JpsSystem(m);
            sys.Sync();
            var j = new JpsPathfinder();
            var a = new AStarPathfinder();
            for (int i = 0; i < 100; i++)
            {
                var s = (rng.Next(64), rng.Next(64));
                var gg = (rng.Next(64), rng.Next(64));
                j.FindPath(sys, s, gg);
                a.FindPath(m, s, gg);
            }
        }

        // ============ 合并基准：随机投点 + .scen，测 JPS(C#/C) 冷/热 与 A* ============
        private static void ComboBench(int q, string? sub)
        {
            string root = FindDir("movingai");
            string dir = string.IsNullOrEmpty(sub) ? root : Path.Combine(root, sub!);
            if (!Directory.Exists(dir)) { Console.WriteLine($"目录不存在：{dir}"); return; }

            var (groups, totalEntries, scenFileCount) = CollectScenGroups(dir, root);
            if (groups.Count == 0) { Console.WriteLine($"未找到可用 .scen 文件：{dir}"); return; }

            string repoRoot = Directory.GetParent(root)?.FullName ?? root;
            string reportsDir = Path.Combine(repoRoot, "benchmark-results");
            Directory.CreateDirectory(reportsDir);
            string scopeTag = string.IsNullOrEmpty(sub) ? "all" : sub!.Replace('/', '-').Replace('\\', '-');
            string reportPath = Path.Combine(reportsDir, $"combo-{scopeTag}-q{q}-{DateTime.Now:yyyyMMdd-HHmmss}.txt");
            var consoleOut = Console.Out;
            var fileOut = new StreamWriter(reportPath) { AutoFlush = true };
            Console.SetOut(new TeeTextWriter(consoleOut, fileOut));
            var progress = new ConsoleProgress(consoleOut);
            void Emit(string s) { progress.Clear(); Console.WriteLine(s); }

            bool nativeEnabled = NativeJps.TryInit(out string nativeInfo);

            // 行着色：ANSI 颜色码只写真实控制台，报告文件仍写纯文本（避免转义码污染报告）。
            bool useColor = !Console.IsOutputRedirected;
            if (useColor && OperatingSystem.IsWindows()) EnableVirtualTerminal();
            void EmitRow(string plain, string colored)
            {
                progress.Clear();
                fileOut.WriteLine(plain);
                consoleOut.WriteLine(useColor ? colored : plain);
            }

            string scope = string.IsNullOrEmpty(sub) ? "movingai/ 全部" : $"movingai/{sub}";
            long dedup = groups.Values.Sum(x => (long)x.Pairs.Count);
            Console.WriteLine($"# JPS(C#/C) 冷·热路径 vs A* · 随机投点 + .scen 合并基准   {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            Console.WriteLine($"构建配置（JPS.Core）：{BuildConfig()}");
            Console.WriteLine($"原生库：{(nativeEnabled ? $"JPS.Native.dll 已加载（{nativeInfo}）→ 同时测 C 版" : $"未启用（{nativeInfo}）→ 仅测 C# 版")}");
            Console.WriteLine($"范围：{scope}，{scenFileCount} 个 .scen / {groups.Count} 张地图；随机投点每图目标 {q} 组，scen 去重后用例 {dedup}（原始 {totalEntries}）");
            Console.WriteLine("流程：每张图只解析一次 → 先随机投点(rand)、后 scen 两段；每段测 C#/C 冷/热与 A* 耗时（多轮取最小）。");
            Console.WriteLine("冷=每次寻路前翻转可走格+Sync 使跳点缓存整表失效（重新扫描）；热=缓存复用。不校验路径（由 JPS.Accuracy 负责）。");
            Console.WriteLine();
            Console.WriteLine("列说明：");
            Console.WriteLine("  map           地图名");
            Console.WriteLine("  size          地图尺寸（宽 x 高）");
            Console.WriteLine("  tag           段：rand=随机投点 / scen=场景用例");
            Console.WriteLine("  pairs         用例对数");
            Console.WriteLine("  JPSexp/A*exp  平均每次寻路的扩展节点数（JPS / A*）");
            if (nativeEnabled)
            {
                Console.WriteLine("  cC# / cC      C# / C 版 JPS 冷路径平均耗时（微秒/次）");
                Console.WriteLine("  wC# / wC      C# / C 版 JPS 热路径平均耗时（微秒/次）");
                Console.WriteLine("  着色：cC/wC 比同类 C# 快=绿 慢=红；四列(cC#/cC/wC#/wC)比 A* 慢=红（红优先）");
            }
            else
            {
                Console.WriteLine("  cC# / wC#     C# 版 JPS 冷 / 热路径平均耗时（微秒/次）");
                Console.WriteLine("  着色(cC#/wC#)：红=比 A* 慢");
            }
            Console.WriteLine("  A*us          A* 平均每次寻路耗时（微秒/次）");
            Console.WriteLine(nativeEnabled
                ? "  A*/cC A*/wC   A*us ÷ C 版冷、热耗时（越大表示 C 版 JPS 相对 A* 越快）"
                : "  A*/cC# A*/wC# A*us ÷ C# 版冷、热耗时");
            Console.WriteLine();
            string hdr = nativeEnabled
                ? $"{"map",-30}{"size",11}{"tag",6}{"pairs",8}{"JPSexp",8}{"A*exp",8}{"cC#",9}{"cC",9}{"wC#",9}{"wC",9}{"A*us",9}{"A*/cC",8}{"A*/wC",8}"
                : $"{"map",-30}{"size",11}{"tag",6}{"pairs",8}{"JPSexp",8}{"A*exp",8}{"cC#",9}{"wC#",9}{"A*us",9}{"A*/cC#",8}{"A*/wC#",8}";
            int rule = nativeEnabled ? 132 : 114;
            Console.WriteLine(hdr);
            Console.WriteLine(new string('-', rule));

            WarmupJit();

            // 累计（各图“多轮最小”之和），rand 与 scen 分开
            double rCJ = 0, rWJ = 0, rCN = 0, rWN = 0, rA = 0; long rP = 0;
            double sCJ = 0, sWJ = 0, sCN = 0, sWN = 0, sA = 0; long sP = 0;
            var sw = new Stopwatch();

            int total = groups.Count, gi = 0;
            foreach (var kv in groups.OrderBy(k => Path.GetRelativePath(root, k.Key), StringComparer.OrdinalIgnoreCase))
            {
                gi++;
                progress.Show($"[{gi}/{total}] {Path.GetRelativePath(root, kv.Key).Replace('\\', '/')}");
                var grp = kv.Value;
                GridMap map;
                try { map = MovingAiMap.Parse(File.ReadAllText(grp.MapPath)); }
                catch { continue; }

                string rel = Path.GetRelativePath(root, grp.MapPath).Replace('\\', '/');
                string name = rel.EndsWith(".map", StringComparison.OrdinalIgnoreCase) ? rel.Substring(0, rel.Length - 4) : rel;

                var walk = new List<(int X, int Y)>();
                for (int y = 0; y < map.Height; y++)
                    for (int x = 0; x < map.Width; x++)
                        if (map.IsWalkable(x, y)) walk.Add((x, y));
                if (walk.Count < 2) continue;

                var system = new JpsSystem(map);
                system.Sync();
                var jps = new JpsPathfinder();
                var astar = new AStarPathfinder();
                using var nat = nativeEnabled ? new NativeMap(map) : null;

                var dpt = walk[0];   // 翻转用可走格（翻 true 再 false → 回原状、仅令 Version 跳变）
                void ForceDirtyCs() { map.SetBlocked(dpt.X, dpt.Y, true); map.SetBlocked(dpt.X, dpt.Y, false); system.Sync(); }

                // 测一段：C# 冷/热、C 冷/热、A*（多轮取最小），输出一行，返回各项 ms 与对数。
                (double cj, double wj, double cn, double wn, double a, int n) RunSegment(string tag, List<(int sx, int sy, int gx, int gy)> pairs)
                {
                    int n = pairs.Count;
                    if (n == 0) return (0, 0, 0, 0, 0, 0);

                    // 展开节点（非计时遍；JPS 顺便预热，冷路径每次 ForceDirty 不受影响）
                    long jExp = 0, aExp = 0;
                    foreach (var p in pairs)
                    {
                        jExp += jps.FindPath(system, (p.sx, p.sy), (p.gx, p.gy)).ExpandedNodes;
                        aExp += astar.FindPath(map, (p.sx, p.sy), (p.gx, p.gy)).ExpandedNodes;
                    }

                    double cj = double.MaxValue, wj = double.MaxValue, a = double.MaxValue;
                    double cn = nat != null ? double.MaxValue : 0, wn = nat != null ? double.MaxValue : 0;
                    for (int rep = 0; rep < 3; rep++)
                    {
                        // C# 冷（每次前 ForceDirty）
                        progress.Show($"[{gi}/{total}] {name} {tag} C#冷 {rep + 1}/3");
                        GC.Collect(); GC.WaitForPendingFinalizers();
                        sw.Restart();
                        foreach (var p in pairs) { ForceDirtyCs(); jps.FindPath(system, (p.sx, p.sy), (p.gx, p.gy)); }
                        sw.Stop(); cj = Math.Min(cj, sw.Elapsed.TotalMilliseconds);

                        // C# 热（Sync 一次后复用）
                        system.Sync();
                        GC.Collect(); GC.WaitForPendingFinalizers();
                        sw.Restart();
                        foreach (var p in pairs) jps.FindPath(system, (p.sx, p.sy), (p.gx, p.gy));
                        sw.Stop(); wj = Math.Min(wj, sw.Elapsed.TotalMilliseconds);

                        if (nat != null)
                        {
                            // C 冷
                            progress.Show($"[{gi}/{total}] {name} {tag} C冷 {rep + 1}/3");
                            GC.Collect(); GC.WaitForPendingFinalizers();
                            sw.Restart();
                            foreach (var p in pairs) { nat.ForceDirty(dpt.X, dpt.Y); nat.Find(p.sx, p.sy, p.gx, p.gy); }
                            sw.Stop(); cn = Math.Min(cn, sw.Elapsed.TotalMilliseconds);

                            // C 热
                            nat.Sync();
                            GC.Collect(); GC.WaitForPendingFinalizers();
                            sw.Restart();
                            foreach (var p in pairs) nat.Find(p.sx, p.sy, p.gx, p.gy);
                            sw.Stop(); wn = Math.Min(wn, sw.Elapsed.TotalMilliseconds);
                        }

                        // A*（无缓存，冷热一致）
                        progress.Show($"[{gi}/{total}] {name} {tag} A* {rep + 1}/3");
                        GC.Collect(); GC.WaitForPendingFinalizers();
                        sw.Restart();
                        foreach (var p in pairs) astar.FindPath(map, (p.sx, p.sy), (p.gx, p.gy));
                        sw.Stop(); a = Math.Min(a, sw.Elapsed.TotalMilliseconds);
                    }

                    double cjus = cj / n * 1000, wjus = wj / n * 1000, aus = a / n * 1000;
                    string size = $"{map.Width}x{map.Height}";
                    string hd = $"{Trunc(name, 30),-30}{size,11}{tag,6}{n,8}{jExp / n,8}{aExp / n,8}";
                    if (nat != null)
                    {
                        double cnus = cn / n * 1000, wnus = wn / n * 1000;
                        string cjStr = $"{cjus,9:F2}", cnStr = $"{cnus,9:F2}", wjStr = $"{wjus,9:F2}", wnStr = $"{wnus,9:F2}";
                        // 优先：四列比 A* 慢=红。否则 cC/wC 比同类 C# 快=绿、慢=红；cC#/wC# 无相对对比。
                        string cjCol = cjus > aus ? Red(cjStr) : cjStr;
                        string cnCol = cnus > aus ? Red(cnStr) : cnus < cjus ? Green(cnStr) : cnus > cjus ? Red(cnStr) : cnStr;
                        string wjCol = wjus > aus ? Red(wjStr) : wjStr;
                        string wnCol = wnus > aus ? Red(wnStr) : wnus < wjus ? Green(wnStr) : wnus > wjus ? Red(wnStr) : wnStr;
                        string tail = $"{aus,9:F2}{aus / Math.Max(0.001, cnus),8:F1}{aus / Math.Max(0.001, wnus),8:F1}";
                        EmitRow(hd + cjStr + cnStr + wjStr + wnStr + tail,   // 顺序：cC# cC wC# wC
                                hd + cjCol + cnCol + wjCol + wnCol + tail);
                    }
                    else
                    {
                        string cjStr = $"{cjus,9:F2}", wjStr = $"{wjus,9:F2}";
                        string cjCol = cjus > aus ? Red(cjStr) : cjStr;
                        string wjCol = wjus > aus ? Red(wjStr) : wjStr;
                        string tail = $"{aus,9:F2}{aus / Math.Max(0.001, cjus),8:F1}{aus / Math.Max(0.001, wjus),8:F1}";
                        EmitRow(hd + cjStr + wjStr + tail, hd + cjCol + wjCol + tail);
                    }
                    return (cj, wj, cn, wn, a, n);
                }

                // ① 随机投点：采样 q 组可解起终点（此处会预热 C# 缓存，计时段每次都 ForceDirty，故无影响）
                var randPairs = new List<(int sx, int sy, int gx, int gy)>(q);
                var rng = new Random(12345);
                int maxAtt = q * 40 + 500;
                for (int at = 0; at < maxAtt && randPairs.Count < q; at++)
                {
                    var s = walk[rng.Next(walk.Count)];
                    var g = walk[rng.Next(walk.Count)];
                    if (s.Equals(g)) continue;
                    if (jps.FindPath(system, s, g).Success) randPairs.Add((s.X, s.Y, g.X, g.Y));
                }
                var r = RunSegment("rand", randPairs);
                rCJ += r.cj; rWJ += r.wj; rCN += r.cn; rWN += r.wn; rA += r.a; rP += r.n;

                // ② scen：去重后有效对
                var scenPairs = new List<(int sx, int sy, int gx, int gy)>(grp.Pairs.Count);
                foreach (var p in grp.Pairs)
                {
                    if (p.sx < 0 || p.sy < 0 || p.gx < 0 || p.gy < 0) continue;
                    if (p.sx >= map.Width || p.sy >= map.Height || p.gx >= map.Width || p.gy >= map.Height) continue;
                    if (!map.IsWalkable(p.sx, p.sy) || !map.IsWalkable(p.gx, p.gy)) continue;
                    scenPairs.Add(p);
                }
                var ss = RunSegment("scen", scenPairs);
                sCJ += ss.cj; sWJ += ss.wj; sCN += ss.cn; sWN += ss.wn; sA += ss.a; sP += ss.n;
            }

            progress.Clear();
            Console.WriteLine(new string('-', rule));

            void Summary(string tag, double cj, double wj, double cn, double wn, double a, long p)
            {
                if (p == 0) return;
                if (nativeEnabled)
                    Console.WriteLine($"[{tag}] {p} 组：C# 冷 {cj:F0}ms/热 {wj:F0}ms  C 冷 {cn:F0}ms/热 {wn:F0}ms  A* {a:F0}ms；" +
                        $"A*/C 冷={a / Math.Max(0.001, cn):F1}x 热={a / Math.Max(0.001, wn):F1}x；C#/C 冷={cj / Math.Max(0.001, cn):F2}x 热={wj / Math.Max(0.001, wn):F2}x");
                else
                    Console.WriteLine($"[{tag}] {p} 组：C# 冷 {cj:F0}ms/热 {wj:F0}ms  A* {a:F0}ms；" +
                        $"A*/C# 冷={a / Math.Max(0.001, cj):F1}x 热={a / Math.Max(0.001, wj):F1}x");
            }
            Summary("rand", rCJ, rWJ, rCN, rWN, rA, rP);
            Summary("scen", sCJ, sWJ, sCN, sWN, sA, sP);

            Console.Out.Flush();
            Console.SetOut(consoleOut);
            fileOut.Dispose();
            Console.WriteLine($"报告已保存：{reportPath}");
        }
    }
}
