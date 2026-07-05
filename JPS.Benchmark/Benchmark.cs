/*
 * Benchmark.cs
 * JPS Pathfinding
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
using System.Collections.Concurrent;
using JPS.Data;
using JPS.Models;
using JPS.Pathfinding;

namespace JPS.Benchmark
{
    internal static class Benchmark
    {
        private const int ResultHeaderRepeat = 50;
        private const int Repeats = 3;


        // Pin the calling thread to one logical processor. Cross-platform via runtime OS dispatch:
        //   Windows -> SetThreadAffinityMask, Linux -> sched_setaffinity, macOS/other -> no portable per-core pinning.
        // DllImports bind lazily on first call, so the library for an OS we don't run on is never loaded; the
        // OperatingSystem.IsXxx() guards ensure each native call only happens on its own platform. A 64-bit mask
        // covers CPUs 0..63; CPU indices beyond that (or nonexistent cores) simply get no pinning.
#pragma warning disable SYSLIB1054
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern IntPtr GetCurrentThread();
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern UIntPtr SetThreadAffinityMask(IntPtr hThread, UIntPtr dwThreadAffinityMask);

        // Linux: sched_setaffinity(pid = 0 -> the calling thread, cpusetsize in bytes, mask). Returns 0 on success.
        [DllImport("libc", SetLastError = true)]
        private static extern int sched_setaffinity(int pid, IntPtr cpusetsize, ref ulong mask);
#pragma warning restore SYSLIB1054

        private static bool TrySetCurrentThreadAffinity(int cpuIndex)
        {
            if (cpuIndex < 0 || cpuIndex >= 64)
                return false;
            ulong mask = 1UL << cpuIndex;
            try
            {
                if (OperatingSystem.IsWindows())
                    return SetThreadAffinityMask(GetCurrentThread(), new UIntPtr(mask)) != UIntPtr.Zero;
                if (OperatingSystem.IsLinux())
                    return sched_setaffinity(0, (IntPtr)sizeof(ulong), ref mask) == 0;
            }
            catch { /* API missing / call blocked -> fall through to no-op */ }
            return false;   // macOS / other: no portable per-core pinning
        }

        // Usage: combo [q] [subdir] [--cpus LIST]
        //   q         rand pairs per map (first bare integer; default 30)
        //   subdir    movingai/ subdirectory to limit the run (first bare non-integer)
        //   --cpus/-c explicit logical CPUs to run on; the count decides the number of worker threads,
        //             and each worker is pinned (affinity) to its CPU. Accepts indices and ranges,
        //             e.g. --cpus 0,2,4,6  or  --cpus 3-9  or  --cpus 0-3,8,10-12.
        //             When omitted, defaults to half the logical CPUs (every other one).
        static int Main(string[] args)
        {
            var rest = new List<string>(args);
            if (rest.Count > 0 && string.Equals(rest[0], "combo", StringComparison.OrdinalIgnoreCase))
                rest.RemoveAt(0);

            // Extract the --cpus / -c option (supports "--cpus 0,2,4", "--cpus=0,2,4", "-c 0-7").
            List<int>? cpus = null;
            for (int i = 0; i < rest.Count; i++)
            {
                string tok = rest[i];
                string? val = null;
                if (tok.StartsWith("--cpus=", StringComparison.OrdinalIgnoreCase)) val = tok["--cpus=".Length..];
                else if (tok.StartsWith("-c=", StringComparison.OrdinalIgnoreCase)) val = tok["-c=".Length..];
                else if (string.Equals(tok, "--cpus", StringComparison.OrdinalIgnoreCase) || string.Equals(tok, "-c", StringComparison.OrdinalIgnoreCase))
                {
                    if (i + 1 < rest.Count) { val = rest[i + 1]; rest.RemoveAt(i + 1); }
                }
                if (val == null)
                    continue;
                cpus = ParseCpuList(val);
                if (cpus == null)
                {
                    Console.WriteLine($"Invalid --cpus value: '{val}'. Use e.g. --cpus 0,2,4,6 or --cpus 3-9.");
                    return 2;
                }
                rest.RemoveAt(i);
                i--;
            }

            // Remaining positionals: first integer = q; first non-integer = movingai/ subdir.
            int q = 30;
            bool qSet = false;
            string? sub = null;
            foreach (var tok in rest)
            {
                if (!qSet && int.TryParse(tok, out int v)) { q = v; qSet = true; }
                else sub ??= tok;
            }

            cpus ??= DefaultCpuList();   // 未指定 → 默认用一半逻辑 CPU（每隔一个取，共 N/2 个）

            int nproc = Environment.ProcessorCount;
            if (cpus.Exists(c => c >= nproc))
                Console.WriteLine($"Warning: some --cpus entries are >= logical CPU count ({nproc}); affinity for those is a no-op.");

            ComboBench(q, sub, cpus);
            return 0;
        }

        // Default worker CPUs: every other logical CPU (0,2,4,...), i.e. half of them — spreads across
        // physical cores on typical SMT-2 topology. Override explicitly with --cpus.
        private static List<int> DefaultCpuList()
        {
            int n = Math.Max(1, Environment.ProcessorCount);
            int half = Math.Max(1, n / 2);
            var list = new List<int>(half);
            for (int i = 0; i < half; i++)
                list.Add(i * 2);
            return list;
        }

        // Parse a CPU spec like "0,2,4,6" or "3-9" or "0-3,8,10-12" into an ordered, de-duplicated list.
        // Returns null on any malformed token or if the result is empty.
        private static List<int>? ParseCpuList(string spec)
        {
            var list = new List<int>();
            var seen = new HashSet<int>();
            foreach (var tok in spec.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
            {
                int dash = tok.IndexOf('-');
                if (dash > 0)
                {
                    if (!int.TryParse(tok[..dash], out int lo) || !int.TryParse(tok[(dash + 1)..], out int hi))
                        return null;
                    if (lo > hi) (lo, hi) = (hi, lo);
                    for (int c = lo; c <= hi; c++)
                        if (c >= 0 && seen.Add(c)) list.Add(c);
                }
                else
                {
                    if (!int.TryParse(tok, out int c) || c < 0)
                        return null;
                    if (seen.Add(c)) list.Add(c);
                }
            }
            return list.Count > 0 ? list : null;
        }

        private static string BuildConfig() =>
            $"corner-cutting={(JpsBuildInfo.CornerCutting ? "on" : "off")}, " +
            $"concurrent-cache={(JpsBuildInfo.ConcurrentCache ? "on" : "off")}";

        // Run a helper program and capture its stdout (null on any failure). Used for host-info probes only.
        private static string? RunCapture(string file, string args)
        {
            try
            {
                var psi = new ProcessStartInfo(file, args)
                {
                    RedirectStandardOutput = true,
                    UseShellExecute = false,
                    CreateNoWindow = true
                };
                using Process? proc = Process.Start(psi);
                if (proc == null)
                    return null;
                string outp = proc.StandardOutput.ReadToEnd();
                proc.WaitForExit();
                return outp;
            }
            catch { return null; }
        }

        // Best-effort friendly CPU model name, e.g. "Intel(R) Core(TM) i7-9700K CPU @ 3.60GHz". The lookup is
        // inherently platform-specific: Windows reads the registry's ProcessorNameString (via reg.exe, so no
        // Microsoft.Win32.Registry package is needed), Linux reads /proc/cpuinfo, macOS uses sysctl. Falls back
        // to "unknown" (Windows further falls back to PROCESSOR_IDENTIFIER, which is family/model/stepping).
        private static string GetCpuModel()
        {
            try
            {
                if (OperatingSystem.IsWindows())
                {
                    string? outp = RunCapture("reg",
                        @"query ""HKLM\HARDWARE\DESCRIPTION\System\CentralProcessor\0"" /v ProcessorNameString");
                    if (outp != null)
                    {
                        int i = outp.IndexOf("REG_SZ", StringComparison.Ordinal);
                        if (i >= 0)
                        {
                            string v = outp[(i + "REG_SZ".Length)..].Trim();   // value follows the type token
                            if (v.Length > 0)
                                return v;
                        }
                    }
                    string? id = Environment.GetEnvironmentVariable("PROCESSOR_IDENTIFIER");
                    return string.IsNullOrWhiteSpace(id) ? "unknown" : id.Trim();
                }
                if (OperatingSystem.IsLinux())
                {
                    foreach (string line in File.ReadLines("/proc/cpuinfo"))
                    {
                        if (line.StartsWith("model name", StringComparison.Ordinal))
                        {
                            int c = line.IndexOf(':');
                            if (c >= 0)
                                return line[(c + 1)..].Trim();
                        }
                    }
                    return "unknown";
                }
                if (OperatingSystem.IsMacOS())
                {
                    string? outp = RunCapture("sysctl", "-n machdep.cpu.brand_string")?.Trim();
                    if (!string.IsNullOrEmpty(outp))
                        return outp;
                }
            }
            catch { /* diagnostics only; never fail the benchmark over host info */ }
            return "unknown";
        }

        // Host/runtime info for report reproducibility, one field per line (kept short so it doesn't wrap).
        private static void PrintHostInfo(Action<string> writeLine)
        {
            writeLine($"Host OS:  {RuntimeInformation.OSDescription.Trim()} ({RuntimeInformation.OSArchitecture})");
            writeLine($"Host CPU: {GetCpuModel()}; {Environment.ProcessorCount} logical cores");
            writeLine($"Runtime:  {RuntimeInformation.FrameworkDescription} ({RuntimeInformation.ProcessArchitecture})");
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

        private sealed class TeeTextWriter : TextWriter
        {
            private readonly TextWriter _a;
            private readonly TextWriter _b;

            public TeeTextWriter(TextWriter a, TextWriter b)
            {
                _a = a;
                _b = b;
            }

            public override Encoding Encoding => _a.Encoding;
            public override void Write(char value) { _a.Write(value); _b.Write(value); }
            public override void Write(string? value) { _a.Write(value); _b.Write(value); }
            public override void WriteLine(string? value) { _a.WriteLine(value); _b.WriteLine(value); }
            public override void Flush() { _a.Flush(); _b.Flush(); }
        }

        private sealed class ConsoleProgress
        {
            private readonly TextWriter _console;
            private readonly bool _enabled;
            private int _lastLen;
            private long _lastShownMs = -100000;
            private const long MinIntervalMs = 250;

            public ConsoleProgress(TextWriter console)
            {
                _console = console;
                _enabled = !Console.IsOutputRedirected;
            }

            public void Show(string text, bool force = false)
            {
                if (!_enabled) return;
                long now = Environment.TickCount64;
                if (!force && now - _lastShownMs < MinIntervalMs) return;
                _lastShownMs = now;

                int max = SafeWidth();
                if (text.Length > max) text = text.Substring(0, max);
                _console.Write('\r');
                _console.Write(text);
                if (_lastLen > text.Length)
                    _console.Write(new string(' ', _lastLen - text.Length));
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

        private sealed class ScenGroup
        {
            public readonly string MapPath;
            public int ScenCount;
            public readonly HashSet<(int sx, int sy, int gx, int gy)> Pairs = new HashSet<(int, int, int, int)>();
            public ScenGroup(string mapPath) { MapPath = mapPath; }
        }

        private sealed class MapJob
        {
            public int Index;
            public string SortKey = string.Empty;
            public ScenGroup Group = null!;
        }

        private sealed class BenchRow
        {
            public string SortKey = string.Empty;
            public string Tag = string.Empty;
            public string Plain = string.Empty;
            public string Colored = string.Empty;
            public double Cj, Wj, Cn, Wn, A;
            public int N;
        }

        // Result of one phase (rand or scen) of one map. Row == null = no matching pairs; Error != null = failed.
        private sealed class PhaseResult
        {
            public BenchRow? Row;
            public string? Error;
        }

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
                    if (line.Length == 0 || line.StartsWith("version", StringComparison.OrdinalIgnoreCase))
                        continue;
                    var t = line.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                    if (t.Length < 9)
                        continue;
                    mapField ??= t[1];
                    pairs.Add((int.Parse(t[4]), int.Parse(t[5]), int.Parse(t[6]), int.Parse(t[7])));
                }

                if (mapField == null || pairs.Count == 0)
                    continue;

                string? mapPath = ResolveMapPath(scenDir, mapField, root);
                if (mapPath == null)
                    continue;

                if (!groups.TryGetValue(mapPath, out var g))
                {
                    g = new ScenGroup(mapPath);
                    groups[mapPath] = g;
                }
                g.ScenCount++;
                foreach (var p in pairs)
                    g.Pairs.Add(p);
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

        private static string Trunc(string s, int max) => s.Length <= max ? s : s.Substring(0, max - 1) + "~";
        private static string Green(string s) => "\x1b[32m" + s + "\x1b[0m";
        private static string Red(string s) => "\x1b[31m" + s + "\x1b[0m";

        // Enable ANSI escape processing so colored output renders in the classic Windows console (Windows only;
        // other terminals handle ANSI natively). Non-Windows builds compile the Win32 imports out -> no-op.
#if WINDOWS
#pragma warning disable SYSLIB1054
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern IntPtr GetStdHandle(int nStdHandle);
        [DllImport("kernel32.dll")]
        private static extern bool GetConsoleMode(IntPtr hConsoleHandle, out uint lpMode);
        [DllImport("kernel32.dll")]
        private static extern bool SetConsoleMode(IntPtr hConsoleHandle, uint dwMode);
#pragma warning restore SYSLIB1054
#endif

        private static void EnableVirtualTerminal()
        {
#if WINDOWS
            if (!OperatingSystem.IsWindows())
                return;
            try
            {
                IntPtr h = GetStdHandle(-11);
                if (GetConsoleMode(h, out uint mode))
                    SetConsoleMode(h, mode | 0x0004);
            }
            catch { }
#endif
        }

        private static void WarmupJit()
        {
            var m = new GridMap(64, 64);
            var rng = new Random(1);
            for (int i = 0; i < 300; i++)
                m.SetBlocked(rng.Next(64), rng.Next(64), true);

            var sys = new JpsSystem(m);
            sys.Sync();
            var j = new JpsPathfinder();
            var a = new AStarPathfinder();

            for (int i = 0; i < 100; i++)
            {
                var s = (rng.Next(64), rng.Next(64));
                var g = (rng.Next(64), rng.Next(64));
                j.FindPath(sys, s, g);
                a.FindPath(m, s, g);
            }
        }

        private static void ComboBench(int q, string? sub, List<int> cpus)
        {
            string root = FindDir("movingai");
            string dir = string.IsNullOrEmpty(sub) ? root : Path.Combine(root, sub);
            if (!Directory.Exists(dir))
            {
                Console.WriteLine($"Directory not found: {dir}");
                return;
            }

            var (groups, totalEntries, scenFileCount) = CollectScenGroups(dir, root);
            if (groups.Count == 0)
            {
                Console.WriteLine($"No usable .scen files found: {dir}");
                return;
            }

            // One worker per requested CPU, capped at the number of maps (no point spinning idle workers).
            int workers = Math.Max(1, Math.Min(cpus.Count, groups.Count));
            string repoRoot = Directory.GetParent(root)?.FullName ?? root;
            string reportsDir = Path.Combine(repoRoot, "benchmark-results");
            Directory.CreateDirectory(reportsDir);
            string scopeTag = string.IsNullOrEmpty(sub) ? "all" : sub.Replace('/', '-').Replace('\\', '-');
            string reportPath = Path.Combine(reportsDir, $"combo-{scopeTag}-q{q}-t{workers}-{DateTime.Now:yyyyMMdd-HHmmss}.txt");

            var consoleOut = Console.Out;
            using var fileOut = new StreamWriter(reportPath) { AutoFlush = true };
            Console.SetOut(new TeeTextWriter(consoleOut, fileOut));

            var progress = new ConsoleProgress(consoleOut);
            object progressLock = new object();
            void ShowProgress(string text, bool force = false)
            {
                lock (progressLock)
                    progress.Show(text, force);
            }
            void ClearProgress()
            {
                lock (progressLock)
                    progress.Clear();
            }

            bool nativeEnabled = NativeJps.TryInit(out string nativeInfo);
            bool useColor = !Console.IsOutputRedirected;
            if (useColor && OperatingSystem.IsWindows())
                EnableVirtualTerminal();

            string scope = string.IsNullOrEmpty(sub) ? "movingai/" : $"movingai/{sub}";
            long dedup = groups.Values.Sum(x => (long)x.Pairs.Count);
            Console.WriteLine($"# JPS(C#/C) cold/hot vs A* benchmark   {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            PrintHostInfo(Console.WriteLine);
            Console.WriteLine($"Build config: {BuildConfig()}");
            Console.WriteLine($"Native: {(nativeEnabled ? $"enabled ({nativeInfo})" : $"disabled ({nativeInfo})")}");
            Console.WriteLine($"Scope: {scope}; {scenFileCount} .scen / {groups.Count} maps; rand target {q}/map; scen dedup {dedup} from {totalEntries}");
            Console.WriteLine($"Parallelism: {workers} map workers pinned to CPUs [{string.Join(",", cpus.Take(workers))}] (default: half of logical CPUs, every other)");
            Console.WriteLine("Timing mode: parallel throughput by map group. Rows are emitted in map dispatch order as results arrive.");
            Console.WriteLine("cold = before each query flip a few cells in a small window then restore + Sync (invalidates the jump-point cache); hot = warm cache reuse.");
            Console.WriteLine();
            Console.WriteLine("Columns:");
            Console.WriteLine("  map          map name (relative path)");
            Console.WriteLine("  size         width x height");
            Console.WriteLine("  tag          rand = random sampled pairs / scen = .scen cases");
            Console.WriteLine("  pairs        start/goal pairs timed");
            Console.WriteLine("  JPSexp/A*exp mean expanded nodes per query (JPS / A*)");
            if (nativeEnabled)
            {
                Console.WriteLine("  cC# / cC     C# / C JPS cold-path mean time (us/query)");
                Console.WriteLine("  wC# / wC     C# / C JPS hot-path mean time (us/query)");
                Console.WriteLine("  A*us         A* mean time per query (us/query)");
                Console.WriteLine("  A*/cC A*/wC  A*us divided by C cold / hot time (higher = C JPS faster than A*)");
                Console.WriteLine("  color: cC/wC vs same-language C# faster=green slower=red; any JPS time slower than A*=red; A*/cC or A*/wC < 1 = red");
            }
            else
            {
                Console.WriteLine("  cC# / wC#    C# JPS cold / hot mean time (us/query)");
                Console.WriteLine("  A*us         A* mean time per query (us/query)");
                Console.WriteLine("  A*/cC# A*/wC# A*us divided by C# cold / hot time (higher = C# JPS faster than A*)");
                Console.WriteLine("  color: cC#/wC# slower than A* = red; A*/cC# or A*/wC# < 1 = red");
            }
            Console.WriteLine();

            string hdr = nativeEnabled
                ? $"{"map",-30}{"size",11}{"tag",6}{"pairs",8}{"JPSexp",8}{"A*exp",8}{"cC#",9}{"cC",9}{"wC#",9}{"wC",9}{"A*us",9}{"A*/cC",8}{"A*/wC",8}"
                : $"{"map",-30}{"size",11}{"tag",6}{"pairs",8}{"JPSexp",8}{"A*exp",8}{"cC#",9}{"wC#",9}{"A*us",9}{"A*/cC#",8}{"A*/wC#",8}";
            int rule = nativeEnabled ? 132 : 114;
            void PrintResultHeader()
            {
                ClearProgress();
                Console.WriteLine(new string('-', rule));
                Console.WriteLine(hdr);
                Console.WriteLine(new string('-', rule));
            }

            var jobs = groups
                .OrderBy(k => Path.GetRelativePath(root, k.Key), StringComparer.OrdinalIgnoreCase)
                .Select((kv, i) => new MapJob
                {
                    Index = i + 1,
                    SortKey = Path.GetRelativePath(root, kv.Key).Replace('\\', '/'),
                    Group = kv.Value
                })
                .ToList();

            WarmupJit();
            GC.Collect();
            GC.WaitForPendingFinalizers();
            GC.Collect();

            var rows = new List<BenchRow>();
            var errors = new List<string>();
            int started = 0;
            int completed = 0;
            int total = jobs.Count;

            PrintResultHeader();
            int resultRows = 0;
            void EmitRow(BenchRow row)
            {
                ClearProgress();
                fileOut.WriteLine(row.Plain);
                consoleOut.WriteLine(useColor ? row.Colored : row.Plain);
                rows.Add(row);
                if (++resultRows % ResultHeaderRepeat == 0)
                    PrintResultHeader();
            }

            // Dispatch jobs to workers in order. Each job submits its two phases (rand / scen) separately,
            // so we allocate a TaskCompletionSource per phase (main waits rand -> scen in job order and prints).
            var jobsQueue = new BlockingCollection<MapJob>();
            var randTcs = new TaskCompletionSource<PhaseResult>[jobs.Count + 1];
            var scenTcs = new TaskCompletionSource<PhaseResult>[jobs.Count + 1];
            for (int i = 0; i < randTcs.Length; i++)
            {
                randTcs[i] = new TaskCompletionSource<PhaseResult>(TaskCreationOptions.RunContinuationsAsynchronously);
                scenTcs[i] = new TaskCompletionSource<PhaseResult>(TaskCreationOptions.RunContinuationsAsynchronously);
            }

            Task[] workerTasks = new Task[workers];
            var workerReady = new TaskCompletionSource<bool>[workers];
            for (int i = 0; i < workers; i++) workerReady[i] = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            for (int wi = 0; wi < workerTasks.Length; wi++)
            {
                int localWorkerIndex = wi; // capture
                workerTasks[wi] = Task.Run(() =>
                {

                    // Pin this worker thread to its assigned CPU (workers == number of requested CPUs, 1:1).
                    try
                    {
                        TrySetCurrentThreadAffinity(cpus[localWorkerIndex]);
                    }
                    catch { }

                    // Give the thread an initial name (Thread.Name can only be set once).
                    try { System.Threading.Thread.CurrentThread.Name ??= $"init-{localWorkerIndex}"; } catch { }

                    // Signal the main thread that this worker finished init and is ready.
                    try { workerReady[localWorkerIndex].TrySetResult(true); } catch { }

                    foreach (var job in jobsQueue.GetConsumingEnumerable())
                    {
                        // Best-effort rename to job-{job.Index} (Thread.Name may already be set; ignore failures).
                        try { System.Threading.Thread.CurrentThread.Name = $"job-{job.Index}"; } catch { }

                        // MapRunner runs rand -> scen for one map, submitting each phase to randTcs/scenTcs as it finishes.
                        // Exceptions are caught inside Run and posted to the unsubmitted phase(s), so both TCS always resolve.
                        try
                        {
                            using var runner = new MapRunner(job, nativeEnabled, q, ShowProgress, () => completed, total);
                            runner.Run(randTcs[job.Index], scenTcs[job.Index]);
                        }
                        finally
                        {
                            int done = Interlocked.Increment(ref completed);
                            ShowProgress($"[{done}/{total} done] {job.SortKey}", true);
                        }
                    }
                });
            }

            // Wait until all workers finish init before dispatching, so the first job is not delayed by thread startup/affinity.
            Task.WhenAll(workerReady.Select(w => w.Task)).GetAwaiter().GetResult();

            // Dispatch all jobs into the queue in order (main thread only enqueues).
            foreach (var job in jobs)
            {
                int localStarted = Interlocked.Increment(ref started);
                ShowProgress($"[{completed}/{total} done, {localStarted}/{total} started] {job.SortKey}", true);
                jobsQueue.Add(job);
            }
            jobsQueue.CompleteAdding();

            // Main thread: consume each job result in dispatch order (= SortKey order).
            // Wait for each job's rand and print it, then wait for its scen and print it.
            // So a map's rand row appears without waiting for its scen; EmitRow reprints the header every 50 rows.
            for (int idx = 1; idx <= jobs.Count; idx++)
            {
                string sortKey = jobs[idx - 1].SortKey;

                var randRes = randTcs[idx].Task.GetAwaiter().GetResult();
                bool jobErrored = false;
                if (randRes.Error != null)
                {
                    errors.Add($"{sortKey}: {randRes.Error}");
                    jobErrored = true;
                }
                else if (randRes.Row != null)
                {
                    EmitRow(randRes.Row);
                }

                var scenRes = scenTcs[idx].Task.GetAwaiter().GetResult();
                if (scenRes.Error != null)
                {
                    // On setup failure both phases carry the same error; if rand already reported it, don't double-count.
                    if (!jobErrored)
                        errors.Add($"{sortKey}: {scenRes.Error}");
                }
                else if (scenRes.Row != null)
                {
                    EmitRow(scenRes.Row);
                }
            }

            // Wait for all worker tasks to finish.
            Task.WaitAll(workerTasks);

            ClearProgress();
            Console.WriteLine(new string('-', rule));

            Summary("rand", rows.Where(r => r.Tag == "rand"));
            Summary("scen", rows.Where(r => r.Tag == "scen"));

            if (errors.Count > 0)
            {
                Console.WriteLine();
                Console.WriteLine($"Skipped maps: {errors.Count}");
                foreach (var e in errors.OrderBy(x => x, StringComparer.OrdinalIgnoreCase).Take(20))
                    Console.WriteLine($"  {e}");
                if (errors.Count > 20)
                    Console.WriteLine($"  ... {errors.Count - 20} more");
            }

            Console.Out.Flush();
            Console.SetOut(consoleOut);
            Console.WriteLine($"Report saved: {reportPath}");

            void Summary(string tag, IEnumerable<BenchRow> seq)
            {
                var list = seq.ToList();
                long p = list.Sum(r => (long)r.N);
                if (p == 0)
                    return;

                double cj = list.Sum(r => r.Cj);
                double wj = list.Sum(r => r.Wj);
                double cn = list.Sum(r => r.Cn);
                double wn = list.Sum(r => r.Wn);
                double a = list.Sum(r => r.A);

                if (nativeEnabled)
                {
                    Console.WriteLine($"[{tag}] {p} pairs: C# cold {cj:F0}ms / hot {wj:F0}ms; " +
                        $"C cold {cn:F0}ms / hot {wn:F0}ms; A* {a:F0}ms; " +
                        $"A*/C cold {a / Math.Max(0.001, cn):F1}x hot {a / Math.Max(0.001, wn):F1}x; " +
                        $"C#/C cold {cj / Math.Max(0.001, cn):F2}x hot {wj / Math.Max(0.001, wn):F2}x");
                }
                else
                {
                    Console.WriteLine($"[{tag}] {p} pairs: C# cold {cj:F0}ms / hot {wj:F0}ms; A* {a:F0}ms; " +
                        $"A*/C# cold {a / Math.Max(0.001, cj):F1}x hot {a / Math.Max(0.001, wj):F1}x");
                }
            }
        }

        // Runs the full benchmark (rand + scen) for one map. Keeping all per-map state in fields turns the
        // former deeply nested local functions into flat methods that read as straight-line code.
        private sealed class MapRunner : IDisposable
        {
            // Injected dependencies.
            private readonly MapJob _job;
            private readonly bool _nativeEnabled;
            private readonly int _q;
            private readonly Action<string, bool> _showProgress;
            private readonly Func<int> _completed;   // global finished-map count, for progress text only
            private readonly int _total;

            // Per-map state (populated by Load).
            private GridMap _map = null!;
            private string _name = string.Empty;
            private JpsSystem _system = null!;
            private JpsPathfinder _jps = null!;
            private AStarPathfinder _astar = null!;
            private NativeMap? _nat;
            private readonly Stopwatch _sw = new Stopwatch();

            // Cold-edit scratch state (random local edits, reused across cold reps).
            private int _coldWindow;
            private int _coldEditCount;
            private int[] _coldSeen = Array.Empty<int>();
            private int _coldSeenStamp;

            public MapRunner(MapJob job, bool nativeEnabled, int q,
                             Action<string, bool> showProgress, Func<int> completed, int total)
            {
                _job = job;
                _nativeEnabled = nativeEnabled;
                _q = q;
                _showProgress = showProgress;
                _completed = completed;
                _total = total;
            }

            public void Dispose() => _nat?.Dispose();

            // Run rand then scen, submitting each phase to its TCS as it finishes. All exceptions are caught and
            // posted to the still-unsubmitted phase(s), so both TCS always resolve (the awaiting main thread never hangs).
            public void Run(TaskCompletionSource<PhaseResult> randOut, TaskCompletionSource<PhaseResult> scenOut)
            {
                try
                {
                    var walk = Load();
                    if (walk == null)
                    {
                        randOut.TrySetResult(new PhaseResult());   // empty (walkable cells < 2) -> no row, no error
                        scenOut.TrySetResult(new PhaseResult());
                        return;
                    }

                    // rand phase: submit as soon as it finishes (don't wait for scen).
                    var rand = RunSegment("rand", SampleRandPairs(walk));
                    randOut.TrySetResult(new PhaseResult { Row = rand });

                    // scen phase: submit when it finishes.
                    var scen = RunSegment("scen", CollectScenPairs());
                    scenOut.TrySetResult(new PhaseResult { Row = scen });
                }
                catch (Exception ex)
                {
                    // Post the error to the unsubmitted phase(s) (TrySetResult is a no-op if already submitted).
                    randOut.TrySetResult(new PhaseResult { Error = ex.Message });
                    scenOut.TrySetResult(new PhaseResult { Error = ex.Message });
                }
            }

            // Parse the map and build the pathfinders + cold-edit state. Returns the walkable cells,
            // or null when the map has fewer than two walkable cells (nothing to benchmark).
            private List<(int X, int Y)>? Load()
            {
                _map = MovingAiMap.Parse(File.ReadAllText(_job.Group.MapPath));

                string rel = _job.SortKey;
                _name = rel.EndsWith(".map", StringComparison.OrdinalIgnoreCase)
                    ? rel.Substring(0, rel.Length - 4)
                    : rel;

                var walk = new List<(int X, int Y)>();
                for (int y = 0; y < _map.Height; y++)
                    for (int x = 0; x < _map.Width; x++)
                        if (_map.IsWalkable(x, y))
                            walk.Add((x, y));
                if (walk.Count < 2)
                    return null;

                _system = new JpsSystem(_map);
                _system.Sync();
                _jps = new JpsPathfinder();
                _astar = new AStarPathfinder();
                _nat = _nativeEnabled ? new NativeMap(_map) : null;

                _coldWindow = Math.Min(16, Math.Min(_map.Width, _map.Height));
                _coldEditCount = Math.Min(24, Math.Max(1, _coldWindow * _coldWindow / 4));
                _coldSeen = new int[_map.Width * _map.Height];
                _coldSeenStamp = 0;
                return walk;
            }

            // Sample up to _q solvable random start/goal pairs.
            private List<(int sx, int sy, int gx, int gy)> SampleRandPairs(List<(int X, int Y)> walk)
            {
                var pairs = new List<(int sx, int sy, int gx, int gy)>(_q);
                var rng = new Random(12345);
                int maxAtt = _q * 40 + 500;
                for (int at = 0; at < maxAtt && pairs.Count < _q; at++)
                {
                    var s = walk[rng.Next(walk.Count)];
                    var g = walk[rng.Next(walk.Count)];
                    if (s.Equals(g))
                        continue;
                    if (_jps.FindPath(_system, s, g).Success)
                    {
                        pairs.Add((s.X, s.Y, g.X, g.Y));
                        if (pairs.Count % 100 == 0 || pairs.Count == _q)
                            Progress($"rand sample {pairs.Count}/{_q}", true);
                    }
                }
                return pairs;
            }

            // Collect the in-bounds, walkable scen pairs for this map.
            private List<(int sx, int sy, int gx, int gy)> CollectScenPairs()
            {
                var grpPairs = _job.Group.Pairs;
                var pairs = new List<(int sx, int sy, int gx, int gy)>(grpPairs.Count);
                foreach (var p in grpPairs)
                {
                    if (p.sx < 0 || p.sy < 0 || p.gx < 0 || p.gy < 0) continue;
                    if (p.sx >= _map.Width || p.sy >= _map.Height || p.gx >= _map.Width || p.gy >= _map.Height) continue;
                    if (!_map.IsWalkable(p.sx, p.sy) || !_map.IsWalkable(p.gx, p.gy)) continue;
                    pairs.Add(p);
                }
                return pairs;
            }

            // Time one segment (rand or scen): expansions, then Repeats rounds of C#/C cold+hot and A*, min per column.
            private BenchRow? RunSegment(string tag, List<(int sx, int sy, int gx, int gy)> pairs)
            {
                int n = pairs.Count;
                if (n == 0)
                    return null;

                long jExp = 0;
                long aExp = 0;
                for (int pi = 0; pi < pairs.Count; pi++)
                {
                    var p = pairs[pi];
                    jExp += _jps.FindPath(_system, (p.sx, p.sy), (p.gx, p.gy)).ExpandedNodes;
                    aExp += _astar.FindPath(_map, (p.sx, p.sy), (p.gx, p.gy)).ExpandedNodes;
                    ShowPathProgress(tag, "exp", pi + 1, n);
                }

                double cj = double.MaxValue;
                double wj = double.MaxValue;
                double a = double.MaxValue;
                double cn = _nat != null ? double.MaxValue : 0;
                double wn = _nat != null ? double.MaxValue : 0;

                for (int rep = 0; rep < Repeats; rep++)
                {
                    Progress($"{tag} C# cold {rep + 1}/{Repeats}");
                    _sw.Restart();
                    {
                        var rngCold = new Random(1000003 + rep * 9176 + _job.Index * 131);
                        var previousEdits = new List<(int x, int y, bool oldBlocked)>();
                        for (int pi = 0; pi < pairs.Count; pi++)
                        {
                            var p = pairs[pi];
                            RestoreCs(previousEdits);
                            previousEdits = MakeColdEdits(rngCold, p.sx, p.sy, p.gx, p.gy);
                            ApplyCs(previousEdits);
                            _system.Sync();
                            _jps.FindPath(_system, (p.sx, p.sy), (p.gx, p.gy));
                            ShowPathProgress(tag, "C#cold", pi + 1, n, rep + 1);
                        }
                        RestoreCs(previousEdits);
                        _system.Sync();
                    }
                    _sw.Stop();
                    cj = Math.Min(cj, _sw.Elapsed.TotalMilliseconds);

                    _system.Sync();
                    Progress($"{tag} C# hot {rep + 1}/{Repeats}");
                    _sw.Restart();
                    for (int pi = 0; pi < pairs.Count; pi++)
                    {
                        var p = pairs[pi];
                        _jps.FindPath(_system, (p.sx, p.sy), (p.gx, p.gy));
                        ShowPathProgress(tag, "C#hot", pi + 1, n, rep + 1);
                    }
                    _sw.Stop();
                    wj = Math.Min(wj, _sw.Elapsed.TotalMilliseconds);

                    if (_nat != null)
                    {
                        Progress($"{tag} C cold {rep + 1}/{Repeats}");
                        _sw.Restart();
                        {
                            var rngCold = new Random(1000003 + rep * 9176 + _job.Index * 131);
                            var previousEdits = new List<(int x, int y, bool oldBlocked)>();
                            for (int pi = 0; pi < pairs.Count; pi++)
                            {
                                var p = pairs[pi];
                                RestoreNative(previousEdits);
                                previousEdits = MakeColdEdits(rngCold, p.sx, p.sy, p.gx, p.gy);
                                ApplyNative(previousEdits);
                                _nat.Sync();
                                _nat.Find(p.sx, p.sy, p.gx, p.gy);
                                ShowPathProgress(tag, "Ccold", pi + 1, n, rep + 1);
                            }
                            RestoreNative(previousEdits);
                            _nat.Sync();
                        }
                        _sw.Stop();
                        cn = Math.Min(cn, _sw.Elapsed.TotalMilliseconds);

                        _nat.Sync();
                        Progress($"{tag} C hot {rep + 1}/{Repeats}");
                        _sw.Restart();
                        for (int pi = 0; pi < pairs.Count; pi++)
                        {
                            var p = pairs[pi];
                            _nat.Find(p.sx, p.sy, p.gx, p.gy);
                            ShowPathProgress(tag, "Chot", pi + 1, n, rep + 1);
                        }
                        _sw.Stop();
                        wn = Math.Min(wn, _sw.Elapsed.TotalMilliseconds);
                    }

                    Progress($"{tag} A* {rep + 1}/{Repeats}");
                    _sw.Restart();
                    for (int pi = 0; pi < pairs.Count; pi++)
                    {
                        var p = pairs[pi];
                        _astar.FindPath(_map, (p.sx, p.sy), (p.gx, p.gy));
                        ShowPathProgress(tag, "A*", pi + 1, n, rep + 1);
                    }
                    _sw.Stop();
                    a = Math.Min(a, _sw.Elapsed.TotalMilliseconds);
                }

                return BuildRow(tag, n, jExp, aExp, cj, wj, cn, wn, a);
            }

            // Format one output row (plain + ANSI-colored) from the per-column timings.
            private BenchRow BuildRow(string tag, int n, long jExp, long aExp, double cj, double wj, double cn, double wn, double a)
            {
                double cjus = cj / n * 1000;
                double wjus = wj / n * 1000;
                double aus = a / n * 1000;
                string size = $"{_map.Width}x{_map.Height}";
                string hd = $"{Trunc(_name, 30),-30}{size,11}{tag,6}{n,8}{jExp / n,8}{aExp / n,8}";

                string plain;
                string colored;
                if (_nat != null)
                {
                    double cnus = cn / n * 1000;
                    double wnus = wn / n * 1000;
                    string cjStr = $"{cjus,9:F2}";
                    string cnStr = $"{cnus,9:F2}";
                    string wjStr = $"{wjus,9:F2}";
                    string wnStr = $"{wnus,9:F2}";
                    string ausStr = $"{aus,9:F2}";
                    double rcn = aus / Math.Max(0.001, cnus);
                    double rwn = aus / Math.Max(0.001, wnus);
                    string rcnStr = $"{rcn,8:F1}";
                    string rwnStr = $"{rwn,8:F1}";

                    string cjCol = cjus > aus ? Red(cjStr) : cjStr;
                    string cnCol = cnus > aus ? Red(cnStr) : cnus < cjus ? Green(cnStr) : cnus > cjus ? Red(cnStr) : cnStr;
                    string wjCol = wjus > aus ? Red(wjStr) : wjStr;
                    string wnCol = wnus > aus ? Red(wnStr) : wnus < wjus ? Green(wnStr) : wnus > wjus ? Red(wnStr) : wnStr;
                    string rcnCol = rcn < 1 ? Red(rcnStr) : rcnStr;
                    string rwnCol = rwn < 1 ? Red(rwnStr) : rwnStr;

                    plain = hd + cjStr + cnStr + wjStr + wnStr + ausStr + rcnStr + rwnStr;
                    colored = hd + cjCol + cnCol + wjCol + wnCol + ausStr + rcnCol + rwnCol;
                }
                else
                {
                    string cjStr = $"{cjus,9:F2}";
                    string wjStr = $"{wjus,9:F2}";
                    string ausStr = $"{aus,9:F2}";
                    double rcj = aus / Math.Max(0.001, cjus);
                    double rwj = aus / Math.Max(0.001, wjus);
                    string rcjStr = $"{rcj,8:F1}";
                    string rwjStr = $"{rwj,8:F1}";

                    string cjCol = cjus > aus ? Red(cjStr) : cjStr;
                    string wjCol = wjus > aus ? Red(wjStr) : wjStr;
                    string rcjCol = rcj < 1 ? Red(rcjStr) : rcjStr;
                    string rwjCol = rwj < 1 ? Red(rwjStr) : rwjStr;

                    plain = hd + cjStr + wjStr + ausStr + rcjStr + rwjStr;
                    colored = hd + cjCol + wjCol + ausStr + rcjCol + rwjCol;
                }

                return new BenchRow
                {
                    SortKey = _job.SortKey,
                    Tag = tag,
                    Plain = plain,
                    Colored = colored,
                    Cj = cj,
                    Wj = wj,
                    Cn = cn,
                    Wn = wn,
                    A = a,
                    N = n
                };
            }

            // Pick _coldEditCount distinct random cells in a small window (excluding start/goal); returns
            // (x, y, oldBlocked) so the edit can be applied then restored, invalidating the jump-point cache.
            private List<(int x, int y, bool oldBlocked)> MakeColdEdits(Random rng, int sx, int sy, int gx, int gy)
            {
                var edits = new List<(int x, int y, bool oldBlocked)>(_coldEditCount);
                int stamp = ++_coldSeenStamp;
                int ox = rng.Next(_map.Width - _coldWindow + 1);
                int oy = rng.Next(_map.Height - _coldWindow + 1);
                int maxAttempts = _coldEditCount * 20 + 200;

                for (int attempts = 0; attempts < maxAttempts && edits.Count < _coldEditCount; attempts++)
                {
                    int x = ox + rng.Next(_coldWindow);
                    int y = oy + rng.Next(_coldWindow);
                    if ((x == sx && y == sy) || (x == gx && y == gy))
                        continue;
                    int id = y * _map.Width + x;
                    if (_coldSeen[id] == stamp)
                        continue;
                    _coldSeen[id] = stamp;
                    edits.Add((x, y, _map.IsBlocked(x, y)));
                }
                return edits;
            }

            private void RestoreCs(List<(int x, int y, bool oldBlocked)> edits)
            {
                foreach (var e in edits)
                    _map.SetBlocked(e.x, e.y, e.oldBlocked);
            }

            private void ApplyCs(List<(int x, int y, bool oldBlocked)> edits)
            {
                foreach (var e in edits)
                    _map.SetBlocked(e.x, e.y, !e.oldBlocked);
            }

            private void RestoreNative(List<(int x, int y, bool oldBlocked)> edits) => _nat?.SetBlockedBatch(edits, apply: false);
            private void ApplyNative(List<(int x, int y, bool oldBlocked)> edits) => _nat?.SetBlockedBatch(edits, apply: true);

            // Progress helpers: both prepend the shared "[done/total done] #index name" prefix.
            private void Progress(string suffix, bool force = false)
                => _showProgress($"[{_completed()}/{_total} done] #{_job.Index} {_name} {suffix}", force);

            private void ShowPathProgress(string tag, string phase, int done, int count, int rep = 0)
            {
                if (done % 100 == 0 || done == count)
                {
                    string repText = rep > 0 ? $" {rep}/{Repeats}" : "";
                    Progress($"{tag} {phase}{repText} {done}/{count}", true);
                }
            }
        }
    }
}
