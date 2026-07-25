using System;
#if JPS_NATIVE_DLLIMPORT_RESOLVER
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
#endif

namespace JPS.Native
{
    public enum NativeJpsError
    {
        NullHandle = -1,
        OutOfBounds = -2,
        Blocked = -3,
        NoPath = -4,
        InvalidArgument = -5,
        OutOfMemory = -6
    }

    public static class NativeJps
    {
#if JPS_NATIVE_DLLIMPORT_RESOLVER
        private static readonly object ResolverLock = new object();
        private static bool _resolverInstalled;
#endif

        public static string? ResolvedPath { get; private set; }

        internal static void EnsureInitialized()
        {
#if JPS_NATIVE_DLLIMPORT_RESOLVER
            if (_resolverInstalled)
                return;
            lock (ResolverLock)
            {
                if (_resolverInstalled)
                    return;
                NativeLibrary.SetDllImportResolver(typeof(NativeJps).Assembly, ResolveLibrary);
                _resolverInstalled = true;
            }
#endif
        }

        public static bool TryInitialize(out string info)
        {
            try
            {
                EnsureInitialized();
                IntPtr system = NativeMethods.jps_system_create(1, 1);
                if (system == IntPtr.Zero)
                {
                    info = "jps_system_create returned NULL.";
                    return false;
                }

                NativeMethods.jps_system_destroy(system);
                info = ResolvedPath ?? "(platform default search path)";
                return true;
            }
            catch (Exception ex)
            {
                info = ex.Message;
                return false;
            }
        }

        public static bool IsError(int result)
        {
            return result < 0;
        }

        public static NativeJpsError GetError(int result)
        {
            if (result >= 0)
                throw new ArgumentOutOfRangeException(nameof(result), "The result is not an error code.");
            return (NativeJpsError)result;
        }

#if JPS_NATIVE_DLLIMPORT_RESOLVER
        private static IntPtr ResolveLibrary(
            string libraryName, System.Reflection.Assembly assembly, DllImportSearchPath? searchPath)
        {
            if (libraryName != NativeMethods.LibraryName)
                return IntPtr.Zero;

            foreach (string candidate in LibraryCandidates())
            {
                IntPtr handle;
                if (File.Exists(candidate) && NativeLibrary.TryLoad(candidate, out handle))
                {
                    ResolvedPath = candidate;
                    return handle;
                }
            }

            return IntPtr.Zero;
        }

        private static IEnumerable<string> LibraryCandidates()
        {
            string fileName = NativeFileName();
            string? directory = AppContext.BaseDirectory;
            for (int depth = 0; depth < 12 && directory != null; depth++)
            {
                yield return Path.Combine(directory, fileName);
                yield return Path.Combine(directory, "x64", "Release", "JPS.Native.dll");
                yield return Path.Combine(directory, "x64", "Debug", "JPS.Native.dll");
                yield return Path.Combine(directory, "JPS.Native", "x64", "Release", "JPS.Native.dll");
                yield return Path.Combine(directory, "JPS.Native", "x64", "Debug", "JPS.Native.dll");
                directory = Directory.GetParent(directory)?.FullName;
            }
        }

        private static string NativeFileName()
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                return "JPS.Native.dll";
            if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
                return "libJPS.Native.dylib";
            return "libJPS.Native.so";
        }
#endif
    }
}
