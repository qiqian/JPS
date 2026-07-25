using System;
using System.Collections.Generic;

namespace JPS.Native
{
    public sealed class NativePathfinder : IDisposable
    {
        private IntPtr _handle;

        public NativePathfinder()
        {
            NativeJps.EnsureInitialized();
            _handle = NativeMethods.jps_pathfinder_create();
            if (_handle == IntPtr.Zero)
                throw new InvalidOperationException("jps_pathfinder_create returned NULL.");
        }

        ~NativePathfinder()
        {
            Dispose(false);
        }

        public ulong MemoryBytes
        {
            get { return NativeMethods.jps_pathfinder_memory_bytes(GetHandle()); }
        }

        public int PathCount
        {
            get { return NativeMethods.jps_pathfinder_path_count(GetHandle()); }
        }

        public bool ReachedGoal
        {
            get { return NativeMethods.jps_pathfinder_reached_goal(GetHandle()) != 0; }
        }

        public int SmoothedPathCount
        {
            get { return NativeMethods.jps_pathfinder_smoothed_path_count(GetHandle()); }
        }

        internal IntPtr Handle
        {
            get { return GetHandle(); }
        }

        public int FindRaw(NativeSystem system, int startX, int startY, int goalX, int goalY)
        {
            if (system == null)
                throw new ArgumentNullException(nameof(system));
            return FindRaw(system.Handle, startX, startY, goalX, goalY, false);
        }

        public int FindRaw(NativeAdapter adapter, int startX, int startY, int goalX, int goalY)
        {
            if (adapter == null)
                throw new ArgumentNullException(nameof(adapter));
            return FindRaw(adapter.SystemHandle, startX, startY, goalX, goalY, false);
        }

        public int FindNearestRaw(NativeSystem system, int startX, int startY, int goalX, int goalY)
        {
            if (system == null)
                throw new ArgumentNullException(nameof(system));
            return FindRaw(system.Handle, startX, startY, goalX, goalY, true);
        }

        public int FindNearestRaw(NativeAdapter adapter, int startX, int startY, int goalX, int goalY)
        {
            if (adapter == null)
                throw new ArgumentNullException(nameof(adapter));
            return FindRaw(adapter.SystemHandle, startX, startY, goalX, goalY, true);
        }

        public (bool ok, List<(int X, int Y)>? path, List<(float X, float Y)>? smooth) Find(
            NativeSystem system, int startX, int startY, int goalX, int goalY)
        {
            int result = FindRaw(system, startX, startY, goalX, goalY);
            return CopyResult(result);
        }

        public (bool ok, List<(int X, int Y)>? path, List<(float X, float Y)>? smooth) Find(
            NativeAdapter adapter, int startX, int startY, int goalX, int goalY)
        {
            int result = FindRaw(adapter, startX, startY, goalX, goalY);
            return CopyResult(result);
        }

        public (bool ok, bool reachedGoal, List<(int X, int Y)>? path,
            List<(float X, float Y)>? smooth) FindNearest(
            NativeSystem system, int startX, int startY, int goalX, int goalY)
        {
            int result = FindNearestRaw(system, startX, startY, goalX, goalY);
            var copied = CopyResult(result);
            return (copied.ok, copied.ok && ReachedGoal, copied.path, copied.smooth);
        }

        public (bool ok, bool reachedGoal, List<(int X, int Y)>? path,
            List<(float X, float Y)>? smooth) FindNearest(
            NativeAdapter adapter, int startX, int startY, int goalX, int goalY)
        {
            int result = FindNearestRaw(adapter, startX, startY, goalX, goalY);
            var copied = CopyResult(result);
            return (copied.ok, copied.ok && ReachedGoal, copied.path, copied.smooth);
        }

        public int CopyPath(int[] outputXy, int capacityPoints)
        {
            ValidateOutput(outputXy, capacityPoints);
            return NativeMethods.jps_pathfinder_copy_path(GetHandle(), outputXy, capacityPoints);
        }

        public int CopySmoothedPath(float[] outputXy, int capacityPoints)
        {
            ValidateOutput(outputXy, capacityPoints);
            return NativeMethods.jps_pathfinder_copy_smoothed_path(
                GetHandle(), outputXy, capacityPoints);
        }

        public List<(int X, int Y)> CopyPath()
        {
            int count = PathCount;
            var buffer = new int[checked(count * 2)];
            int copied = CopyPath(buffer, count);
            var path = new List<(int X, int Y)>(copied);
            for (int i = 0; i < copied; i++)
                path.Add((buffer[i * 2], buffer[i * 2 + 1]));
            return path;
        }

        public List<(float X, float Y)> CopySmoothedPath()
        {
            int count = SmoothedPathCount;
            var buffer = new float[checked(count * 2)];
            int copied = CopySmoothedPath(buffer, count);
            var path = new List<(float X, float Y)>(copied);
            for (int i = 0; i < copied; i++)
                path.Add((buffer[i * 2], buffer[i * 2 + 1]));
            return path;
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        private int FindRaw(
            IntPtr system, int startX, int startY, int goalX, int goalY, bool nearest)
        {
            return nearest
                ? NativeMethods.jps_pathfinder_find_path_nearest(
                    GetHandle(), system, startX, startY, goalX, goalY)
                : NativeMethods.jps_pathfinder_find_path(
                    GetHandle(), system, startX, startY, goalX, goalY);
        }

        private (bool ok, List<(int X, int Y)>? path,
            List<(float X, float Y)>? smooth) CopyResult(int result)
        {
            if (result < 0)
                return (false, null, null);
            return (true, CopyPath(), CopySmoothedPath());
        }

        private void Dispose(bool disposing)
        {
            if (_handle != IntPtr.Zero)
            {
                NativeMethods.jps_pathfinder_destroy(_handle);
                _handle = IntPtr.Zero;
            }
        }

        private IntPtr GetHandle()
        {
            if (_handle == IntPtr.Zero)
                throw new ObjectDisposedException(nameof(NativePathfinder));
            return _handle;
        }

        private static void ValidateOutput(Array output, int capacityPoints)
        {
            if (output == null)
                throw new ArgumentNullException(nameof(output));
            if (capacityPoints < 0 || checked(capacityPoints * 2) > output.Length)
                throw new ArgumentOutOfRangeException(nameof(capacityPoints));
        }
    }
}
