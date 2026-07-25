using System;
using System.Collections.Generic;

namespace JPS.Native
{
    public sealed class NativeMap : IDisposable
    {
        private readonly NativeSystem _system;
        private readonly NativePathfinder _pathfinder;
        private readonly IntPtr _systemHandle;
        private readonly IntPtr _pathfinderHandle;
        private int[] _editBuffer = Array.Empty<int>();
        private bool _disposed;

        public NativeMap(int width, int height, byte[] cells)
        {
            _system = new NativeSystem(width, height, cells);
            try
            {
                _pathfinder = new NativePathfinder();
                _systemHandle = _system.Handle;
                _pathfinderHandle = _pathfinder.Handle;
            }
            catch
            {
                _system.Dispose();
                throw;
            }
        }

        public NativeMap(int width, int height, Func<int, int, bool> isBlocked)
        {
            _system = new NativeSystem(width, height, isBlocked);
            try
            {
                _pathfinder = new NativePathfinder();
                _systemHandle = _system.Handle;
                _pathfinderHandle = _pathfinder.Handle;
            }
            catch
            {
                _system.Dispose();
                throw;
            }
        }

        public ulong SystemMemoryBytes
        {
            get
            {
                ThrowIfDisposed();
                return NativeMethods.jps_system_memory_bytes(_systemHandle);
            }
        }

        public ulong PathfinderMemoryBytes
        {
            get
            {
                ThrowIfDisposed();
                return NativeMethods.jps_pathfinder_memory_bytes(_pathfinderHandle);
            }
        }

        public ulong MemoryBytes
        {
            get { return SystemMemoryBytes + PathfinderMemoryBytes; }
        }

        public int Find(int startX, int startY, int goalX, int goalY)
        {
            ThrowIfDisposed();
            return NativeMethods.jps_pathfinder_find_path(
                _pathfinderHandle, _systemHandle, startX, startY, goalX, goalY);
        }

        public void ForceDirty(int x, int y)
        {
            ThrowIfDisposed();
            NativeMethods.jps_system_set_blocked(_systemHandle, x, y, 1);
            NativeMethods.jps_system_set_blocked(_systemHandle, x, y, 0);
            NativeMethods.jps_system_sync(_systemHandle);
        }

        public void SetBlocked(int x, int y, bool blocked)
        {
            ThrowIfDisposed();
            NativeMethods.jps_system_set_blocked(_systemHandle, x, y, blocked ? 1 : 0);
        }

        public void SetBlockedBatch(
            IReadOnlyList<(int x, int y, bool oldBlocked)> edits, bool apply)
        {
            ThrowIfDisposed();
            if (edits == null)
                throw new ArgumentNullException(nameof(edits));
            int count = edits.Count;
            if (count == 0)
                return;

            int required = checked(count * 3);
            if (_editBuffer.Length < required)
                _editBuffer = new int[required];
            for (int i = 0; i < count; i++)
            {
                var edit = edits[i];
                _editBuffer[i * 3] = edit.x;
                _editBuffer[i * 3 + 1] = edit.y;
                _editBuffer[i * 3 + 2] =
                    (apply ? !edit.oldBlocked : edit.oldBlocked) ? 1 : 0;
            }

            NativeMethods.jps_system_set_blocked_batch(_systemHandle, _editBuffer, count);
        }

        public void Sync()
        {
            ThrowIfDisposed();
            NativeMethods.jps_system_sync(_systemHandle);
        }

        public (bool ok, List<(int X, int Y)>? path) FindAndCopy(
            int startX, int startY, int goalX, int goalY)
        {
            int result = Find(startX, startY, goalX, goalY);
            return result < 0
                ? (false, null)
                : (true, _pathfinder.CopyPath());
        }

        public void Dispose()
        {
            if (_disposed)
                return;
            _pathfinder.Dispose();
            _system.Dispose();
            _disposed = true;
        }

        private void ThrowIfDisposed()
        {
            if (_disposed)
                throw new ObjectDisposedException(nameof(NativeMap));
        }
    }
}
