using System;

namespace JPS.Native
{
    public sealed class NativeAdapter : IDisposable
    {
        private IntPtr _handle;
        private IntPtr _systemHandle;

        public NativeAdapter(int width, int height, int obstaclePadding)
        {
            ValidateArguments(width, height, obstaclePadding);
            NativeJps.EnsureInitialized();
            _handle = NativeMethods.jps_adapter_create(width, height, obstaclePadding);
            if (_handle == IntPtr.Zero)
                throw new InvalidOperationException("jps_adapter_create returned NULL.");
            InitializeSystemHandle();
            Width = width;
            Height = height;
        }

        public NativeAdapter(int width, int height, int obstaclePadding, byte[] cells)
        {
            ValidateArguments(width, height, obstaclePadding);
            if (cells == null)
                throw new ArgumentNullException(nameof(cells));
            if (cells.Length != checked(width * height))
                throw new ArgumentException("The buffer length must equal width * height.", nameof(cells));

            NativeJps.EnsureInitialized();
            _handle = NativeMethods.jps_adapter_create_from_buffer(
                width, height, obstaclePadding, cells, cells.Length);
            if (_handle == IntPtr.Zero)
                throw new InvalidOperationException("jps_adapter_create_from_buffer returned NULL.");
            InitializeSystemHandle();
            Width = width;
            Height = height;
        }

        public NativeAdapter(
            int width, int height, int obstaclePadding, Func<int, int, bool> isBlocked)
            : this(width, height, obstaclePadding, BuildCells(width, height, isBlocked))
        {
        }

        ~NativeAdapter()
        {
            Dispose(false);
        }

        public int Width { get; }

        public int Height { get; }

        public int ObstaclePadding
        {
            get { return NativeMethods.jps_adapter_obstacle_padding(GetHandle()); }
        }

        public int DynamicObstacleCount
        {
            get { return NativeMethods.jps_adapter_dynamic_obstacle_count(GetHandle()); }
        }

        public int DynamicCoveredCellCount
        {
            get { return NativeMethods.jps_adapter_dynamic_covered_cell_count(GetHandle()); }
        }

        public ulong MemoryBytes
        {
            get { return NativeMethods.jps_adapter_memory_bytes(GetHandle()); }
        }

        internal IntPtr SystemHandle
        {
            get
            {
                GetHandle();
                return _systemHandle;
            }
        }

        public int SetObstaclePadding(int obstaclePadding)
        {
            return NativeMethods.jps_adapter_set_obstacle_padding(GetHandle(), obstaclePadding);
        }

        public bool IsStaticBlocked(int x, int y)
        {
            return NativeMethods.jps_adapter_is_static_blocked(GetHandle(), x, y) != 0;
        }

        public bool IsBlocked(int x, int y)
        {
            return NativeMethods.jps_adapter_is_blocked(GetHandle(), x, y) != 0;
        }

        public int UpdateDynamicObstacle(int id, int x, int y, int width, int height)
        {
            return NativeMethods.jps_adapter_update_dynamic_obstacle(
                GetHandle(), id, x, y, width, height);
        }

        public bool TryGetDynamicObstacle(
            int id, out int x, out int y, out int width, out int height)
        {
            int result = NativeMethods.jps_adapter_get_dynamic_obstacle(
                GetHandle(), id, out x, out y, out width, out height);
            if (result < 0)
                throw new InvalidOperationException("jps_adapter_get_dynamic_obstacle failed: " +
                    ((NativeJpsError)result).ToString());
            return result != 0;
        }

        public int ClearDynamicObstacles()
        {
            return NativeMethods.jps_adapter_clear_dynamic_obstacles(GetHandle());
        }

        public void Sync()
        {
            NativeMethods.jps_adapter_sync(GetHandle());
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        private void Dispose(bool disposing)
        {
            if (_handle != IntPtr.Zero)
            {
                NativeMethods.jps_adapter_destroy(_handle);
                _handle = IntPtr.Zero;
                _systemHandle = IntPtr.Zero;
            }
        }

        private IntPtr GetHandle()
        {
            if (_handle == IntPtr.Zero)
                throw new ObjectDisposedException(nameof(NativeAdapter));
            return _handle;
        }

        private static byte[] BuildCells(
            int width, int height, Func<int, int, bool> isBlocked)
        {
            ValidateArguments(width, height, 0);
            if (isBlocked == null)
                throw new ArgumentNullException(nameof(isBlocked));
            var cells = new byte[checked(width * height)];
            int index = 0;
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    cells[index++] = isBlocked(x, y) ? (byte)1 : (byte)0;
            return cells;
        }

        private void InitializeSystemHandle()
        {
            _systemHandle = NativeMethods.jps_adapter_system(_handle);
            if (_systemHandle == IntPtr.Zero)
            {
                IntPtr adapter = _handle;
                _handle = IntPtr.Zero;
                NativeMethods.jps_adapter_destroy(adapter);
                throw new InvalidOperationException("jps_adapter_system returned NULL.");
            }
        }

        private static void ValidateArguments(int width, int height, int obstaclePadding)
        {
            if (width <= 0 || width > short.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(width));
            if (height <= 0 || height > short.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(height));
            if (obstaclePadding < 0)
                throw new ArgumentOutOfRangeException(nameof(obstaclePadding));
            checked
            {
                _ = width * height;
            }
        }
    }
}
