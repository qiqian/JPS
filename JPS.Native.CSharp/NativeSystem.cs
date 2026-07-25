using System;

namespace JPS.Native
{
    public sealed class NativeSystem : IDisposable
    {
        private IntPtr _handle;

        public NativeSystem(int width, int height)
        {
            ValidateDimensions(width, height);
            Width = width;
            Height = height;
            _handle = CreateHandle(width, height);
        }

        public NativeSystem(int width, int height, byte[] cells)
        {
            ValidateDimensions(width, height);
            if (cells == null)
                throw new ArgumentNullException(nameof(cells));
            if (cells.Length != checked(width * height))
                throw new ArgumentException(
                    "The buffer length must equal width * height.", nameof(cells));

            Width = width;
            Height = height;
            _handle = CreateHandle(width, height);
            try
            {
                NativeMethods.jps_system_set_blocked_buffer(_handle, cells, cells.Length);
                NativeMethods.jps_system_sync(_handle);
            }
            catch
            {
                NativeMethods.jps_system_destroy(_handle);
                _handle = IntPtr.Zero;
                throw;
            }
        }

        public NativeSystem(int width, int height, Func<int, int, bool> isBlocked)
            : this(width, height, BuildCells(width, height, isBlocked))
        {
        }

        ~NativeSystem()
        {
            Dispose(false);
        }

        public int Width { get; }

        public int Height { get; }

        public ulong MemoryBytes
        {
            get { return NativeMethods.jps_system_memory_bytes(GetHandle()); }
        }

        internal IntPtr Handle
        {
            get { return GetHandle(); }
        }

        public void SetBlocked(int x, int y, bool blocked)
        {
            NativeMethods.jps_system_set_blocked(GetHandle(), x, y, blocked ? 1 : 0);
        }

        public bool IsBlocked(int x, int y)
        {
            return NativeMethods.jps_system_is_blocked(GetHandle(), x, y) != 0;
        }

        public void ClearAll()
        {
            NativeMethods.jps_system_clear_all(GetHandle());
        }

        public void SetBlockedBuffer(byte[] cells)
        {
            if (cells == null)
                throw new ArgumentNullException(nameof(cells));
            int expected = checked(Width * Height);
            if (cells.Length != expected)
                throw new ArgumentException("The buffer length must equal width * height.", nameof(cells));
            NativeMethods.jps_system_set_blocked_buffer(GetHandle(), cells, cells.Length);
        }

        public void SetBlockedBatch(int[] xyv, int editCount)
        {
            if (xyv == null)
                throw new ArgumentNullException(nameof(xyv));
            if (editCount < 0 || checked(editCount * 3) > xyv.Length)
                throw new ArgumentOutOfRangeException(nameof(editCount));
            NativeMethods.jps_system_set_blocked_batch(GetHandle(), xyv, editCount);
        }

        public void Sync()
        {
            NativeMethods.jps_system_sync(GetHandle());
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
                NativeMethods.jps_system_destroy(_handle);
                _handle = IntPtr.Zero;
            }
        }

        private IntPtr GetHandle()
        {
            if (_handle == IntPtr.Zero)
                throw new ObjectDisposedException(nameof(NativeSystem));
            return _handle;
        }

        private static void ValidateDimensions(int width, int height)
        {
            if (width <= 0 || width > short.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(width));
            if (height <= 0 || height > short.MaxValue)
                throw new ArgumentOutOfRangeException(nameof(height));
            checked
            {
                _ = width * height;
            }
        }

        private static IntPtr CreateHandle(int width, int height)
        {
            NativeJps.EnsureInitialized();
            IntPtr handle = NativeMethods.jps_system_create(width, height);
            if (handle == IntPtr.Zero)
                throw new InvalidOperationException("jps_system_create returned NULL.");
            return handle;
        }

        private static byte[] BuildCells(int width, int height, Func<int, int, bool> isBlocked)
        {
            ValidateDimensions(width, height);
            if (isBlocked == null)
                throw new ArgumentNullException(nameof(isBlocked));

            var cells = new byte[checked(width * height)];
            int index = 0;
            for (int y = 0; y < height; y++)
                for (int x = 0; x < width; x++)
                    cells[index++] = isBlocked(x, y) ? (byte)1 : (byte)0;
            return cells;
        }
    }
}
