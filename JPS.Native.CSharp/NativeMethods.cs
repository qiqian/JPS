using System;
using System.Runtime.InteropServices;
using System.Security;

namespace JPS.Native
{
    [SuppressUnmanagedCodeSecurity]
    internal static class NativeMethods
    {
#if (UNITY_IOS && !UNITY_EDITOR) || JPS_NATIVE_IOS
        internal const string LibraryName = "__Internal";
#else
        internal const string LibraryName = "JPS.Native";
#endif

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr jps_system_create(int width, int height);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_system_destroy(IntPtr system);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_system_width(IntPtr system);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_system_height(IntPtr system);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong jps_system_memory_bytes(IntPtr system);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_system_set_blocked(IntPtr system, int x, int y, int blocked);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_system_is_blocked(IntPtr system, int x, int y);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_system_clear_all(IntPtr system);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_system_set_blocked_buffer(
            IntPtr system, [In] byte[] cells, int count);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_system_set_blocked_batch(
            IntPtr system, [In] int[] xyv, int editCount);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_system_sync(IntPtr system);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr jps_adapter_create(int width, int height, int obstaclePadding);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr jps_adapter_create_from_buffer(
            int width, int height, int obstaclePadding, [In] byte[] cells, int count);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_adapter_destroy(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_width(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_height(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_obstacle_padding(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_dynamic_obstacle_count(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_dynamic_covered_cell_count(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong jps_adapter_memory_bytes(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_set_obstacle_padding(IntPtr adapter, int obstaclePadding);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_is_static_blocked(IntPtr adapter, int x, int y);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_is_blocked(IntPtr adapter, int x, int y);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_update_dynamic_obstacle(
            IntPtr adapter, int id, int x, int y, int width, int height);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_get_dynamic_obstacle(
            IntPtr adapter, int id, out int x, out int y, out int width, out int height);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_adapter_clear_dynamic_obstacles(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr jps_adapter_system(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_adapter_sync(IntPtr adapter);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr jps_pathfinder_create();

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void jps_pathfinder_destroy(IntPtr pathfinder);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong jps_pathfinder_memory_bytes(IntPtr pathfinder);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_pathfinder_find_path(
            IntPtr pathfinder, IntPtr system, int startX, int startY, int goalX, int goalY);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_pathfinder_find_path_nearest(
            IntPtr pathfinder, IntPtr system, int startX, int startY, int goalX, int goalY);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_pathfinder_path_count(IntPtr pathfinder);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_pathfinder_reached_goal(IntPtr pathfinder);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_pathfinder_smoothed_path_count(IntPtr pathfinder);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_pathfinder_copy_path(
            IntPtr pathfinder, [Out] int[] outputXy, int capacityPoints);

        [DllImport(LibraryName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int jps_pathfinder_copy_smoothed_path(
            IntPtr pathfinder, [Out] float[] outputXy, int capacityPoints);
    }
}
