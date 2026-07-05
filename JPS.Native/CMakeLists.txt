cmake_minimum_required(VERSION 3.22)   # aligned with the latest LTS NDK (r27)

# Suppress developer/compatibility warnings coming from bundled NDK CMake files
# (those ship cmake_minimum_required with older versions like 3.6 and trigger
# warnings such as "Compatibility with CMake < 3.10 will be removed...").
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)
project(jps_native C)

set(CMAKE_C_STANDARD 11)

if(NOT ANDROID)
  message(WARNING "This CMakeLists is intended for Android NDK builds (ANDROID variable not set).")
endif()

file(GLOB JPS_NATIVE_SOURCES
	"${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

add_library(jps SHARED ${JPS_NATIVE_SOURCES})
set_target_properties(jps PROPERTIES OUTPUT_NAME "jps")
target_include_directories(jps PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Common optimization flags
target_compile_options(jps PRIVATE -fPIC -O3 -funroll-loops)

# Hide internal symbols. ELF exports every non-static symbol by default (unlike a Windows DLL, which
# exports nothing unless marked dllexport). With -fvisibility=hidden only symbols explicitly given
# default visibility via JPS_API (see jps_export.h -> __attribute__((visibility("default")))) are
# exported, trimming the .so's dynamic symbol table to the public API (jps_system_* / jps_pathfinder_*)
# and matching the Windows DLL. Internal modules (grid_map / min_heap / jump_point_cache / smoother)
# become hidden; cross-.c calls inside the .so still resolve normally.
target_compile_options(jps PRIVATE -fvisibility=hidden)

# Deterministic / cross-platform-consistent floating point.
# The core pathfinding is all-integer; floats appear only in path smoothing (cell centers cx+0.5f).
# To keep the smoothed output bit-identical across ABIs (x86_64 / arm64 / armv7 / x86):
#   -ffp-contract=off : never fuse a*b+c into an FMA. An FMA rounds once, a separate mul+add rounds twice,
#                       so an arch with FMA (ARM, x86 FMA3) would diverge from one without it — the single
#                       biggest source of x86-vs-ARM float mismatch. clang defaults to 'on', so set it.
#   -fno-fast-math    : strict IEEE-754 (no reassociation / approximate reciprocals). -O3 does NOT enable
#                       fast-math, but state it explicitly so it can't leak in via toolchain defaults.
target_compile_options(jps PRIVATE -ffp-contract=off -fno-fast-math)

# ABI-specific tuning: enable SIMD backend defines and arch flags
if(CMAKE_ANDROID_ARCH_ABI STREQUAL "arm64-v8a")
  target_compile_options(jps PRIVATE -march=armv8-a)
  target_compile_definitions(jps PRIVATE JPS_SIMD_NEON=1)
elseif(CMAKE_ANDROID_ARCH_ABI STREQUAL "armeabi-v7a")
  target_compile_options(jps PRIVATE -mfpu=neon -mfloat-abi=softfp)
  target_compile_definitions(jps PRIVATE JPS_SIMD_NEON=1)
elseif(CMAKE_ANDROID_ARCH_ABI STREQUAL "x86_64")
  target_compile_options(jps PRIVATE -msse2)
  target_compile_definitions(jps PRIVATE JPS_SIMD_SSE2=1)
elseif(CMAKE_ANDROID_ARCH_ABI STREQUAL "x86")
  # 32-bit x86: force SSE scalar math so floats stay 32/64-bit IEEE instead of x87's 80-bit extended
  # precision — otherwise intermediate float results differ from x86_64 / ARM.
  target_compile_options(jps PRIVATE -msse2 -mfpmath=sse)
  target_compile_definitions(jps PRIVATE JPS_SIMD_SSE2=1)
endif()

# Enable LTO for Android clang builds so cross-translation-unit calls (eg. pathfinder.c -> jump_point_cache.c)
# can be inlined. Only enable when building with Clang/LLVM under Android.
if(ANDROID)
  if(CMAKE_C_COMPILER_ID MATCHES "Clang")
	message(STATUS "Enabling ThinLTO for Android Clang build with lld")
    target_compile_options(jps PRIVATE -flto=thin)
    target_link_options(jps PRIVATE -flto=thin -fuse-ld=lld)
  endif()
endif()

# Ensure the shared library (.so) is copied to a predictable output directory after build
set(OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/lib/${CMAKE_ANDROID_ARCH_ABI})
add_custom_command(TARGET jps POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_DIR}
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:jps> ${OUTPUT_DIR}
)
