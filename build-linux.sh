#!/usr/bin/env bash
#
# build-linux.sh — 在 Linux（arm64 / x86_64）上构建并准备运行 JPS.Accuracy 与 JPS.Benchmark。
#
# 做三件事：
#   1) 用 cc/gcc/clang 把 JPS.Native 的 6 个 .c + 1 个 .cpp（pathfinder.cpp）编成 libJPS.Native.so
#      （沿用 CMakeLists.txt 的关键编译选项，尤其是保证平滑路径与 C# 逐位一致的浮点确定性选项）；
#   2) dotnet build 两个托管工具（net10.0，框架依赖，跑在本机 .NET 运行时上）；
#   3) 把 .so 复制到各自的托管输出目录，使运行期 P/Invoke（DllImport "JPS.Native"）能找到它。
#
# 前置依赖：C 编译器（cc/gcc/clang）+ .NET 10 SDK。
#
# 用法：
#   bash build-linux.sh            # 构建全部（native + 两个工具）
#   CONFIG=Debug bash build-linux.sh
#   CC=clang bash build-linux.sh
#   LTO=0 bash build-linux.sh                       # 关掉 -flto（个别工具链链接期 LTO 有问题时）
#   EXTRA_CFLAGS="-mcpu=native" bash build-linux.sh # 追加 CPU 专属调优
#
# 构建后从仓库根目录运行：
#   dotnet JPS.Accuracy/bin/Release/net10.0/JPS.Accuracy.dll                 # 全量正确性
#   dotnet JPS.Accuracy/bin/Release/net10.0/JPS.Accuracy.dll dao-map 100    # 子集
#   dotnet JPS.Benchmark/bin/Release/net10.0/JPS.Benchmark.dll combo 1000   # 基准
#   dotnet JPS.Benchmark/bin/Release/net10.0/JPS.Benchmark.dll combo 1000 bg512-map
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CONFIG="${CONFIG:-Release}"
LTO="${LTO:-1}"
NATIVE_DIR="JPS.Native"
SO_NAME="libJPS.Native.so"
TFM="net10.0"

# ---- 选 C 编译器 ----
CC="${CC:-}"
if [ -z "$CC" ]; then
  for c in cc gcc clang; do
    if command -v "$c" >/dev/null 2>&1; then CC="$c"; break; fi
  done
fi
if [ -z "$CC" ]; then
  echo "错误：找不到 C 编译器（需要 cc/gcc/clang）。请安装 build-essential 或 clang。" >&2
  exit 1
fi
if ! command -v dotnet >/dev/null 2>&1; then
  echo "错误：找不到 dotnet（需要 .NET 10 SDK）。" >&2
  exit 1
fi

# ---- 按架构选 SIMD/arch 选项（与 CMakeLists.txt 的 ABI 分支对应）----
ARCH="$(uname -m)"
ARCH_FLAGS=()
case "$ARCH" in
  aarch64|arm64)      ARCH_FLAGS=(-march=armv8-a) ;;              # armv8 基线含 NEON；jps_simd.h 靠 __aarch64__ 选 NEON
  armv7l|armv7|armhf) ARCH_FLAGS=(-mfpu=neon) ;;                  # 32 位 ARM：开 NEON（float-abi 随工具链默认）
  x86_64|amd64)       ARCH_FLAGS=(-msse2) ;;
  i686|i386)          ARCH_FLAGS=(-msse2 -mfpmath=sse) ;;         # 32 位 x86：用 SSE 而非 x87，保证 float 与其它 ABI 一致
  *) echo "警告：未识别架构 '$ARCH'，不加架构专属选项（aarch64/x86_64 仍会自动走对应 SIMD）。" >&2 ;;
esac

# ---- 与 CMake/NDK 构建一致的公共选项（见 JPS.Native/CMakeLists.txt）----
#   -ffp-contract=off -fno-fast-math : 让平滑路径的浮点结果与 C# 参考逐位一致（accuracy 会逐点校验）
#   -fvisibility=hidden              : 只导出 JPS_API 标注的公共 jps_* 接口，和 Windows DLL 的导出面一致
# 不含 -std（按语言分别给）：.c 走 C11，pathfinder.cpp 走 C++17；后者加 -fno-exceptions -fno-rtti
# 去掉 C++ 异常/RTTI，于是无 libstdc++ 依赖，仍能用 C driver（$CC）链接。
CFLAGS=(-O3 -funroll-loops -fPIC -fvisibility=hidden -ffp-contract=off -fno-fast-math)
[ "$LTO" = "1" ] && CFLAGS+=(-flto)
# shellcheck disable=SC2206
[ -n "${EXTRA_CFLAGS:-}" ] && CFLAGS+=($EXTRA_CFLAGS)
CFLAGS+=("${ARCH_FLAGS[@]}")

echo "==> [1/3] 编译 $SO_NAME  (arch=$ARCH, CC=$CC, config=$CONFIG, lto=$LTO)"
OBJDIR="$(mktemp -d)"; trap 'rm -rf "$OBJDIR"' EXIT
OBJS=()
for src in "$NATIVE_DIR"/*.c; do
  o="$OBJDIR/$(basename "$src").o"
  "$CC" -std=c11 "${CFLAGS[@]}" -I"$NATIVE_DIR" -c "$src" -o "$o"; OBJS+=("$o")
done
for src in "$NATIVE_DIR"/*.cpp; do
  o="$OBJDIR/$(basename "$src").o"
  "$CC" -std=c++17 -fno-exceptions -fno-rtti "${CFLAGS[@]}" -I"$NATIVE_DIR" -c "$src" -o "$o"; OBJS+=("$o")
done
"$CC" "${CFLAGS[@]}" -shared "${OBJS[@]}" -o "$NATIVE_DIR/$SO_NAME"
echo "    -> $NATIVE_DIR/$SO_NAME"

# ---- 构建两个托管工具（框架依赖；非 Windows 上 vcxproj 引用已按平台条件化排除）----
echo "==> [2/3] dotnet build 托管工具 (config=$CONFIG)"
dotnet build JPS.Accuracy/JPS.Accuracy.csproj   -c "$CONFIG" --nologo -v minimal
dotnet build JPS.Benchmark/JPS.Benchmark.csproj -c "$CONFIG" --nologo -v minimal

# ---- 把 .so 放到各托管输出目录旁，供 DllImport("JPS.Native") 运行期解析 ----
echo "==> [3/3] 复制 $SO_NAME 到托管输出目录"
ACC_OUT="JPS.Accuracy/bin/$CONFIG/$TFM"
BEN_OUT="JPS.Benchmark/bin/$CONFIG/$TFM"
for out in "$ACC_OUT" "$BEN_OUT"; do
  if [ -d "$out" ]; then
    cp -f "$NATIVE_DIR/$SO_NAME" "$out/"
    echo "    -> $out/$SO_NAME"
  else
    echo "警告：输出目录不存在：$out（托管构建可能失败）" >&2
  fi
done

cat <<EOF

完成。从仓库根目录运行：

  # 正确性（全部地图、全部用例；耗时较长）
  dotnet $ACC_OUT/JPS.Accuracy.dll
  # 子集： dotnet $ACC_OUT/JPS.Accuracy.dll dao-map 100

  # 基准（combo，每图 1000 组随机 + 官方 .scen）
  dotnet $BEN_OUT/JPS.Benchmark.dll combo 1000
  # 子集： dotnet $BEN_OUT/JPS.Benchmark.dll combo 1000 bg512-map

结果分别写入 accuracy-results/ 与 benchmark-results/（相对仓库根，位置与运行时的 cwd 无关）。
EOF
