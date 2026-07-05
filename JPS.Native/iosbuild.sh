#!/usr/bin/env bash
#
# iosbuild.sh — 把 JPS.Native 编成 iOS 静态库（.a），并打包成 JPS.Native.xcframework。
#
# 必须在 **macOS** 上运行，且已装 Xcode 命令行工具（xcode-select --install）。
# 产物（默认 build-ios/ 下）：
#   iphoneos/libJPS.Native.a            真机     (arm64)
#   iphonesimulator/libJPS.Native.a     模拟器   (arm64 + x86_64，lipo 合并)
#   include/                            公共头（消费方 #include "jps.h" 即可）
#   JPS.Native.xcframework              把真机 + 模拟器两片按平台分装，拖进 Xcode 直接用
#
# 编译选项与 CMakeLists.txt / build-linux.sh 保持一致：
#   -ffp-contract=off -fno-fast-math  让平滑路径浮点结果与其它 ABI 逐位一致
#   -fvisibility=hidden               只导出 JPS_API 标注的公共 jps_* 接口
# iOS 是 arm64 → jps_simd.h 靠 __aarch64__ 自动走 NEON，无需额外定义。
#
# 用法：
#   bash iosbuild.sh                      # 真机 + 模拟器 + xcframework（默认）
#   bash iosbuild.sh --device-only        # 只编真机 arm64 静态库
#   bash iosbuild.sh --no-xcframework     # 编两片 .a 但不打 xcframework
#   MIN_VERSION=12.0 bash iosbuild.sh     # 改最低部署版本（默认 13.0）
#
# Unity / 原生 iOS 集成提示：静态库会被链接进 App 本体，托管侧 P/Invoke 用
#   [DllImport("__Internal")]  —— iOS 上静态链接的符号统一走 "__Internal"。
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MIN_VERSION="${MIN_VERSION:-13.0}"
BUILD_ROOT="${BUILD_ROOT:-build-ios}"
LIB_NAME="libJPS.Native.a"
FRAMEWORK_NAME="JPS.Native"
DEVICE_ONLY=0
MAKE_XCFRAMEWORK=1

# ---- parse args ----
while [ $# -gt 0 ]; do
  case "$1" in
    --min-version)   MIN_VERSION="$2"; shift 2 ;;
    --build-root)    BUILD_ROOT="$2"; shift 2 ;;
    --device-only)   DEVICE_ONLY=1; shift ;;
    --no-xcframework) MAKE_XCFRAMEWORK=0; shift ;;
    -h|--help)
      sed -n '2,30p' "$0"
      exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 1 ;;
  esac
done

# ---- 环境检查 ----
if [ "$(uname -s)" != "Darwin" ]; then
  echo "错误：iOS 静态库只能在 macOS 上用 Xcode 工具链编译（当前不是 macOS）。" >&2
  exit 1
fi
if ! command -v xcrun >/dev/null 2>&1; then
  echo "错误：找不到 xcrun。请先安装 Xcode 命令行工具：xcode-select --install" >&2
  exit 1
fi

# ---- 编译选项（与 CMakeLists.txt 公共选项一致）----
# 不含 -std（按语言分别给）：.c 走 C11，pathfinder.cpp 走 C++17（+ -fno-exceptions -fno-rtti）。
CFLAGS=(-O3 -funroll-loops -fPIC -fvisibility=hidden -ffp-contract=off -fno-fast-math)
# shellcheck disable=SC2206
[ -n "${EXTRA_CFLAGS:-}" ] && CFLAGS+=($EXTRA_CFLAGS)

rm -rf "$BUILD_ROOT"
mkdir -p "$BUILD_ROOT"

# 公共头：只发布 jps.h（自包含，内含导出宏 / 返回码 / 不透明句柄 / 全部公共函数）。
HDR_DIR="$BUILD_ROOT/include"
mkdir -p "$HDR_DIR"
cp -f ./jps.h "$HDR_DIR"/

# build_slice <sdk> <version-min-flag> <arch...> → 产出 $BUILD_ROOT/<sdk>/libJPS.Native.a
# 每个 arch 先各自编 .o 并归档成 .a，再 lipo 合并同一 SDK 下的多 arch。
build_slice() {
  local sdk="$1" verflag="$2"; shift 2
  local archs=("$@")
  local sdkpath clang
  sdkpath="$(xcrun --sdk "$sdk" --show-sdk-path)"
  clang="$(xcrun --sdk "$sdk" -f clang)"

  local archlibs=()
  local arch src objdir archlib
  for arch in "${archs[@]}"; do
    objdir="$BUILD_ROOT/obj/$sdk-$arch"
    mkdir -p "$objdir"
    for src in *.c; do
      "$clang" -arch "$arch" -isysroot "$sdkpath" "$verflag" -std=c11 "${CFLAGS[@]}" \
               -I. -c "$src" -o "$objdir/$src.o"
    done
    for src in *.cpp; do
      "$clang" -arch "$arch" -isysroot "$sdkpath" "$verflag" -std=c++17 -fno-exceptions -fno-rtti "${CFLAGS[@]}" \
               -I. -c "$src" -o "$objdir/$src.o"
    done
    archlib="$BUILD_ROOT/obj/$sdk-$arch.a"
    xcrun libtool -static -o "$archlib" "$objdir"/*.o
    archlibs+=("$archlib")
  done

  local outdir="$BUILD_ROOT/$sdk"
  mkdir -p "$outdir"
  if [ "${#archlibs[@]}" -gt 1 ]; then
    lipo -create -output "$outdir/$LIB_NAME" "${archlibs[@]}"
  else
    cp -f "${archlibs[0]}" "$outdir/$LIB_NAME"
  fi
  echo "  -> $outdir/$LIB_NAME  (${archs[*]}, min $MIN_VERSION)"
}

echo "==> [1] 真机静态库 iphoneos/arm64"
build_slice iphoneos "-miphoneos-version-min=$MIN_VERSION" arm64

if [ "$DEVICE_ONLY" -eq 0 ]; then
  echo "==> [2] 模拟器静态库 iphonesimulator/arm64+x86_64"
  build_slice iphonesimulator "-mios-simulator-version-min=$MIN_VERSION" arm64 x86_64
fi

if [ "$MAKE_XCFRAMEWORK" -eq 1 ] && [ "$DEVICE_ONLY" -eq 0 ]; then
  echo "==> [3] 打包 $FRAMEWORK_NAME.xcframework"
  if ! command -v xcodebuild >/dev/null 2>&1; then
    echo "警告：找不到 xcodebuild，跳过 xcframework（两片 .a 已生成）。" >&2
  else
    xcodebuild -create-xcframework \
      -library "$BUILD_ROOT/iphoneos/$LIB_NAME" -headers "$HDR_DIR" \
      -library "$BUILD_ROOT/iphonesimulator/$LIB_NAME" -headers "$HDR_DIR" \
      -output "$BUILD_ROOT/$FRAMEWORK_NAME.xcframework" >/dev/null
    echo "  -> $BUILD_ROOT/$FRAMEWORK_NAME.xcframework"
  fi
fi

# 清掉中间 .o / 单 arch 归档，只留最终产物。
rm -rf "$BUILD_ROOT/obj"

echo ""
echo "完成。产物在 $BUILD_ROOT/："
echo "  - 真机 .a：      $BUILD_ROOT/iphoneos/$LIB_NAME"
[ "$DEVICE_ONLY" -eq 0 ] && echo "  - 模拟器 .a：    $BUILD_ROOT/iphonesimulator/$LIB_NAME"
[ "$MAKE_XCFRAMEWORK" -eq 1 ] && [ "$DEVICE_ONLY" -eq 0 ] && echo "  - xcframework：   $BUILD_ROOT/$FRAMEWORK_NAME.xcframework（推荐，拖进 Xcode 即用）"
echo "  - 公共头：        $BUILD_ROOT/include/（#include \"jps.h\"）"
echo ""
echo "iOS 上静态库链接进 App 本体，托管侧 P/Invoke 用 [DllImport(\"__Internal\")]。"
