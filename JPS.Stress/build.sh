#!/usr/bin/env bash
#
# build.sh — 编译独立的 JPS C 压力测试（stress.c + JPS.Native 源码，静态直连，无 P/Invoke）。
#
# 用 -DJPS_STATIC 让 jps.h 的 JPS_API 变空（既不 export 也不 import），把 6 个 native .c 和
# stress.c 一起编成一个可执行文件。跑在有 C 编译器（cc/gcc/clang）的任意平台。
#
# 用法：
#   bash build.sh                 # 生成 ./stress
#   CC=gcc bash build.sh
#   bash build.sh && ./stress ../movingai/bg512-map/AR0011SR.map --rand 2000
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
NATIVE="../JPS.Native"

CC="${CC:-}"
if [ -z "$CC" ]; then
  for c in cc clang gcc; do
    if command -v "$c" >/dev/null 2>&1; then CC="$c"; break; fi
  done
fi
[ -z "$CC" ] && { echo "找不到 C 编译器（需要 cc/clang/gcc）。" >&2; exit 1; }

# 架构：x86_64 / aarch64 的 SIMD 后端由 jps_simd.h 靠预定义宏自动选，无需额外 flag；
# 32 位平台补上开关。
ARCH_FLAGS=()
case "$(uname -m)" in
  armv7l|armv7|armhf) ARCH_FLAGS=(-mfpu=neon) ;;
  i686|i386)          ARCH_FLAGS=(-msse2 -mfpmath=sse) ;;
esac

mkdir -p bin

echo "==> CC=$CC  编译 stress（JPS_STATIC，含 $NATIVE/*.c）"
"$CC" -std=c11 -O2 "${ARCH_FLAGS[@]}" -DJPS_STATIC \
      -I"$NATIVE" stress.c "$NATIVE"/*.c -o bin/stress

echo "    -> ./bin/stress"
echo ""
echo "示例："
echo "  ./bin/stress ../movingai/bg512-map/AR0011SR.map --rand 2000"
echo "  ./bin/stress ../movingai/dao-map/orz900d.map --rand 500 --reps 3"
echo "  ./bin/stress <map> --no-scen --rand 100000     # 只跑随机对、量大"
