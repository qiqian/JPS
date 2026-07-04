#!/usr/bin/env bash
set -euo pipefail

# ndkbuild.sh
# Linux / macOS helper to build JPS.Native with Android NDK + CMake for multiple ABIs.
# Auto-downloaded NDKs are kept under ndk/<platform>/ (linux|darwin|windows) so different
# OSes sharing this repo never clobber each other.
# Usage examples:
#   ./ndkbuild.sh                     # uses ANDROID_NDK_HOME or ndk/<platform>/... or auto-download
#   ./ndkbuild.sh --ndk-path /path/to/ndk --abis "arm64-v8a;armeabi-v7a" --api 21

NDK_PATH="${ANDROID_NDK_HOME:-}"
# Android targets ARM only (arm64-v8a covers all modern 64-bit devices; Play Store requires 64-bit).
# Add armeabi-v7a via --abis for legacy 32-bit ARM. x86/x86_64 emulator ABIs are intentionally omitted.
ABIS="arm64-v8a"
API=21
BUILD_ROOT="build-android"
# Latest LTS NDK (LLVM 18, default 16 KB page-size alignment, min API 21). Only used by the auto-download
# path below. If a newer LTS / patch exists, bump this — see https://developer.android.com/ndk/downloads
NDK_VERSION="r27d"

print_usage() {
  cat <<EOF
Usage: $0 [--ndk-path PATH] [--abis "arm64-v8a"] [--api 21] [--build-root build-android]

If --ndk-path and ANDROID_NDK_HOME are not provided, the script looks for a copy of NDK
$NDK_VERSION under ndk/<platform>/ next to this script. If none is found, it downloads
NDK $NDK_VERSION from Google's servers into ndk/<platform>/ (supports Linux and macOS/darwin).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ndk-path)   NDK_PATH="$2"; shift 2;;
    --abis)       ABIS="$2"; shift 2;;
    --api)        API="$2"; shift 2;;
    --build-root) BUILD_ROOT="$2"; shift 2;;
    -h|--help)    print_usage; exit 0;;
    *) echo "Unknown argument: $1" >&2; print_usage; exit 2;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "$NDK_PATH" ]]; then
  # Detect platform up-front so the NDK is kept under ndk/<platform>/ (OSes stay isolated).
  uname_s="$(uname -s)"
  case "$uname_s" in
    Linux*)   platform=linux;  ndk_file="android-ndk-$NDK_VERSION-linux.zip";;
    Darwin*)  platform=darwin; ndk_file="android-ndk-$NDK_VERSION-darwin.zip";;
    MINGW*|MSYS*|CYGWIN*|Windows_NT*) platform=windows; ndk_file="android-ndk-$NDK_VERSION-windows.zip";;
    *) echo "Automatic NDK download is supported only on Windows, Linux and macOS. Please install NDK and set --ndk-path or ANDROID_NDK_HOME." >&2; exit 1;;
  esac

  ndk_dir="$SCRIPT_DIR/ndk/$platform"

  # Look for a copy of the requested NDK version under ndk/<platform>/ only. A different version is
  # NOT silently reused, so bumping NDK_VERSION actually takes effect (triggers a fresh download).
  local_ndk=""
  for d in "$ndk_dir"/android-ndk-$NDK_VERSION*; do
    if [[ -f "$d/build/cmake/android.toolchain.cmake" ]]; then
      local_ndk="$d"
      break
    fi
  done

  if [[ -n "$local_ndk" ]]; then
    NDK_PATH="$local_ndk"
    echo "Found local NDK at: $NDK_PATH"
  else
    echo "NDK not found under $ndk_dir. Preparing to obtain NDK $NDK_VERSION for $platform..."
    mkdir -p "$ndk_dir"

    url="https://dl.google.com/android/repository/$ndk_file"
    download_path="$ndk_dir/$ndk_file"

    if [[ -f "$download_path" ]]; then
      echo "Found existing NDK archive: $download_path (skipping download)"
    else
      if command -v curl >/dev/null 2>&1; then
        echo "Downloading $url ..."
        curl -L --fail -o "$download_path" "$url"
      elif command -v wget >/dev/null 2>&1; then
        echo "Downloading $url ..."
        wget -O "$download_path" "$url"
      else
        echo "Neither curl nor wget found. Please install one to enable automatic download." >&2
        exit 1
      fi
    fi

    # Extract into ndk/<platform>/ (skip if already extracted there).
    extracted=""
    for d in "$ndk_dir"/android-ndk-$NDK_VERSION*; do
      if [[ -f "$d/build/cmake/android.toolchain.cmake" ]]; then extracted="$d"; break; fi
    done

    if [[ -z "$extracted" ]]; then
      if command -v unzip >/dev/null 2>&1; then
        echo "Extracting $download_path ..."
        unzip -q -o "$download_path" -d "$ndk_dir"
      elif command -v pwsh >/dev/null 2>&1; then
        echo "Extracting with pwsh Expand-Archive ..."
        pwsh -NoProfile -Command "Expand-Archive -Path \"$download_path\" -DestinationPath \"$ndk_dir\" -Force"
      elif command -v powershell.exe >/dev/null 2>&1; then
        echo "Extracting with powershell Expand-Archive ..."
        powershell.exe -NoProfile -Command "Expand-Archive -Path \"$download_path\" -DestinationPath \"$ndk_dir\" -Force"
      else
        echo "unzip not found and no PowerShell available to extract archive. Please install unzip or provide NDK manually." >&2
        exit 1
      fi

      for d in "$ndk_dir"/android-ndk-$NDK_VERSION*; do
        if [[ -f "$d/build/cmake/android.toolchain.cmake" ]]; then extracted="$d"; break; fi
      done
      if [[ -z "$extracted" ]]; then
        echo "Failed to locate extracted NDK folder after extraction." >&2
        exit 1
      fi
    fi

    NDK_PATH="$extracted"
    echo "NDK installed to: $NDK_PATH"
  fi
fi

if ! command -v cmake >/dev/null 2>&1; then
  echo "cmake not found in PATH. Please install CMake and ensure it's available." >&2
  exit 1
fi

echo "Using NDK: $NDK_PATH"
echo "ABIs: $ABIS"
echo "API: $API"
echo "Build root: $BUILD_ROOT"

# NDK CMake cross-compilation REQUIRES the toolchain file; without it ANDROID/CMAKE_ANDROID_ARCH_ABI
# are unset, so the SIMD arch flags/defines in CMakeLists never apply and the build fails.
TOOLCHAIN="$NDK_PATH/build/cmake/android.toolchain.cmake"
if [[ ! -f "$TOOLCHAIN" ]]; then
  echo "Android CMake toolchain not found: $TOOLCHAIN" >&2
  echo "The NDK path looks wrong or incomplete (expected <ndk>/build/cmake/android.toolchain.cmake)." >&2
  exit 1
fi

# detect CPU count
if command -v nproc >/dev/null 2>&1; then
  JOBS=$(nproc)
elif [[ "$(uname -s)" = "Darwin" ]]; then
  JOBS=$(sysctl -n hw.ncpu)
else
  JOBS=4
fi

IFS=';'
for abi in $ABIS; do
  abi=$(echo "$abi" | xargs)
  if [[ -z "$abi" ]]; then
    continue
  fi

  build_dir="$BUILD_ROOT/$abi"
  mkdir -p "$build_dir"

  args=(
    -S .
    -B "$build_dir"
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"
    -DANDROID_NDK="$NDK_PATH"
    -DANDROID_ABI="$abi"
    -DANDROID_PLATFORM=android-$API
    -DCMAKE_BUILD_TYPE=Release
    -DANDROID_STL=c++_static
  )

  echo "Configuring for ABI: $abi..."
  cmake "${args[@]}"

  echo "Building for ABI: $abi..."
  cmake --build "$build_dir" --config Release -- -j"$JOBS"

  echo "Built $abi -> $build_dir"
done

echo "All ABIs built. Each libjps.so is under $BUILD_ROOT/<abi>/lib/<abi>/"
