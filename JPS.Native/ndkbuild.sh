#!/usr/bin/env bash
set -euo pipefail

# ndkbuild.sh
# Linux / macOS helper to build JPS.Native with Android NDK + CMake for multiple ABIs.
# Usage examples:
#   ./ndkbuild.sh                     # uses ANDROID_NDK_HOME or repo-local ndk or auto-download
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

If --ndk-path and ANDROID_NDK_HOME are not provided, the script will look for a local
android-ndk* or ndk* folder in the repository. If none is found, it will attempt to
download NDK $NDK_VERSION from Google's servers (supports Linux and macOS/darwin).
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
	--ndk-path)
	  NDK_PATH="$2"; shift 2;;
	--abis)
	  ABIS="$2"; shift 2;;
	--api)
	  API="$2"; shift 2;;
	--build-root)
	  BUILD_ROOT="$2"; shift 2;;
	-h|--help)
	  print_usage; exit 0;;
	*)
	  echo "Unknown argument: $1" >&2; print_usage; exit 2;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -z "$NDK_PATH" ]]; then
  # Look for a repo-local copy of the requested NDK version only. A different local version is NOT
  # silently reused, so bumping NDK_VERSION actually takes effect (triggers a fresh download below).
  local_ndk=""
  for d in "$SCRIPT_DIR"/android-ndk-$NDK_VERSION* "$SCRIPT_DIR"/ndk-$NDK_VERSION*; do
	if [[ -d "$d" ]]; then
	  local_ndk="$d"
	  break
	fi
  done

  if [[ -n "$local_ndk" ]]; then
	NDK_PATH="$local_ndk"
	echo "Found local NDK at: $NDK_PATH"
  else
	echo "NDK not found in repository. Preparing to obtain NDK $NDK_VERSION..."

	uname_s="$(uname -s)"
	case "$uname_s" in
	  Linux*)   platform=linux; ndk_file="android-ndk-$NDK_VERSION-linux.zip";;
	  Darwin*)  platform=darwin; ndk_file="android-ndk-$NDK_VERSION-darwin.zip";;
	  MINGW*|MSYS*|CYGWIN*|Windows_NT*) platform=windows; ndk_file="android-ndk-$NDK_VERSION-windows.zip";;
	  *) echo "Automatic NDK download is supported only on Windows, Linux and macOS. Please install NDK and set --ndk-path or ANDROID_NDK_HOME." >&2; exit 1;;
	esac

	url="https://dl.google.com/android/repository/$ndk_file"
	download_path="$SCRIPT_DIR/$ndk_file"

	# if archive already exists, skip download
	if [[ -f "$download_path" ]]; then
	  echo "Found existing NDK archive: $download_path (skipping download)"
	else
	  # download using curl or wget
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

	# if already extracted, use it; otherwise extract
	extracted=""
	for d in "$SCRIPT_DIR"/android-ndk-$NDK_VERSION* "$SCRIPT_DIR"/ndk*; do
	  if [[ -d "$d" ]]; then
		extracted="$d"
		break
	  fi
	done

	if [[ -n "$extracted" ]]; then
	  NDK_PATH="$extracted"
	  echo "Found existing extracted NDK at: $NDK_PATH"
	else
	  # extract archive
	  if [[ -f "$download_path" ]]; then
		if command -v unzip >/dev/null 2>&1; then
		  echo "Extracting $download_path ..."
		  unzip -q -o "$download_path" -d "$SCRIPT_DIR"
		else
		  # try PowerShell Expand-Archive if available (Windows)
		  if command -v pwsh >/dev/null 2>&1; then
			echo "Extracting with pwsh Expand-Archive ..."
			pwsh -NoProfile -Command "Expand-Archive -Path \"$download_path\" -DestinationPath \"$SCRIPT_DIR\" -Force"
		  elif command -v powershell.exe >/dev/null 2>&1; then
			echo "Extracting with powershell Expand-Archive ..."
			powershell.exe -NoProfile -Command "Expand-Archive -Path \"$download_path\" -DestinationPath \"$SCRIPT_DIR\" -Force"
		  else
			echo "unzip not found and no PowerShell available to extract archive. Please install unzip or provide NDK manually." >&2
			exit 1
		  fi
		fi
	  else
		echo "NDK archive not available for extraction: $download_path" >&2
		exit 1
	  fi

	  # find extracted folder
	  for d in "$SCRIPT_DIR"/android-ndk-$NDK_VERSION* "$SCRIPT_DIR"/ndk*; do
		if [[ -d "$d" ]]; then
		  extracted="$d"
		  break
		fi
	  done

	  if [[ -z "$extracted" ]]; then
		echo "Failed to locate extracted NDK folder after extraction." >&2
		exit 1
	  fi

	  NDK_PATH="$extracted"
	  echo "NDK installed to: $NDK_PATH"
	fi
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
