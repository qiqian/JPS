@echo off
setlocal enabledelayedexpansion

REM ndkbuild.bat
REM Windows helper to build JPS.Native with Android NDK + CMake for one or more ABIs.
REM Mirrors ndkbuild.sh. Usage examples:
REM   ndkbuild.bat
REM   ndkbuild.bat --ndk-path C:\path\to\ndk --abis "arm64-v8a;armeabi-v7a" --api 21

set "NDK_PATH=%ANDROID_NDK_HOME%"
REM Android targets ARM only (arm64-v8a covers all modern 64-bit devices; Play Store requires 64-bit).
REM Add armeabi-v7a via --abis for legacy 32-bit ARM. x86/x86_64 emulator ABIs are intentionally omitted.
set "ABIS=arm64-v8a"
set "API=21"
set "BUILD_ROOT=build-android"
REM Latest LTS NDK (LLVM 18, 16 KB page-size alignment, min API 21). Only used by the auto-download path.
REM If a newer LTS/patch exists, bump this - see https://developer.android.com/ndk/downloads
set "NDK_VERSION=r27d"

REM ---- parse args ----
:parse
if "%~1"=="" goto after_parse
if /i "%~1"=="--ndk-path"   ( set "NDK_PATH=%~2" & shift & shift & goto parse )
if /i "%~1"=="--abis"       ( set "ABIS=%~2"     & shift & shift & goto parse )
if /i "%~1"=="--api"        ( set "API=%~2"      & shift & shift & goto parse )
if /i "%~1"=="--build-root" ( set "BUILD_ROOT=%~2" & shift & shift & goto parse )
if /i "%~1"=="-h"     goto usage
if /i "%~1"=="--help" goto usage
echo Unknown argument: %~1 1>&2
goto usage

:after_parse
set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
REM Build/configure relative to the script dir (CMakeLists.txt lives here), like the .sh's `-S .`.
cd /d "%SCRIPT_DIR%"

if "%NDK_PATH%"=="" call :find_or_get_ndk
if "%NDK_PATH%"=="" (
  echo Failed to obtain NDK. Set --ndk-path or ANDROID_NDK_HOME. 1>&2
  exit /b 1
)

where cmake >nul 2>&1
if errorlevel 1 (
  echo cmake not found in PATH. Please install CMake and ensure it's available. 1>&2
  exit /b 1
)

REM NDK CMake cross-compilation REQUIRES the toolchain file; without it ANDROID/CMAKE_ANDROID_ARCH_ABI
REM are unset, so the SIMD arch flags/defines in CMakeLists never apply and the build fails.
set "TOOLCHAIN=%NDK_PATH%\build\cmake\android.toolchain.cmake"
if not exist "%TOOLCHAIN%" (
  echo Android CMake toolchain not found: %TOOLCHAIN% 1>&2
  echo The NDK path looks wrong or incomplete ^(expected ^<ndk^>\build\cmake\android.toolchain.cmake^). 1>&2
  exit /b 1
)

REM On Windows the default CMake generator is Visual Studio, which cannot cross-compile for Android.
REM Force Ninja - the NDK ships one under prebuilt\windows-x86_64\bin; else fall back to PATH.
set "NINJA=%NDK_PATH%\prebuilt\windows-x86_64\bin\ninja.exe"
if not exist "%NINJA%" set "NINJA=ninja"

set "JOBS=%NUMBER_OF_PROCESSORS%"
if "%JOBS%"=="" set "JOBS=4"

echo Using NDK:   %NDK_PATH%
echo ABIs:        %ABIS%
echo API:         %API%
echo Build root:  %BUILD_ROOT%
echo Ninja:       %NINJA%

REM ---- iterate ABIs (semicolon-separated) ----
for %%a in ("%ABIS:;=" "%") do (
  set "ABI=%%~a"
  if not "!ABI!"=="" (
    call :build_abi "!ABI!"
    if errorlevel 1 exit /b 1
  )
)

echo All ABIs built. Each libjps.so is under %BUILD_ROOT%\^<abi^>\lib\^<abi^>\
exit /b 0

REM ============================================================
:build_abi
set "ABI=%~1"
set "BUILD_DIR=%BUILD_ROOT%\%ABI%"
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"

echo Configuring for ABI: %ABI%...
cmake -G Ninja -DCMAKE_MAKE_PROGRAM="%NINJA%" -S . -B "%BUILD_DIR%" ^
  -DCMAKE_TOOLCHAIN_FILE="%TOOLCHAIN%" ^
  -DANDROID_NDK="%NDK_PATH%" ^
  -DANDROID_ABI="%ABI%" ^
  -DANDROID_PLATFORM=android-%API% ^
  -DCMAKE_BUILD_TYPE=Release ^
  -DANDROID_STL=c++_static
if errorlevel 1 exit /b 1

echo Building for ABI: %ABI%...
cmake --build "%BUILD_DIR%" --config Release --parallel %JOBS%
if errorlevel 1 exit /b 1

echo Built %ABI% -^> %BUILD_DIR%
exit /b 0

REM ============================================================
:find_or_get_ndk
REM Keep auto-downloaded NDKs under ndk\<platform> to avoid interfering with other scripts.
set "platform=windows"
set "NDK_DIR=%SCRIPT_DIR%\ndk\%platform%"
if not exist "%NDK_DIR%" mkdir "%NDK_DIR%"

REM Look for a repo-local copy under ndk\<platform> only. A different local version is NOT reused.
for /d %%d in ("%NDK_DIR%\android-ndk-%NDK_VERSION%*" "%NDK_DIR%\ndk-%NDK_VERSION%*") do (
  if exist "%%d\build\cmake\android.toolchain.cmake" (
    set "NDK_PATH=%%d"
    echo Found local NDK at: %%d
    exit /b 0
  )
)

echo NDK not found under %NDK_DIR%. Preparing to obtain NDK %NDK_VERSION% for %platform%...
set "NDK_FILE=android-ndk-%NDK_VERSION%-windows.zip"
set "URL=https://dl.google.com/android/repository/%NDK_FILE%"
set "DOWNLOAD_PATH=%NDK_DIR%\%NDK_FILE%"

if exist "%DOWNLOAD_PATH%" (
  echo Found existing NDK archive: %DOWNLOAD_PATH% (skipping download)
) else (
  echo Downloading %URL% to %DOWNLOAD_PATH% ...
  rem Use BITS for robust download/resume (PowerShell Start-BitsTransfer)
  powershell -NoProfile -Command "Try { Start-BitsTransfer -Source '%URL%' -Destination '%DOWNLOAD_PATH%'; exit 0 } Catch { Write-Error $_; exit 1 }"
  if errorlevel 1 (
    echo Download failed. Install NDK manually and pass --ndk-path. 1>&2
    exit /b 1
  )
)

REM already extracted?
for /d %%d in ("%NDK_DIR%\android-ndk-%NDK_VERSION%*") do (
  if exist "%%d\build\cmake\android.toolchain.cmake" (
    set "NDK_PATH=%%d"
    echo Found existing extracted NDK at: %%d
    exit /b 0
  )
)

echo Extracting %DOWNLOAD_PATH% ...
powershell -NoProfile -Command "Expand-Archive -Path '%DOWNLOAD_PATH%' -DestinationPath '%NDK_DIR%' -Force"
if errorlevel 1 (
  echo Extraction failed. 1>&2
  exit /b 1
)

for /d %%d in ("%NDK_DIR%\android-ndk-%NDK_VERSION%*") do (
  if exist "%%d\build\cmake\android.toolchain.cmake" (
    set "NDK_PATH=%%d"
    echo NDK installed to: %%d
    exit /b 0
  )
)

echo Failed to locate extracted NDK folder after extraction. 1>&2
exit /b 1

REM ============================================================
:usage
echo Usage: %~nx0 [--ndk-path PATH] [--abis "arm64-v8a"] [--api 21] [--build-root build-android]
echo.
echo If --ndk-path and ANDROID_NDK_HOME are not provided, the script looks for a local
echo android-ndk-%NDK_VERSION%* folder next to this script. If none is found, it downloads
echo NDK %NDK_VERSION% from Google's servers and extracts it (PowerShell Expand-Archive).
exit /b 0
