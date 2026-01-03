# Rebuild NLopt with Luksan, install to third_party, rebuild FastPlanner, copy DLL, run main and plotting script
# Usage: Run from PowerShell as Administrator (if needed)

set -e
$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $MyInvocation.MyCommand.Definition
# repo root
$repo = Resolve-Path "$root\.." | Select-Object -ExpandProperty Path
Write-Host "Repo root: $repo"

$nlopt_src = Join-Path $repo "files\nlopt-master"
$nlopt_build = Join-Path $nlopt_src "build"
$nlopt_install = Join-Path $repo "third_party\nlopt"

# Configure NLopt
Write-Host "Configure NLopt..."
cmake -S $nlopt_src -B $nlopt_build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DNLOPT_LUKSAN=ON -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=$nlopt_install

# Build & Install NLopt
Write-Host "Building and installing NLopt..."
cmake --build $nlopt_build --config Release --target INSTALL -- /m

# Reconfigure FastPlanner to use installed NLopt
$fast_root = $repo
$fast_build = Join-Path $fast_root "build"
Write-Host "Reconfigure FastPlanner..."
cmake -S $fast_root -B $fast_build -G "Visual Studio 17 2022" -A x64 -DNLopt_DIR=$nlopt_install\lib\cmake\nlopt

# Build FastPlanner
Write-Host "Building FastPlanner..."
cmake --build $fast_build --config Release -- /m

# Copy NLopt DLL to build output (ensure runtime picks the right DLL)
$src_dll = Join-Path $nlopt_install "bin\nlopt.dll"
$dst_dll = Join-Path $fast_build "Release\nlopt.dll"
if (Test-Path $src_dll) {
    Copy-Item -Force $src_dll $dst_dll
    Write-Host "Copied $src_dll to $dst_dll"
} else {
    Write-Warning "NLopt DLL not found at $src_dll"
}

# Run main.exe
$exe = Join-Path $fast_build "Release\main.exe"
if (Test-Path $exe) {
    Write-Host "Running main.exe..."
    & $exe
} else {
    Write-Warning "Executable not found: $exe"
}

# Run plotting script
$plot = Join-Path $fast_root "scripts\plot_trajectory.py"
$csv = Join-Path $repo "trajectory.csv"
if ((Test-Path $plot) -and (Test-Path $csv)) {
    Write-Host "Plotting trajectory..."
    python $plot $csv
} else {
    Write-Warning "Plot script or CSV missing: $plot, $csv"
}

Write-Host "Done."