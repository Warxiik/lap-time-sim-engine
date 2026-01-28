#
# perf_runner.ps1 - Performance benchmarking helper script for Windows
#
# Builds and runs the benchmark tool with various configurations.
# Supports Release/Debug builds and custom iteration counts.
#
# Usage:
#   .\tools\perf_runner.ps1 [options]
#
# Options:
#   -Release       Build in Release mode (default)
#   -Debug         Build in Debug mode
#   -Iterations    Number of benchmark iterations (default: 100)
#   -Clean         Clean build directory before building
#

param(
    [switch]$Release,
    [switch]$Debug,
    [int]$Iterations = 100,
    [switch]$Clean,
    [switch]$Help
)

$ErrorActionPreference = "Stop"

function Write-Header {
    Write-Host "========================================" -ForegroundColor Blue
    Write-Host "  Lap Time Simulator - Performance Run" -ForegroundColor Blue
    Write-Host "========================================" -ForegroundColor Blue
    Write-Host ""
}

function Write-Usage {
    Write-Host "Usage: .\perf_runner.ps1 [options]"
    Write-Host ""
    Write-Host "Options:"
    Write-Host "  -Release      Build in Release mode (default)"
    Write-Host "  -Debug        Build in Debug mode"
    Write-Host "  -Iterations   Number of benchmark iterations (default: 100)"
    Write-Host "  -Clean        Clean build directory before building"
    Write-Host "  -Help         Show this help message"
    Write-Host ""
    Write-Host "Examples:"
    Write-Host "  .\perf_runner.ps1                      # Release build, 100 iterations"
    Write-Host "  .\perf_runner.ps1 -Debug -Iterations 50"
    Write-Host "  .\perf_runner.ps1 -Release -Clean -Iterations 200"
}

if ($Help) {
    Write-Usage
    exit 0
}

# Determine build type
$BuildType = "Release"
if ($Debug) {
    $BuildType = "Debug"
}

# Find project root (script is in tools/)
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$BuildDir = Join-Path $ProjectRoot "cmake-build-$($BuildType.ToLower())"

Write-Header

Write-Host "Configuration:" -ForegroundColor Yellow
Write-Host "  Build type:  $BuildType"
Write-Host "  Iterations:  $Iterations"
Write-Host "  Build dir:   $BuildDir"
Write-Host ""

# Clean build if requested
if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "Cleaning build directory..." -ForegroundColor Yellow
    Remove-Item -Recurse -Force $BuildDir
}

# Create build directory
if (-not (Test-Path $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

Push-Location $BuildDir

try {
    # Configure
    Write-Host "Configuring CMake ($BuildType)..." -ForegroundColor Yellow
    cmake -DCMAKE_BUILD_TYPE="$BuildType" $ProjectRoot
    if ($LASTEXITCODE -ne 0) {
        throw "CMake configuration failed"
    }

    # Build benchmark target
    Write-Host "Building benchmark..." -ForegroundColor Yellow
    cmake --build . --target benchmark --config $BuildType --parallel
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed"
    }

    Write-Host ""
    Write-Host "Build successful!" -ForegroundColor Green
    Write-Host ""

    # Find the executable
    $BenchmarkExe = $null
    $PossiblePaths = @(
        (Join-Path $BuildDir "benchmark.exe"),
        (Join-Path $BuildDir "$BuildType\benchmark.exe"),
        (Join-Path $BuildDir "Debug\benchmark.exe"),
        (Join-Path $BuildDir "Release\benchmark.exe")
    )

    foreach ($path in $PossiblePaths) {
        if (Test-Path $path) {
            $BenchmarkExe = $path
            break
        }
    }

    if (-not $BenchmarkExe) {
        throw "Benchmark executable not found"
    }

    # Run benchmark
    Write-Host "Running benchmark..." -ForegroundColor Yellow
    Write-Host ""

    $Stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    & $BenchmarkExe $Iterations
    $Stopwatch.Stop()

    if ($LASTEXITCODE -ne 0) {
        throw "Benchmark failed"
    }

    Write-Host ""
    Write-Host "Total script time: $($Stopwatch.Elapsed.TotalSeconds.ToString('F2')) seconds" -ForegroundColor Cyan
    Write-Host "Benchmark complete!" -ForegroundColor Green
}
finally {
    Pop-Location
}
