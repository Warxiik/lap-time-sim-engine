#!/bin/bash
#
# perf_runner.sh - Performance benchmarking helper script
#
# Builds and runs the benchmark tool with various configurations.
# Supports Release/Debug builds and custom iteration counts.
#
# Usage:
#   ./tools/perf_runner.sh [options]
#
# Options:
#   -r, --release    Build in Release mode (default)
#   -d, --debug      Build in Debug mode
#   -i, --iterations Number of benchmark iterations (default: 100)
#   -c, --clean      Clean build directory before building
#   -h, --help       Show this help message
#

set -e

# Default configuration
BUILD_TYPE="Release"
ITERATIONS=100
CLEAN_BUILD=false
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Lap Time Simulator - Performance Run${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

print_usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -r, --release     Build in Release mode (default)"
    echo "  -d, --debug       Build in Debug mode"
    echo "  -i, --iterations  Number of benchmark iterations (default: 100)"
    echo "  -c, --clean       Clean build directory before building"
    echo "  -h, --help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                     # Release build, 100 iterations"
    echo "  $0 -d -i 50            # Debug build, 50 iterations"
    echo "  $0 --release -c -i 200 # Clean Release build, 200 iterations"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--release)
            BUILD_TYPE="Release"
            shift
            ;;
        -d|--debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        -i|--iterations)
            ITERATIONS="$2"
            shift 2
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
done

print_header

BUILD_DIR="${PROJECT_ROOT}/cmake-build-${BUILD_TYPE,,}"

echo -e "${YELLOW}Configuration:${NC}"
echo "  Build type:  ${BUILD_TYPE}"
echo "  Iterations:  ${ITERATIONS}"
echo "  Build dir:   ${BUILD_DIR}"
echo ""

# Clean build if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf "${BUILD_DIR}"
fi

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Configure
echo -e "${YELLOW}Configuring CMake (${BUILD_TYPE})...${NC}"
cmake -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" "${PROJECT_ROOT}"

# Build benchmark target
echo -e "${YELLOW}Building benchmark...${NC}"
cmake --build . --target benchmark --parallel

# Check if build succeeded
if [ ! -f "${BUILD_DIR}/benchmark" ] && [ ! -f "${BUILD_DIR}/benchmark.exe" ]; then
    echo -e "${RED}Build failed - benchmark executable not found${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Build successful!${NC}"
echo ""

# Run benchmark
echo -e "${YELLOW}Running benchmark...${NC}"
echo ""

# Find the executable (handle both Unix and Windows via WSL)
if [ -f "${BUILD_DIR}/benchmark.exe" ]; then
    BENCHMARK_EXE="${BUILD_DIR}/benchmark.exe"
else
    BENCHMARK_EXE="${BUILD_DIR}/benchmark"
fi

# Execute with timing
time "${BENCHMARK_EXE}" "${ITERATIONS}"

echo ""
echo -e "${GREEN}Benchmark complete!${NC}"
