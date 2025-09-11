#!/bin/bash

# Debug version of install.sh with enhanced error reporting
set -e  # Exit on any error
set -o pipefail  # Exit on pipe failures

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# Symbols
CHECK="✓"
CROSS="✗"
ARROW="→"
STAR="★"

# Enhanced error reporting
debug_info() {
    echo -e "${BLUE}[DEBUG]${NC} $1"
}

error_exit() {
    echo -e "${RED}${CROSS} ERROR:${NC} $1"
    echo -e "${YELLOW}Script failed at line $2${NC}"
    echo -e "${YELLOW}Command: $3${NC}"
    exit 1
}

# Trap errors and provide detailed information
trap 'error_exit "An error occurred" $LINENO "$BASH_COMMAND"' ERR

# Function to print colored output
print_status() {
    echo -e "${GREEN}${CHECK}${NC} $1"
}

print_error() {
    echo -e "${RED}${CROSS}${NC} $1"
}

print_info() {
    echo -e "${BLUE}${ARROW}${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_header() {
    echo
    echo -e "${PURPLE}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${PURPLE}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${STAR}${NC}"
    echo
}

print_section() {
    echo
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo -e "${CYAN} $1${NC}"
    echo -e "${YELLOW}════════════════════════════════════════${NC}"
    echo
}

# Pre-flight checks
print_header "UAV Gazebo Simulation Environment Setup - DEBUG MODE"

debug_info "Starting pre-flight checks..."

# Check if we're running as root
if [ "$EUID" -eq 0 ]; then
    print_error "Do not run this script as root!"
    exit 1
fi

# Check required environment variables
debug_info "Checking environment variables..."
if [ -z "${DEV_DIR}" ]; then
    print_error "DEV_DIR environment variable is not set"
    print_info "Please set it using: export DEV_DIR=/path/to/your/dev/directory"
    exit 1
fi
print_status "DEV_DIR=$DEV_DIR"

# Check if DEV_DIR exists and is writable
if [ ! -d "$DEV_DIR" ]; then
    print_error "DEV_DIR directory does not exist: $DEV_DIR"
    exit 1
fi

if [ ! -w "$DEV_DIR" ]; then
    print_error "DEV_DIR is not writable: $DEV_DIR"
    exit 1
fi

# Set up directory variables
ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src
PX4_DIR=$DEV_DIR/PX4-Autopilot

debug_info "Directory structure:"
debug_info "  ROS2_WS: $ROS2_WS"
debug_info "  ROS2_SRC: $ROS2_SRC"
debug_info "  PX4_DIR: $PX4_DIR"

# Check basic commands
print_section "Checking Required Commands"

commands=("git" "curl" "wget" "apt-get" "pip3" "python3")
for cmd in "${commands[@]}"; do
    if command -v "$cmd" &> /dev/null; then
        print_status "$cmd found"
    else
        print_error "$cmd not found - please install it first"
        exit 1
    fi
done

# Check internet connectivity
print_info "Checking internet connectivity..."
if ping -c 1 google.com &> /dev/null; then
    print_status "Internet connectivity OK"
else
    print_error "No internet connectivity - required for downloading packages"
    exit 1
fi

print_status "All pre-flight checks passed!"
print_info "You can now run the main install.sh script"
print_info "If you encounter errors, please share the exact error message"

echo
echo -e "${CYAN}To run the main installation:${NC}"
echo -e "${CYAN}  ./install.sh${NC}"
echo
echo -e "${YELLOW}Common issues and solutions:${NC}"
echo -e "${YELLOW}  1. Permission denied: Make sure DEV_DIR is writable${NC}"
echo -e "${YELLOW}  2. Git clone fails: Check internet connection and credentials${NC}"
echo -e "${YELLOW}  3. Build fails: Check if all dependencies are installed${NC}"
echo -e "${YELLOW}  4. Python install fails: Try running without --break-system-packages${NC}"
