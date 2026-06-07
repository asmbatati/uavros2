#!/bin/bash

# ---------------------------------------------------------------------------
# uav_gz_sim install.sh
#
# Flags:
#   --minimal              Skip YOLOv8 + GeographicLib + QGC downloads (CI/headless).
#   --simulators=LIST      Comma-separated subset of:
#                          gazebo,webots,mujoco,isaac,pybullet,genesis
#                          Default: gazebo (only PX4-coupled sim by default).
#   --help                 Print this header and exit.
#
# Env:
#   DEV_DIR  Parent of PX4-Autopilot and ros2_ws (default: $HOME/drone_arm_ws)
# ---------------------------------------------------------------------------

MINIMAL=0
SIMULATORS="gazebo"

for arg in "$@"; do
    case "$arg" in
        --minimal)        MINIMAL=1 ;;
        --simulators=*)   SIMULATORS="${arg#--simulators=}" ;;
        --help|-h)        sed -n '3,16p' "$0"; exit 0 ;;
        *)                echo "Unknown argument: $arg (use --help)"; exit 1 ;;
    esac
done

sim_enabled() {
    case ",${SIMULATORS}," in
        *",$1,"*) return 0 ;;
        *)        return 1 ;;
    esac
}

# Auto-detect DEV_DIR from this script's path. The expected layout is
#     $DEV_DIR/ros2_ws/src/uav_gz_sim/install.sh
# so three `dirname`s up from this script lands at DEV_DIR. Honour an
# explicit $DEV_DIR if the user set it (overrides the auto-detect).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DETECTED_DEV_DIR="$(cd "$SCRIPT_DIR/../../.." 2>/dev/null && pwd)"
if [ -z "${DEV_DIR:-}" ]; then
    if [ -n "$DETECTED_DEV_DIR" ] && [ -d "$DETECTED_DEV_DIR/ros2_ws/src/uav_gz_sim" ]; then
        export DEV_DIR="$DETECTED_DEV_DIR"
    else
        export DEV_DIR="$HOME/drone_arm_ws"
    fi
fi

# Source ROS2 environment if available
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Fix git ownership issues for container environments
git config --global --add safe.directory '*' 2>/dev/null || true

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

# Function to detect if running inside px4_ros2_jazzy container
detect_container() {
    local in_container=false
    
    # Check for Docker environment
    if [ -f /.dockerenv ]; then
        in_container=true
    fi
    
    # Check container hostname
    if [ "$(hostname)" = "px4-dev" ]; then
        in_container=true
    fi
    
    # Check for container-specific environment variables
    if [ "$CONTAINER_NAME" = "px4_ros2_jazzy" ] || [ -n "$LOCAL_USER_ID" ]; then
        in_container=true
    fi
    
    # Check if we're in the expected container working directory
    if [ "$(pwd)" = "$DEV_DIR" ] && [ -f /.dockerenv ]; then
        in_container=true
    fi
    
    echo $in_container
}

# Function to check Ubuntu version
check_ubuntu_version() {
    if ! command -v lsb_release &> /dev/null; then
        print_error "lsb_release not found. Installing lsb-release..."
        sudo apt-get update -qq
        sudo apt-get install -y lsb-release
    fi
    
    local ubuntu_version=$(lsb_release -rs)
    print_info "Ubuntu version: $ubuntu_version"
    
    if [[ "$ubuntu_version" != "24.04" && "$ubuntu_version" != "22.04" && "$ubuntu_version" != "20.04" ]]; then
        print_warning "Unsupported Ubuntu version: $ubuntu_version. Supported versions: 24.04, 22.04, 20.04"
        return 1
    fi
    
    return 0
}

# Function to check and install ROS2 Jazzy
check_ros2_jazzy() {
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        print_status "ROS2 Jazzy found"
        return 0
    fi
    
    print_info "ROS2 Jazzy not found. Installing..."
    
    # Add ROS2 repository
    sudo apt update && sudo apt install -y curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Install ROS2 Jazzy
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
    
    # Add to bashrc if not already there
    if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    fi
    
    print_status "ROS2 Jazzy installed"
    return 0
}

# Function to check and install Gazebo Harmonic
check_gazebo_harmonic() {
    if command -v gz &> /dev/null; then
        local gz_version=$(gz --version | head -n1)
        print_info "Gazebo version: $gz_version"
        if [[ "$gz_version" == *"harmonic"* ]] || gz sim --version &> /dev/null; then
            print_status "Gazebo Harmonic found"
            return 0
        fi
    fi
    
    print_info "Gazebo Harmonic not found. Installing..."
    
    # Add Gazebo repository
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    
    # Install Gazebo Harmonic
    sudo apt update
    sudo apt install -y gz-harmonic
    
    print_status "Gazebo Harmonic installed"
    return 0
}

# Function to install PX4 development dependencies
install_px4_dependencies() {
    print_info "Installing PX4 development dependencies..."
    
    # Update package list
    sudo apt-get update -qq
    
    # Install basic dependencies
    sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
        astyle \
        build-essential \
        cmake \
        cppcheck \
        file \
        g++ \
        gcc \
        gdb \
        git \
        git-lfs \
        lcov \
        libfuse2 \
        libxml2-dev \
        libxml2-utils \
        make \
        ninja-build \
        python3 \
        python3-dev \
        python3-pip \
        python3-setuptools \
        python3-wheel \
        rsync \
        shellcheck \
        unzip \
        zip \
        wget \
        curl \
        ca-certificates \
        gnupg \
        lsb-release
    
    # Install Python dependencies
    local ubuntu_version=$(lsb_release -rs)
    if [[ "$ubuntu_version" == "24.04" ]]; then
        python3 -m pip install --break-system-packages \
            argcomplete argparse cerberus coverage empy future jinja2 jsonschema \
            kconfiglib lxml matplotlib numpy nunavut packaging pandas pkgconfig \
            psutil pygments wheel pymavlink pyros-genmsg pyserial pyulog pyyaml \
            requests setuptools six toml
    else
        python3 -m pip install --user \
            argcomplete argparse cerberus coverage empy future jinja2 jsonschema \
            kconfiglib lxml matplotlib numpy nunavut packaging pandas pkgconfig \
            psutil pygments wheel pymavlink pyros-genmsg pyserial pyulog pyyaml \
            requests setuptools six toml
    fi
    
    print_status "PX4 dependencies installed"
}

# Function to check all dependencies for host environment
check_host_dependencies() {
    print_section "Checking Host Dependencies"
    
    local deps_ok=true
    
    # Check Ubuntu version
    if ! check_ubuntu_version; then
        deps_ok=false
    fi
    
    # Check ROS2 Jazzy
    if ! check_ros2_jazzy; then
        deps_ok=false
    fi
    
    # Check Gazebo Harmonic
    if ! check_gazebo_harmonic; then
        deps_ok=false
    fi
    
    # Install PX4 dependencies
    install_px4_dependencies
    
    if [ "$deps_ok" = true ]; then
        print_status "All host dependencies are satisfied"
    else
        print_error "Some dependencies failed to install"
        exit 1
    fi
}

# This script sets up the uav_gz_sim simulation environment
print_header "UAV Gazebo Simulation Environment Setup"

# Detect environment
IN_CONTAINER=$(detect_container)

if [ "$IN_CONTAINER" = "true" ]; then
    print_status "Running inside px4_ros2_jazzy container - skipping dependency checks"
    print_info "Container environment detected with pre-installed dependencies"
    print_info "Git ownership issues resolved for container environment"
else
    print_warning "Running on host system - checking and installing dependencies"
    check_host_dependencies
fi

print_info "Checking environment variables..."
print_status "DEV_DIR=$DEV_DIR"
if [ -n "$DETECTED_DEV_DIR" ] && [ "$DEV_DIR" = "$DETECTED_DEV_DIR" ]; then
    print_info "DEV_DIR auto-detected from script location"
elif [ "$DEV_DIR" = "$HOME/drone_arm_ws" ]; then
    print_info "Using default DEV_DIR. You can override with: export DEV_DIR=<your_directory>"
else
    print_info "Using explicitly-set DEV_DIR=$DEV_DIR"
fi
print_info "MINIMAL=$MINIMAL"
print_info "SIMULATORS=$SIMULATORS"

# Print git credentials WITHOUT echoing the token (security).
if [ -n "$GIT_USER" ]; then
    print_info "Git credentials: GIT_USER=$GIT_USER, GIT_TOKEN=$([ -n "$GIT_TOKEN" ] && echo '<set>' || echo '<unset>')"
else
    print_info "Git credentials: not set (public clone)"
fi

ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src
PX4_DIR=$DEV_DIR/PX4-Autopilot
OSQP_SRC=$DEV_DIR

# Make sure that PX4 root directory is set
if [ -z "${PX4_DIR}" ]; then
  print_error "PX4_DIR environment variable is not set. Set it using export PX4_DIR=<PX4-ROOT_directory_that_contains_PX4-Autopilot>"
  exit 1
fi

# Make sure that ROS2_WS directory is set
if [ -z "${ROS2_WS}" ]; then
  print_error "ROS2_WS environment variable is not set. Set it using export ROS2_WS=<ROS2_WS_directory_that_contains_ros2_ws>"
  exit 1
fi

if [ ! -d "$ROS2_WS" ]; then
  print_info "Creating $ROS2_SRC"
  mkdir -p $ROS2_SRC
  print_status "Created ROS2 workspace directory"
fi

print_section "Setting up Git Repository URLs"

SIM_PKG_URL='https://github.com/asmbatati/uav_gz_sim.git'
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    SIM_PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/asmbatati/uav_gz_sim.git
    print_status "Using authenticated Git URL"
else
    SIM_PKG_URL=https://github.com/asmbatati/uav_gz_sim.git
    print_warning "Using public Git URL (no authentication)"
fi

print_info "Installing git-lfs..."
sudo apt-get install git-lfs
print_status "Git LFS installed"

print_section "Cloning UAV Gazebo Simulation Package"

# Clone the uav_gz_sim if it doesn't exist
if [ ! -d "$ROS2_SRC/uav_gz_sim" ]; then
    print_info "Cloning uav_gz_sim repository..."
    cd $ROS2_SRC || { print_error "Failed to change to $ROS2_SRC"; exit 1; }
    if git clone $SIM_PKG_URL; then
        cd uav_gz_sim || { print_error "Failed to change to uav_gz_sim directory"; exit 1; }
        git lfs install || print_warning "Git LFS install failed - continuing without LFS"
        git lfs pull || print_warning "Git LFS pull failed - continuing without LFS"
        git submodule update --init --remote --recursive || print_warning "Submodule update failed - continuing"
        print_status "uav_gz_sim cloned successfully"
    else
        print_error "Failed to clone uav_gz_sim repository"
        exit 1
    fi
else
    print_info "Updating existing uav_gz_sim repository..."
    cd $ROS2_SRC/uav_gz_sim
    
    # Check for local modifications
    local_changes=$(git status --porcelain)
    
    if [[ -n "$local_changes" ]]; then
        print_warning "Local modifications detected - skipping repository update"
        echo -e "${YELLOW}Modified files:${NC}"
        git status --short
        print_info "Continuing with installation using your current local files..."
    else
        print_info "No local modifications found - updating from remote..."
        if git fetch origin && git pull origin main; then
            print_status "Repository updated successfully"
        else
            print_warning "Failed to update repository - continuing with current version"
        fi
    fi
    
    # Always run `git submodule update --init --recursive` so a stale local
    # checkout still ends up with the docker submodule populated. The
    # `--remote` flag (which fetches the latest commit on each submodule's
    # tracked branch) is only added when the working tree is clean to avoid
    # surprising the user with submodule updates underneath local changes.
    git lfs install
    if [[ -z "$local_changes" ]]; then
        git lfs pull
        git submodule update --init --remote --recursive
        print_status "Git LFS pulled and submodules updated (incl. remote tip)"
    else
        git submodule update --init --recursive
        print_info "Local modifications detected: ran submodule init/checkout"
        print_info "but skipped --remote (would fetch latest tip on each submodule)"
    fi

    print_status "uav_gz_sim repository ready for installation"
fi

# Copy bash.sh to DEV_DIR and setup bashrc sourcing
print_info "Setting up bash aliases and environment..."
if [ -f "$ROS2_SRC/uav_gz_sim/scripts/bash.sh" ]; then
    cp $ROS2_SRC/uav_gz_sim/scripts/bash.sh $DEV_DIR/bash.sh
    print_status "bash.sh copied to $DEV_DIR"
    
    # Check if bash.sh is already sourced in bashrc
    if ! grep -q "source.*$DEV_DIR/bash.sh" ~/.bashrc; then
        echo "" >> ~/.bashrc
        echo "# Source UAV simulation environment" >> ~/.bashrc
        echo "source $DEV_DIR/bash.sh" >> ~/.bashrc
        print_status "bash.sh added to ~/.bashrc"
    else
        print_status "bash.sh already sourced in ~/.bashrc"
    fi
else
    print_warning "bash.sh not found in uav_gz_sim/scripts/"
fi

# Check for QGroundControl AppImage and download if not present
if [ "$MINIMAL" = "1" ]; then
    print_info "Skipping QGroundControl setup (--minimal)"
else
print_info "Setting up QGroundControl..."

# Install QGroundControl dependencies
print_info "Installing QGroundControl dependencies..."
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev

# Remove modem manager that interferes with serial ports
print_info "Removing modem manager (interferes with serial ports)..."
sudo apt-get remove modemmanager -y || true

# Add user to dialout group for serial port access
print_info "Adding user to dialout group for serial port access..."
if [ -n "$USER" ]; then
    sudo usermod -a -G dialout $USER
else
    print_warning "USER environment variable not set - skipping dialout group addition"
    print_info "You may need to manually add your user to the dialout group: sudo usermod -a -G dialout \$USER"
fi

print_status "QGroundControl dependencies installed"

# Check for QGroundControl AppImage
QGC_FILENAME="QGroundControl-x86_64.AppImage"
QGC_PATH="$DEV_DIR/$QGC_FILENAME"
QGC_SYMLINK="$DEV_DIR/QGroundControl.AppImage"

if [ -f "$QGC_PATH" ]; then
    print_status "QGroundControl AppImage found at $QGC_PATH"
else
    print_info "QGroundControl AppImage not found. Downloading..."
    
    # Download QGroundControl AppImage with correct filename
    if wget -q --show-progress -O "$QGC_PATH" "https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage"; then
        # Make it executable
        chmod +x "$QGC_PATH"
        print_status "QGroundControl AppImage downloaded and made executable"
    else
        print_error "Failed to download QGroundControl AppImage"
        print_warning "You can manually download it from: https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html"
    fi
fi

# Create symlink for backward compatibility with existing alias
if [ -f "$QGC_PATH" ] && [ ! -L "$QGC_SYMLINK" ]; then
    ln -sf "$QGC_FILENAME" "$QGC_SYMLINK"
    print_status "Created symlink for QGroundControl compatibility"
fi

print_warning "Note: You may need to logout and login again for serial port permissions to take effect"
fi  # end --minimal QGC block

print_section "Setting up PX4-Autopilot"

# Clone and build PX4-Autopilot if it doesn't exist
if [ ! -d "$PX4_DIR" ]; then
    print_info "Cloning PX4-Autopilot..."
    cd $DEV_DIR
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    cd $PX4_DIR
    print_info "Cleaning PX4 build environment..."
    make submodulesclean
    make distclean
    make clean
    print_status "PX4-Autopilot cloned and configured"
else
    print_info "PX4_DIR=$PX4_DIR already exists"
    cd $PX4_DIR
    print_info "Cleaning existing PX4 build environment..."
    make submodulesclean
    make distclean
    make clean
fi

print_info "Preparing PX4 build environment..."
cd $PX4_DIR || { print_error "Failed to change to PX4 directory"; exit 1; }

# Ensure all submodules are properly initialized (especially mavlink + the
# Tools/simulation/gz submodule we copy assets into). Skip the first
# `make px4_sitl` build here - we do a single build AFTER copying our
# models / airframes so the embedded ROMFS picks them up. Saves ~10 min.
print_info "Initializing PX4 submodules..."
git submodule sync --recursive
git submodule update --init --recursive

print_section "Configuring PX4 Gazebo Models"

# Handle git operations for gazebo models BEFORE copying files. Tools/simulation/gz
# is a PX4 submodule (PX4-gazebo-models); guard the git ops in case the submodule
# isn't initialized for some reason.
GZ_SIM_DIR="$PX4_DIR/Tools/simulation/gz"
if [ ! -d "$GZ_SIM_DIR" ]; then
    print_error "Expected directory $GZ_SIM_DIR does not exist."
    print_info "Run \`cd $PX4_DIR && git submodule update --init --recursive\` and re-run install.sh."
    exit 1
fi
print_info "Setting up PX4 gazebo models repository at $GZ_SIM_DIR..."
cd "$GZ_SIM_DIR"

if [ -d ".git" ] || git rev-parse --git-dir > /dev/null 2>&1; then
    # Get current branch/commit info
    print_info "Current git state:"
    git status --porcelain
    git branch

    # Force reset everything to a clean state. This also wipes any models
    # that were previously copied in by a past install run, so the copy
    # below starts from a known-clean tree.
    print_info "Forcing repository to clean state..."
    git stash push -u -m "Auto-stash before install.sh" || true
    git reset --hard HEAD || true
    git clean -fdx || true

    # Fetch latest from origin to ensure we have all branches
    git fetch origin || true

    # Force checkout main branch (this will work even from detached HEAD)
    print_info "Checking out main branch..."
    git checkout -B main origin/main || git checkout main || true
    print_status "Repository is now clean and on main branch"
else
    print_warning "$GZ_SIM_DIR is not a git repo; skipping reset/clean (continuing with raw copy)"
fi

print_section "Copying Configuration Files"

# Copy files to $PX4_DIR
print_info "Copying models to ${PX4_DIR}/Tools/simulation/gz/models/"
if [ -d "${ROS2_SRC}/uav_gz_sim/models" ]; then
    cp -r ${ROS2_SRC}/uav_gz_sim/models/* ${PX4_DIR}/Tools/simulation/gz/models/ || {
        print_error "Failed to copy models"
        exit 1
    }
    print_status "Models copied"
else
    print_error "Models directory not found at ${ROS2_SRC}/uav_gz_sim/models"
    exit 1
fi

print_info "Copying worlds to ${PX4_DIR}/Tools/simulation/gz/worlds/"
if [ -d "${ROS2_SRC}/uav_gz_sim/worlds" ]; then
    cp -r ${ROS2_SRC}/uav_gz_sim/worlds/* ${PX4_DIR}/Tools/simulation/gz/worlds/ || {
        print_error "Failed to copy worlds"
        exit 1
    }
    print_status "Worlds copied"
else
    print_error "Worlds directory not found at ${ROS2_SRC}/uav_gz_sim/worlds"
    exit 1
fi

print_info "Copying airframe configs to ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"
if [ -d "${ROS2_SRC}/uav_gz_sim/config/px4" ]; then
    cp -r ${ROS2_SRC}/uav_gz_sim/config/px4/* ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/ || {
        print_error "Failed to copy airframe configurations"
        exit 1
    }
    print_status "Airframe configurations copied"
else
    print_error "PX4 config directory not found at ${ROS2_SRC}/uav_gz_sim/config/px4"
    exit 1
fi

# PX4 keeps an explicit airframe list in CMakeLists.txt that is used to embed
# the ROMFS tarball at build time. Copying files alone is not enough — they
# must also appear in that list. Strip any prior entries that match our
# airframe FILENAMES exactly (so stock PX4 IDs like 4010_gz_x500_mono_cam
# are preserved), then re-inject the current list after the
# "# [22000, 22999] Reserve for custom models" marker.
print_info "Registering airframes in PX4 CMakeLists.txt"
CMAKE_AIRFRAMES="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
UAV_AIRFRAMES=$(ls "${ROS2_SRC}/uav_gz_sim/config/px4" | grep -E '^[0-9]+_gz_' | sort)
if [ -f "$CMAKE_AIRFRAMES" ] && [ -n "$UAV_AIRFRAMES" ]; then
    python3 - "$CMAKE_AIRFRAMES" <<PY
import sys, re
path = sys.argv[1]
ours = """${UAV_AIRFRAMES}""".strip().splitlines()
src = open(path).read()
# Strip ONLY the exact airframe filenames we own, plus any stray COLCON_IGNORE
# (some prior install runs left one behind). Stock PX4 entries are untouched.
for name in ours:
    src = re.sub(r'(?m)^[ \t]*' + re.escape(name) + r'[ \t]*\n', '', src)
src = re.sub(r'(?m)^[ \t]*COLCON_IGNORE[ \t]*\n', '', src)
new_block = '\n'.join(ours) + '\n'
marker = '# [22000, 22999] Reserve for custom models\n'
if marker in src:
    src = src.replace(marker, marker + new_block, 1)
else:
    # Fallback: append at end-of-list (before final closing paren)
    src = re.sub(r'\)\s*$', new_block + ')\n', src)
open(path, 'w').write(src)
print('updated', path)
PY
    print_status "Airframe list registered in PX4 CMakeLists.txt"
else
    print_warning "Could not patch $CMAKE_AIRFRAMES — PX4 build may not include new airframes"
fi

# Build px4_sitl
print_info "Rebuilding PX4 SITL with new configurations..."
cd $PX4_DIR || { print_error "Failed to change to PX4 directory"; exit 1; }

# Ensure submodules are still properly initialized after file copying
git submodule update --init --recursive

if make px4_sitl; then
    print_status "PX4 SITL rebuilt successfully"
else
    print_error "Failed to rebuild PX4 SITL with new configurations"
    exit 1
fi

cd $DEV_DIR

if [ "$MINIMAL" = "1" ]; then
    print_info "Skipping YOLOv8 ROS Package (--minimal)"
else
print_section "Setting up YOLOv8 ROS Package"

if [ ! -d "$ROS2_SRC/yolov8_ros" ]; then
    print_info "Cloning yolov8_ros repository..."
    cd $ROS2_SRC
    git clone https://github.com/mgonzs13/yolov8_ros.git
    cd $ROS2_SRC/yolov8_ros && git checkout 2.0.1
    print_status "yolov8_ros cloned and checked out to version 2.0.1"
else
    print_info "Updating existing yolov8_ros repository..."
    cd $ROS2_SRC/yolov8_ros && git pull origin && git checkout 2.0.1
    print_status "yolov8_ros updated and checked out to version 2.0.1"
fi
fi  # end --minimal yolov8 block

print_section "Setting up MAVROS and Dependencies"

# these mavlink and mavros versions are working for ros2 jazzy
# Sept 17, 2023
print_info "Cloning mavlink package..."
if [ ! -d "$ROS2_SRC/mavlink" ]; then
    cd $ROS2_SRC
    git clone  https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
    cd $ROS2_SRC/mavlink && git checkout release/jazzy/mavlink
    print_status "mavlink cloned and checked out to jazzy release"
else
    print_status "mavlink already exists"
fi

# Custom mavros pkg is required to handle TF issues in multi-vehicle simulation
print_info "Cloning custom mavros package..."
if [ ! -d "$ROS2_SRC/mavros" ]; then
    cd $ROS2_SRC
    git clone  https://github.com/mavlink/mavros.git
    print_status "mavros cloned successfully"
else
    print_status "mavros already exists"
fi

# Clone eigen_stl_containers dependency for mavros
print_info "Cloning eigen_stl_containers dependency..."
if [ ! -d "$ROS2_SRC/eigen_stl_containers" ]; then
    cd $ROS2_SRC
    git clone https://github.com/ros/eigen_stl_containers.git
    print_status "eigen_stl_containers cloned successfully"
else
    print_status "eigen_stl_containers already exists"
fi

print_section "Installing Dependencies"

# Initialize rosdep only if not already initialized
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    print_info "Initializing rosdep..."
    cd $ROS2_WS && rosdep init
    print_status "rosdep initialized"
else
    print_status "rosdep already initialized"
fi

print_info "Updating and installing ROS dependencies..."
cd $ROS2_WS && sudo apt update && rosdep update && rosdep install --from-paths src --ignore-src -r -y
print_status "ROS dependencies installed"

# Install RMW Zenoh for ROS2 communication
print_info "Installing RMW Zenoh for ROS2..."
sudo apt install -y ros-jazzy-rmw-zenoh-cpp
print_status "RMW Zenoh installed"

# Runtime deps that aren't declared in package.xml but are needed at launch
# time across simulators (arm control, URDF expansion, MoveIt integration).
print_info "Installing simulation runtime deps (xacro, ros2_control, gz_ros2_control, MoveIt, robot/joint state publishers)..."
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-moveit \
    || print_warning "Some runtime deps failed to install; check apt output above"
print_status "Simulation runtime deps installed"

# Install missing Python dependencies for ROS2 message generation
print_info "Installing Python dependencies..."
if pip3 install --break-system-packages lark empy catkin_pkg; then
    print_status "Python dependencies installed"
else
    print_warning "Failed to install Python dependencies with --break-system-packages, trying without..."
    if pip3 install --user lark empy catkin_pkg; then
        print_status "Python dependencies installed (user mode)"
    else
        print_error "Failed to install Python dependencies"
        exit 1
    fi
fi

# Install GeographicLib datasets for MAVROS
if [ "$MINIMAL" = "1" ]; then
    print_info "Skipping GeographicLib datasets (--minimal)"
else
    print_info "Installing GeographicLib datasets..."
    sudo apt-get update
    sudo apt-get install -y geographiclib-tools
    sudo geographiclib-get-geoids egm96-5
    print_status "GeographicLib datasets installed"
fi

# Per-simulator dependency installation
install_simulator_deps() {
    local sim="$1"
    print_section "Installing simulator deps: $sim"
    case "$sim" in
        gazebo)
            # Already covered by check_gazebo_harmonic and ros-jazzy-ros-gz
            sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge || \
                print_warning "ros-gz install failed (may already be present)"
            ;;
        webots)
            sudo apt-get install -y ros-jazzy-webots-ros2 || \
                print_warning "webots_ros2 install failed (manual install may be required)"
            ;;
        mujoco)
            pip3 install --break-system-packages mujoco || pip3 install --user mujoco || \
                print_warning "mujoco install failed"
            # ros2_control is now installed unconditionally above (it's needed for
            # gazebo arm control too); the mujoco branch is just a no-op reminder.
            print_info "Note: mujoco_ros2_control may require source build; see docs/SIMULATORS.md"
            ;;
        isaac)
            print_info "Isaac Sim is not pip-installable; install via NVIDIA Omniverse Launcher."
            print_info "See docs/SIMULATORS.md for the PegasusSimulator setup."
            ;;
        pybullet)
            pip3 install --break-system-packages pybullet || pip3 install --user pybullet || \
                print_warning "pybullet install failed"
            ;;
        genesis)
            print_info "Genesis is scaffolded only; see docs/SIMULATORS.md."
            ;;
        *)
            print_warning "Unknown simulator: $sim"
            ;;
    esac
}

IFS=',' read -ra SIM_LIST <<< "$SIMULATORS"
for sim in "${SIM_LIST[@]}"; do
    install_simulator_deps "$sim"
done

print_section "Building ROS2 Packages"

# Function to clean CMake cache for failed packages
clean_cmake_cache() {
    print_warning "Cleaning CMake cache due to build errors..."
    cd $ROS2_WS && rm -rf build/ install/ log/
    print_status "Build cache cleaned - retrying build..."
}

# Function to build packages with retry on error
build_with_retry() {
    local build_cmd="$1"
    local description="$2"
    
    print_info "$description"
    cd $ROS2_WS
    
    # First attempt
    if eval "$build_cmd"; then
        print_status "$description completed successfully"
        return 0
    else
        print_error "$description failed - cleaning cache and retrying..."
        clean_cmake_cache
        
        # Second attempt after cleaning cache
        print_info "Retrying: $description"
        if eval "$build_cmd"; then
            print_status "$description completed successfully (after cache clean)"
            return 0
        else
            print_error "$description failed even after cache clean"
            return 1
        fi
    fi
}

# Ensure ROS2 environment is sourced before building
source /opt/ros/jazzy/setup.bash

# Build mavros (Step 1/3)
if ! build_with_retry "source /opt/ros/jazzy/setup.bash && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros --executor sequential" "Building mavros (Step 1/3)"; then
    print_error "Failed to build mavros. Exiting."
    exit 1
fi

# Build mavros_extras (Step 2/3)
if ! build_with_retry "source /opt/ros/jazzy/setup.bash && MAKEFLAGS='j1 -l1' colcon build --packages-up-to mavros_extras --executor sequential" "Building mavros_extras (Step 2/3)"; then
    print_error "Failed to build mavros_extras. Exiting."
    exit 1
fi

# Build all remaining packages (Step 3/3)
if ! build_with_retry "source /opt/ros/jazzy/setup.bash && colcon build" "Building all remaining packages (Step 3/3)"; then
    print_error "Failed to build all packages. Exiting."
    exit 1
fi

print_header "Installation Complete!"

print_status "All packages are built successfully"
print_status "Models and airframe config files are copied to ${PX4_DIR}"
print_info "To run the simulation:"
echo -e "  ${CYAN}# in terminal 1:${NC}"
echo -e "  ${CYAN}source ${ROS2_WS}/install/setup.bash${NC}"
echo -e "  ${CYAN}zenoh${NC}"
echo -e "  ${CYAN}# in terminal 2:${NC}"
echo -e "  ${CYAN}source ${ROS2_WS}/install/setup.bash${NC}"
echo -e "  ${CYAN}ros2 launch uav_gz_sim sim.launch.py${NC}"

# Re-source ~/.bashrc so the newly-added bash.sh aliases are picked up in
# THIS shell when install.sh is invoked via `source ./install.sh`. When
# run as `./install.sh` this is a no-op for the parent shell - the user
# needs to open a new terminal or `source ~/.bashrc` themselves; we tell
# them so.
if [ -f "$HOME/.bashrc" ]; then
    source "$HOME/.bashrc" 2>/dev/null || true
fi
print_info "If you ran this as \`./install.sh\` (not \`source ./install.sh\`),"
print_info "open a new terminal or run \`source ~/.bashrc\` to pick up the aliases."

cd $HOME
