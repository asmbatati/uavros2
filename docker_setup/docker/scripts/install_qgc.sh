#!/usr/bin/env bash
set -e

echo "Installing QGroundControl (QGC) Dependencies..."

# Update package lists
sudo apt-get update -y --quiet

# Install GStreamer and plugins for video streaming
sudo apt-get install -y --quiet --no-install-recommends \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    python3-gi \
    python3-gst-1.0

# Install Qt, fuse, and keyboard/cursor dependencies required by QGC AppImage
sudo apt-get install -y --quiet --no-install-recommends \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev

# Clean up apt caches to reduce image size
sudo apt-get clean autoclean
sudo rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

echo "Dependencies installed successfully."

# Define where QGroundControl should be placed based on user request
QGC_DIR="/home/user/shared_volume"
sudo mkdir -p "${QGC_DIR}"

echo "Downloading QGroundControl Latest AppImage to ${QGC_DIR}..."
sudo wget -q -O "${QGC_DIR}/QGroundControl.AppImage" "https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage"

# Make it executable
sudo chmod +x "${QGC_DIR}/QGroundControl.AppImage"

# Ensure the correct ownership if it's going into the user directory
if id "user" &>/dev/null; then
    sudo chown -R user:user "${QGC_DIR}"
fi

echo "QGroundControl installed successfully at ${QGC_DIR}/QGroundControl.AppImage."
