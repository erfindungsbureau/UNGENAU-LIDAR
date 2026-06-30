#!/bin/bash
# Install RoboStack (ROS Noetic via conda) on Jetson TX2
# Run as user 'nvidia'
# See docs/setup-ros.md for full details
set -e

MINIFORGE_INSTALLER="$HOME/Miniforge3-aarch64.sh"
MINIFORGE_DIR="$HOME/miniforge3"
ENV_NAME="ros-noetic"

echo "=== RoboStack ROS Noetic Setup ==="

# 1. Install Miniforge if not present
if [ ! -d "$MINIFORGE_DIR" ]; then
    if [ ! -f "$MINIFORGE_INSTALLER" ]; then
        echo "[1/4] Downloading Miniforge3 for aarch64..."
        wget -q --show-progress \
          https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh \
          -O "$MINIFORGE_INSTALLER"
    fi
    echo "[1/4] Installing Miniforge3..."
    bash "$MINIFORGE_INSTALLER" -b -p "$MINIFORGE_DIR"
    echo "Miniforge installed at $MINIFORGE_DIR"
else
    echo "[1/4] Miniforge already installed at $MINIFORGE_DIR"
fi

# 2. Initialize conda
source "$MINIFORGE_DIR/etc/profile.d/conda.sh"
conda config --set always_yes true

# 3. Configure RoboStack channels
echo "[2/4] Configuring conda channels for RoboStack..."
conda config --add channels conda-forge
conda config --add channels robostack-staging
conda config --set channel_priority strict

# 4. Create ROS Noetic environment
if conda env list | grep -q "^$ENV_NAME "; then
    echo "[3/4] ROS Noetic environment already exists"
else
    echo "[3/4] Creating ROS Noetic environment..."
    echo "      (20-40 minutes on ARM64 with internet, faster with local package cache)"
    mamba create -n "$ENV_NAME" \
        ros-noetic-ros-base \
        ros-noetic-rosbag \
        ros-noetic-tf2-ros \
        ros-noetic-sensor-msgs \
        ros-noetic-nav-msgs \
        ros-noetic-geometry-msgs \
        compilers cmake pkg-config make ninja colcon-common-extensions \
        -c robostack-staging -c conda-forge \
        2>&1 | tee /tmp/robostack-install.log
fi

# 5. Add to .bashrc
if ! grep -q "miniforge3/etc/profile.d/conda.sh" "$HOME/.bashrc" 2>/dev/null; then
    echo "" >> "$HOME/.bashrc"
    echo "# Conda (Miniforge)" >> "$HOME/.bashrc"
    echo "source \$HOME/miniforge3/etc/profile.d/conda.sh" >> "$HOME/.bashrc"
fi

echo "[4/4] Done! Activate with: conda activate $ENV_NAME"
echo ""
echo "Next step: build ROS workspace"
echo "  See: ~/lidar-webui/BUILD_ROS.md  or  docs/setup-ros.md"
echo "=== Installation complete ==="
