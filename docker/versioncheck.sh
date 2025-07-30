#!/bin/bash
set -e

echo "🔍 Checking critical library versions..."

# --------------------------
# APT-installed libraries
# --------------------------
libs=(
  "ros-noetic-roscpp"
  "ros-noetic-roslib"
  "ros-noetic-tf2-ros"
  "ros-noetic-tf2-geometry-msgs"
  "ros-noetic-geometry-msgs"
  "ros-noetic-visualization-msgs"
  "ros-noetic-nav-msgs"
  "ros-noetic-xmlrpcpp"
  "libboost-all-dev"
  "libeigen3-dev"
  "libxmlrpcpp-dev"
  "libstdc++6"
)

for pkg in "${libs[@]}"; do
  echo "🔹 $pkg"
  if dpkg -s "$pkg" &>/dev/null; then
    dpkg -s "$pkg" | grep -E "Package|Version"
  else
    echo "❌ $pkg not installed"
  fi
  echo ""
done

# --------------------------
# apriltag_ros commit
# --------------------------
APRILTAG_DIR="/home/shuoyuan/catkin_slam_ws/src/apriltag_ros"
echo "📦 Checking apriltag_ros in $APRILTAG_DIR"
if [ -d "$APRILTAG_DIR/.git" ]; then
  cd "$APRILTAG_DIR"
  echo "✅ apriltag_ros commit:"
  git rev-parse HEAD
  echo "📅 Commit date:"
  git log -1 --pretty=format:"%ad"
else
  echo "❌ apriltag_ros not found or not a git repo"
fi
echo ""

# --------------------------
# GTSAM commit
# --------------------------
GTSAM_DIR="/home/shuoyuan/drivers/gtsam"
echo "📦 Checking GTSAM in $GTSAM_DIR"
if [ -d "$GTSAM_DIR/.git" ]; then
  cd "$GTSAM_DIR"
  echo "✅ GTSAM commit:"
  git rev-parse HEAD
  echo "📅 Commit date:"
  git log -1 --pretty=format:"%ad"
else
  echo "❌ GTSAM not found or not a git repo"
fi
echo ""

# --------------------------
# Eigen version via macros
# --------------------------
echo "📦 Eigen version (from header macros)"
eigen_macros="/usr/include/eigen3/Eigen/src/Core/util/Macros.h"
if [ -f "$eigen_macros" ]; then
  grep -E "#define EIGEN_WORLD_VERSION|#define EIGEN_MAJOR_VERSION|#define EIGEN_MINOR_VERSION" "$eigen_macros"
else
  echo "❌ Could not find Eigen Macros.h"
fi
echo ""

# --------------------------
# GCC version
# --------------------------
echo "📦 GCC Version (C++ compiler):"
gcc --version | head -n 1
echo ""

# --------------------------
# libstdc++ version
# --------------------------
echo "📦 libstdc++6 version (underlies cmath, vector, algorithm, cxxabi.h):"
dpkg -s libstdc++6 | grep -E 'Package|Version' || echo "❌ libstdc++6 not installed"

