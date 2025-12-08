#!/bin/bash

# ==========================================================#
# Student Prototype Educational Repository (SuPER_SLAM)     #
# ==========================================================#

# Default Values
KITTI_ROOT="/data"
OUTPUT_DIR="./results"
BUILD_DIR="./build"
EXECUTABLE_NAME="super_slam"
EVAL_SCRIPT="./utils/compare_trajectory.py"

usage() {
    echo "Usage: $0 -d <date> -n <drive> [-k <kitti_root>] [-o <output_dir>]"
    exit 1
}

# Parse args
while getopts "d:n:k:o:" opt; do
    case $opt in
        d) DATE="$OPTARG" ;;
        n) DRIVE="$OPTARG" ;;
        k) KITTI_ROOT="$OPTARG" ;;
        o) OUTPUT_DIR="$OPTARG" ;;
        *) usage ;;
    esac
done

# Mandatory checks
if [ -z "$DATE" ] || [ -z "$DRIVE" ]; then
    echo "Error: Date and Drive arguments are required."
    usage
fi
# Update output dir for date
OUTPUT_DIR="${OUTPUT_DIR}/${DATE}_drive_${DRIVE}"
# Output trajectory filename
TRAJ_FILE="${OUTPUT_DIR}/logs/${DATE}_drive_${DRIVE}_poses.txt"

mkdir -p "$OUTPUT_DIR/logs"

echo ""
echo "=========================================================="
echo " Building SuPER_SLAM"
echo "=========================================================="
echo ""
echo "Build Directory: $BUILD_DIR"

# Configure build
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake ..
    cd ..
fi

cmake --build "$BUILD_DIR" -- -j$(nproc)
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "Build successful."

# Config file path
CONFIG_FILE="./config/kitti_${DATE}_drive_${DRIVE}.yaml"

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Config file not found:"
    echo "  $CONFIG_FILE"
    exit 1
fi

echo ""
echo "=========================================================="
echo " Running SuPER_SLAM"
echo "=========================================================="
echo ""
echo "$BUILD_DIR/$EXECUTABLE_NAME" "$CONFIG_FILE"

"$BUILD_DIR/$EXECUTABLE_NAME" "$CONFIG_FILE"
if [ $? -ne 0 ]; then
    echo "SLAM exited with errors!"
    exit 1
fi

echo ""
echo "SLAM run complete."

# Ensure trajectory exists
if [ ! -f "$TRAJ_FILE" ]; then
    echo "Error: Expected trajectory file not found:"
    echo "  $TRAJ_FILE"
    echo "Make sure your C++ SLAM saves the trajectory using this exact filename."
    exit 1
fi

echo ""
echo "=========================================================="
echo " Evaluating Trajectory"
echo "=========================================================="

python3 "$EVAL_SCRIPT" \
    --input "$TRAJ_FILE" \
    --kitti_root "$KITTI_ROOT" \
    --date "$DATE" \
    --drive "$DRIVE" \
    --output "$OUTPUT_DIR/plots"
# 3D Trajectory Plot
# python3 ./utils/plot_3d.py --input "$TRAJ_FILE" --output "$OUTPUT_DIR/plots/trajectory_3d.png"

if [ $? -ne 0 ]; then
    echo "Evaluation script failed!"
    exit 1
fi

echo ""
echo "=========================================================="
echo " SuPER_SLAM Pipeline Completed Successfully!"
echo "=========================================================="
exit 0