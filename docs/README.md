# SuPER_SLAM – Student Prototype Educational Repository

## Overview
**SuPER_SLAM** is a modular stereo visual odometry and SLAM system designed for KITTI datasets. It estimates camera poses, smooths them using a Kalman Filter, and visualizes trajectories in both 2D and 3D.

## Features
- **Stereo Visual Odometry**
  - ORB feature detection and matching.
  - Stereo matching with depth filtering.
  - Temporal matching across frames.
- **Pose Estimation**
  - Solve PnP problem with RANSAC.
  - Outlier rejection using Kalman Filter motion gating.
- **Kalman Filter**
  - Smooths camera translation while preserving rotation.
  - Handles noisy or partially missing measurements.
- **Visualization**
  - Display images, stereo matches, and temporal matches.
  - Plot 2D (XZ plane) and 3D trajectories using `matplotlibcpp` and Python.
- **Configuration**
  - YAML-based configuration for dataset paths, tracker parameters, PnP solver, and Kalman Filter.
- **Evaluation**
  - Python evaluation scripts compare estimated trajectory with KITTI ground truth.

## Project Structure

```

SUPER_SLAM/
├── CMakeLists.txt                # Build configuration
├── Dockerfile                    # Ubuntu 22.04 dev environment
├── super_slam.sh                 # Build and run script
├── config/                       # YAML configuration files
│   └── kitti_2011_09_26_drive_0005.yaml
├── include/                      # Header files
│   └── vo/                       # Visual odometry module headers
│       ├── camera.hpp
│       ├── config.hpp
│       ├── feature.hpp
│       ├── feature_tracker.hpp
│       ├── filter.hpp
│       ├── frame.hpp
│       ├── kitti_loader.hpp
│       ├── logger.hpp
│       └── viz.hpp
│   └── matplotlibcpp.h            # C++ Matplotlib wrapper
├── modules/                       # Implementation modules
│   ├── frontend/
│   ├── backend/
│   ├── io/
│   ├── geometry/
│   └── viz/
├── src/
│   ├── camera.cpp
│   ├── pnp_solver.cpp
│   └── main.cpp
└── utils/
|   ├── compare_trajectory.py     # KITTI trajectory evaluation
|   └── plot_trajectory_3d.py     # Optional 3D plot of trajectory

````

## Dependencies

- **C++ Libraries**
  - OpenCV
  - Eigen3
  - yaml-cpp
  - matplotlibcpp (header-only)
- **Python Packages** (for evaluation/plotting)
  - numpy
  - matplotlib
  - pykitti
  - scipy
- **Build tools**
  - CMake >= 3.10
  - GCC / g++
  - Python3 development headers

## Setup

### 1.1 Dockerfile (Recommended)
```bash
docker build -t super_slam .
docker run -it --rm -v $(pwd):/app -v /path/to/dataset:/data -e DISPLAY=$DISPLAY --net=host -v /tmp/.X11-unix:/tmp/.X11-unix super-slam:latest
````

### 1.2 DockerHub Pull
```bash
docker pull joshuajoset/super-slam:latest
docker run -it --rm -v $(pwd):/app -v /path/to/dataset:/data -e DISPLAY=$DISPLAY --net=host -v /tmp/.X11-unix:/tmp/.X11-unix joshuajosey/super-slam:latest
````

### 2. Local Build

```bash
mkdir -p build
cd build
cmake ..
cmake --build . -- -j$(nproc)
```

## Configuration

Each KITTI drive has a YAML config file under `config/`.
Example: `config/kitti_2011_09_26_drive_0005.yaml`

Key parameters:

* **Dataset**

  * `root_path`: path to KITTI dataset root
  * `drive_date` / `drive_number`: drive identifier
  * `output_path`: where logs and plots are saved
* **Feature Tracker**

  * `num_features`: ORB feature count
  * `fast_threshold`: FAST threshold
  * `max_depth`: maximum feature depth in meters
* **PnP Solver**

  * `min_inliers`, `use_extrinsic_guess`, `ransac_iterations`, `reprojection_error`, `confidence`
* **Kalman Filter**

  * `initial_uncertainty`, `process_noise`, `measurement_noise`, `motion_gate_dist`

## Usage

### Run with Shell Script

```bash
./super_slam.sh -d 2011_09_26 -n 0005 -k /data -o ./results
```

* `-d`: drive date
* `-n`: drive number
* `-k`: KITTI root directory (optional)
* `-o`: output directory (optional)

### Direct Execution

```bash
./build/super_slam ./config/kitti_2011_09_26_drive_0005.yaml
```

## Outputs
All outputs stored in results/<date>_drive_<n>/ by default
* `logs/app.log`
* `logs/<date>_drive_<n>_poses.txt`
* `logs/<date>_drive_<n>_poses_raw.txt`
* `plots/<date>_drive_<n>_trajectory.png`
* `plots/<date>_drive_<n>_trajectory_3d.png`
* `plots/<date>_drive_<n>_comparison.png`

## Evaluation

Compare your trajectory with KITTI ground truth using:

```bash
python3 utils/compare_trajectory.py \
    --input results/<date>_drive_<n>/logs/<date>_drive_<n>_poses.txt` \
    --kitti_root /data/2011_09_26 \
    --date 2011_09_26 \
    --drive 0005 \
    --output results/plots
```

## Notes / Tips

* Kalman filter smooths **translation**.
* Motion gating prevents large jumps from noisy PnP results.
* For faster runs, reduce ORB features or limit dataset frames.

