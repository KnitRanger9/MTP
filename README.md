# Edge Deployed Visual SLAM with Control Aware Latency

#### Name: Aritra Maji
#### Roll: DA24M004
#### Guide: Dr. Nirav P. Bhatt, Dr. Ramkrishna Pasumarthy

---

## Introduction

This repository contains implementations of three visual SLAM configurations, targeting
edge deployment with latency-aware control:

1. **SuperPoint-based SLAM** — Python, monocular stereo with a learned feature detector
2. **Stereo ORB-SLAM3** — C++, stereo mode using a Stereolabs ZED camera
3. **RGB-D Inertial ORB-SLAM3** — ROS 2, RGB-D + IMU fusion in a Docker environment

---

## Code Quality Standards

All Python source files in this repository conform to the following submission requirements:

| Standard | Tool | Spec |
|---|---|---|
| Formatting | [`black`](https://github.com/psf/black) | max line length 88 |
| Docstrings | — | [Google style guide](https://google.github.io/styleguide/pyguide.html#38-comments-and-docstrings) |
| Type checking | [`mypy`](https://mypy.readthedocs.io/) | all arguments type-hinted |
| Linting | [`ruff`](https://docs.astral.sh/ruff/) / [`flake8`](https://flake8.pycqa.org/) | sorted imports, no errors |

### Running the toolchain

```bash
# Format
black --line-length 88 .

# Type-check
mypy train2_updated_3.py

# Lint
ruff check .
# or
flake8 --max-line-length 88 .
```

---

## Part 1: SuperPoint-based SLAM

**Code file:** [train2_updated_3.py](train2_updated_3.py)
<br>
**Weights:** [Download from Github](https://github.com/magicleap/SuperPointPretrainedNetwork/blob/master/superpoint_v1.pth)

This module implements a stereo visual odometry pipeline using the
[SuperPoint](https://github.com/magicleap/SuperPointPretrainedNetwork) self-supervised
keypoint detector and descriptor. It runs on a ZED camera in stereo mode, performs
temporal feature matching between consecutive frames, estimates camera pose via the
essential matrix, triangulates 3-D map points from the stereo pair, and renders a live
3-D trajectory plot.

### Dependencies

| Package | Minimum version |
|---|---|
| `torch` | 0.4 |
| `opencv-python` | 3.4 |
| `numpy` | — |
| `scikit-image` | — |
| `matplotlib` | — |
| `pyzed` | ZED SDK ≥ 4.x |

### Setup

```bash
# 1. Create workspace
mkdir Dev && cd Dev

# 2. Clone SuperPoint weights repository (Magic Leap)
git clone https://github.com/magicleap/SuperPointPretrainedNetwork.git
# Download superpoint_v1.pth from the repository and place it in your working directory

# 3. Install Python dependencies
pip install torch opencv-python scikit-image matplotlib numpy

# 4. Install ZED SDK
# Follow the official Stereolabs guide: https://www.stereolabs.com/docs/installation/
# Then install the Python API:
pip install pyzed
```

### Usage

```bash
python train2_updated_3.py
```

The script reads from the ZED camera automatically using `ZedCAM`. Adjust the
`SUPERPOINT_WEIGHTS_PATH` variable at the top of `__main__` to point to your
`superpoint_v1.pth` file.

### Key configuration parameters

| Parameter | Default | Description |
|---|---|---|
| `conf_thresh` | `0.015` | Keypoint confidence threshold |
| `nms_dist` | `4` | NMS suppression radius (pixels) |
| `max_kps` | `1000` | Maximum keypoints per frame |
| `nn_thresh` | `0.75` | Lowe ratio-test threshold |
| `baseline` | `0.12 m` | ZED stereo baseline |
| `depth_mode` | `NEURAL` | ZED depth estimation mode |
| `camera_fps` | `15` | ZED capture frame rate |

### Output

- Live 3-D trajectory and map-point visualization via `matplotlib`
- Press **q** to exit and display the final trajectory

### License

See [Magic Leap LICENSE](https://github.com/magicleap/SuperPointPretrainedNetwork/blob/master/LICENSE)
and Stereolabs SDK license for relevant terms.

---

## Part 2: Stereo ORB-SLAM3

**Code file:** [ORB_SLAM3/Examples/Stereo/stereo_ZED.cc](ORB_SLAM3/Examples/Stereo/stereo_ZED.cc) <br>
**Settings file:** [ORB_SLAM3/Examples/Stereo/ZED_stereo.yaml](ORB_SLAM3/Examples/Stereo/ZED_stereo.yaml)

Runs ORB-SLAM3 in stereo mode with a ZED camera. Captures left/right grayscale frames,
retrieves ZED pose and IMU data, feeds image pairs to ORB-SLAM3, and logs trajectories
and per-frame tracking latency.

### System requirements

- Ubuntu 20.04 (fresh install recommended)
- ≥ 16 GB RAM (32 GB recommended)
- ≥ 4 CPU cores

### Dependency installation

```bash
# Core build tools and libraries
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt-get install -y \
    build-essential cmake git \
    libgtk2.0-dev pkg-config \
    libavcodec-dev libavformat-dev libswscale-dev \
    python-dev python-numpy libtbb2 libtbb-dev \
    libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev \
    libjasper-dev libglew-dev libboost-all-dev \
    libssl-dev libeigen3-dev libcanberra-gtk-module
```

### OpenCV installation (versions 4.2.0 and 3.2.0)

For each version, clone the source and build without CUDA:

```bash
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=Release \
      -D WITH_CUDA=OFF \
      -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j3 && sudo make install
```

> **Note:** Before building, add the three preprocessor defines to
> `cap_ffmpeg_impl.hpp` as documented in
> [Kevin Robb's guide](https://github.com/kevin-robb/orb_slam_implementation).

### Pangolin installation (pinned commit for compatibility)

```bash
cd ~/Dev
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d
mkdir build && cd build
cmake .. -D CMAKE_BUILD_TYPE=Release
make -j3 && sudo make install
```

### ORB-SLAM3 installation

```bash
cd ~/Dev
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
git checkout ef9784101fbd28506b52f233315541ef8ba7af57
```

Apply the patches to `LoopClosing.h`, `CMakeLists.txt`, and `System.cc` described in
[Kevin Robb's guide](https://github.com/kevin-robb/orb_slam_implementation), then:

```bash
./build.sh
```

### Integrating the ZED example

```bash
# Replace src/, include/, and Examples/ contents with this repository's files
cp -r ORB_SLAM3/src/*        ~/Dev/ORB_SLAM3/src/
cp -r ORB_SLAM3/include/*    ~/Dev/ORB_SLAM3/include/
cp -r ORB_SLAM3/Examples/*   ~/Dev/ORB_SLAM3/Examples/

# Build the Stereo example
cd ~/Dev/ORB_SLAM3/Examples/Stereo
mkdir build && cd build
cmake ..
make -j2
```

### Usage

```bash
<path_to_executable> <path_to_vocabulary> <path_to_settings> [trajectory_file_name]

# Example — ZED camera
./Examples/Stereo/build/stereo_zed \
    ../Vocabulary/ORBvoc.txt \
    ./Examples/Stereo/ZED_stereo.yaml \
    dataset-ZED_stereo

# Example — EuRoC dataset
./Examples/Stereo/stereo_euroc \
    ./Vocabulary/ORBvoc.txt \
    ./Examples/Stereo/EuRoC.yaml \
    ~/Datasets/EuRoc/MH01 \
    ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt \
    dataset-MH01_stereo
```

### Output files

| File | Content |
|---|---|
| `ZED_poses.txt` | ZED odometry: `timestamp tx ty tz qx qy qz qw` |
| `IMU_data.txt` | IMU per frame: `timestamp ax ay az gx gy gz` |
| `ORB_Latency.txt` | Per-frame tracking latency: `timestamp tracking_time_s` |
| `With_LoopClosure.txt` | ORB-SLAM3 trajectory with loop closure |
| `Without_LoopClosure.txt` | ORB-SLAM3 trajectory without loop closure |

### ORB_SLAM3 source tree

```
ORB_SLAM3/
├── src/
│   ├── Atlas.cc
│   ├── CameraModels/KannalaBrandt8.cpp
│   ├── CameraModels/Pinhole.cpp
│   ├── Config.cc
│   ├── Converter.cc
│   ├── Frame.cc
│   ├── FrameDrawer.cc
│   ├── G2oTypes.cc
│   ├── GeometricTools.cc
│   ├── ImuTypes.cc
│   ├── KeyFrame.cc
│   ├── KeyFrameDatabase.cc
│   ├── LocalMapping.cc
│   ├── LoopClosing.cc
│   ├── Map.cc
│   ├── MapDrawer.cc
│   ├── MapPoint.cc
│   ├── MLPnPsolver.cpp
│   ├── OptimizableTypes.cpp
│   ├── Optimizer.cc
│   ├── ORBextractor.cc
│   ├── ORBmatcher.cc
│   ├── Settings.cc
│   ├── Sim3Solver.cc
│   ├── System.cc
│   ├── Tracking.cc
│   ├── TwoViewReconstruction.cc
│   └── Viewer.cc
└── Examples/
    ├── Stereo/
    │   ├── ZED_stereo.yaml
    │   ├── stereo_ZED.cc
    │   ├── EuRoC.yaml
    │   └── ...
    └── RGB-D-Inertial/
        ├── zed_mini_rgbd_inertial.yaml
        └── ...
```

### License

See [ORB-SLAM3 LICENSE](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/LICENSE)
and Stereolabs SDK license for relevant terms.

---

## Part 3: RGB-D Inertial ORB-SLAM3 (ROS 2)

**Launch file:** [ros2_ws/launch/rgbd.launch.py](ros2_ws/launch/rgbd.launch.py) <br>
**Settings file:** [ORB_SLAM3/Examples/RGB-D-Inertial/zed_mini_rgbd_inertial.yaml](ORB_SLAM3/Examples/RGB-D-Inertial/zed_mini_rgbd_inertial.yaml)

Runs ORB-SLAM3 in RGB-D inertial mode inside a Docker container, subscribing to a ZED
Mini camera via ROS 2. Pose estimates from both ORB-SLAM3 and the ZED odometry API are
recorded for comparison.

### System requirements

- Ubuntu 22.04
- Docker installed

### Setup

#### 1. Clone and initialise the Docker wrapper

```bash
git clone https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git
cd ORB-SLAM3-ROS2-Docker
git submodule update --init --recursive --remote
```

#### 2. Install Docker

```bash
sudo chmod +x container_root/shell_scripts/docker_install.sh
./container_root/shell_scripts/docker_install.sh
```

#### 3. Build the Docker image (without CUDA)

```bash
sudo docker build --build-arg USE_CI=false -t orb-slam3-humble:22.04 .
echo "xhost +" >> ~/.bashrc
source ~/.bashrc
```

#### 4. Launch the container

```bash
sudo docker compose run orb_slam3_22_humble
# Verify X11 forwarding inside the container:
xeyes
```

#### 5. Build ORB-SLAM3 and the ROS 2 wrapper (inside container)

```bash
cd /home/orb/ORB_SLAM3 && sudo chmod +x build.sh && ./build.sh
cd /root/colcon_ws && colcon build --symlink-install && source install/setup.bash
```

#### 6. Copy project-specific files

```bash
# Copy launch file
cp orb_slam3_ros2_wrapper/launch/rgbd.launch.py \
   ORB-SLAM3-ROS2-Docker/orb_slam3_ros2_wrapper/launch/

# Copy settings file
cp ORB_SLAM3/Examples/RGB-D-Inertial/zed_mini_rgbd_inertial.yaml \
   ORB-SLAM3-ROS2-Docker/orb_slam3_ros2_wrapper/params/
```

#### 7. Configure ROS 2 environment (inside and outside Docker)

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
```

Update node names and topic remappings in `rgbd.launch.py` using `rqt_graph` to match
the live node graph.

### Running

```bash
# Outside Docker — launch ZED camera node
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zedm \
    publish_imu_tf:=false \
    enable_positional_tracking:=true \
    sarea_memory:=false

# Inside Docker — launch ORB-SLAM3
ros2 launch orb_slam3_ros2_wrapper rgbd.launch.py

# Record pose estimates (outside Docker)
python3 save_poses.py
```

### Published ROS 2 topics

| Topic | Type | Description |
|---|---|---|
| `map_points` | `PointCloud2` | SLAM feature point cloud |
| `robot_pose_slam` | `PoseStamped` | Estimated pose in global frame |
| `slam_info` | `SlamInfo` | SLAM statistics |
| `map_data` | `MapData` | Continuous map data |

### Available ROS 2 services

| Service | Description |
|---|---|
| `orb_slam3/get_map_data` | Retrieve current map |
| `orb_slam3/get_all_landmarks_in_map` | Get all map feature points |
| `orb_slam3/reset_mapping` | Clear current mapping instance |

### Launch file tree

```
ORB-SLAM3-ROS2-Docker/orb_slam3_ros2_wrapper/
├── launch/
│   ├── rgbd.launch.py
│   ├── unirobot.launch.py
│   └── zed_mini_stereo_imu.launch.py
└── params/
    ├── gazebo_rgbd.yaml
    ├── orbbec_astra.yaml
    ├── rgbd-ros-params.yaml
    ├── zed2i_kratos.yaml
    ├── zed2i.yaml
    └── zed_mini_stereo_imu.yaml
```

### Output files

| File | Content |
|---|---|
| `zed_mini_poses_zed.txt` | Pose estimates from ZED Mini SLAM API |
| `zed_mini_poses_slam.txt` | Pose estimates from ORB-SLAM3 (RGB-D inertial) |

### License

See [ORB-SLAM3-ROS2-Docker LICENSE](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker/blob/master/LICENSE)
and Stereolabs SDK license for relevant terms.

---


