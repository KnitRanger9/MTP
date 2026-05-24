# Edge Deployed Visual SLAM with control aware latency
#### Name: Aritra Maji
#### Roll: DA24M004
#### Guide: Dr. Nirav P. Bhatt, Dr. Ramkrishna Pasumarthy

## Introduction: 
The repository contains codes of three slam settings:
1. Superpoint based SLAM (in python)
2. Stereo based ORB-SLAM3 (in cpp)
3. RGBD based ORB-SLAM3 (in ros2, cpp, python)

## Part 1: Superpoint based SLAM
Code file: train2_updated.py
<br>
link: [Click Here](https://drive.google.com/file/d/1_P4dLI2Mz064J_z3YtkUUDM0CrtstLi6/view?usp=sharing)

### Setup
1. Make an empty directory `mkdir Dev && cd Dev`
2. Download superpoint weights from official repository by [Magic Leap](https://github.com/magicleap/SuperPointPretrainedNetwork.git)
3. Install all requirements: `pip install -r requirements.txt`
4. Setup ZED Camera following the official ZED SDK manual

### Instructions

Download and run **train_updated_3.py**

### License
See Magic Leap LICENSE and Stereolabs SDK license for relevant terms.
<br>
<br>

## Part 2: Stereo based ORB-SLAM3
code file: stereo_ZED.cc
<br>
settings file: ZED_stereo.yaml


This example shows how to run ORB-SLAM3 in stereo mode using a Stereolabs ZED camera. It captures left/right grayscale images, retrieves ZED pose and IMU data, feeds image pairs to ORB-SLAM3, and logs trajectories and performance metrics.

### Output
- `ZED_poses.txt` — ZED odometry: `timestamp tx ty tz qx qy qz qw` (saved when ZED pose is valid).
- `IMU_data.txt` — IMU measurements per frame: `timestamp accel_x accel_y accel_z gyro_x gyro_y gyro_z`.
- `ORB_Latency.txt` — per-frame SLAM tracking latency: `timestamp tracking_time_seconds`.
- Final SLAM outputs: `With_LoopClosure.txt` and `Without_LoopClosure.txt` (trajectory formats saved by the ORB-SLAM3 API).

### Command line

Usage:

```
./stereo_realsense_t265 path_to_vocabulary path_to_settings (trajectory_file_name)
```

- `path_to_vocabulary`: ORB vocabulary file used by ORB-SLAM3 (e.g., `ORBvoc.txt`).
- `path_to_settings`: YAML settings file for your camera and SLAM configuration.
- `trajectory_file_name` (optional): An extra filename passed into the System constructor (example-specific).

### Instructions
- Follow ORB-SLAM3 installation guide by [Kevin Robb](https://github.com/kevin-robb/orb_slam_implementation.git) 
- Replace the files inside `src`, `hpp` and `Examples` folders with the contents of the repository.
- Build ORB_SLAM3 based on the installation guide.
- Inside `/Examples/Stereo` run:
    - `mkdir build && cd build`
    - `cmake ..`
    - `make -j2`
- Run the slam algorithm using bash: `./Examples/Stereo/stereo_euroc ./Vocabulary/ORBvoc.txt ./Examples/Stereo/EuRoC.yaml ~/Datasets/EuRoc/MH01 ./Examples/Stereo/EuRoC_TimeStamps/MH01.txt dataset-MH01_stereo`

Install and build steps will vary by platform. Example placeholders:

### Output files
- `ZED_poses.txt` — raw ZED pose logs
- `IMU_data.txt` — logged IMU data
- `ORB_Latency.txt` — per-frame SLAM processing time
- `With_LoopClosure.txt` — saved ORB-SLAM3 trajectory (with loop closure)
- `Without_LoopClosure.txt` — saved ORB-SLAM3 trajectory (without loop closure)

### License
See ORB-SLAM3 LICENSE and Stereolabs SDK license for relevant terms.
<br>
<br>

## Part 3: RGBD based inertial ORB-SLAM3
code file: rgbd.launch.py
<br>
settings file: ZED_stereo.yaml

### Setup
- Clone the github repo from [suchetanrs's repository](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git) and follow the instructions [build without CUDA]
- Copy the launch file `orb_slam3_ros2_wrapper/launch/rgbd.launch.py` to the repo folder 
- Run these three lines inside and outside Docker:
    - `export ROS_DOMAIN_ID=0`
    - `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
    - `unset CYCLONEDDS_URI`
- Run the ZED camera node outside Docker using `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm publish_imu_tf:=false enable_positional_tracking:=true sarea_memory:=false`
- Run ORB-SLAM3 launch file: `ros2 launch orb_slam3_ros2_wrapper zed_mini_stereo_imu.launch.py`

### Output files
- `zed_mini_poses_zed.txt`: contains pose estimates of ZED Mini camera's SLAM API
- `zed_mini_poses_slam.txt`: contains pose estimates of ORB-SLAM3 in rgbd-inertial model