#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')
    
    # Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='False')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value='zed')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', 
        default_value=os.path.join(orb_wrapper_pkg, 'params', 'zed_mini_stereo_imu.yaml'),
        description='Path to ROS2 params YAML')
    
    vocabulary_file_path = "/home/orb/ORB_SLAM3/Vocabulary/ORBvoc.txt"
    zed_mini_config_path = "/home/orb/ORB_SLAM3/Examples/RGB-D-Inertial/zed_mini_rgbd_inertial.yaml"
    
    # Params
    params_file = LaunchConfiguration('params_file')
    robot_namespace = LaunchConfiguration('robot_namespace')
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=robot_namespace,      # Namespace params under zed_mini/
        param_rewrites={},             # Required argument (empty dict OK)
        convert_types=True)
    
    # ORB-SLAM3 Node (stereo_imu executable)
    orb_slam3_node = Node(
        package='orb_slam3_ros2_wrapper',
        executable='rgbd',  # ✅ Stereo+IMU mode
        name='orb_slam3',
        output='screen',
        namespace=robot_namespace,
        arguments=[vocabulary_file_path, zed_mini_config_path],
        parameters=[configured_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            # Existing IMU remapping
        ('/zed/imu', '/zed/zed_node/imu/data_raw'), 
        
        # RGB-D Image remappings
        ('/zed/camera/image_raw', '/zed/zed_node/rgb/image_rect_color'),
        ('/zed/depth/image_raw','/zed/zed_node/depth/depth_registered'),
        
        ])
#    pose_saver_node = Node(
#        package='orb_slam3_ros2_wrapper',
#        executable='save_poses.py',  # Your Python script
#        name='pose_saver',
#        output='screen',
#        parameters=[{'output_file': '/home/orb/zed_mini_poses_live.txt'}],
#        namespace=robot_namespace 
#    )
    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        declare_params_file_cmd,
        orb_slam3_node
    ])

