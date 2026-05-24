#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        
        # Declare parameters for both output files
        self.declare_parameter('slam_output_file', 'zed_mini_poses_slam.txt')
        self.declare_parameter('zed_output_file', 'zed_mini_poses_zed.txt')
        
        self.slam_output_file = self.get_parameter('slam_output_file').value
        self.zed_output_file = self.get_parameter('zed_output_file').value
        
        # Define topics
        slam_topic = '/zed/robot_pose_slam'  # ORB-SLAM3 output
        zed_topic = '/zed/zed_node/pose'     # ZED wrapper internal output
        
        # Create subscriptions
        self.slam_sub = self.create_subscription(
            PoseStamped, slam_topic, self.slam_pose_callback, 10)
            
        self.zed_sub = self.create_subscription(
            PoseStamped, zed_topic, self.zed_pose_callback, 10)
            
        self.get_logger().info(f'Saving SLAM poses from {slam_topic} to {self.slam_output_file}')
        self.get_logger().info(f'Saving ZED poses from {zed_topic} to {self.zed_output_file}')
        
        # Initialize both files with the TUM CSV header
        with open(self.slam_output_file, 'w') as f:
            f.write('# timestamp tx ty tz qx qy qz qw\n')
            
        with open(self.zed_output_file, 'w') as f:
            f.write('# timestamp tx ty tz qx qy qz qw\n')
    
    def slam_pose_callback(self, msg):
        self.save_pose(msg, self.slam_output_file)

    def zed_pose_callback(self, msg):
        self.save_pose(msg, self.zed_output_file)
        
    def save_pose(self, msg, filename):
        # Format timestamp to seconds.nanoseconds
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Format TUM line
        pose_line = f'{t:.6f} {msg.pose.position.x} {msg.pose.position.y} {msg.pose.position.z} ' \
                   f'{msg.pose.orientation.x} {msg.pose.orientation.y} {msg.pose.orientation.z} {msg.pose.orientation.w}\n'
        
        # Append to the specified file
        with open(filename, 'a') as f:
            f.write(pose_line)

def main():
    rclpy.init()
    node = PoseSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
