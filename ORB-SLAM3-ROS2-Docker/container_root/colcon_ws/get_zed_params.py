import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import qos_profile_sensor_data # <-- The magic fix for missing data
import numpy as np

class ZedConfigExtractor(Node):
    def __init__(self):
        super().__init__('zed_config_extractor')
        
        # Exact frames based on your /zed/ namespace
        self.target_frame = 'zed_left_camera_optical_frame'
        self.source_frame = 'zed_imu_link'
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to the rectified camera info using the correct QoS profile
        self.subscription = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/left/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data)
        
        self.timer = self.create_timer(1.0, self.on_timer)
        self.got_camera_info = False
        self.got_tf = False

    def camera_info_callback(self, msg):
        if not self.got_camera_info:
            self.get_logger().info('\n--- 1. CAMERA INTRINSICS (Copy to YAML) ---')
            self.get_logger().info(f'Camera.fx: {msg.k[0]:.6f}')
            self.get_logger().info(f'Camera.fy: {msg.k[4]:.6f}')
            self.get_logger().info(f'Camera.cx: {msg.k[2]:.6f}')
            self.get_logger().info(f'Camera.cy: {msg.k[5]:.6f}')
            
            # Extract distortion parameters safely
            d = msg.d
            self.get_logger().info('\n--- 2. DISTORTION PARAMS ---')
            self.get_logger().info(f'Camera.k1: {d[0] if len(d) > 0 else 0.0}')
            self.get_logger().info(f'Camera.k2: {d[1] if len(d) > 1 else 0.0}')
            self.get_logger().info(f'Camera.p1: {d[2] if len(d) > 2 else 0.0}')
            self.get_logger().info(f'Camera.p2: {d[3] if len(d) > 3 else 0.0}')
            self.got_camera_info = True

    def on_timer(self):
        if not self.got_tf:
            try:
                # Look up the transform from IMU to Left Camera Optical Frame
                t = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    rclpy.time.Time())
                
                # Translation
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z
                
                # Rotation (Quaternion)
                x = t.transform.rotation.x
                y = t.transform.rotation.y
                z = t.transform.rotation.z
                w = t.transform.rotation.w
                
                # Convert Quaternion to 3x3 Rotation Matrix
                rot_mat = np.array([
                    [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
                    [2*(x*y + z*w),       1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
                    [2*(x*z - y*w),       2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
                ])
                
                # Build 4x4 Transformation Matrix
                T = np.eye(4)
                T[:3, :3] = rot_mat
                T[0, 3] = tx
                T[1, 3] = ty
                T[2, 3] = tz
                
                # Format for ORB-SLAM3 YAML
                data_str = ", ".join([f"{val:.6f}" for val in T.flatten()])
                
                self.get_logger().info('\n--- 3. T_b_c1 (IMU to Left Camera) ---')
                self.get_logger().info('T_b_c1: !!opencv-matrix')
                self.get_logger().info('   rows: 4')
                self.get_logger().info('   cols: 4')
                self.get_logger().info('   dt: f')
                self.get_logger().info(f'   data: [{data_str}]')
                
                self.got_tf = True
                
            except TransformException as ex:
                self.get_logger().warn(f'Waiting for TF tree... {ex}')

        if self.got_camera_info and self.got_tf:
            self.get_logger().info('\nSuccess! Copy these values to your ORB-SLAM3 YAML. Press Ctrl+C to exit.')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ZedConfigExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
