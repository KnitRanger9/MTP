import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/zed/orb_slam3/pose', # Matches your robot_namespace
            self.listener_callback,
            10)
        self.file = open('/home/orb/ORB_SLAM3/Trajectory.txt', 'w')

    def listener_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        # TUM Format: timestamp x y z qx qy qz qw
        self.file.write(f"{t:.6f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.file.close()
        node.destroy_node()
        rclpy.shutdown()
