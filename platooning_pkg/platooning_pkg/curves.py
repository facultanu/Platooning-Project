import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import scipy.interpolate as spi

class LeaderPathPublisher(Node):
    def __init__(self):
        super().__init__('leader_path_publisher')
        
        # Publisher for the shared platoon trajectory
        self.path_pub = self.create_publisher(Path, '/platoon_path', 10)
        
        # Publish at 1 Hz (since the path is static, 1 Hz is plenty)
        self.timer = self.create_timer(1.0, self.publish_path)
        
        # Pre-compute the path message once to save CPU
        self.path_msg = self.create_path_message()
        self.get_logger().info("Leader Trajectory Publisher Started. Broadcasting B-spline path.")

    def create_path_message(self):
        path = Path()
        # Ensure this matches your global tracking frame (e.g., 'map' or 'odom')
        path.header.frame_id = "odom" 
        
        path_x, path_y = generate_custom_spline()
        
        for i in range(len(path_x)):
            pose = PoseStamped()
            pose.header.frame_id = path.header.frame_id
            
            pose.pose.position.x = float(path_x[i])
            pose.pose.position.y = float(path_y[i])
            pose.pose.position.z = 0.0
            
            # Calculate yaw (theta) for orientation using the next point
            if i < len(path_x) - 1:
                yaw = math.atan2(path_y[i+1] - path_y[i], path_x[i+1] - path_x[i])
            else:
                yaw = math.atan2(path_y[i] - path_y[i-1], path_x[i] - path_x[i-1])
            
            # Convert yaw to quaternion
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            path.poses.append(pose)
            
        return path

    def publish_path(self):
        # Update timestamp and publish
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LeaderPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()