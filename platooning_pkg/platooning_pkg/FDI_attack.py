import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random

class TimeBombFDIAttacker(Node):
    def __init__(self):
        super().__init__('fdi_attack_node')
        
        self.declare_parameter('target_name', 'tb3_leader')
        self.declare_parameter('attack_time', 15.0) # Seconds before attack triggers
        
        self.target_name = self.get_parameter('target_name').value
        self.attack_time = self.get_parameter('attack_time').value
        self.start_time = self.get_clock().now()
        
        # Subscribe to clean data
        self.sub = self.create_subscription(
            Odometry, f'/{self.target_name}/odom', self.odom_callback, 10)
        
        # Publish to spoofed topic
        self.pub = self.create_publisher(
            Odometry, f'/{self.target_name}/odom_spoofed', 10)
            
        self.get_logger().info(f"MITM Active on {self.target_name}. Waiting {self.attack_time}s to inject poison...")

    def odom_callback(self, msg):
        spoofed_msg = msg
        
        # Check how much time has passed
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if elapsed_time >= self.attack_time:
            # TRIGGER THE HACK
            # Adding up to 0.5m to position and up to 0.2m/s to velocity
            # Check Section VI.B
            spoofed_msg.pose.pose.position.x += random.uniform(-0.5, 0.5)
            spoofed_msg.twist.twist.linear.x += random.uniform(-0.2, 0.2)
            # This is the delta_j(t)
            
            # Print a warning once when the attack starts
            if elapsed_time < self.attack_time + 0.1:
                self.get_logger().warn("!!! EXECUTING FDI ATTACK NOW !!!")
        
        self.pub.publish(spoofed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimeBombFDIAttacker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()