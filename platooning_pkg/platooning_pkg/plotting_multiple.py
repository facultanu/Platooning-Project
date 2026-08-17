import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        # Generate a unique filename
        self.filename = f"platoon_data_{int(time.time())}.csv"
        
        # Write header: we track positions (x) and velocities (v) for all three vehicles
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'l_x', 'l_v', 'f1_x', 'f1_v', 'f2_x', 'f2_v', 'f3_x', 'f3_v'])

        # Subscribe to odometry topics
        self.create_subscription(Odometry, '/tb3_leader/odom', lambda msg: self.odom_cb(msg, 'l'), 10)
        self.create_subscription(Odometry, '/tb3_follower1/odom', lambda msg: self.odom_cb(msg, 'f1'), 10)
        self.create_subscription(Odometry, '/tb3_follower2/odom', lambda msg: self.odom_cb(msg, 'f2'), 10)
        # CORRECTED: Changed the lambda parameter from 'f2' to 'f3'
        self.create_subscription(Odometry, '/tb3_follower3/odom', lambda msg: self.odom_cb(msg, 'f3'), 10)
        
        self.states = {'l': {'x':3.0, 'v':0.0}, 'f1': {'x':2.0, 'v':0.0}, 'f2': {'x':1.0, 'v':0.0}, 'f3': {'x':0.0, 'v':0.0}}
        
        # Log data at 25Hz to match the control loop frequency
        self.timer = self.create_timer(0.04, self.log_data) 
        self.get_logger().info(f"Logging data to {self.filename}")

    def odom_cb(self, msg, robot_id):
        # Extract x position and linear velocity from odometry
        self.states[robot_id]['x'] = msg.pose.pose.position.x
        self.states[robot_id]['v'] = msg.twist.twist.linear.x

    def log_data(self):
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.get_clock().now().nanoseconds, 
                self.states['l']['x'], self.states['l']['v'],
                self.states['f1']['x'], self.states['f1']['v'],
                self.states['f2']['x'], self.states['f2']['v'],
                self.states['f3']['x'], self.states['f3']['v']
            ])

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()