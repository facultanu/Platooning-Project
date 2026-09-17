import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String

class PlatoonLeaderController(Node):
    def __init__(self):
        super().__init__('platoon_leader_controller')
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/platoon_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/tb3_leader/cmd_vel', 10)
        
        # Subscribers
        self.leader_pose_sub = self.create_subscription(
            Pose, '/tb3_leader/pose', self.leader_pose_callback, 10)
        self.t3_pose_sub = self.create_subscription(
            Pose, '/tb3_T3/pose', self.t3_pose_callback, 10)
        
        # State variables
        self.leader_x = None
        self.leader_y = None
        self.leader_yaw = None
        self.t3_x = None
        self.t3_y = None
        
        self.current_state = "CRUISING"
        
        # Latch variables to stop the accordion effect
        self.wait_ticks = 0
        self.wait_started = False
        self.wait_completed = False
        
        # T3 Speed tracking
        self.t3_speed = 0.0
        self.last_t3_x = None
        self.last_t3_time = None

        # Velocity state for the linear acceleration limiter
        self.current_v = 0.0
        
        # Control loop at 20 Hz (0.05s) for smooth driving
        self.timer = self.create_timer(0.05, self.control_loop)

    # --- Helper function to get Yaw from Quaternion ---
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def leader_pose_callback(self, msg):
        # --- NEW: Outlier Rejection Filter ---
        if self.leader_x is None:
            # Only lock onto the stream if it matches the expected spawn point (~1.2m)
            if abs(msg.position.x - 1.2) > 1.0:
                return
        else:
            # Ignore ghost readings that imply impossible teleportation
            if abs(msg.position.x - self.leader_x) > 0.5:
                return 
                
        self.leader_x = msg.position.x
        self.leader_y = msg.position.y
        self.leader_yaw = self.get_yaw_from_quaternion(msg.orientation)

    def t3_pose_callback(self, msg):
        # --- NEW: Outlier Rejection Filter ---
        if self.t3_x is None:
            # Only lock onto the stream if it matches T3's expected spawn point (~2.5m)
            if abs(msg.position.x - 2.5) > 1.0:
                return
        else:
            # Ignore ghost readings
            if abs(msg.position.x - self.t3_x) > 0.5:
                return 
                
        current_time = self.get_clock().now()
        self.t3_x = msg.position.x
        self.t3_y = msg.position.y
        
        # Estimate T3's linear speed safely
        if self.last_t3_x is not None and self.last_t3_time is not None:
            dt = (current_time - self.last_t3_time).nanoseconds / 1e9
            
            if dt >= 0.1:
                raw_speed = (self.t3_x - self.last_t3_x) / dt
                raw_speed = max(min(raw_speed, 0.25), -0.25)
                
                if abs(raw_speed) < 0.01:
                    raw_speed = 0.0
                    
                self.t3_speed = (0.2 * raw_speed) + (0.8 * self.t3_speed)
                
                self.last_t3_x = self.t3_x
                self.last_t3_time = current_time
        else:
            self.last_t3_x = self.t3_x
            self.last_t3_time = current_time

    def control_loop(self):
        if self.leader_x is None or self.t3_x is None:
            return

        delta_y_initial = abs(self.t3_y) 
        distance_x = self.t3_x - self.leader_x
        
        if delta_y_initial < 0.15:
            maneuver = "DEPASIRE"
        else:
            maneuver = "DEVANSARE"
            
        new_state = self.current_state
        target_y = 0.0 
        
        if maneuver == "DEVANSARE":
            if distance_x < -0.3:
                new_state = "DEVANSARE (Completed)"
            else:
                new_state = "DEVANSARE (Cruising)"
                
        elif maneuver == "DEPASIRE":
            # PHASE 1: Approaching
            if not self.wait_started:
                if distance_x > 0.6:
                    target_y = 0.0
                    new_state = "DEPASIRE (Approaching)"
                else:
                    self.wait_started = True 
            
            # PHASE 2: Waiting
            if self.wait_started and not self.wait_completed:
                if self.wait_ticks < 100:
                    self.wait_ticks += 1
                    target_y = 0.0
                    new_state = "DEPASIRE (Waiting)"
                else:
                    self.wait_completed = True
            
            # PHASE 3: Passing and Merging
            if self.wait_completed:
                if distance_x >= -0.6:
                    target_y = 0.35
                    if abs(self.leader_y - 0.35) > 0.05 and self.current_state != "DEPASIRE (Passing)":
                        new_state = "DEPASIRE (Lane Change Out)"
                    else:
                        new_state = "DEPASIRE (Passing)"
                else:
                    target_y = 0.0
                    if abs(self.leader_y) < 0.05:
                        new_state = "DEPASIRE (Completed)"
                    else:
                        new_state = "DEPASIRE (Lane Change In)"

        # Clean Debugging
        if new_state != self.current_state:
            self.get_logger().info(f"Transitioned to: {new_state} | Distance X: {distance_x:.2f}m")
            
            if new_state == "DEPASIRE (Completed)":
                self.get_logger().info(f"Procedure {maneuver} executed successfully. Back in original lane.\n")
            elif new_state == "DEVANSARE (Completed)":
                self.get_logger().info(f"Procedure {maneuver} executed successfully. Cruised past T3.\n")
                
            self.current_state = new_state

        state_msg = String()
        state_msg.data = self.current_state
        self.state_pub.publish(state_msg)

        # 5. Kinematic Control (Proportional Controller)
        desired_v = 0.0
        
        if new_state == "DEPASIRE (Waiting)":
            desired_v = self.t3_speed
        else:
            desired_v = 0.16 
        
        error_y = target_y - self.leader_y
        desired_yaw = math.atan2(error_y, 0.5) 
        
        error_yaw = desired_yaw - self.leader_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
        
        k_p_angular = 1.5
        desired_w = k_p_angular * error_yaw
        
        # Linear Acceleration Limiter
        max_dv = 0.01  
        
        if desired_v > self.current_v + max_dv:
            self.current_v += max_dv
        elif desired_v < self.current_v - max_dv:
            self.current_v -= max_dv
        else:
            self.current_v = desired_v

        # Publish the smoothed linear velocity and immediate angular velocity
        twist = Twist()
        twist.linear.x = self.current_v
        twist.angular.z = desired_w
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PlatoonLeaderController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()