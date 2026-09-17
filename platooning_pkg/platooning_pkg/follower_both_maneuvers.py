import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Pose, Twist

class PlatoonFollowerController(Node):
    def __init__(self):
        super().__init__('platoon_follower_controller')
        
        # Declare parameters so the same script works for any follower
        self.declare_parameter('my_name', 'tb3_follower1')
        self.declare_parameter('front_name', 'tb3_leader') 
        self.declare_parameter('target_name', 'tb3_T3') 
        
        self.my_name = self.get_parameter('my_name').get_parameter_value().string_value
        self.front_name = self.get_parameter('front_name').get_parameter_value().string_value
        self.target_name = self.get_parameter('target_name').get_parameter_value().string_value
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.my_name}/cmd_vel', 10)
        
        # Subscribers
        self.my_pose_sub = self.create_subscription(
            Pose, f'/{self.my_name}/pose', self.my_pose_callback, 10)
        self.front_pose_sub = self.create_subscription(
            Pose, f'/{self.front_name}/pose', self.front_pose_callback, 10)
        self.t3_pose_sub = self.create_subscription(
            Pose, f'/{self.target_name}/pose', self.t3_pose_callback, 10)
        
        # State variables
        self.my_x = None
        self.my_y = None
        self.my_yaw = None

        self.front_x = None
        self.front_y = None

        self.t3_x = None
        self.t3_y = None
        
        self.current_state = "CRUISING"
        self.desired_gap = 0.3  # Meters
        
        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def my_pose_callback(self, msg):
        self.my_x = msg.position.x
        self.my_y = msg.position.y
        self.my_yaw = self.get_yaw_from_quaternion(msg.orientation)

    def front_pose_callback(self, msg):
        self.front_x = msg.position.x
        self.front_y = msg.position.y

    def t3_pose_callback(self, msg):
        self.t3_x = msg.position.x
        self.t3_y = msg.position.y

    def control_loop(self):
        # Wait for all data to be available
        if None in [self.my_x, self.my_y, self.front_x, self.front_y, self.t3_x, self.t3_y]:
            return

        # ---------------------------------------------------------
        # 1. LATERAL CONTROL: Independent Overtaking State Machine
        # ---------------------------------------------------------
        delta_y_initial = abs(self.t3_y)
        distance_to_t3 = self.t3_x - self.my_x
        
        maneuver = "DEPASIRE" if delta_y_initial < 0.15 else "DEVANSARE"
        new_state = self.current_state
        target_y = 0.0  
        
        if maneuver == "DEVANSARE":
            target_y = 0.0
            if distance_to_t3 < -0.3:
                new_state = "DEVANSARE (Completed)"
            else:
                new_state = "DEVANSARE (Cruising)"
                
        elif maneuver == "DEPASIRE":
            if distance_to_t3 == 0.6:
                target_y = 0.35
                new_state = "DEPASIRE (Lane Change Out)"
            elif -0.6 <= distance_to_t3 < 0.6:
                target_y = 0.35
                new_state = "DEPASIRE (Passing)"
            else:
                target_y = 0.0
                if abs(self.my_y) < 0.05:
                    new_state = "DEPASIRE (Completed)"
                else:
                    new_state = "DEPASIRE (Lane Change In)"

        if new_state != self.current_state:
            self.get_logger().info(f"[{self.my_name}] Transitioned to: {new_state} | Dist to T3: {distance_to_t3:.2f}m")
            self.current_state = new_state

        # ---------------------------------------------------------
        # 2. LONGITUDINAL CONTROL: Artificial Potential Field (APF)
        # ---------------------------------------------------------
        actual_dist = math.hypot(self.front_x - self.my_x, self.front_y - self.my_y)
        
        D = 0.20       # Eq (2) Obstacle avoidance area (Hard safety boundary)
        rho = self.desired_gap     # Desired following distance
        
        if actual_dist <= D:
            linear_x = 0.0
            self.get_logger().warn(f"[{self.my_name}] Safety boundary D breached! Emergency Stop.", throttle_duration_sec=1.0)
            
        elif D < actual_dist <= rho:
            # REPULSIVE POTENTIAL: Distance is shrinking below the desired gap.
            eta_p = 1e-4 
            v_rep = -eta_p * ((1.0 / (actual_dist - D)) - (1.0 / (rho - D))) * (1.0 / ((actual_dist - D)**2))
            linear_x = v_rep
            
        else:
            # ATTRACTIVE POTENTIAL: Follower has fallen behind the desired gap.
            eta_a = 1.25
            v_att = eta_a * (1.0 - (rho / actual_dist))
            linear_x = v_att

        # Clamp speed for safety (allows slight braking if repulsed)
        linear_x = max(-0.1, min(0.3, linear_x))

        # ---------------------------------------------------------
        # 3. KINEMATIC COMMANDS
        # ---------------------------------------------------------
        twist = Twist()
        twist.linear.x = linear_x 
        
        error_y = target_y - self.my_y
        desired_yaw = math.atan2(error_y, 0.5) 
        error_yaw = desired_yaw - self.my_yaw
        error_yaw = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
        
        k_p_angular = 1.5
        twist.angular.z = k_p_angular * error_yaw
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PlatoonFollowerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()