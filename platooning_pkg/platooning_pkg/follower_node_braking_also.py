import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import osqp
import scipy.sparse as sparse

class ECBFPlatoonFollower(Node):
    def __init__(self):
        super().__init__('ecbf_follower_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 1. Namespaces & Offsets
        self.declare_parameter('my_name', 'tb3_follower1')
        self.declare_parameter('target_name', 'tb3_leader')
        self.declare_parameter('my_spawn_x', 0.0)
        self.declare_parameter('target_spawn_x', 0.0)
        
        my_name = self.get_parameter('my_name').value
        target_name = self.get_parameter('target_name').value
        self.my_spawn_x = self.get_parameter('my_spawn_x').value
        self.target_spawn_x = self.get_parameter('target_spawn_x').value

        # 2. System & Control Parameters
        self.L = 0.05               
        self.v_max = 0.22           
        self.v_min = 0.0            # Prevents the backward reversing spike
        self.omega_max = 2.84       
        
        # --- The new safe distance ---
        self.D_safe = 0.3           # Absolute minimum stopping distance (m)
        self.tau = 0.8              # Time-gap (seconds) for velocity-scaling
        self.gamma = 3              # Tracking gain
        self.alpha_cbf = 2.0        # How aggressively the forcefield pushes back
        
        # 3. State Variables
        self.my_state = np.zeros((3, 1))
        
        # --- NOISE FILTERING ---
        self.my_v_filtered = 0.0    # Smoothed velocity to prevent OSQP jitter
        self.alpha_filter = 0.1     # Tuning: 10% new reading, 90% history
        
        self.target_state = np.zeros((3, 1))
        self.target_v = 0.0
        self.target_w = 0.0
        
        self.my_state_received = False
        self.target_state_received = False

        # 4. ROS Interfaces
        self.create_subscription(Odometry, f'/{my_name}/odom', self.my_odom_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{target_name}/odom', self.target_odom_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, f'/{target_name}/cmd_vel', self.target_cmd_callback, qos_profile_system_default)
        self.cmd_pub = self.create_publisher(Twist, f'/{my_name}/cmd_vel', qos_profile_system_default)

        # 5. Control Loop (25 Hz)
        self.timer = self.create_timer(0.04, self.control_loop)
        self.get_logger().info(f"[{my_name}] Velocity-Aware ECBF Started. Target: [{target_name}]")

    def my_odom_callback(self, msg):
        """ Updating position and applyinf the low-pass filter for removing the oscilations """
        x, y, theta = self.extract_pose(msg)
        self.my_state = np.array([[x + self.my_spawn_x], [y], [theta]])
        
        # Apply Low-Pass Filter to odometry velocity to kill the jitter
        raw_v = msg.twist.twist.linear.x
        self.my_v_filtered = (self.alpha_filter * raw_v) + ((1.0 - self.alpha_filter) * self.my_v_filtered)
        
        self.my_state_received = True

    def target_odom_callback(self, msg):
        """ Target odometry extraction"""
        x, y, theta = self.extract_pose(msg)
        self.target_state = np.array([[x + self.target_spawn_x], [y], [theta]])
        self.target_state_received = True

    def target_cmd_callback(self, msg):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z

    def extract_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        return x, y, theta

    def get_virtual_point(self, state):
        """ Here is the virtual/fictive point of the robot """
        x, y, theta = state[0,0], state[1,0], state[2,0]
        return np.array([[x + self.L * math.cos(theta)], [y + self.L * math.sin(theta)]])

    def control_loop(self):
        if not (self.my_state_received and self.target_state_received):
            return

        # 1. Nominal Control (Clean Signal)
        # We calculate the nominal control using the leader's velocity to maintain
        # the original smoothness, effectively acting as a feed-forward term.
        my_theta = float(self.my_state[2, 0])
        target_theta = float(self.target_state[2, 0])
        
        x_my_virt = self.get_virtual_point(self.my_state)
        print(f"virtual x: {x_my_virt}\n")
        x_tgt_virt = self.get_virtual_point(self.target_state)
        print(f"target x: {x_tgt_virt}\n")

        # Transformation to virtual/fictive point logic
        T_tgt = np.array([
            [math.cos(target_theta), -self.L * math.sin(target_theta)],
            [math.sin(target_theta),  self.L * math.cos(target_theta)]
        ])
        u_tgt_virt = T_tgt @ np.array([[self.target_v], [self.target_w]])

        # Nominal control law
        u_nom = u_tgt_virt - self.gamma * (x_my_virt - x_tgt_virt)

        # 2. Safety Constraint (Filtered Signal)
        # We use the filtered velocity to calculate the Dynamic Headway boundary,
        # ensuring the safety barrier is jitter-free but physically accurate.
        dx = x_tgt_virt - x_my_virt
        print(f"distance until target: {dx}\n")
        dist_sq = dx[0,0]**2 + dx[1,0]**2
        
        v_my_norm = abs(self.my_v_filtered)
        D_dynamic = self.D_safe + (self.tau * v_my_norm) # distance increases as the target's speed increases
        # proportional to the predecessor's speed
        
        # Control Barrier Function
        h = dist_sq - D_dynamic**2
        print(f"h is: {h}\n")
        # We want A_cbf * u <= b_cbf
        A_cbf = 2 * dx.T 
        b_cbf = np.array([float((A_cbf @ u_tgt_virt + self.alpha_cbf * h).item())])

        # 3. Optimization (Feasibility-Aware QP)
        # Slack variable added to improve feasibility during emergency braking
        T_FL = np.array([
            [math.cos(my_theta), math.sin(my_theta)],
            [-math.sin(my_theta)/self.L, math.cos(my_theta)/self.L]
        ])
        
        # P matrix penalizes deviations from nominal and slack variable usage
        P = sparse.csc_matrix(np.diag([1.0, 1.0, 10000.0]))
        q = np.array([-u_nom[0,0], -u_nom[1,0], 0.0])

        A_cbf_slack = np.hstack([A_cbf, [[-1.0]]]) 
        T_FL_slack = np.hstack([T_FL, np.zeros((2, 1))])
        
        A_stack = np.vstack([A_cbf_slack, T_FL_slack])
        A = sparse.csc_matrix(A_stack)

        # Set physical bounds: v_min = 0.0 ensures no backward movement
        l = np.array([-np.inf, self.v_min, -self.omega_max])
        u_upper = np.array([b_cbf[0], self.v_max, self.omega_max])

        # https://osqp.org/docs/index.html# (OSQP)
        prob = osqp.OSQP()
        prob.setup(P, q, A, l, u_upper, verbose=False)
        res = prob.solve()

        if res.info.status != 'solved':
            v_cmd, omega_cmd = 0.0, 0.0
        else:
            u_opt = np.array([[res.x[0]], [res.x[1]]])
            vw = T_FL @ u_opt
            v_cmd = float(vw[0, 0])
            omega_cmd = float(vw[1, 0])

        twist_msg = Twist()
        twist_msg.linear.x = v_cmd
        twist_msg.angular.z = omega_cmd
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ECBFPlatoonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()