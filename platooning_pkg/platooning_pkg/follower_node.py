import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import osqp
import scipy.sparse as sparse

class PlatoonFollowerNode(Node):
    def __init__(self):
        super().__init__('platoon_follower_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 1. Declare Parameters for Namespaces
        self.declare_parameter('my_name', 'tb3_follower1')
        self.declare_parameter('target_name', 'tb3_leader')
        
        # --- Spawn offset parameters ---
        self.declare_parameter('my_spawn_x', 0.0)
        self.declare_parameter('target_spawn_x', 0.0)
        
        my_name = self.get_parameter('my_name').value
        target_name = self.get_parameter('target_name').value
        
        self.my_spawn_x = self.get_parameter('my_spawn_x').value
        self.target_spawn_x = self.get_parameter('target_spawn_x').value

        # 2. System & Control Parameters
        self.L = 0.05               # Offset distance (m) matching single_agent
        self.v_max = 0.22           # Max linear velocity (m/s)
        self.omega_max = 2.84       # Max angular velocity (rad/s)
        
        self.D_safe = 0.3          # 30cm safe following distance
        self.gamma = 3            # Tracking gain
        self.alpha_cbf = 2.0        # How aggressively the forcefield pushes back
        
        # 3. State Variables
        self.my_state = np.zeros((3, 1)) # Follower coordonates
        self.target_state = np.zeros((3, 1)) # leader coordonates
        self.target_v = 0.0
        self.target_w = 0.0
        
        self.my_state_received = False
        self.target_state_received = False

        # 4. Subscribers & Publishers
        self.create_subscription(Odometry, f'/{my_name}/odom', self.my_odom_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, f'/{target_name}/odom', self.target_odom_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, f'/{target_name}/cmd_vel', self.target_cmd_callback, qos_profile_system_default)
        
        self.cmd_pub = self.create_publisher(Twist, f'/{my_name}/cmd_vel', qos_profile_system_default)

        # 5. Control Loop (25 Hz)
        self.timer = self.create_timer(0.04, self.control_loop)
        self.get_logger().info(f"[{my_name}] CBF-QP Started. Following: [{target_name}] at {self.D_safe}m")

    def my_odom_callback(self, msg):
        """ Extract and store follower position from odometry """
        x, y, theta = self.extract_pose(msg)
        self.my_state = np.array([[x + self.my_spawn_x], [y], [theta]])
        self.my_state_received = True

    def target_odom_callback(self, msg):
        """ Extract and store target position from odometry """
        x, y, theta = self.extract_pose(msg)
        self.target_state = np.array([[x + self.target_spawn_x], [y], [theta]])
        self.target_state_received = True

    def target_cmd_callback(self, msg):
        """ Here we capture target's linear and angular speed """
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
        return np.array([
            [x + self.L * math.cos(theta)],
            [y + self.L * math.sin(theta)]
        ])

    def control_loop(self):
        if not (self.my_state_received and self.target_state_received):
            return

        # Current poses and virtual points for both robots
        my_theta = float(self.my_state[2, 0])
        target_theta = float(self.target_state[2, 0])

        # Current virtual/fictive point coordonates
        x_my_virt = self.get_virtual_point(self.my_state)
        x_tgt_virt = self.get_virtual_point(self.target_state)

        # Predecessor's virtual velocity (Feedforward)
        T_tgt = np.array([
            [math.cos(target_theta), -self.L * math.sin(target_theta)],
            [math.sin(target_theta),  self.L * math.cos(target_theta)]
        ]) # Check Eq. 4. This converts the 2 speeds into virtual/fictive point

        u_tgt_virt = T_tgt @ np.array([[self.target_v], [self.target_w]])

        # ---------------------------------------------------------
        # 1. NOMINAL CONTROL (Track the leader's exact position)
        # ---------------------------------------------------------

        u_nom = u_tgt_virt - self.gamma * (x_my_virt - x_tgt_virt)

        # ---------------------------------------------------------
        # 2. CBF SAFETY MATH (The Forcefield)
        # ---------------------------------------------------------
        dx = x_tgt_virt - x_my_virt
        print(f"The distance between the 2 bots is: {dx}")
        # CBF safety Constraint
        dist_sq = dx[0,0]**2 + dx[1,0]**2
        h = dist_sq - self.D_safe**2 # We want this to be positive
        print(f"The h is: {h}")

        # A_cbf * u_my <= b_cbf
        A_cbf = 2 * dx.T  # Shape: (1, 2)
        b_cbf = np.array([float(A_cbf @ u_tgt_virt + self.alpha_cbf * h)])

        # ---------------------------------------------------------
        # 3. ACTUATOR LIMIT MATH
        # ---------------------------------------------------------
        # T_FL maps u_virt back to [v, w]^T (Thrang Paper)
        T_FL = np.array([
            [math.cos(my_theta), math.sin(my_theta)],
            [-math.sin(my_theta)/self.L, math.cos(my_theta)/self.L]
        ])
        
        # ---------------------------------------------------------
        # 4. OSQP FORMULATION (WITH SLACK VARIABLE)
        # ---------------------------------------------------------
        # Variables: u_vec = [u_virt_x, u_virt_y, slack]
        # Objective: min 1/2 u^T P u + q^T u
        # We heavily penalize the slack variable (10000) so it is only used in emergencies
        P = sparse.csc_matrix(np.diag([1.0, 1.0, 10000.0]))
        q = np.array([-u_nom[0,0], -u_nom[1,0], 0.0])

        # Constraints: A_stack * u_vec <= u_upper
        # A_cbf * u_my - slack <= b_cbf
        A_cbf_slack = np.hstack([A_cbf, [[-1.0]]])  # Shape: (1, 3)
        
        # Actuator constraints don't use the slack variable
        T_FL_slack = np.hstack([T_FL, np.zeros((2, 1))])  # Shape: (2, 3)
        
        # Stack them together
        A_stack = np.vstack([A_cbf_slack, T_FL_slack])
        A = sparse.csc_matrix(A_stack)

        l = np.array([
            -np.inf,           # CBF only has an upper bound here
            0.0,       # Min v
            -self.omega_max    # Min w
        ])
        
        u_upper = np.array([
            b_cbf[0],          # CBF limit
            self.v_max,        # Max v
            self.omega_max     # Max w
        ])

        # Solve QP
        prob = osqp.OSQP()
        prob.setup(P, q, A, l, u_upper, verbose=False)
        res = prob.solve()

        if res.info.status != 'solved':
            self.get_logger().warn("QP Solver Failed! Emergency Stop.")
            v_cmd, omega_cmd = 0.0, 0.0
        else:
            # Map optimized virtual control back to v, w (Ignoring the slack variable)
            u_opt = np.array([[res.x[0]], [res.x[1]]])
            vw = T_FL @ u_opt
            v_cmd = float(vw[0, 0])
            omega_cmd = float(vw[1, 0])
            
            # Print a warning if the safety forcefield was breached
            if res.x[2] > 0.01:
                self.get_logger().warn(f"Safety breach! Recovering... (Slack active: {res.x[2]:.2f})")
    
        # Here it is used Euclidian Distance. If the leader turns, the safety forcefield "pushes"
        # along that straight line. This means that the robot in front "pushes" the follower sideways, it does not
        # "tow" it
        # In thsi scenario, we do not consider braking and accelerating, only moving at a constant speed

        # Publish Command
        twist_msg = Twist()
        twist_msg.linear.x = v_cmd
        twist_msg.angular.z = omega_cmd
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlatoonFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()