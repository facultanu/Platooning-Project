import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cvxpy as cp

class ResilientCBFController(Node):
    def __init__(self):
        super().__init__('platoon_follower_node')
        
        # 1. Declare ROS 2 Parameters
        self.declare_parameter('my_name', 'tb3_follower1')
        self.declare_parameter('target_name', 'tb3_leader')
        self.declare_parameter('my_offset', 1.0)
        self.declare_parameter('target_offset', 2.0)
        
        # 2. Retrieve Parameters
        # Keeping generic so we can have the platoon of 3 cars
        self.my_name = self.get_parameter('my_name').value
        self.target_name = self.get_parameter('target_name').value
        self.my_offset = self.get_parameter('my_offset').value
        self.target_offset = self.get_parameter('target_offset').value
        
        # PAPER Reference: Section II.C (FDI Attack Model) and Section II.D
        self.dt = 0.05
        self.time_headway = 0.8    # eta (niu) - Time-gap used in CTH policy
        self.standstill_gap = 0.3  # iota - Desired baseline distance between cars
        self.epsilon = 0.5         # FDI worst-case bound (epsilon) from Assumption 2
        self.l_safe = 1.0          # Lipschitz constant for safety (l_safe) from Assumption 3
        
        self.my_state = {'q': 0.0, 'v': 0.0, 'a': 0.0}
        self.neighbors = {} 
        
        # 3. Dynamic Subscribers & Publishers
        self.sub_odom = self.create_subscription(
            Odometry, f'/{self.my_name}/odom', self.odom_callback, 10)
            
        # PAPER Reference: Section II.C - FDI Attack Model
        # This subscribes to the neighbor's state which may contain injected false data: xhat_j(t) = x_j(t) + delta_j(t)
        self.sub_neighbor = self.create_subscription(
            Odometry, f'/{self.target_name}/odom', self.neighbor_callback, 10)
        
        self.pub_cmd = self.create_publisher(
            Twist, f'/{self.my_name}/cmd_vel', 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info(f"{self.my_name} is now tracking {self.target_name}!")

    def odom_callback(self, msg):
        self.my_state['q'] = msg.pose.pose.position.x + self.my_offset
        self.my_state['v'] = msg.twist.twist.linear.x

    def neighbor_callback(self, msg):
        self.neighbors[self.target_name] = {
            'q': msg.pose.pose.position.x + self.target_offset,
            'v': msg.twist.twist.linear.x
        }

    def compute_nominal_resilient_control(self):
        if self.target_name not in self.neighbors:
            return 0.0
            
        q_lead = self.neighbors[self.target_name]['q']
        v_lead = self.neighbors[self.target_name]['v']
        q_me = self.my_state['q']
        v_me = self.my_state['v']
        
        kq = 2.0
        kv = 4.0
        
        # PAPER Reference: Equation (2) - Constant Time Headway (CTH) policy
        # d_{i,j}(t) = iota + eta v_i(t)
        d_desired = self.standstill_gap + (self.time_headway * v_me)
        q_target = q_lead - d_desired
        
        err_q = q_me - q_target  
        err_v = v_me - v_lead    

        # PAPER Reference: Equation (3) - Resilient Nominal Controller (u_{i,res})
        # Simplified for double integrator dynamics (ignoring acceleration tracking) and single predecessor topology
        u_res = -kq * err_q - kv * err_v
        return u_res

    def control_loop(self):
        if self.target_name not in self.neighbors:
            return

        u_res = self.compute_nominal_resilient_control()
        
        # PAPER Reference: Section IV.A - Relaxation of String Stability
        # Slack variables introduced to prevent QP (Quadratic Programming) solver infeasibility
        u = cp.Variable(1)
        rho = cp.Variable(1) # Note: String stability is not fully implemented here, but slack is reserved.
        
        w1 = 10.0 # penalty weight
        
        # PAPER Reference: Equation (22) - Relaxed Resilient-CBF-QP Objective Function
        # Minimizing the deviation from the nominal resilient control while penalizing the slack variable
        cost = cp.Minimize(cp.square(u - u_res) + w1 * cp.square(rho))
        
        constraints = []
        
        # PAPER Reference: Section III.A - Bounded State Constraints & Section IV.B.1 (Parameter Predesign)
        # Enforcing symmetric physical acceleration limits of the TurtleBot3 actuator (a_{min} = -a_{max})
        a_max = 0.06
        constraints.append(u <= a_max)
        constraints.append(u >= -a_max)
        
        q_lead = self.neighbors[self.target_name]['q']
        q_me = self.my_state['q']
        v_me = self.my_state['v']
        
        # =====================================================================
        # 1. SAFETY CBF
        # =====================================================================
        # PAPER Reference: Equation (13) - Real safety requirement
        # Note: The standstill_gap (iota) buffer is omitted here compared to the actual paper equation.
        h_safe = (q_lead - q_me) - self.time_headway * v_me
        
        # PAPER Reference: Theorem 1 and Equation (15) - Boundary Robustness
        # Shrinks the admissible set by the worst-case FDI attack bound (l * epsilon) to ensure safety against false data.
        h_robust = h_safe - (self.l_safe * self.epsilon)
        
        alpha = 1.0 
        # PAPER Reference: Derived from Theorem 1 (Equation 14) adapted for simplified double-integrator dynamics
        # This translates the continuous CBF condition into a linear constraint for the QP solver.
        constraints.append(self.time_headway * u <= -v_me + alpha * h_robust)
        
        prob = cp.Problem(cost, constraints)
        try:
            # Resolves the constrained optimization problem
            prob.solve(solver=cp.OSQP)
            u_opt = u.value[0]
        except:
            # PAPER Reference: Section IV.B, Remark 5 - Fallback Strategy
            # When QP becomes infeasible (e.g., safety constraint conflicts with a_max), switch to a fail-safe maximum braking mode.
            u_opt = -a_max

        cmd = Twist()
        
        # Calculate new velocity based on optimized acceleration
        new_v = self.my_state['v'] + (u_opt * self.dt)
        
        # PAPER Reference: Section VI.B (Experimental Validation)
        # Clamp velocity to TurtleBot3 physical bounds [0.0, 0.22] m/s to prevent impossible actuator commands.
        v_max = 0.22
        cmd.linear.x = max(0.0, min(v_max, new_v))
        
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ResilientCBFController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()