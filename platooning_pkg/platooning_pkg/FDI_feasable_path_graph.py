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
        
        # Parameters (mapped from the paper's Nomenclature)
        # We will assume that the communication happens between consecutive cars
        # (leader<->follower1, follower1<->follower2 and so on)
        self.dt = 0.05
        self.time_headway = 0.4    # eta (niu) (Time-gap) 
        self.standstill_gap = 0.3  # iota (desired distance between cars) 
        self.epsilon = 0.5         # FDI worst-case bound 
        self.l_safe = 1.0          # Lipschitz constant for safety 
        
        self.my_state = {'q': 0.0, 'v': 0.0, 'a': 0.0}
        self.neighbors = {} 
        
        # 3. Dynamic Subscribers & Publishers
        self.sub_odom = self.create_subscription(
            Odometry, f'/{self.my_name}/odom', self.odom_callback, 10)
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
        
        # PAPER Reference: Equation (2) - Desirable distance calculation 
        d_desired = self.standstill_gap + (self.time_headway * v_me)
        q_target = q_lead - d_desired
        
        err_q = q_me - q_target  
        err_v = v_me - v_lead    
        
        # PAPER Reference: Equation (3) - Resilient Nominal Controller 
        # (Simplified here for a single predecessor topology)
        u_res = -kq * err_q - kv * err_v
        return u_res

    def control_loop(self):
        if self.target_name not in self.neighbors:
            return

        u_res = self.compute_nominal_resilient_control()

        # Here used CVXPY and OSQP for solving this
        # https://www.cvxpy.org/index.html (CVXPY)
        # https://osqp.org/docs/index.html# (OSQP)

        u = cp.Variable(1)
        rho = cp.Variable(1) # Slack variable for soft constraints
        
        w1 = 10.0 # penalty weight
        
        # PAPER Reference: Equation (23) - Objective Function 
        # Minimizing deviation from nominal control while penalizing the slack variable
        cost = cp.Minimize(cp.square(u - u_res) + w1 * cp.square(rho))
        
        constraints = []
        
        a_max = 0.06
        # PAPER Reference: Section III.A - Bounded State Constraints 
        # (Acceleration limits for the Turtlebot actuator)
        constraints.append(u <= a_max)
        constraints.append(u >= -a_max)
        
        q_lead = self.neighbors[self.target_name]['q']
        v_lead = self.neighbors[self.target_name]['v']
        q_me = self.my_state['q']
        v_me = self.my_state['v']
        
        # PAPER Reference: Equation (13) - The raw Safety Constraint. I subtracted the standstill_gap
        # from this
        h_safe = (q_lead - q_me) - self.standstill_gap - self.time_headway * v_me
        
        # PAPER Reference: Equation (15) and Theorem 1 - Boundary Robustness 
        # Shrinking the safe space by worst-case attack bound (l * epsilon)
        h_robust = h_safe - (self.l_safe * self.epsilon)
        
        alpha = 1.0 
        constraints.append(self.time_headway * u <= (v_lead - v_me) + alpha * h_robust)

        # FEASIBILITY CBF WITH ADAPTIVE RELAXATION (Theorem 3)
        alpha_fea = 1.0
        
        # 1. Define the Slack Variable (rho_fea)
        # Note: fea = feasable
        # We must also add this to the cost function at the top of the loop!
        rho_fea = cp.Variable(1)
        
        # 2. Split into conservative part and residual Delta (Eq 34)
        # Delta = s_5 * (q_{i-1} - q_i - eta * v_i - l_safe{i-1} * epsilon) = alpha * h_robust
        Delta = alpha * h_robust
        
        # 3. Formulate the Relaxed Feasibility Constraint (Eq 36a)
        # u <= alpha_fea * ((v_lead - v_me) + eta * a_max) + rho_fea
        right_side_fea = alpha_fea * ((v_lead - v_me) + self.time_headway * a_max)
        constraints.append(u <= right_side_fea + rho_fea)
        
        # 4. Enforce the Bound on the Relaxation (Eq 36b)
        constraints.append(rho_fea <= alpha_fea * Delta)
        constraints.append(rho_fea >= 0) # Slack must be non-negative

        # REDEFINE THE COST FUNCTION
        w_fea = 100.0 # High penalty ensures relaxation is only used when strictly necessary
        cost = cp.Minimize(cp.square(u - u_res) + w_fea * cp.square(rho_fea))
        # Wish to minimize this this (check Eq. 22)
        
        prob = cp.Problem(cost, constraints)
        try:
            prob.solve(solver=cp.OSQP) # OSQP = Operator Splitting Quadratic Programming (the solver)
            # used OSQP in follower_node_braking_also
            # Other solvers available: https://www.cvxpy.org/tutorial/solvers/index.html
            u_opt = u.value[0]
        except:
            # PAPER Reference: Section IV.B (Remark 5) - Fallback to fail-safe maximum braking 
            u_opt = -a_max

        # Basically CVXPY and OSQP work together: CVXPY defines variables, objectives and contrains
        # OSQP handles the numerical solving

        cmd = Twist()
        
        # Calculate new velocity based on optimized acceleration
        new_v = self.my_state['v'] + (u_opt * self.dt)
        
        # PAPER Reference: Experimental Validation (Section VI.B) - Velocity physical limits 
        # Clamp velocity to TurtleBot3 physical limits [0.0, 0.22] m/s
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