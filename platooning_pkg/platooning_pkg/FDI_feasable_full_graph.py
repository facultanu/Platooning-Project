import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import cvxpy as cp

class ResilientCBFController(Node):
    def __init__(self):
        super().__init__('platoon_follower_node')
        
        # =====================================================================
        # 1. ROS 2 PARAMETERS & PLATOON SETUP
        # =====================================================================
        self.declare_parameter('my_name', 'tb3_follower1')
        self.declare_parameter('target_name', 'tb3_leader')
        self.declare_parameter('spawn_gap', 1.0) 
        
        self.my_name = self.get_parameter('my_name').value
        self.target_name = self.get_parameter('target_name').value
        self.spawn_gap = self.get_parameter('spawn_gap').value
        
        # PAPER Reference: Section II.A (Network Model)
        self.platoon = ['tb3_leader', 'tb3_follower1', 'tb3_follower2', 'tb3_follower3']
        if self.my_name in self.platoon:
            self.my_idx = self.platoon.index(self.my_name)
        else:
            self.my_idx = 0
            
        # =====================================================================
        # 2. CONTROLLER & CBF PARAMETERS FROM THE PAPER
        # =====================================================================
        self.dt = 0.05
        self.standstill_gap = 0.3  # \iota - Standstill desired distance
        self.time_headway = 0.4    # \eta (niu)  - Constant Time Headway (CTH)
        self.epsilon = 0.5         # \epsilon - Worst-case FDI attack bound (maximum value the hacker can add to the distance)
        self.l_safe = 1.0          # l_safe - Safety Lipschitz constant

        # Epsilon vs standstill_gap:

        # standstill_gap bigger advantage:
        # - better safety and robustness (easier to keep this bigger distance). This also means that
        # when one follower (in the future the leader) is attacked, the robots have more time to brake and not crash
        # Disadvantage:
        # - not practical in "crowded cities" and not that good string stability (takes a lot of space)

        # Epsilon smaller advantage:
        # - better string stability more efficient (smaller worst-case amrgin, vehicles can be closer)
        # DisadvantAAGE:
        # - not very robust (if the hacker adds a bigger distance then epsilon,
        # the CBF-QP (Control Barrier Function - Quadratic Programming) will fail)
        
        self.my_state = {'q': 0.0, 'v': 0.0, 'a': 0.0}
        self.neighbors = {} 
        self.dropped_neighbors = [] 
        self.suspicion_scores = {} 
        
        self.target_info = {'e': 0.0, 'dot_e': 0.0}
        self.last_v_lead = 0.0
        self.start_time = self.get_clock().now().nanoseconds
        self.last_time_lead = self.start_time
        
        # =====================================================================
        # 3. DYNAMIC SUBSCRIBERS & PUBLISHERS
        # Basically making the full-graph communication style 
        # =====================================================================
        self.sub_odom = self.create_subscription(
            Odometry, f'/{self.my_name}/odom', self.odom_callback, 10)
        
        self.neighbor_subs = []
        for robot_name in self.platoon:
            if robot_name != self.my_name:
                sub = self.create_subscription(
                    Odometry, f'/{robot_name}/odom', 
                    lambda msg, name=robot_name: self.neighbor_callback(msg, name), 10)
                self.neighbor_subs.append(sub)
            
        self.pub_info = self.create_publisher(
            Vector3, f'/{self.my_name}/platoon_info', 10)
            
        if self.target_name != 'tb3_leader':
            self.sub_target_info = self.create_subscription(
                Vector3, f'/{self.target_name}/platoon_info', self.target_info_callback, 10)
        
        self.pub_cmd = self.create_publisher(Twist, f'/{self.my_name}/cmd_vel', 10)
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"{self.my_name} tracking {self.target_name} (Resilient Full-Graph Active)!")


    def odom_callback(self, msg):
        self.my_state['q'] = msg.pose.pose.position.x
        self.my_state['v'] = msg.twist.twist.linear.x

    def neighbor_callback(self, msg, name):
        v_j = msg.twist.twist.linear.x
        a_j = 0.0
        
        if name == self.target_name: 
            current_time = self.get_clock().now().nanoseconds
            dt_seconds = (current_time - self.last_time_lead) / 1e9
            if dt_seconds > 0:
                a_j = (v_j - self.last_v_lead) / dt_seconds
            self.last_v_lead = v_j
            self.last_time_lead = current_time

        self.neighbors[name] = {'q': msg.pose.pose.position.x, 'v': v_j, 'a': a_j}

    def target_info_callback(self, msg):
        self.target_info['e'] = msg.x
        self.target_info['dot_e'] = msg.y

    # =====================================================================
    # 4. NOMINAL RESILIENT CONTROLLER (Section II.D)
    # =====================================================================
    def compute_nominal_resilient_control(self):
        if not self.neighbors:
            return 0.0
            
        q_me = self.my_state['q']
        v_me = self.my_state['v']
        a_me = self.my_state.get('a', 0.0)
        
        kq = 2.0
        kv = 4.0
        ka = 1.0  # PAPER Reference: Equation (3)
        deviations = {}
        
        for j_name, j_state in self.neighbors.items():
            if j_name not in self.platoon:
                continue
                
            j_idx = self.platoon.index(j_name)
            
            # =================================================================
            # DIRECTED PREDECESSOR TRACKING (Section II.A)
            # Only synchronize with preceding vehicles ahead of us (j < i).
            # Never allow downstream followers (j >= i) to drag us backward!
            # =================================================================
            if j_idx >= self.my_idx:
                continue
                
            vector_dist = self.my_idx - j_idx 
            
            # PAPER Reference: Equation (2) - Constant Time Headway (CTH)
            d_desired = vector_dist * (self.standstill_gap + (self.time_headway * v_me))
            q_target = j_state['q'] - d_desired
            
            err_q = q_me - q_target  
            err_v = v_me - j_state['v']    
            err_a = a_me - j_state.get('a', 0.0)
            
            raw_score = abs(err_q)+ abs(err_v)
            
            if j_name not in self.suspicion_scores:
                self.suspicion_scores[j_name] = raw_score
            else:
                if raw_score > 0.4:
                    self.suspicion_scores[j_name] = raw_score
                else:
                    self.suspicion_scores[j_name] = (0.95 * self.suspicion_scores[j_name]) + (0.05 * raw_score)
                
            deviations[j_name] = {
                'err_q': err_q, 
                'err_v': err_v, 
                'err_a': err_a, 
                'norm': self.suspicion_scores[j_name]
            }
            
        # PAPER Reference: Section II.D - Outlier Rejection & Assumption 1
        F_attacks = 1  # Set this to the maximum number of simultaneous attackers (modify if wanted)
        # Here the leader can't be affected
        suspicious_candidates = [name for name in deviations.keys() if name != 'tb3_leader']
        
        # Sort so the highest suspicion scores are at the front of the list
        suspicious_candidates.sort(key=lambda name: deviations[name]['norm'], reverse=True)
        
        attack_threshold = 0.45 
        elapsed_time = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
        
        # Clear the dropped list every loop before re-evaluating
        self.dropped_neighbors = []
        
        if elapsed_time > 15.0:  # We ckeck the suspect(s) after the attack has been initialized
            # Check up to F_attacks number of vehicles
            for i in range(min(F_attacks, len(suspicious_candidates))):
                suspect = suspicious_candidates[i]
                if deviations[suspect]['norm'] > attack_threshold:
                    self.dropped_neighbors.append(suspect)
                    self.get_logger().warn(f"ATTACK DETECTED! Outlier rejection dropped {suspect} (Score: {deviations[suspect]['norm']:.2f})")

        # PAPER Reference: Equation (3) - Resilient Nominal Controller u_{i,res}
        u_res_total = 0.0
        for j_name, dev in deviations.items():
            if j_name not in self.dropped_neighbors:
                u_res_total += (-kq * dev['err_q']) - (kv * dev['err_v']) - (ka * dev['err_a'])
                
        return u_res_total

    # =====================================================================
    # 5. RESILIENT CBF-QP CONTROL LOOP (Sections III, IV, V)
    # =====================================================================
    def control_loop(self):
        if self.target_name not in self.neighbors:
            return

        u_res = self.compute_nominal_resilient_control()

        # Defining CVXPY variables 
        u = cp.Variable(1)
        rho = cp.Variable(1)      
        rho_fea = cp.Variable(1)  

        # Here I add the constrains/restrictions/limitations (that s.t (subject to))
        constraints = []
        a_max = 0.06
        constraints.append(u <= a_max)
        constraints.append(u >= -a_max)
        
        # DYNAMIC CBF REPOSITIONING
        cbf_target_name = self.target_name
        if cbf_target_name in self.dropped_neighbors:
            for idx in reversed(range(self.my_idx)):
                candidate = self.platoon[idx]
                if candidate not in self.dropped_neighbors:
                    cbf_target_name = candidate
                    break
                    
        q_lead = self.neighbors[cbf_target_name]['q']
        v_lead = self.neighbors[cbf_target_name]['v']
        a_lead = self.neighbors[cbf_target_name]['a']
        q_me = self.my_state['q']
        v_me = self.my_state['v']
        
        target_idx = self.platoon.index(cbf_target_name)
        target_dist = self.my_idx - target_idx
        
        actual_gap = target_dist * self.standstill_gap
        actual_headway = target_dist * self.time_headway

        # PAPER Reference: Assumption 1 & Section V.B Conditional Relaxation
        # - Leader vehicle is secure (EXTEND THIS TO LEADER ALSO BEING AFFECTED)
        elapsed_time = (self.get_clock().now().nanoseconds - self.start_time) / 1e9

        if cbf_target_name == 'tb3_leader':
            active_epsilon = 0.0
        else:
            target_score = self.suspicion_scores.get(cbf_target_name, 0.0)
            
            # GRACE PERIOD FIX: 
            # Give the platoon 12 seconds to physically close the 1.0m spawn gaps.
            # After 12s, if a vehicle acts suspiciously, ACTIVATE the CBF shield.
            if elapsed_time > 12.0 and target_score > 0.25:
                active_epsilon = self.epsilon
            else:
                active_epsilon = 0.0

        # A. SAFETY CBF (Section III.B)
        h_safe = (q_lead - q_me) - actual_gap - (actual_headway * v_me)
        h_robust = h_safe - (self.l_safe * active_epsilon)
        alpha = 1.0 
        constraints.append(actual_headway * u <= (v_lead - v_me) + alpha * h_robust)

        # B. FEASIBILITY CBF (Section IV.B.2 & Theorem 3)
        Delta = max(alpha * h_robust, 0.0)
        gamma_fea = 1.0
        alpha_fea = 1.0
        h_F = (v_lead - v_me) + alpha * h_robust + (actual_headway * a_max)
        coeff_u = 1.0 + (alpha * actual_headway)
        right_side_fea = alpha * (v_lead - v_me) + gamma_fea * h_F
        
        constraints.append(coeff_u * u <= right_side_fea + rho_fea)
        constraints.append(rho_fea <= alpha_fea * Delta)
        constraints.append(rho_fea >= 0)

        # C. STRING STABILITY CBF (Section V.B.2)
        if cbf_target_name == 'tb3_leader':
            err_lead = 0.0
            dot_err_lead = 0.0
        else:
            err_lead = self.target_info['e']
            dot_err_lead = self.target_info['dot_e']
        
        err_me = (q_lead - q_me) - actual_gap - (actual_headway * v_me)
        dot_err_me_partial = v_lead - v_me  
        
        def sgn(x):
            if x > 0: return 1.0
            elif x < 0: return -1.0
            else: return 0.0
            
        s_me = sgn(err_me)
        s_lead = sgn(err_lead)
        l_str = 1.0 
        
        accel_threshold = 0.02
        if abs(a_lead) >= accel_threshold:
            h_str = abs(err_lead) - abs(err_me) - (l_str * active_epsilon)
        else:
            h_str = abs(err_lead) - abs(err_me) + (l_str * active_epsilon)
            
        gamma_str = 1.0
        right_side_str = (-s_lead * dot_err_lead) + (s_me * dot_err_me_partial) - (gamma_str * h_str)
        constraints.append((s_me * actual_headway * u) + rho >= right_side_str)

        # D. OPTIMIZATION OBJECTIVE (Section IV.A & Equation 37)
        w_str = 10.0   
        w_fea = 100.0
        # Here we define the optimization problem from Eq. 37
        cost = cp.Minimize(cp.square(u - u_res) + w_str * cp.square(rho) + w_fea * cp.square(rho_fea))
        prob = cp.Problem(cost, constraints)

        # Here we use OSQP to solve this
        try:
            prob.solve(solver=cp.OSQP)
            if prob.status in [cp.INFEASIBLE, cp.INFEASIBLE_INACCURATE]:
                u_opt = -a_max
            elif u.value is None:
                u_opt = -a_max
            else:
                u_opt = u.value[0]
        except Exception as e:
            self.get_logger().warn(f"Solver Error: {e}. Fallback braking.")
            u_opt = -a_max

        msg_info = Vector3()
        msg_info.x = err_me
        msg_info.y = dot_err_me_partial - (actual_headway * u_opt) 
        msg_info.z = u_opt 
        self.pub_info.publish(msg_info)

        cmd = Twist()
        new_v = self.my_state['v'] + (u_opt * self.dt)
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