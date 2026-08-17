import rclpy
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import matplotlib.pyplot as plt

# Import the custom spline track from our utility file
from platooning_pkg.trajectory_utils_single_agent import generate_custom_spline

class SplineTrackerNode(Node):
    def __init__(self):
        super().__init__('spline_tracker_node')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 1. System Parameters
        self.L = 0.05               # Offset distance (m)
        self.v_max = 0.22           # Max linear velocity (m/s)
        self.omega_max = 2.84       # Max angular velocity (rad/s)      
        self.u_max = min(self.v_max, self.L * self.omega_max)
        
        # 2. Controller Parameters
        self.gamma = 3.0  
        self.P = np.diag([0.3, 0.3])
        self.v_desired = 0.20  # Cruise speed for our Feedforward controller
        # Even though in this case the speed will not be over 0.15 m/s, it is
        # a good thing to have like a limit or something
        
        # 3. Load the custom track & Pre-calculate exact curvature tangents
        self.path_x, self.path_y = generate_custom_spline()
        
        # Calculate the mathematical derivative (tangent) of the track at every point
        grad_x = np.gradient(self.path_x)
        grad_y = np.gradient(self.path_y)
        norms = np.hypot(grad_x, grad_y)
        norms = np.maximum(norms, 1e-6)  # Prevent division by zero
        self.path_tx = grad_x / norms    # Normalized X direction vector
        self.path_ty = grad_y / norms    # Normalized Y direction vector
        
        self.current_path_idx = 0  
        self.tick_count = 0
        
        self.xi = np.zeros((3, 1))  
        self.state_received = False 
        self.start_time = None
        
        # 4. Plotting Arrays
        self.history_t, self.history_x, self.history_y = [], [], []
        self.history_x_virt, self.history_y_virt = [], []
        self.history_v, self.history_w = [], []
        
        # 5. ROS 2 Publishers and Subscribers
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_system_default)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        
        # 25 Hz control loop
        self.timer = self.create_timer(0.04, self.control_loop)
        self.get_logger().info("Optimal Feedforward + Feedback Tracking Started...")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.xi = np.array([[x], [y], [theta]])
        self.state_received = True
        if self.start_time is None:
            self.start_time = self.get_clock().now()

    def control_loop(self):
        if not self.state_received:
            return
            
        theta = float(self.xi[2, 0])
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # Calculate Virtual Point
        x_virt = np.array([
            [float(self.xi[0, 0]) + self.L * math.cos(theta)],
            [float(self.xi[1, 0]) + self.L * math.sin(theta)]
        ])
        # This calculates the recent real-world oordonates of the virtual/fictive point based on the current
        # pose and heading
        
        # --- PATH PROJECTION ALGORITHM ---
        # Look at the next 50 points to find the absolute closest point to the robot
        search_range = 50
        end_search_idx = min(self.current_path_idx + search_range, len(self.path_x))
        
        dx = self.path_x[self.current_path_idx:end_search_idx] - x_virt[0, 0]
        dy = self.path_y[self.current_path_idx:end_search_idx] - x_virt[1, 0]
        dists = dx**2 + dy**2
        
        local_closest_idx = int(np.argmin(dists))
        # Evaluates the distance between the virtual point and the path points
        min_dist = dists[local_closest_idx]
        
        if min_dist < 0.25:
            self.current_path_idx += local_closest_idx
        
        # The reference target is set to the matched closest index on the trajectory path
        target_idx = min(self.current_path_idx, len(self.path_x) - 1)
        
        x_ref = np.array([
            [self.path_x[target_idx]],
            [self.path_y[target_idx]]
        ])
        
        # FEEDFORWARD CONTROL
        # Drive exactly along the path's mathematical tangent vector direction at our cruise speed
        if target_idx < len(self.path_x) - 1:
            u_ff = self.v_desired * np.array([
                [self.path_tx[target_idx]], 
                [self.path_ty[target_idx]]
            ])
        else:
            # If we are at the finish line, turn off feedforward to stop pushing forward
            u_ff = np.zeros((2, 1))

        # What FeedForward does: it looks ahead and anticipates what the speed and angular velocity
        # should be.
            
        # FEEDBACK CONTROL (The Corrector)
        # Fix any tiny deviations from the centerline
        x_err = x_virt - x_ref
        u_fb = -self.gamma * np.dot(self.P, x_err) # Check Eq. 23
        
        # Combine them to build the nominal input command!
        u_nom = u_ff + u_fb
        
        # Saturation check to respect maximum actuation bounds
        norm_u = np.linalg.norm(u_nom)
        if norm_u > self.u_max:
            u = self.u_max * (u_nom / norm_u)
        else:
            u = u_nom
            
        T_FL = np.array([
            [math.cos(theta), math.sin(theta)],
            [-math.sin(theta)/self.L, math.cos(theta)/self.L]
        ])
        # Check Equations (1) - (6), where it is explained the feedback linearization.
        # This T_FL is the inverse of the matrix from (4), which transforms the virtual input u in
        # the linear velocity v and angular speed w
        
        vw = np.dot(T_FL, u)
        v_cmd = float(vw[0, 0])
        omega_cmd = float(vw[1, 0])
        
        # Record data
        self.history_t.append(t)
        self.history_x.append(float(self.xi[0, 0]))
        self.history_y.append(float(self.xi[1, 0]))
        self.history_x_virt.append(float(x_virt[0, 0]))
        self.history_y_virt.append(float(x_virt[1, 0]))
        self.history_v.append(v_cmd)
        self.history_w.append(omega_cmd)
        
        # Publish
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = v_cmd
        twist_msg.twist.angular.z = omega_cmd
        self.publisher_.publish(twist_msg)
        
        # Print status log every 1 second (50 ticks)
        self.tick_count += 1
        if self.tick_count % 50 == 0:
            progress_pct = (self.current_path_idx / len(self.path_x)) * 100
            self.get_logger().info(f"Tracking Progress: {progress_pct:.1f}% | Speed: {v_cmd:.2f} m/s | Error: {np.linalg.norm(x_err):.3f}m")
        
        # Stop if we physically reach the end of the track
        if target_idx == len(self.path_x) - 1 and np.linalg.norm(x_err) < 0.1:
            self.get_logger().info("Finish line reached! Generating plots...")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            self.timer.cancel()
            self.plot_results()

    def plot_results(self):
        plt.figure('Trajectory Tracking', facecolor='w')
        plt.plot(self.path_x, self.path_y, 'k:', linewidth=1.5, label='Reference Track')
        plt.plot(self.history_x, self.history_y, 'b-', linewidth=1.5, label='Robot Center')
        plt.plot(self.history_x_virt, self.history_y_virt, 'g--', linewidth=1.5, label='Virtual Point')
        plt.title('Optimal Path Following (Feedforward + Feedback)')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid(True)
        plt.legend(loc='best')
        plt.axis('equal')

        plt.figure('Control Inputs', facecolor='w')
        plt.subplot(2, 1, 1)
        plt.plot(self.history_t, self.history_v, 'b-', linewidth=1.5)
        plt.axhline(self.v_max, color='r', linestyle='--')
        plt.axhline(-self.v_max, color='r', linestyle='--')
        plt.title('Linear Velocity (v)')
        plt.ylabel('m/s')
        plt.grid(True)

        plt.subplot(2, 1, 2)
        plt.plot(self.history_t, self.history_w, 'b-', linewidth=1.5)
        plt.axhline(self.omega_max, color='r', linestyle='--')
        plt.axhline(-self.omega_max, color='r', linestyle='--')
        plt.title('Angular Velocity (omega)')
        plt.xlabel('Time (s)')
        plt.ylabel('rad/s')
        plt.grid(True)

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = SplineTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()