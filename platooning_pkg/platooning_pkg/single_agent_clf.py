import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
import matplotlib.pyplot as plt

class SaturatedControllerNode(Node):
    def __init__(self):
        super().__init__('saturated_controller_node')
        
        # Force the node to use Gazebo's simulated clock
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 1. System Parameters
        self.L = 0.05               # Offset distance (m)
        # distance between the robot and the fictive/cirtual point
        # This robot can't go sideways, so we can't linearize directly in the center. Instead, we take
        # a virtual/fictive point 

        # limits of the TurtleBot3 Burger
        self.v_max = 0.22           # Max linear velocity (m/s)
        self.omega_max = 2.84       # Max angular velocity (rad/s)

        # Check Eq.8. This is the saturated control. This ensures that we cannot pass both limits
        # that the robot has
        self.u_max = min(self.v_max, self.L * self.omega_max)
        
        # 2. Controller Parameters (Check Eq. 23)
        self.gamma = 3.0
        self.P = np.diag([0.3, 0.3])
        # B is the identity matrix, so I did not write it in the code
        self.x_axis = 1
        self.y_axis = 2
        self.x_ref = np.array([[self.x_axis + self.L * math.cos(math.atan2(self.y_axis, self.x_axis))], [self.y_axis + self.L * math.sin(math.atan2(self.y_axis, self.x_axis))]])
        # Assuming we always start from (0,0)
        # Target Position + the error
        # I want to be exactly in the (X,Y) coordonates
        # The idea is that this script ensures the fictive point arrives at the desired target. However, we want
        # the Burger to arrive there, so have modified the XY target like this:

        # First I calculated the alpha angle with atan2(y,x). Explanation: tan alpha = y/x. From here alpha = tan^-1(y/x)
        # , so alpha = atan (y/x)

        # Then added at the x_axis target L*cos(alpha) and at the y_axis target L*sin(alpha).
        # This way, the robot will be much closer to the target point
        
        # 3. State Variables
        self.xi = np.zeros((3, 1))  # [x, y, theta]
        self.state_received = False # Don't calculate until we know where we are
        
        # 4. Data Recording Arrays (For Plotting)
        self.history_t = []
        self.history_x = []
        self.history_y = []
        self.history_x_virt = []
        self.history_y_virt = []
        self.history_v = []
        self.history_w = []
        self.start_time = None
        
        # 5. ROS2 Publishers, Subscribers, and Timers
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_system_default)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        
        # Run the control loop at 25 Hz (0.04s)
        timer_period = 0.04 
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info("Saturated Controller Node Started. Chasing target...")

    def odom_callback(self, msg):
        """ Update robot state from odometry """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.xi = np.array([[x], [y], [theta]])
        self.state_received = True
        
        if self.start_time is None:
            # Record the exact time we got our first message
            self.start_time = self.get_clock().now()

    def control_loop(self):
        """ The main math block (runs at 25 Hz) """
        if not self.state_received:
            return # Wait for odometry
            
        theta = float(self.xi[2, 0])
        
        # Calculate the actual position of the virtual point 'x'
        x_virt = np.array([
            [float(self.xi[0, 0]) + self.L * math.cos(theta)],
            [float(self.xi[1, 0]) + self.L * math.sin(theta)]
        ])
        # This calculates the recent real-world oordonates of the virtual/fictive point based on the current
        # pose and heading
        
        # Calculate remaining distance
        x_err = x_virt - self.x_ref
        
        # Nominal Controller Design (Check Eq. 23)
        u_nom = -self.gamma * np.dot(self.P, x_err)
        print(f"Nominal controller: {u_nom}")

        # This is the saturation
        norm_u = np.linalg.norm(u_nom) # norm of the command
        print(f"Nominal command normalized: {norm_u}")
        if norm_u > self.u_max: # if this exceeds the max
            u = self.u_max * (u_nom / norm_u) # it limits the command to u_max
            print(f"Command too high. Normalizing to: {u}")
        else: # if it does not exceed
            u = u_nom  # keep the command
            print(f"Command ok. Keep it {u}")
            
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
        
        # --- RECORD DATA FOR PLOTTING ---
        current_time_sec = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.history_t.append(current_time_sec)
        self.history_x.append(float(self.xi[0, 0]))
        self.history_y.append(float(self.xi[1, 0]))
        self.history_x_virt.append(float(x_virt[0, 0]))
        self.history_y_virt.append(float(x_virt[1, 0]))
        self.history_v.append(v_cmd)
        self.history_w.append(omega_cmd)
        
        # --- PUBLISH COMMANDS ---
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = v_cmd
        twist_msg.twist.angular.z = omega_cmd
        self.publisher_.publish(twist_msg)
        
        # --- STOP & PLOT LOGIC ---
        if np.linalg.norm(x_err) < 0.001:
            self.get_logger().info("Target reached! Stopping and generating plots...")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            
            # Cancel the timer so the loop stops
            self.timer.cancel()
            
            # Trigger the plotting function
            self.plot_results()

    def plot_results(self):
        """ Generate MATLAB-style plots using the recorded history """
        # Plot 1: X-Y Trajectory
        plt.figure('TurtleBot3 Trajectory', facecolor='w')
        plt.plot(self.history_x, self.history_y, 'b-', linewidth=1.5, label='Robot Center')
        plt.plot(self.history_x_virt, self.history_y_virt, 'g--', linewidth=1.5, label='Virtual Point')
        plt.plot(self.x_ref[0,0], self.x_ref[1,0], 'ro', markersize=8, label='Target')
        plt.title('Robot Trajectory in X-Y Plane')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.grid(True)
        plt.legend(loc='best')
        plt.axis('equal')

        # Plot 2: Velocity Profiles
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
        plt.show() # This opens the windows and pauses the script until you close them

def main(args=None):
    rclpy.init(args=args)
    controller_node = SaturatedControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()