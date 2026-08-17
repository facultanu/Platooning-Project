import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Ensure the TurtleBot3 model is set to burger
    set_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    
    # Get paths for Gazebo Harmonic and Turtlebot3 models
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    models_dir = os.path.join(tb3_gazebo_dir, 'models')
    
    # Tell Gazebo where to find the TurtleBot3 meshes/materials
    set_model_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir)

    # Start Gazebo Harmonic server and client with an empty world
    # The '-r' flag starts the simulation automatically (unpaused)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # Robot configurations (Name, X_pos, Y_pos)
    robots = [
        {'name': 'tb3_leader', 'x': '3.0', 'y': '0.0'}, # Leader
        {'name': 'tb3_follower1', 'x': '2.0', 'y': '0.0'}, # Follower 1
        {'name': 'tb3_follower2', 'x': '1.0', 'y': '0.0'}, # Follower 2
        {'name': 'tb3_follower3', 'x': '0.0', 'y': '0.0'}, # Follower 3
    ]

    # Path to the Jazzy-updated SDF file
    # Path to our custom Harmonic SDF file
    pkg_dir = get_package_share_directory('platooning_pkg')
    model_path = os.path.join(pkg_dir, 'models', 'tb3_harmonic.sdf')

    spawn_cmds = []
    
    for robot in robots:
        # 1. Spawn the robot in Gazebo Harmonic
        spawn_cmds.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', robot['name'],
                    '-file', model_path,
                    '-x', robot['x'],
                    '-y', robot['y'],
                    '-z', '0.05'
                ],
                output='screen'
            )
        )
        
        # 2. Bridge the topics between Gazebo and ROS 2
        # Gazebo publishes to /model/<name>/odometry, so we remap it to /<name>/odom 
        # so it matches our Python script expectations.
        spawn_cmds.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    # Bridge Twist (Velocity Commands) TO Gazebo
                    f'/model/{robot["name"]}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    # Bridge Odometry (State) FROM Gazebo
                    f'/model/{robot["name"]}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
                ],
                remappings=[
                    (f'/model/{robot["name"]}/cmd_vel', f'/{robot["name"]}/cmd_vel'),
                    (f'/model/{robot["name"]}/odometry', f'/{robot["name"]}/odom')
                ],
                output='screen'
            )
        )

    return LaunchDescription([
        set_env,
        set_model_path,
        gz_sim,
    ] + spawn_cmds)