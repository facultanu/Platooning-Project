import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Set the model type
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    # Get directories
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    models_dir = os.path.join(tb3_gazebo_dir, 'models')
    
    # CRITICAL STEP
    # Tell Gazebo Harmonic where to find the TurtleBot3 meshes
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + models_dir
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = models_dir

    # 1. Start Gazebo Harmonic natively
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    ld = LaunchDescription([gz_sim])

    # 2. Define our robots
    robots = [
        {'name': 'tb3_leader',    'x': '0.0',  'y': '0.0', 'z': '0.1'}, 
        {'name': 'tb3_follower1', 'x': '-0.3', 'y': '0.0', 'z': '0.1'},
        {'name': 'tb3_follower2', 'x': '-0.6', 'y': '0.0', 'z': '0.1'}
    ]

    sdf_file = os.path.join(models_dir, 'turtlebot3_burger', 'model.sdf')

# Read the raw SDF into a string
    with open(sdf_file, 'r') as f:
        base_sdf = f.read()

    # 3. Spawn each robot
    # 3. Spawn each robot and build its ROS-Gazebo Bridge
    for robot in robots:
        # Dynamically inject the namespace into the hardcoded plugin topics
        robot_sdf = base_sdf.replace('>cmd_vel<', f'>model/{robot["name"]}/cmd_vel<')
        robot_sdf = robot_sdf.replace('>odom<', f'>model/{robot["name"]}/odom<')
        
        # 2. Spawn using the dynamically rewritten string (-string instead of -file)
        spawn_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', robot['name'],
                '-string', robot_sdf,
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', robot['z']
            ],
            output='screen'
        )
        ld.add_action(spawn_cmd)

        # 3. The Bridge (Translating isolated Gazebo topics to clean ROS topics)
        bridge_cmd = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/model/{robot["name"]}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/model/{robot["name"]}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
            remappings=[
                (f'/model/{robot["name"]}/cmd_vel', f'/{robot["name"]}/cmd_vel'),
                (f'/model/{robot["name"]}/odom', f'/{robot["name"]}/odom')
            ],
            output='screen'
        )
        ld.add_action(bridge_cmd)

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    ld.add_action(clock_bridge)
    return ld