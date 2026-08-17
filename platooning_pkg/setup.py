import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'platooning_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Existing launch files inclusion
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Added this line to include the custom Gazebo Harmonic SDF models
        (os.path.join('share', package_name, 'models'), glob('models/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radu2003andrei',
    maintainer_email='radu2003andrei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # CSV generator
            'plotter_node = platooning_pkg.plotting_multiple:main',
            # visual plot
            'visualizer = platooning_pkg.visualization:main',
            # targer tracking
            'target_tracker = platooning_pkg.single_agent_clf:main',
            # trajectory tracking
            'trajectory_tracker = platooning_pkg.single_agent_traj_trak:main',
            # follower node (only constant speed and a straight line)
            'follower_node = platooning_pkg.follower_node:main',
            # follower node (here we also include braking)
            'braking_follower = platooning_pkg.follower_node_braking_also:main',
            # FDI-not-resistant
            'FDI_not_resistent = platooning_pkg.FDI_not_resistent:main',
            # FDI attack
            'FDI_attack = platooning_pkg.FDI_attack:main',
            # FDI-resistant path-graph
            'FDI_feasable_path_graph = platooning_pkg.FDI_feasable_path_graph:main',
            # FDI-resistant full-graph
            'FDI_feasable_full_graph = platooning_pkg.FDI_feasable_full_graph:main',
            # Curve platooning
            'curve_platoon_follower = platooning_pkg.curve_platoon_follower:main',
        ],
    },
)