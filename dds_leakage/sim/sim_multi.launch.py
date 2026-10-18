"""Simulation with five TurtleBot3 Burger robots, each in its own namespace (/tb3_1 .. /tb3_5)."""
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name = namespace, x [m], y [m], yaw [rad]) of each robot. The name is also the model name in
# Gazebo. Irregular positions and headings, so the robots are not lined up; the areas below hold
# if there is no drift, and the closest two trajectories get to each other is ~1.85 m.
ROBOTS = [
    ('tb3_1', 0.0, 0.0, 0.4),     # square:   x from -0.2 to 0.6 m, y from 0 to 0.8 m
    ('tb3_2', 3.1, -0.6, 2.0),    # triangle: x from 2.3 to 3.1 m,  y from -0.6 to 0.1 m
    ('tb3_3', 0.7, 2.8, -1.2),    # circle:   x from 0.7 to 1.7 m,  y from 2.5 to 3.5 m
    ('tb3_4', 3.4, 2.0, 0.9),     # up_down:  x from 3.4 to 4.3 m,  y from 2.0 to 3.2 m
    ('tb3_5', 6.0, 0.6, -2.4),    # hexagon:  x from 5.6 to 6.7 m,  y from -0.6 to 0.6 m
]


def model_without_imu():
    """
    Copy of the TurtleBot3 Burger model without the IMU, written to a temporary file.

    The IMU is alone in imu_link (no mass or geometry), so removing the link and its joint does
    not change the robot's dynamics; only the /<name>/imu topic disappears.
    """
    model_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_burger')
    tree = ET.parse(os.path.join(model_dir, 'model.sdf'))
    model = tree.getroot().find('model')
    imu = model.findall('link[@name="imu_link"]') + model.findall('joint[@name="imu_joint"]')
    for element in imu:
        model.remove(element)
    out = tempfile.NamedTemporaryFile('wb', prefix='burger_no_imu_', suffix='.sdf', delete=False)
    tree.write(out)  # no XML declaration: spawn_entity.py rejects one that names an encoding
    out.close()
    return out.name


def generate_launch_description():
    sim_dir = os.path.dirname(os.path.abspath(__file__))
    world = os.path.join(sim_dir, 'empty_with_state.world')
    params = os.path.join(sim_dir, 'gazebo_params.yaml')
    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    model_sdf = model_without_imu()

    gui = LaunchConfiguration('gui')

    actions = [
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzserver.launch.py')),
            launch_arguments={'world': world, 'params_file': params}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),
    ]

    for name, x, y, yaw in ROBOTS:
        # -robot_namespace puts the robot's plugins (cmd_vel, odom, scan, ...) under /<name>
        actions.append(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{name}',
            arguments=[
                '-entity', name,
                '-file', model_sdf,
                '-robot_namespace', name,
                '-x', str(x), '-y', str(y), '-z', '0.01', '-Y', str(yaw),
            ],
            output='screen',
        ))

    return LaunchDescription(actions)
