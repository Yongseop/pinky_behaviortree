from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os import environ

def generate_launch_description():
    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    # Declare arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='office_world.world',
        description='Name of the Gazebo world file to load'
    )

    # Paths
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    pinky_worlds_path = get_package_share_directory('pinky_worlds')

    # PathJoinSubstitution for dynamic path resolution
    world_file_path = PathJoinSubstitution([
        pinky_worlds_path, 'world', LaunchConfiguration('world_name')
    ])

    # Set GAZEBO_MODEL_PATH to the model folder of pinky_worlds package
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([pinky_worlds_path, 'model'])
    )

    # Include gzserver.launch.py
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_path, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={
            'verbose': 'true',
            'physics': 'ode',
            'lockstep': 'true',
            'world': world_file_path
        }.items()
    )

    # Include gzclient.launch.py
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_path, 'launch', 'gzclient.launch.py'])
        )
    )

    return LaunchDescription([
        set_gazebo_model_path,  # Set GAZEBO_MODEL_PATH
        world_name_arg,         # World name argument
        gzserver,               # Gazebo server
        gzclient                # Gazebo client
    ])