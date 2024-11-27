from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 모델 경로 설정
    pinky_worlds_model_path = os.path.join(
        get_package_share_directory('pinky_worlds'), 'model'
    )

    # GAZEBO_MODEL_PATH 환경 변수 설정
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=pinky_worlds_model_path
    )

    # Gazebo를 실행하는 노드
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        arguments=['--verbose', '-s', 'libgazebo_ros_factory.so', 'office_world.world'],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo_node
    ])