from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 공유 디렉토리 가져오기
    pinky_slam_share = get_package_share_directory('pinky_slam')

    # 매개변수 경로 직접 설정
    slam_config_path = os.path.join(pinky_slam_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config_path = os.path.join(pinky_slam_share, 'rviz', 'gmapping.rviz')

    # 경로 디버깅
    print(f"SLAM Config Path: {slam_config_path}")
    print(f"RViz Config Path: {rviz_config_path}")

    return LaunchDescription([
        # SLAM 노드 실행
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom')
            ]
        ),

        # RViz2 노드 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )
    ])