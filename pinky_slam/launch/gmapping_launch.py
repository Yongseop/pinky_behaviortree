# ~/ros2_ws/src/pinky_slam/launch/gmapping_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 공유 디렉토리 가져오기
    pinky_slam_share = get_package_share_directory('pinky_slam')

    return LaunchDescription([
        # SLAM 모드 인자 선언
        DeclareLaunchArgument(
            'slam_mode',
            default_value='mapping',
            description='Mode for SLAM: mapping or localization'
        ),

        # SLAM 노드 실행
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_mode'),
                PathJoinSubstitution([
                    pinky_slam_share,
                    'config',
                    'slam_toolbox_params.yaml'
                ])
            ],
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
            arguments=['-d', PathJoinSubstitution([
                pinky_slam_share,
                'rviz',
                'gmapping.rviz'
            ])],
        )
    ])