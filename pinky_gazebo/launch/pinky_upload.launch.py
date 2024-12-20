from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Paths
    pinky_description_path = get_package_share_directory('pinky_description')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('pinky_description'),
                        'urdf',
                        'robot.urdf.xacro',
                    ]),
                ]),
        }]
    )

    # Spawn robot node
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'pinky',
            '-topic', 'robot_description',
            '-timeout', '20.0',
            '-x', '-1.1',
            '-y', '2.0',
            '-Y', '-1.5708',
            '-package_to_model'
        ],
        prefix="bash -c 'sleep 2.0; $0 $@' ",
        parameters=[{'use_sim_time': True}]
    )
    
    # 컨트롤러 로드 명령어 정의
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_cont'],
        output='screen'
    )


    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_base_controller],
            )
        ),
        robot_state_publisher,       # Robot State Publisher
        spawn_robot,                 # Spawn robot
    ])