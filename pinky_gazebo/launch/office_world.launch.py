from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    TimerAction,
    ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from os import environ

def generate_launch_description():
    # 환경 변수 설정
    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    
    # Launch 인자 선언
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='office_world.world',
        description='Name of the Gazebo world file to load'
    )

    # 패키지 경로 가져오기
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    pinky_gazebo_path = get_package_share_directory('pinky_gazebo')

    # 월드 파일 경로 설정
    world_file_path = PathJoinSubstitution([
        pinky_gazebo_path, 'world', LaunchConfiguration('world_name')
    ])

    # GAZEBO_MODEL_PATH 환경 변수 설정
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([pinky_gazebo_path, 'model'])
    )

    # Gazebo 서버 포함
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_path, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={
            'verbose': 'true',
            'physics': 'ode',
            'lockstep': 'false',
            'world': world_file_path
        }.items()
    )

    # Gazebo 클라이언트 포함
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_path, 'launch', 'gzclient.launch.py'])
        )
    )
    
    # pinky_upload.launch.py 포함
    pinky_upload_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pinky_gazebo_path, 'launch', 'pinky_upload.launch.py'
            ])
        )
    )
    
   
    
    # pinky_upload_launch 및 컨트롤러 로드 명령어를 지연 실행 (예: 5초 후)
    delayed_actions = TimerAction(
        period=2.0,  # 지연 시간 (초)
        actions=[
            pinky_upload_launch
        ]
    )
    return LaunchDescription([
        set_gazebo_model_path,  # GAZEBO_MODEL_PATH 설정
        world_name_arg,         # 월드 이름 인자
        gzserver,               # Gazebo 서버 실행
        gzclient,               # Gazebo 클라이언트 실행
        delayed_actions         # pinky_upload 및 컨트롤러 로드
    ])