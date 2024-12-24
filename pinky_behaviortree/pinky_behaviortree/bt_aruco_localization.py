import py_trees
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import numpy as np
import math

class ArucoLocalization(py_trees.behaviour.Behaviour):
    def __init__(self, name="ArucoLocalization", yaml_path='marker_positions.yaml'):
        super().__init__(name)
        self.node = None
        self.initialpose_publisher = None
        self.yaml_path = yaml_path
        self.marker_positions = None
        self.detected_marker_id = None
        self.marker_tvec = None
        self.robot_pose = None
        # Blackboard 클라이언트 설정
        self.blackboard = self.attach_blackboard_client(name="ArucoLocalization")
        self.blackboard.register_key(key="detected_marker_id", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="marker_tvec", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="current_robot_pose", access=py_trees.common.Access.READ)
        
    def setup(self, **kwargs):
        try:
            self.node = kwargs.get('node')
            if not self.node:
                self.node = rclpy.create_node('aruco_localization_node')
            
            # Initial pose publisher
            self.initialpose_publisher = self.node.create_publisher(
                PoseWithCovarianceStamped,
                '/initialpose',
                10
            )
            
            # Load marker positions from yaml
            self.load_marker_positions()
            return True
        except Exception as e:
            self.node.get_logger().error(f'Setup failed: {str(e)}')
            return False
        
    def load_marker_positions(self):
        try:
            yaml_path = 'marker_positions.yaml'  # 절대 경로나 정확한 상대 경로가 필요할 수 있습니다
            with open(yaml_path, 'r') as file:
                self.marker_positions = yaml.safe_load(file)
                self.node.get_logger().info(f'Loaded marker positions: {self.marker_positions}')
                if self.marker_positions is None:
                    raise ValueError("Failed to load marker positions from YAML")
        except Exception as e:
            self.node.get_logger().error(f'Failed to load marker positions: {str(e)}')
            self.marker_positions = {}  # 기본 빈 딕셔너리로 초기화
            
    def update(self):
        self.detected_marker_id = self.blackboard.detected_marker_id
        self.marker_tvec = self.blackboard.marker_tvec
        
        try:
            marker_key = f'marker_{self.detected_marker_id}'
            marker_global_pos = self.marker_positions[marker_key]['position']
            
            # 1. 카메라 좌표계에서 로봇 좌표계로 변환
            marker_robot_x = self.marker_tvec[2]     # 카메라의 z축이 로봇의 x축
            marker_robot_y = -self.marker_tvec[0]    # 카메라의 -x축이 로봇의 y축
            
            # 마커의 상대 벡터 크기
            marker_size = self.marker_positions[marker_key]['marker_size']
            scaling_factor = marker_size / 0.5  # 마커의 실제 크기의 절반으로 스케일링
            
            # 마커 벡터의 크기와 각도
            distance = math.sqrt(marker_robot_x**2 + marker_robot_y**2) * scaling_factor
            
            # 마커를 바라보는 로봇의 방향 (로봇 프레임에서)
            local_yaw = math.atan2(marker_robot_y, marker_robot_x)
            
            # 마커의 글로벌 위치와 로봇의 로컬 관측으로부터 로봇의 글로벌 위치/방향 계산
            robot_x = marker_global_pos[0] - distance * math.cos(local_yaw)
            robot_y = marker_global_pos[1] - distance * math.sin(local_yaw)
            robot_yaw = local_yaw  # 마커를 바라보는 방향이 로봇의 전방

            self.node.get_logger().info(f"""
                [DEBUG INFO]
                Marker ID: {self.detected_marker_id}
                Global marker position: ({marker_global_pos[0]:.3f}, {marker_global_pos[1]:.3f})
                Local marker vector (x, y): ({marker_robot_x:.3f}, {marker_robot_y:.3f})
                Scaled distance: {distance:.3f}
                Local yaw (deg): {math.degrees(local_yaw):.2f}
                Robot calculated position: ({robot_x:.3f}, {robot_y:.3f})
                Robot yaw (deg): {math.degrees(robot_yaw):.2f}
                
                Raw tvec data: {self.marker_tvec}
            """)

            # Create and publish initialpose
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.node.get_clock().now().to_msg()
            
            initial_pose.pose.pose.position.x = robot_x
            initial_pose.pose.pose.position.y = robot_y
            initial_pose.pose.pose.position.z = 0.0
            
            initial_pose.pose.pose.orientation.z = math.sin(robot_yaw/2)
            initial_pose.pose.pose.orientation.w = math.cos(robot_yaw/2)
            
            # ... 나머지 코드는 동일 ...
            
            # Set covariance
            initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            
            # Publish
            self.initialpose_publisher.publish(initial_pose)
            self.node.get_logger().info('Published new initial pose based on marker detection')
            
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.node.get_logger().error(f'Failed to calculate new pose: {str(e)}')
            return py_trees.common.Status.FAILURE
            
    def terminate(self, new_status):
        pass