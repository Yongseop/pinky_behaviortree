import py_trees
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import numpy as np
import math
from ament_index_python.packages import get_package_share_directory
import os
import cv2

class ArucoLocalization(py_trees.behaviour.Behaviour):
    def __init__(self, name="ArucoLocalization", yaml_path='marker_positions.yaml'):
        super().__init__(name)
        self.node = None
        self.initialpose_publisher = None
        self.yaml_path = yaml_path
        self.marker_positions = None
        self.detected_marker_id = None
        self.marker_tvec = None
        self.marker_rvec = None
        self.robot_pose = None
        # Blackboard 클라이언트 설정
        self.blackboard = self.attach_blackboard_client(name="ArucoLocalization")
        self.blackboard.register_key(key="detected_marker_id", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="marker_tvec", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="marker_rvec", access=py_trees.common.Access.READ)  # 추가
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
            pkg_share_path = get_package_share_directory('pinky_behaviortree')
            yaml_path = os.path.join(pkg_share_path, 'marker_positions.yaml')
            with open(yaml_path, 'r') as file:
                self.marker_positions = yaml.safe_load(file)
                # 디버그 로그 추가
                self.node.get_logger().info(f'Loaded markers: {list(self.marker_positions.keys())}')
                if self.marker_positions is None:
                    raise ValueError("Failed to load marker positions from YAML")
        except Exception as e:
            self.node.get_logger().error(f'Failed to load marker positions: {str(e)}')
            self.marker_positions = {}
            
    def update(self):
        self.detected_marker_id = self.blackboard.detected_marker_id
        self.marker_tvec = self.blackboard.marker_tvec
        self.marker_rvec = self.blackboard.marker_rvec 
        try:
            marker_key = f'marker_{self.detected_marker_id}'
            marker_global_pos = self.marker_positions[marker_key]['position']
            marker_global_quat = self.marker_positions[marker_key]['orientation']
            
            # OpenCV -> ROS 좌표계 변환 행렬
            cv_to_ros = np.array([
                [0, 0, 1],
                [-1, 0, 0],
                [0, -1, 0]
            ])
            
            # 1. ArUco에서 얻은 마커의 pose를 ROS 좌표계로 변환
            marker_rot_mat, _ = cv2.Rodrigues(self.marker_rvec)
            marker_rot_ros = cv_to_ros @ marker_rot_mat
            
            marker_pos_ros = cv_to_ros @ self.marker_tvec
            
            marker_transform = np.eye(4)
            marker_transform[:3, :3] = marker_rot_ros
            marker_transform[:3, 3] = marker_pos_ros
            
            # 2. 마커의 글로벌 pose 변환은 그대로 유지
            global_rot_mat = self.quaternion_to_rotation_matrix(marker_global_quat)
            global_transform = np.eye(4)
            global_transform[:3, :3] = global_rot_mat
            global_transform[:3, 3] = marker_global_pos
            
            # 3. 로봇의 pose 계산
            robot_transform = global_transform @ np.linalg.inv(marker_transform)
            
            # 4. 결과에서 위치와 방향 추출
            robot_pos = robot_transform[:3, 3]
            robot_rot_mat = robot_transform[:3, :3]
            robot_quat = self.rotation_matrix_to_quaternion(robot_rot_mat)

            # 로그 출력
            self.node.get_logger().info(f"""
                [DEBUG INFO]
                Marker ID: {self.detected_marker_id}
                Global marker position: {marker_global_pos}
                Global marker orientation: {marker_global_quat}
                Calculated robot position: {robot_pos}
                Calculated robot orientation: {robot_quat}
            """)

            # Initial pose 메시지 생성 및 발행
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.node.get_clock().now().to_msg()
            
            initial_pose.pose.pose.position.x = robot_pos[0]
            initial_pose.pose.pose.position.y = robot_pos[1]
            initial_pose.pose.pose.position.z = 0.0
            
            initial_pose.pose.pose.orientation.w = robot_quat[0]
            initial_pose.pose.pose.orientation.x = robot_quat[1]
            initial_pose.pose.pose.orientation.y = robot_quat[2]
            initial_pose.pose.pose.orientation.z = robot_quat[3]
            
            # Covariance 설정
            initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
            
            self.initialpose_publisher.publish(initial_pose)
            self.node.get_logger().info('Published new initial pose based on marker detection')
            
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.node.get_logger().error(f'Failed to calculate new pose: {str(e)}')
            return py_trees.common.Status.FAILURE

    def quaternion_to_rotation_matrix(self, quaternion):
        w, x, y, z = quaternion
        
        rot_matrix = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        
        return rot_matrix

    def rotation_matrix_to_quaternion(self, R):
        trace = np.trace(R)
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
            
        return np.array([w, x, y, z])
            
    def terminate(self, new_status):
        pass