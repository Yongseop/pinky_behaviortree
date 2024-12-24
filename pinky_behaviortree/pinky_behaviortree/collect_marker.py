import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import numpy as np
import yaml
import signal
import sys

class MarkerLocalization(Node):
    def __init__(self):
        super().__init__('marker_localization')
        
        # 카메라 파라미터 설정
        WIDTH = 640
        HEIGHT = 360
        FOV = 1.08
        fx = WIDTH / (2 * np.tan(FOV / 2))
        self.camera_matrix = np.array([
            [fx, 0, WIDTH/2],
            [0, fx, HEIGHT/2],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros(5, dtype=np.float32)
        
        self.bridge = CvBridge()
        self.marker_positions = {}
        self.current_robot_pose = None
        
        # YAML 파일 경로 설정
        self.yaml_path = 'marker_positions.yaml'
        
        # 구독자 설정
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        # ArUco 설정
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('Marker Localization Node has been initialized')
        
    def pose_callback(self, msg):
        self.current_robot_pose = msg
        
    def signal_handler(self, sig, frame):
        self.get_logger().info('Shutting down...')
        cv2.destroyAllWindows()
        self.save_to_yaml()
        rclpy.shutdown()
        sys.exit(0)
        
    def save_to_yaml(self):
        try:
            yaml_data = {}
            for marker_id, data in self.marker_positions.items():
                yaml_data[f'marker_{marker_id}'] = {
                    'position': data['position'].tolist(),
                    'marker_size': 0.5
                }
            
            with open(self.yaml_path, 'w') as file:
                yaml.dump(yaml_data, file, default_flow_style=False)
                self.get_logger().info(f'Saved {len(yaml_data)} markers to {self.yaml_path}')
        except Exception as e:
            self.get_logger().error(f'Error saving YAML: {str(e)}')
            
    def image_callback(self, msg):
        try:
            if self.current_robot_pose is None:
                return
                
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.5, self.camera_matrix, self.dist_coeffs)
                cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                self.current_detection = {}
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    cv_image = cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, 
                                            rvecs[i], tvecs[i], 0.1)
                    
                    # 로봇의 현재 위치와 방향
                    robot_pos = self.current_robot_pose.pose.pose.position
                    robot_quat = self.current_robot_pose.pose.pose.orientation
                    
                    # yaw 각도 계산
                    yaw = np.arctan2(2.0 * (robot_quat.w * robot_quat.z + robot_quat.x * robot_quat.y),
                                    1.0 - 2.0 * (robot_quat.y * robot_quat.y + robot_quat.z * robot_quat.z))
                    
                    # 마커의 위치를 map 좌표계로 변환
                    marker_robot_x = tvecs[i][0][2]  # 카메라 z축이 로봇의 x축
                    marker_robot_y = -tvecs[i][0][0]  # 카메라 -x축이 로봇의 y축
                    
                    # 로봇 좌표를 map 좌표로 변환
                    marker_map_x = robot_pos.x + (marker_robot_x * np.cos(yaw) - marker_robot_y * np.sin(yaw))
                    marker_map_y = robot_pos.y + (marker_robot_x * np.sin(yaw) + marker_robot_y * np.cos(yaw))
                    marker_map_z = tvecs[i][0][1]  # z 좌표는 그대로 유지
                    
                    map_position = np.array([marker_map_x, marker_map_y, marker_map_z])
                    
                    self.current_detection[marker_id] = {
                        'position': map_position
                    }
                    
                   #  self.get_logger().info(f'Marker {marker_id} at map position: {map_position}')
            
            # 키 입력 처리
            key = cv2.waitKey(1)
            if key == ord('c') and hasattr(self, 'current_detection'):
                self.marker_positions.update(self.current_detection)
                self.save_to_yaml()
                self.get_logger().info('Captured and saved marker positions')
            elif key == ord('q'):
                self.signal_handler(None, None)
            
            cv2.imshow("Camera View", cv_image)
                
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MarkerLocalization()
    rclpy.spin(node)

if __name__ == '__main__':
    main()