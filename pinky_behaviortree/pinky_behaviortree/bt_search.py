import py_trees
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np

class SearchBehaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name="SearchBehaviour"):
        super().__init__(name)
        self.node = None
        self.cmd_vel_publisher = None
        self.image_subscription = None
        
        self.rotation_count = 0
        self.max_rotations = 10
        
        # 상태 관리
        self.current_phase = 0  # 0: rotate, 1: stop, 2: detect
        self.phase_start_time = None
        
        # Marker detection 관련 설정
        self.bridge = CvBridge()
        self.marker_detected = False
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
        
        # ArUco 설정
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        
    def setup(self, **kwargs):
        try:
            self.node = kwargs.get('node')
            if not self.node:
                self.node = rclpy.create_node('search_behaviour_node')
            
            # cmd_vel publisher
            self.cmd_vel_publisher = self.node.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )
            
            # camera subscriber
            self.image_subscription = self.node.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10
            )
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Setup failed: {str(e)}')
            return False
    
    def image_callback(self, msg):
        try:
            if self.current_phase != 2:  # detect 단계가 아니면 처리하지 않음
                return
                
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None and len(ids) > 0:
                self.marker_detected = True
                self.node.get_logger().info('Marker detected!')
            
        except Exception as e:
            self.node.get_logger().error(f'Error in image_callback: {str(e)}')
    
    def update(self):
        if self.rotation_count >= self.max_rotations and not self.marker_detected:
            self.node.get_logger().warning('Failed: Maximum rotation attempts reached')
            return py_trees.common.Status.FAILURE

        if not self.phase_start_time:
            self.phase_start_time = self.node.get_clock().now()
        
        current_time = self.node.get_clock().now()
        elapsed_time = (current_time - self.phase_start_time).nanoseconds / 1e9
        
        # Rotate phase
        if self.current_phase == 0:
            if elapsed_time < 3.0:  # 3초 동안 회전
                twist = Twist()
                twist.angular.z = 0.5
                self.cmd_vel_publisher.publish(twist)
                self.node.get_logger().info(f'Rotating... (attempt {self.rotation_count + 1}/{self.max_rotations})')
                return py_trees.common.Status.RUNNING
            else:
                self.current_phase = 1
                self.phase_start_time = None
                return py_trees.common.Status.RUNNING
        
        # Stop phase
        elif self.current_phase == 1:
            if elapsed_time < 1.0:  # 1초 동안 정지
                twist = Twist()
                self.cmd_vel_publisher.publish(twist)
                self.node.get_logger().info('Stopping...')
                return py_trees.common.Status.RUNNING
            else:
                self.current_phase = 2
                self.phase_start_time = None
                self.marker_detected = False
                return py_trees.common.Status.RUNNING
        
        # Detect phase
        else:  # current_phase == 2
            if elapsed_time < 2.0:  # 2초 동안 마커 탐지 시도
                if self.marker_detected:
                    self.node.get_logger().info('Success: Marker was detected')
                    # 여기에 마커 위치 보정 코드 추가 예정
                    return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.RUNNING
            else:
                self.current_phase = 0
                self.phase_start_time = None
                self.rotation_count += 1  # 회전 시도 횟수 증가
                return py_trees.common.Status.RUNNING

    def initialise(self):
        self.current_phase = 0
        self.phase_start_time = None
        self.marker_detected = False
        self.rotation_count = 0
    
    def terminate(self, new_status):
        # 종료 시 로봇 정지
        if self.cmd_vel_publisher:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)