import py_trees
import rclpy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

class DetectMarker(py_trees.behaviour.Behaviour):
    def __init__(self, name="DetectMarker", detection_time=2.0):
        super().__init__(name)
        self.node = None
        self.bridge = CvBridge()
        self.marker_detected = False
        self.start_time = None
        self.image_subscription = None
        self.detection_time = detection_time
        self.detected_marker_id = None
        self.marker_tvec = None
        self.marker_rvec = None
        self.current_robot_pose = None
        self.blackboard = self.attach_blackboard_client(name="DetectMarker")
        self.blackboard.register_key(key="detected_marker_id", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="marker_tvec", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="marker_rvec", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="current_robot_pose", access=py_trees.common.Access.WRITE)
        
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
                self.node = rclpy.create_node('detect_marker_node')
            
            self.image_subscription = self.node.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10
            )

            self.pose_subscription = self.node.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.pose_callback,
                10
            )
            return True
        except Exception as e:
            self.node.get_logger().error(f'Setup failed: {str(e)}')
            return False

    def pose_callback(self, msg):
        self.current_robot_pose = msg.pose.pose
            
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            if ids is not None and len(ids) > 0:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.5, self.camera_matrix, self.dist_coeffs)
                self.marker_detected = True
                self.detected_marker_id = ids[0][0]
                self.marker_tvec = tvecs[0][0]
                self.marker_rvec = rvecs[0][0]
                self.node.get_logger().info(f'Marker {self.detected_marker_id} detected!')
            
        except Exception as e:
            self.node.get_logger().error(f'Error in image_callback: {str(e)}')
            
    def update(self):
        if not self.start_time:
            self.start_time = self.node.get_clock().now()
            self.marker_detected = False

        current_time = self.node.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time < self.detection_time:
            if self.marker_detected:
                self.blackboard.detected_marker_id = self.detected_marker_id
                self.blackboard.marker_tvec = self.marker_tvec
                self.blackboard.marker_rvec = self.marker_rvec
                self.blackboard.current_robot_pose = self.current_robot_pose
                self.node.get_logger().info('Success: Marker was detected')
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info('Fail: Marker detection timed out')
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.FAILURE:
            # 실패 시 blackboard 데이터 초기화
            self.blackboard.detected_marker_id = None
            self.blackboard.marker_tvec = None
            self.blackboard.current_robot_pose = None
        self.marker_detected = False
        self.start_time = None