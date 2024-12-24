import py_trees
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

class CheckLocalizationStatus(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckLocalizationStatus"):
        super().__init__(name)
        self.node = None
        self.subscription = None
        self.cmd_vel_publisher = None
        self.latest_pose = None
        self.blackboard = py_trees.blackboard.Blackboard()
        self.start_time = None
        self.toggle_direction = True
        self.tick_counter = 0  # tick 카운터 추가
        self.ticks_per_toggle = 10  # 토글 전환에 필요한 tick 수 설정
        self.is_terminated = False
        
        self.thresholds = {
            'x_uncertainty': 0.012,
            'y_uncertainty': 0.012,
            'theta_uncertainty': 0.025
        }
        
    def setup(self, **kwargs):
        try:
            self.node = kwargs.get('node')
            if not self.node:
                self.node = rclpy.create_node('localization_check_node')

            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            
            # AMCL pose subscriber
            self.subscription = self.node.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.amcl_pose_callback,
                qos
            )
            
            # cmd_vel publisher
            self.cmd_vel_publisher = self.node.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )
            
            self.start_time = self.node.get_clock().now()
            return True
        except Exception as e:
            self.node.get_logger().error(f'Setup failed: {str(e)}')
            return False

    def amcl_pose_callback(self, msg):
        self.latest_pose = msg
        covariance = msg.pose.covariance
        
        self.blackboard.x_uncertainty = covariance[0]
        self.blackboard.y_uncertainty = covariance[7]
        self.blackboard.theta_uncertainty = covariance[35]

    def move_robot(self):
        """로봇을 앞뒤로 움직이는 함수"""
        twist = Twist()

        # 설정된 tick 주기마다 방향 전환
        if self.tick_counter % self.ticks_per_toggle == 0:
            self.toggle_direction = not self.toggle_direction

        if self.toggle_direction:
            twist.linear.x = 1.0
            twist.angular.z = -0.5
        else:
            twist.linear.x = -1.0
            twist.angular.z = 0.5

        self.cmd_vel_publisher.publish(twist)
        self.tick_counter += 1  # tick 카운터 증가
        self.node.get_logger().info(f"Tick: {self.tick_counter}, Direction: {'Forward' if self.toggle_direction else 'Backward'}")

    def stop_robot(self):
        """로봇을 정지시키는 함수"""
        if self.cmd_vel_publisher and self.cmd_vel_publisher.handle:
            try:
                twist = Twist()
                self.cmd_vel_publisher.publish(twist)
            except Exception as e:
                self.node.get_logger().error(f"Failed to publish stop command: {e}")
        else:
            self.node.get_logger().warning("cmd_vel_publisher is invalid. Cannot stop robot.")

    def update(self):
        """비헤이비어 트리 업데이트 함수"""
        current_time = self.node.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        # 10초 동안 로봇 움직이기
        if elapsed_time < 10.0:
            self.move_robot()
            self.node.get_logger().info(f'Moving robot: {elapsed_time:.1f} seconds elapsed')
            return py_trees.common.Status.RUNNING
        else:
            # 로봇 정지 시도
            if self.cmd_vel_publisher and self.cmd_vel_publisher.handle:
                self.stop_robot()
            else:
                self.node.get_logger().warning("cmd_vel_publisher is invalid. Cannot stop robot.")
            
            # Localization 상태 평가
            if self.latest_pose is None:
                self.node.get_logger().info("No pose received. Localization failed.")
                return py_trees.common.Status.FAILURE

            is_lost = (
                self.blackboard.x_uncertainty > self.thresholds['x_uncertainty']
                or self.blackboard.y_uncertainty > self.thresholds['y_uncertainty']
                or self.blackboard.theta_uncertainty > self.thresholds['theta_uncertainty']
            )

            if is_lost:
                self.node.get_logger().warning("Localization failed due to high uncertainty.")
                return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().info("Localization succeeded.")
                return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """종료 시 정리"""
        if self.is_terminated:  # 이미 종료된 경우 추가 호출 방지
            return
        self.is_terminated = True  # 종료 플래그 설정

        if self.cmd_vel_publisher and self.cmd_vel_publisher.handle:
            try:
                self.stop_robot()
            except Exception as e:
                self.node.get_logger().warning(f"Error stopping robot: {e}")
        else:
            self.node.get_logger().warning("cmd_vel_publisher already destroyed or invalid.")

        # Subscription 및 Publisher 제거
        if self.subscription:
            try:
                self.node.destroy_subscription(self.subscription)
            except Exception as e:
                self.node.get_logger().error(f"Error destroying subscription: {e}")

        if self.cmd_vel_publisher and self.cmd_vel_publisher.handle:
            try:
                self.node.destroy_publisher(self.cmd_vel_publisher)
            except Exception as e:
                self.node.get_logger().error(f"Error destroying publisher: {e}")

        # 상태 초기화
        self.latest_pose = None

        # 상태에 따른 로그 출력
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info("Localization succeeded. Behavior terminated successfully.")
        elif new_status == py_trees.common.Status.FAILURE:
            self.node.get_logger().info("Localization failed. Behavior terminated with failure.")
