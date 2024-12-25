import py_trees
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
import time

class NavigateToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name="NavigateToGoal", goal_pose=None):
        super().__init__(name)
        self.node = None
        self.action_client = None
        self.cmd_vel_publisher = None  
        self.goal_pose = goal_pose if goal_pose else self._default_goal()
        self.send_goal_future = None
        self.get_result_future = None
        self.feedback = None
        self.subscription = None
        self.latest_pose = None
        
        self.check_start_time = None
        self.check_delay = 5.0 
        self.check_enabled = False

        self.blackboard = self.attach_blackboard_client(name="NavigateToGoal")
        self.blackboard.register_key(key="action_client", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_future", access=py_trees.common.Access.WRITE)

        self.thresholds = {
            'x_uncertainty': 0.06,
            'y_uncertainty': 0.06,
            'theta_uncertainty': 0.125
        }

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get('node')
            if not self.node:
                self.node = rclpy.create_node('navigate_to_goal_node')
            
            self.action_client = ActionClient(
                self.node,
                NavigateToPose,
                'navigate_to_pose'
            )
            
            self.cmd_vel_publisher = self.node.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )
            
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            
            self.subscription = self.node.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.amcl_pose_callback,
                qos
            )
            
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.node.get_logger().error('Navigation action server not available')
                return False
                
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Setup failed: {str(e)}')
            return False

    def initialise(self):
        super().initialise()
        self.check_start_time = time.time()
        self.check_enabled = False
        self.node.get_logger().info("[NavigateToGoal] 시작, uncertainty check는 3초 후에 활성화됩니다")

    def reset_navigation_state(self):
        self.send_goal_future = None
        self.get_result_future = None
        self.feedback = None

    def _default_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 3.6
        goal.pose.position.y = 1.4
        goal.pose.orientation.w = 1.0
        goal.pose.orientation.z = 0.0
        return goal

    def amcl_pose_callback(self, msg):
        self.latest_pose = msg

    def check_localization(self):
        if self.latest_pose is None:
            return False
        covariance = self.latest_pose.pose.covariance
        is_lost = (
            covariance[0] > self.thresholds['x_uncertainty'] or
            covariance[7] > self.thresholds['y_uncertainty'] or
            covariance[35] > self.thresholds['theta_uncertainty']
        )
        return is_lost

    def stop_robot(self):
        try:
            stop_cmd = Twist()
            for _ in range(5):
                self.cmd_vel_publisher.publish(stop_cmd)
                time.sleep(0.1)
            self.node.get_logger().info("Robot stop command sent")
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Failed to stop robot: {str(e)}")

    def update(self):
        """메인 업데이트 로직"""
        if not self.check_enabled:
            elapsed = time.time() - self.check_start_time
            if elapsed >= self.check_delay:
                self.check_enabled = True
                self.node.get_logger().info("[NavigateToGoal] Uncertainty check 활성화됨")

        # Navigation 로직
        if not self.send_goal_future:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.goal_pose
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            
            self.send_goal_future = self.action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
            self.blackboard.goal_future = self.send_goal_future
            self.node.get_logger().info("[NavigateToGoal] New goal 전송")
            return py_trees.common.Status.RUNNING

        if self.send_goal_future.done():
            goal_handle = self.send_goal_future.result()
            if not goal_handle.accepted:
                self.node.get_logger().error('Goal rejected')
                self.reset_navigation_state()
                return py_trees.common.Status.FAILURE

            if not self.get_result_future:
                self.get_result_future = goal_handle.get_result_async()
                return py_trees.common.Status.RUNNING

            if self.get_result_future.done():
                try:
                    self.get_result_future.result()
                    self.node.get_logger().info('Navigation succeeded')
                    return py_trees.common.Status.SUCCESS
                except Exception as e:
                    self.node.get_logger().error(f'Navigation failed: {str(e)}')
                    self.reset_navigation_state()
                    return py_trees.common.Status.FAILURE

        # Uncertainty 체크 (check_enabled일 때만)
        if self.check_enabled and self.check_localization():
            self.node.get_logger().warn("High localization uncertainty detected")
            if self.send_goal_future and self.send_goal_future.done():
                goal_handle = self.send_goal_future.result()
                if goal_handle and goal_handle.accepted:
                    goal_handle.cancel_goal_async()
            self.stop_robot()
            self.reset_navigation_state()
            return py_trees.common.Status.FAILURE
                    
        return py_trees.common.Status.RUNNING

    def feedback_callback(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def terminate(self, new_status):
        """Behaviour가 종료될 때 호출되는 콜백"""
        if new_status == py_trees.common.Status.FAILURE:
            self.stop_robot()
        
        if self.action_client and self.send_goal_future:
            if not self.send_goal_future.done():
                self.send_goal_future.cancel()
        
        self.reset_navigation_state()

class NavigationMonitor(py_trees.behaviour.Behaviour):
    def __init__(self, name="NavigationMonitor"):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.node = None
        
    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if not self.node:
            self.node = rclpy.create_node('navigation_monitor_node')
        return True
        
    def update(self):
        if hasattr(self.blackboard, 'navigation_failed') and self.blackboard.navigation_failed:
            self.node.get_logger().warn("Navigation failure detected")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS