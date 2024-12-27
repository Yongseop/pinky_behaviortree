import rclpy
import py_trees_ros
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.decorators import Inverter
from py_trees.decorators import Decorator
from py_trees.composites import Selector
from py_trees import logging as log_tree
import py_trees
import py_trees.decorators
import time
from geometry_msgs.msg import Twist, PoseStamped
import math

from .bt_rotate_robot import RotateRobot
from .bt_stop_robot import StopRobot
from .bt_detect_marker import DetectMarker
from .bt_aruco_localization import ArucoLocalization
from .bt_navigate_to_goal import NavigateToGoal, NavigationMonitor
from .bt_navigation_manager import NavigationManager

class Counter(py_trees.decorators.Decorator):
    def __init__(self, child, name="Counter", max_count=10):
        super().__init__(name=name, child=child)
        self.max_count = max_count
        self.count = 0
        self.node = None
        self.navigation_cancelled = False
        self.cmd_vel_publisher = None
        self.blackboard = self.attach_blackboard_client(name="Counter")
        self.blackboard.register_key(key="action_client", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="goal_future", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get('node')
            if not self.node:
                self.node = rclpy.create_node('counter_node')

            # cmd_vel publisher 추가
            self.cmd_vel_publisher = self.node.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )
            return True
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Counter setup failed: {str(e)}')
            return False

    def stop_robot(self):
        """로봇을 강제로 정지"""
        stop_cmd = Twist()
        for _ in range(10):  # 여러번 publish
            self.cmd_vel_publisher.publish(stop_cmd)
            time.sleep(0.1)

    def update(self):
        # Navigation 취소 및 로봇 정지
        if not self.navigation_cancelled:
            # 1. goal 취소
            if self.blackboard.exists('goal_future'):
                goal_future = self.blackboard.goal_future
                if goal_future and not goal_future.done():
                    self.node.get_logger().info("Canceling current navigation goal")
                    goal_future.cancel()
                    # Clear the goal_future from blackboard
                    self.blackboard.goal_future = None
            
            # 2. 로봇 강제 정지
            self.stop_robot()
            
            # 3. navigation2가 완전히 취소되길 기다림
            time.sleep(1.0)
            
            self.navigation_cancelled = True

        if self.count >= self.max_count:
            return py_trees.common.Status.FAILURE

        status = self.decorated.status
        if status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info("Marker detection succeeded, resetting navigation state")
            # Navigation 상태 초기화
            self.navigation_cancelled = False
            if hasattr(self.blackboard, 'goal_future'):
                self.blackboard.goal_future = None
            return py_trees.common.Status.SUCCESS
            
        elif status == py_trees.common.Status.FAILURE:
            self.count += 1
            if self.count >= self.max_count:
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
            
        return py_trees.common.Status.RUNNING

    def initialise(self):
        """Reset counter state when starting"""
        self.count = 0
        self.navigation_cancelled = False
        if hasattr(self.blackboard, 'goal_future'):
            self.blackboard.goal_future = None

def create_pose(x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = math.cos(yaw/2)
    goal.pose.orientation.z = math.sin(yaw/2)
    return goal

def make_bt():
    root = Selector(name="Root", memory=True)

    # Points 정의
    nav_points = [
        create_pose(3.6, 1.4, 0.0),   # Point 1
        create_pose(1.5, 1.0, 1.57),  # Point 2
        create_pose(1.6, -0.3, 3.14)   # Point 3
    ]

    # Navigation sequence
    nav_sequence = Sequence(
        name="NavigationSequence",
        memory=True,
        children=[
            NavigationManager(name="NavigationManager", points=nav_points)
        ]
    )
    root.add_child(nav_sequence)

    # Recovery sequence
    rotate_detect_sequence = Sequence(
        name="RotateDetectSequence",
        memory=True,
        children=[
            Sequence(  # 회전 및 정지를 위한 하위 시퀀스
                name="RotateStopSequence",
                memory=True,
                children=[
                    RotateRobot(rotation_time=3.0),  # 3초 회전
                    StopRobot(stop_time=1.0)         # 1초 정지
                ]
            ),
            Sequence(  # 마커 감지 및 위치 보정을 위한 하위 시퀀스
                name="DetectLocalizeSequence",
                memory=True,
                children=[
                    DetectMarker(detection_time=2.0),  # 2초 동안 마커 감지
                    ArucoLocalization()
                ]
            )
        ]
    )
    rotation_counter = Counter(child=rotate_detect_sequence, max_count=10)
    root.add_child(rotation_counter)

    return root

def main():
    rclpy.init(args=None)
    log_tree.level = log_tree.Level.INFO

    root = make_bt()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    tree.setup(timeout=15)
    tree.tick_tock(period_ms=100.0)

    node = tree.node

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트 발생. 종료합니다.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
