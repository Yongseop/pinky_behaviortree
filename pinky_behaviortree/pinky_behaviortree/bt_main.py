import rclpy
import py_trees_ros
import py_trees
import py_trees.decorators
import time

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.decorators import Inverter
from py_trees.decorators import Decorator
from py_trees.composites import Selector
from py_trees.decorators import Repeat 
from py_trees import logging as log_tree

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from .bt_rotate_robot import RotateRobot
from .bt_stop_robot import StopRobot
from .bt_detect_marker import DetectMarker
from .bt_check_localization import CheckLocalizationStatus
from .bt_aruco_localization import ArucoLocalization
from .bt_navigate_to_goal import NavigateToGoal, NavigationMonitor

class Counter(py_trees.decorators.Decorator):
    def __init__(self, child, name="Counter", max_count=10):
        super().__init__(name=name, child=child)
        self.max_count = max_count
        self.count = 0
        self.node = None
        self.navigation_cancelled = False
        self.cmd_vel_publisher = None
        self.blackboard = self.attach_blackboard_client(name="Counter")
        self.blackboard.register_key(key="action_client", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_future", access=py_trees.common.Access.READ)

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
            
            # 2. 로봇 강제 정지
            self.stop_robot()
            
            # 3. navigation2가 완전히 취소되길 기다림
            time.sleep(1.0)
            
            self.navigation_cancelled = True

        if self.count >= self.max_count:
            return py_trees.common.Status.FAILURE

        self.decorated.tick()
        status = self.decorated.status

        if status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info("Marker detection succeeded, returning to navigation")
            return py_trees.common.Status.SUCCESS
            
        elif status == py_trees.common.Status.FAILURE:
            self.count += 1
            if self.count >= self.max_count:
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
            
        else:  # RUNNING
            return py_trees.common.Status.RUNNING

    def initialise(self):
        self.count = 0
        self.navigation_cancelled = False


def create_recovery_subtree() -> py_trees.behaviour.Behaviour:
    """
    회전 + 스탑 + 마커탐지 + 위치보정 시퀀스
    Counter 데코레이터를 씌워서, 마커 검출을 max_count번 시도
    """
    # 1) 회전 + 정지
    rotate_stop_seq = Sequence(
        name="RotateStopSequence",
        memory=True,
        children=[
            RotateRobot(rotation_time=3.0),
            StopRobot(stop_time=1.0)
        ]
    )
    # 2) 마커 감지 + 위치 보정
    detect_localize_seq = Sequence(
        name="DetectLocalizeSequence",
        memory=True,
        children=[
            DetectMarker(detection_time=2.0),
            ArucoLocalization()
        ]
    )
    # 묶은 시퀀스(회전+정지→마커보정)
    rotate_detect_seq = Sequence(
        name="RotateDetectSequence",
        memory=True,
        children=[
            rotate_stop_seq,
            detect_localize_seq
        ]
    )
    # Counter로 감싼 뒤 반환
    return Counter(child=rotate_detect_seq, max_count=10)


def create_navigate_or_recover(
    name: str,
    goal_pose: PoseStamped
) -> py_trees.behaviour.Behaviour:
    """
    Fallback(Selector) 구성:
    1) NavigateToGoal(특정 웨이포인트)
    2) Recovery Subtree (Counter + RotateDetectSequence)
    """
    fallback = Selector(name=name, memory=False)

    nav_to_goal = NavigateToGoal(name=f"Nav_{name}", goal_pose=goal_pose)
    recovery_subtree = create_recovery_subtree()

    fallback.add_children([nav_to_goal, recovery_subtree])
    return fallback

def waypoint_pose(x: float, y: float, theta_z: float = 0.0, theta_w: float = 1.0) -> PoseStamped:
    """
    편의를 위해 pose 생성
    """
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    # Orientation은 z,w만 사용(평면 회전)
    goal.pose.orientation.z = theta_z
    goal.pose.orientation.w = theta_w
    return goal

def make_waypoints_sequence() -> py_trees.behaviour.Behaviour:
    """
    3개의 웨이포인트를 순서대로 이동:
    [Fallback(Goal1), Fallback(Goal2), Fallback(Goal3)]
    모든 웨이포인트 성공 시 SUCCESS 반환
    (단, 무한 반복하고 싶다면 뒤에서 Repeat로 감쌀 예정)
    """
    seq = Sequence(name="WaypointsSequence", memory=True)

    # 첫째 지점 (3.6, 1.4)
    wp1 = create_navigate_or_recover(
        name="WP1",
        goal_pose=waypoint_pose(3.6, 1.4, theta_z=0.0, theta_w=1.0)
    )

    # 둘째 지점 (2.0, 2.0)
    wp2 = create_navigate_or_recover(
        name="WP2",
        goal_pose=waypoint_pose(2.0, 2.0, theta_z=0.0, theta_w=1.0)
    )

    # 셋째 지점 (0.0, 0.3)
    wp3 = create_navigate_or_recover(
        name="WP3",
        goal_pose=waypoint_pose(0.0, 0.3, theta_z=0.0, theta_w=1.0)
    )

    seq.add_children([wp1, wp2, wp3])
    return seq


def make_bt():
    """
    최종 트리:
    - WaypointsSequence를 무한 반복하도록 감쌈
    """
    # 3개 웨이포인트 시퀀스
    waypoints_seq = make_waypoints_sequence()

    # 무한 반복 데코레이터
    # py_trees.decorators.Repeat(once=False)는 자식이 SUCCESS 혹은 FAILURE가 되어도
    # 다시 자식을 initialise 하고 RUNNING으로 만들어 재실행
    repeated_waypoints = Repeat(
        child=waypoints_seq,
        name="RepeatWaypointsForever",
        num_success=100
    )
    return repeated_waypoints

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
            # if root.status == py_trees.common.Status.SUCCESS:
            #     node.get_logger().info("작업이 성공적으로 완료되었습니다. 종료합니다.")
            #     break
            # elif root.status == py_trees.common.Status.FAILURE:
            #     node.get_logger().info("작업이 실패했습니다. 종료합니다.")
            #     break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트 발생. 종료합니다.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()