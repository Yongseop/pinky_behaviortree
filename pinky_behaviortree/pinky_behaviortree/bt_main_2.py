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

from .bt_check_localization import CheckLocalizationStatus
from .bt_aruco_localization import ArucoLocalization
from .bt_detect_marker import DetectMarker

class Counter(py_trees.decorators.Decorator):
    def __init__(self, child, name="Counter", max_count=10):
        super().__init__(name=name, child=child)
        self.max_count = max_count
        self.count = 0

    def update(self):
        if self.count >= self.max_count:
            return py_trees.common.Status.FAILURE

        self.decorated.tick()
        status = self.decorated.status

        if status == py_trees.common.Status.SUCCESS:
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

def make_bt():
    root = Selector(name="Root", memory=True)

    check_status = CheckLocalizationStatus()
    root.add_child(check_status)

    detect_sequence = Sequence(
        name="DetectSequence",
        memory=True,
        children=[
            DetectMarker(detection_time=2.0),  # 2초 동안 마커 감지
            ArucoLocalization()
        ]
    )
    rotation_counter = Counter(child=detect_sequence, max_count=10)

    root.add_children([rotation_counter])

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
            if root.status == py_trees.common.Status.SUCCESS:
                node.get_logger().info("작업이 성공적으로 완료되었습니다. 종료합니다.")
                break
            elif root.status == py_trees.common.Status.FAILURE:
                node.get_logger().info("작업이 실패했습니다. 종료합니다.")
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트 발생. 종료합니다.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()