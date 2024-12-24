import rclpy
import time
import py_trees_ros
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.decorators import Inverter
from py_trees.decorators import Decorator
from py_trees.composites import Selector
from py_trees import logging as log_tree
import py_trees
from .bt_check_localization import CheckLocalizationStatus

def make_bt():
    root = Selector(name="sequence", memory=True)

    check_status = CheckLocalizationStatus()
    root.add_child(check_status)

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
            if root.status in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE]:
                node.get_logger().info("Tree has finished execution. Shutting down.")
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Shutting down.")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()