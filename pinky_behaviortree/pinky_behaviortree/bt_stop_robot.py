import py_trees
import rclpy
from geometry_msgs.msg import Twist

class StopRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name="StopRobot", stop_time=1.0):
        super().__init__(name)
        self.node = None
        self.cmd_vel_publisher = None
        self.start_time = None
        self.stop_time = stop_time
        
    def setup(self, **kwargs):
        try:
            self.node = kwargs.get('node')
            if not self.node:
                self.node = rclpy.create_node('stop_robot_node')
            
            self.cmd_vel_publisher = self.node.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )
            return True
        except Exception as e:
            self.node.get_logger().error(f'Setup failed: {str(e)}')
            return False
            
    def initialise(self):
        self.start_time = None
            
    def update(self):
        if not self.start_time:
            self.start_time = self.node.get_clock().now()
            
        current_time = self.node.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        if elapsed_time < self.stop_time:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            self.node.get_logger().info('Stopping...')
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS
            
    def terminate(self, new_status):
        if self.cmd_vel_publisher:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)