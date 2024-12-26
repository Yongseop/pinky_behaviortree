import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import math
import csv
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AMCL_Subscriber(Node):
    def __init__(self):
        super().__init__('amcl_pose_subscriber')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT 
        
        # Subscription
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            qos
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # Timers
        self.create_timer(1.25, self.publish_cmd_vel)
        self.create_timer(10, self.compute_rotated_pose)

        # State variables
        self.latest_pose = None
        self.amcl_data = []
        self.shutdown = False
        self.initial_pose_time = None  # Time when initial pose is set
        self.toggle_direction = True

    def amcl_pose_callback(self, msg):
        self.latest_pose = msg.pose.pose

        # Extract and log the diagonal covariance values
        covariance = msg.pose.covariance
        x_uncertainty = covariance[0]
        y_uncertainty = covariance[7]
        theta_uncertainty = covariance[35]
        print(f'Diagonal Covariance: x={x_uncertainty:.4f}, y={y_uncertainty:.4f}, theta={theta_uncertainty:.4f}')

        # Save to AMCL data list
        self.amcl_data.append({
            "timestamp": self.get_clock().now().to_msg().sec,
            "x_uncertainty": x_uncertainty,
            "y_uncertainty": y_uncertainty,
            "theta_uncertainty": theta_uncertainty,
            "initial_pose_set": self.initial_pose_time is not None
        })

    def publish_cmd_vel(self):
        twist = Twist()
        if self.toggle_direction:
            twist.linear.x = 1.5  # Forward movement
            twist.angular.z = -1.0
        else:
            twist.linear.x = -1.5
            twist.angular.z = 1.0  # Rotate in place
        self.cmd_vel_publisher.publish(twist)
        self.toggle_direction = not self.toggle_direction

    def compute_rotated_pose(self):
        if self.latest_pose:
            q = self.latest_pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            new_yaw = yaw + math.radians(75)
            new_yaw = (new_yaw + math.pi) % (2 * math.pi) - math.pi

            new_pose = PoseWithCovarianceStamped()
            new_pose.header.stamp = self.get_clock().now().to_msg()
            new_pose.header.frame_id = "map"
            new_pose.pose.pose.position.x = self.latest_pose.position.x + 1.0
            new_pose.pose.pose.position.y = self.latest_pose.position.y + 1.0
            new_pose.pose.pose.position.z = self.latest_pose.position.z

            new_pose.pose.pose.orientation.z = math.sin(new_yaw / 2.0)
            new_pose.pose.pose.orientation.w = math.cos(new_yaw / 2.0)

            if not self.shutdown:
                self.shutdown = True
                self.initial_pose_publisher.publish(new_pose)
                self.initial_pose_time = self.get_clock().now().to_msg().sec
                print(f'Published New Initial Pose: Position(x: {new_pose.pose.pose.position.x}, y: {new_pose.pose.pose.position.y}), '
                      f'Theta: {new_yaw} radians')

                self.create_timer(10.0, self.delayed_shutdown)
        else:
            print('Cannot compute rotated pose. No AMCL Pose available yet.')

    def save_to_csv(self):
        with open('amcl_pose_data.csv', 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=["timestamp", "x_uncertainty", "y_uncertainty", "theta_uncertainty", "initial_pose_set"])
            writer.writeheader()
            writer.writerows(self.amcl_data)
        print('Data saved to CSV.')

    def delayed_shutdown(self):
        self.save_to_csv()
        print('Shutting down node now.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AMCL_Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()