import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav2_msgs.msg import ParticleCloud
import math
import csv
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AMCL_Monitor(Node):
    def __init__(self):
        super().__init__('amcl_monitor')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT 
        
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            qos
        )
        self.particle_subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_cloud_callback,
            qos
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_pose = None
        self.particle_data = []
        self.amcl_data = []
        self.start_time = self.get_clock().now()
        self.toggle_direction = True
        self.baseline_calculated = False

        self.thresholds = {
            'x_uncertainty': 0.012,
            'y_uncertainty': 0.012,
            'theta_uncertainty': 0.025,
            'y_variance': 0.01
        }

        self.create_timer(1.25, self.publish_cmd_vel)

    def amcl_pose_callback(self, msg):
        self.latest_pose = msg.pose.pose
        covariance = msg.pose.covariance
        x_uncertainty = covariance[0]
        y_uncertainty = covariance[7]
        theta_uncertainty = covariance[35]

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        self.amcl_data.append({
            "timestamp": self.get_clock().now().to_msg().sec,
            "x_uncertainty": x_uncertainty,
            "y_uncertainty": y_uncertainty,
            "theta_uncertainty": theta_uncertainty,
            "initial_pose_set": False
        })

        if elapsed_time >= 10.0 and not self.baseline_calculated:
            self.baseline_calculated = True
            self.calculate_baseline_and_check()

    def particle_cloud_callback(self, msg):
        particle_positions = [(p.pose.position.x, p.pose.position.y) for p in msg.particles]
        weights = [p.weight for p in msg.particles]

        x_positions = [pos[0] for pos in particle_positions]
        y_positions = [pos[1] for pos in particle_positions]

        x_variance = np.var(x_positions)
        y_variance = np.var(y_positions)
        max_weight = max(weights) if weights else 0.0
        mean_weight = np.mean(weights) if weights else 0.0

        self.particle_data.append({
            "timestamp": self.get_clock().now().to_msg().sec,
            "x_variance": x_variance,
            "y_variance": y_variance,
            "max_weight": max_weight,
            "mean_weight": mean_weight,
            "initial_pose_set": False
        })

    def publish_cmd_vel(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time < 10.5:
            self.get_logger().info(f'Moving robot: {elapsed_time:.1f} seconds elapsed')
            twist = Twist()
            if self.toggle_direction:
                twist.linear.x = 1.5
                twist.angular.z = -1.0
            else:
                twist.linear.x = -1.5
                twist.angular.z = 1.0
            self.cmd_vel_publisher.publish(twist)
            self.toggle_direction = not self.toggle_direction
        else:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)

    def calculate_baseline_and_check(self):
        if len(self.amcl_data) < 5:
            return

        baseline_data = [d for d in self.amcl_data if d['timestamp'] <= self.get_clock().now().to_msg().sec]
        
        avg_x_uncertainty = np.mean([d['x_uncertainty'] for d in baseline_data])
        avg_y_uncertainty = np.mean([d['y_uncertainty'] for d in baseline_data])
        avg_theta_uncertainty = np.mean([d['theta_uncertainty'] for d in baseline_data])

        self.get_logger().info('Starting localization monitoring')
        self.get_logger().info(f'Baseline values - X: {avg_x_uncertainty:.4f}, Y: {avg_y_uncertainty:.4f}, Theta: {avg_theta_uncertainty:.4f}')

        current_data = self.amcl_data[-1]  # Get the latest data point
        self.check_localization_status(
            current_data['x_uncertainty'],
            current_data['y_uncertainty'],
            current_data['theta_uncertainty']
        )

    def check_localization_status(self, x_uncertainty, y_uncertainty, theta_uncertainty):
        is_lost = False
        messages = []

        if x_uncertainty > self.thresholds['x_uncertainty']:
            messages.append(f'High X uncertainty: {x_uncertainty:.4f}')
            is_lost = True
        
        if y_uncertainty > self.thresholds['y_uncertainty']:
            messages.append(f'High Y uncertainty: {y_uncertainty:.4f}')
            is_lost = True
            
        if theta_uncertainty > self.thresholds['theta_uncertainty']:
            messages.append(f'High Theta uncertainty: {theta_uncertainty:.4f}')
            is_lost = True

        if is_lost:
            self.get_logger().warning('Possible localization loss detected!')
            for msg in messages:
                self.get_logger().warning(msg)
        else:
            self.get_logger().info('Localization status: OK')

    def save_to_csv(self):
        with open('amcl_pose_data.csv', 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=["timestamp", "x_uncertainty", "y_uncertainty", "theta_uncertainty", "initial_pose_set"])
            writer.writeheader()
            writer.writerows(self.amcl_data)

        with open('particle_cloud_data.csv', 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=["timestamp", "x_variance", "y_variance", "max_weight", "mean_weight", "initial_pose_set"])
            writer.writeheader()
            writer.writerows(self.particle_data)

        self.get_logger().info('Data saved to CSV files')

def main(args=None):
    rclpy.init(args=args)
    node = AMCL_Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_to_csv()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
