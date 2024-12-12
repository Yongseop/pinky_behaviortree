import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav2_msgs.msg import ParticleCloud
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT 
        self.toggle_direction = True
        
        # Subscriptions
        self.amcl_subscription = self.create_subscription(
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

        # Publisher for movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.amcl_data = []
        self.particle_data = []
        self.data_collection_done = False

        # Thresholds
        self.amcl_threshold = {
            "x_uncertainty": 0.07,
            "y_uncertainty": 0.07,
            "theta_uncertainty": 0.1
        }
        self.particle_threshold = {
            "x_variance": 0.05,
            "y_variance": 0.05
        }

        # Start data collection timer
        self.create_timer(1.25, self.publish_cmd_vel)  # Command velocity timer
        self.create_timer(20, self.stop_data_collection)  # Stop after 10 seconds

    def amcl_pose_callback(self, msg):
        if not self.data_collection_done:
            covariance = msg.pose.covariance
            x_uncertainty = covariance[0]
            y_uncertainty = covariance[7]
            theta_uncertainty = covariance[35]

            self.amcl_data.append({
                "x_uncertainty": x_uncertainty,
                "y_uncertainty": y_uncertainty,
                "theta_uncertainty": theta_uncertainty
            })

    def particle_cloud_callback(self, msg):
        if not self.data_collection_done:
            x_positions = [p.pose.position.x for p in msg.particles]
            y_positions = [p.pose.position.y for p in msg.particles]

            x_variance = np.var(x_positions)
            y_variance = np.var(y_positions)

            self.particle_data.append({
                "x_variance": x_variance,
                "y_variance": y_variance
            })

    def publish_cmd_vel(self):
        if not self.data_collection_done:
            twist = Twist()
            if self.toggle_direction:
                twist.linear.x = 1.5  # Forward movement
                twist.angular.z = -1.0
            else:
                twist.linear.x = -1.5
                twist.angular.z = 1.0  # Rotate in place
            self.cmd_vel_publisher.publish(twist)
            self.toggle_direction = not self.toggle_direction

    def stop_data_collection(self):
        self.data_collection_done = True
        self.analyze_data()

    def analyze_data(self):
        # Calculate mean values for AMCL data
        amcl_means = {
            "x_uncertainty": np.mean([d["x_uncertainty"] for d in self.amcl_data]),
            "y_uncertainty": np.mean([d["y_uncertainty"] for d in self.amcl_data]),
            "theta_uncertainty": np.mean([d["theta_uncertainty"] for d in self.amcl_data])
        }

        # Check if mean values exceed thresholds
        amcl_lost = (
            amcl_means["x_uncertainty"] > self.amcl_threshold["x_uncertainty"] or
            amcl_means["y_uncertainty"] > self.amcl_threshold["y_uncertainty"] or
            amcl_means["theta_uncertainty"] > self.amcl_threshold["theta_uncertainty"]
        )

        # Calculate mean values for Particle Cloud data
        particle_means = {
            "x_variance": np.mean([d["x_variance"] for d in self.particle_data]),
            "y_variance": np.mean([d["y_variance"] for d in self.particle_data])
        }

        # Check if mean values exceed thresholds
        particle_lost = (
            particle_means["x_variance"] > self.particle_threshold["x_variance"] or
            particle_means["y_variance"] > self.particle_threshold["y_variance"]
        )

        # Print results
        print(f'amcl_lost: {amcl_lost}, threshold: {self.amcl_threshold}')
        print(f'particle_lost: {particle_lost}, threshold: {self.particle_threshold}')
        if amcl_lost or particle_lost:
            print("[RESULT] Localization Lost Detected!")
        else:
            print("[RESULT] Localization Stable.")

        # Save data for future analysis
        self.save_data()
        self.terminate_node()

    def save_data(self):
        # Save AMCL data
        with open('amcl_data.csv', 'w') as f:
            f.write("x_uncertainty,y_uncertainty,theta_uncertainty\n")
            for d in self.amcl_data:
                f.write(f"{d['x_uncertainty']},{d['y_uncertainty']},{d['theta_uncertainty']}\n")

        # Save Particle Cloud data
        with open('particle_data.csv', 'w') as f:
            f.write("x_variance,y_variance\n")
            for d in self.particle_data:
                f.write(f"{d['x_variance']},{d['y_variance']}\n")

        print("Data saved to CSV files.")

    def terminate_node(self):
        print("Shutting down node...")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()