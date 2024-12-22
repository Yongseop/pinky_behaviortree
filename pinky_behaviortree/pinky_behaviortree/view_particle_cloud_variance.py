import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

class ParticleCloudVarianceLogger(Node):
    def __init__(self):
        super().__init__('particle_cloud_logger')
        
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT 
        
        
        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_cloud_callback,
            qos
        )
    def particle_cloud_callback(self, msg):
        # Extract x and y positions of particles
        x_positions = [particle.pose.position.x for particle in msg.particles]
        y_positions = [particle.pose.position.y for particle in msg.particles]

        # Compute variance for x and y
        x_variance = np.var(x_positions)
        y_variance = np.var(y_positions)

        # Print the variances
        print('Particle Cloud Variance:')
        print(f'  X Variance: {x_variance:.6f}')
        print(f'  Y Variance: {y_variance:.6f}')
        print(f'Total Particles: {len(msg.particles)}')


def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudVarianceLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()