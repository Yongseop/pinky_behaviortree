import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class AMCLPoseCovarianceLogger(Node):
    def __init__(self):
        super().__init__('amcl_pose_covariance_logger')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

    def amcl_pose_callback(self, msg):
        covariance = msg.pose.covariance
        print('AMCL Pose Covariance:')
        print(
            f"[ {covariance[0]:.4f}, {covariance[1]:.4f}, {covariance[2]:.4f}, {covariance[3]:.4f}, {covariance[4]:.4f}, {covariance[5]:.4f} ]")
        print(
            f"[ {covariance[6]:.4f}, {covariance[7]:.4f}, {covariance[8]:.4f}, {covariance[9]:.4f}, {covariance[10]:.4f}, {covariance[11]:.4f} ]")
        print(
            f"[ {covariance[12]:.4f}, {covariance[13]:.4f}, {covariance[14]:.4f}, {covariance[15]:.4f}, {covariance[16]:.4f}, {covariance[17]:.4f} ]")
        print(
            f"[ {covariance[18]:.4f}, {covariance[19]:.4f}, {covariance[20]:.4f}, {covariance[21]:.4f}, {covariance[22]:.4f}, {covariance[23]:.4f} ]")
        print(
            f"[ {covariance[24]:.4f}, {covariance[25]:.4f}, {covariance[26]:.4f}, {covariance[27]:.4f}, {covariance[28]:.4f}, {covariance[29]:.4f} ]")
        print(
            f"[ {covariance[30]:.4f}, {covariance[31]:.4f}, {covariance[32]:.4f}, {covariance[33]:.4f}, {covariance[34]:.4f}, {covariance[35]:.4f} ]")


def main(args=None):
    rclpy.init(args=args)
    node = AMCLPoseCovarianceLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
