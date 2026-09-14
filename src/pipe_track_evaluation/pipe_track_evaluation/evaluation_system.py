import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class MetricsNode(Node):

    def __init__(self):
        super().__init__('metrics_node')

        # Subscriptions
        self.sub_gt = self.create_subscription(
            Odometry, 'holocean/odom', self.gt_cb, 10)
        # Assuming you publish your estimated position to another topic:
        self.sub_est = self.create_subscription(
            Odometry, 'auv/estimated_odom', self.est_cb, 10)

        self.gt_path = []
        self.est_path = []

    def gt_cb(self, msg):
        pos = msg.pose.pose.position
        self.gt_path.append((pos.x, pos.y))

    def est_cb(self, msg):
        pos = msg.pose.pose.position
        self.est_path.append((pos.x, pos.y))

    def calculate_rmse(self):
        # Example logic: match timestamps and calculate Euclidean distance error
        pass

    def plot_trajectory(self):
        # Call this when the mission finishes to save the graph
        gt_x, gt_y = zip(*self.gt_path)
        est_x, est_y = zip(*self.est_path)

        plt.plot(gt_x, gt_y, label='Ground Truth', color='blue')
        plt.plot(est_x, est_y, label='Estimated', color='red', linestyle='--')
        plt.legend()
        plt.savefig(
            '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/results/trajectory.png')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
