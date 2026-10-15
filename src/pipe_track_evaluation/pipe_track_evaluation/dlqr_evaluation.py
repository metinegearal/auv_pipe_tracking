"""Generate a DLQR report once per model published by the controller."""

import json

from pipe_track_evaluation.dlqr_analysis import write_outputs
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class DlqrEvaluation:
    """Attach DLQR reporting to the existing evaluation node."""

    def __init__(self, node, results_dir):
        self.node = node
        self.results_dir = results_dir
        self.last_model = None
        self.subscription = node.create_subscription(
            String, '/motion/dlqr_model', self.on_model,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.get_logger().info('Waiting for the controller model on /motion/dlqr_model')

    def on_model(self, message):
        """Regenerate files after a controller restart with different gains."""
        if message.data == self.last_model:
            return
        try:
            report, plot = write_outputs(
                json.loads(message.data), self.results_dir)
        except (ValueError, KeyError, TypeError, OSError, ArithmeticError) as error:
            self.node.get_logger().error(f'DLQR evaluation failed: {error}')
            return
        self.last_model = message.data
        self.node.get_logger().info(
            f'DLQR evaluation saved: {report.resolve()} and {plot.resolve()}')
