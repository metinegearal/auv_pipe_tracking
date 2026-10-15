"""Schedule progress saves and finalize reports on the existing metrics node."""

import math


class MetricsReporting:
    """Keep live summaries current and retry finalization until data is available."""

    def __init__(self, node):
        self.node = node
        self.finish_requested = False
        node.declare_parameter('metrics_update_period', 5.0)
        period = float(node.get_parameter('metrics_update_period').value)
        if not math.isfinite(period) or period <= 0:
            raise ValueError('metrics_update_period must be positive and finite')
        self.timer = node.create_timer(period, self.checkpoint)

    def checkpoint(self):
        """Save text/CSV during the run; generate plots when completion is pending."""
        if self.node.is_finished or not self.node.gt_positions:
            return
        self._save(final=self.finish_requested)

    def finish(self):
        """Handle mission completion and Ctrl+C through the same final save path."""
        if self.node.is_finished:
            return
        self.finish_requested = True
        if not self.node.gt_positions:
            self.node.get_logger().warn('No trajectory data yet; final report is pending.')
            return
        self._save(final=True)

    def _save(self, final):
        try:
            saved = self.node.evaluate_and_plot(generate_plots=final)
        except (OSError, ValueError, ArithmeticError) as error:
            self.node.get_logger().error(f'Metrics save failed; will retry: {error}')
            return
        if final and saved:
            # Mark finished only after files have actually been written.
            self.node.is_finished = True
