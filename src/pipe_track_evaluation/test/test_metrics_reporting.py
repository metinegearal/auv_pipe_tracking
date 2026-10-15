"""Check report scheduling, completion races, and write-failure recovery."""

from unittest.mock import Mock

from pipe_track_evaluation.metrics_reporting import MetricsReporting


def reporter_with_data():
    """Provide the small node interface required by the reporting helper."""
    node = Mock()
    node.get_parameter.return_value.value = 5.0
    node.is_finished = False
    node.gt_positions = [(1.0, 2.0)]
    node.evaluate_and_plot.return_value = True
    return MetricsReporting(node), node


def test_running_summary_is_saved_without_mission_completion():
    reporter, node = reporter_with_data()
    reporter.checkpoint()
    node.evaluate_and_plot.assert_called_once_with(generate_plots=False)
    assert not node.is_finished


def test_finish_writes_final_plot_only_once():
    reporter, node = reporter_with_data()
    reporter.finish()
    reporter.finish()
    reporter.checkpoint()
    node.evaluate_and_plot.assert_called_once_with(generate_plots=True)
    assert node.is_finished


def test_early_finish_waits_for_first_trajectory_sample():
    reporter, node = reporter_with_data()
    node.gt_positions = []
    reporter.finish()
    reporter.checkpoint()
    node.evaluate_and_plot.assert_not_called()
    assert not node.is_finished
    node.gt_positions.append((1.0, 2.0))
    reporter.checkpoint()
    node.evaluate_and_plot.assert_called_once_with(generate_plots=True)
    assert node.is_finished


def test_failed_final_save_is_retried_without_marking_finished():
    reporter, node = reporter_with_data()
    node.evaluate_and_plot.side_effect = [OSError('directory unavailable'), True]
    reporter.finish()
    assert not node.is_finished
    reporter.checkpoint()
    assert node.is_finished
    assert node.evaluate_and_plot.call_count == 2


def test_no_report_without_trajectory_data():
    reporter, node = reporter_with_data()
    node.gt_positions = []
    reporter.checkpoint()
    node.evaluate_and_plot.assert_not_called()
