import csv
import math
import os

from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, MagneticField
from std_msgs.msg import Bool


class MetricsNode(Node):

    # Polyline representing the ground-truth pipeline segments
    PIPELINE_WAYPOINTS = [
        (41, -66.2), (38.4, -68.6), (36.8, -69),
        (36.4, -76), (31.1, -76), (30.9, -69.3),
        (23.3, -69.3), (20.3, -65), (13.5, -65),
    ]

    def __init__(self):
        super().__init__('metrics_node')

        self.declare_parameter(
            'results_dir',
            'results'
        )
        self.results_dir = self.get_parameter('results_dir').value
        os.makedirs(self.results_dir, exist_ok=True)

        # Topic Subscriptions
        self.sub_gt = self.create_subscription(
            Odometry, 'holocean/odom', self.gt_cb, 10
        )
        self.magnetometer_sub = self.create_subscription(
            MagneticField, 'holocean/mag', self.magnetometer_cb, 10
        )
        self.sub_est = self.create_subscription(
            Odometry, 'estimation/odom', self.est_cb, 10
        )
        self.sub_finish = self.create_subscription(
            Bool, '/movement/finished_execution', self.finish_cb, 10
        )

        # Latency Subscriptions
        self.sub_mask = self.create_subscription(
            Image, '/object/mask', self.mask_cb, 10
        )
        self.sub_waypoint = self.create_subscription(
            PointStamped, '/trajectory/waypoint', self.waypoint_cb, 10
        )

        self.yaw = 0.0
        self.time_of_mask = 0.0

        # Time series buffers
        self.timestamps = []
        self.gt_positions = []
        self.est_positions = []
        self.speeds = []
        self.cross_track_errors = []
        self.heading_errors_deg = []

        # Latency buffers
        self.perception_latencies = []
        self.pipeline_latencies = []

        self.start_time = None
        self.rel_time = 0.0
        self.prev_pos = (0.0, 0.0)
        self.is_finished = False

        self.get_logger().info('✅ Metrics Evaluation Node Started.')

    # ------------------------------------------------------------------
    # Math Helpers: Geometry & Quaternion
    # ------------------------------------------------------------------
    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def point_to_segment_projection(p, a, b):
        px, py = p
        ax, ay = a
        bx, by = b

        dx = bx - ax
        dy = by - ay
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq == 0.0:
            return math.hypot(px - ax, py - ay), math.atan2(dy, dx)

        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
        proj_x = ax + t * dx
        proj_y = ay + t * dy

        dist = math.hypot(px - proj_x, py - proj_y)
        seg_heading = math.atan2(dy, dx)
        return dist, seg_heading

    def compute_pipeline_deviation(self, point, auv_yaw):
        min_dist = float('inf')
        target_heading = 0.0

        for i in range(len(self.PIPELINE_WAYPOINTS) - 1):
            seg_a = self.PIPELINE_WAYPOINTS[i]
            seg_b = self.PIPELINE_WAYPOINTS[i + 1]
            dist, heading = self.point_to_segment_projection(point, seg_a, seg_b)
            if dist < min_dist:
                min_dist = dist
                target_heading = heading

        heading_err = math.atan2(
            math.sin(auv_yaw - target_heading),
            math.cos(auv_yaw - target_heading)
        )
        return min_dist, math.degrees(abs(heading_err))

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def mask_cb(self, msg: Image):
        if self.is_finished:
            return
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        latency_ms = (now.nanoseconds - msg_time.nanoseconds) / 1e6
        self.perception_latencies.append(latency_ms)

    def waypoint_cb(self, msg: PointStamped):
        if self.is_finished:
            return
        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        latency_ms = (now.nanoseconds - msg_time.nanoseconds) / 1e6
        self.pipeline_latencies.append(latency_ms)

    def magnetometer_cb(self, msg: MagneticField):
        self.yaw = math.atan2(msg.magnetic_field.x, msg.magnetic_field.y) - (math.pi / 2)
        if self.yaw < -math.pi:
            self.yaw += 2 * math.pi

    def gt_cb(self, msg: Odometry):
        if self.is_finished:
            return

        self.rel_time += 1.0 / 30.0
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = self.yaw

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        if speed == 0.0 and self.prev_pos != (0.0, 0.0):
            dt = 1.0 / 30.0
            dx = px - self.prev_pos[0]
            dy = py - self.prev_pos[1]
            speed = math.hypot(dx, dy) / dt

        self.prev_pos = (px, py)

        cte, head_err = self.compute_pipeline_deviation((px, py), yaw)

        self.timestamps.append(self.rel_time)
        self.gt_positions.append((px, py))
        self.speeds.append(speed)
        self.cross_track_errors.append(cte)
        self.heading_errors_deg.append(head_err)

    def est_cb(self, msg: Odometry):
        if self.is_finished:
            return
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.est_positions.append((px, py))

    def finish_cb(self, msg: Bool):
        if msg.data and not self.is_finished:
            self.is_finished = True
            self.get_logger().info('🏁 Mission complete received. Evaluating...')
            self.evaluate_and_plot()

    # ------------------------------------------------------------------
    # Metrics Evaluation & Plot Generation
    # ------------------------------------------------------------------
    def evaluate_and_plot(self):
        if not self.gt_positions:
            self.get_logger().warn('No trajectory data recorded to evaluate!')
            return

        ctes = np.array(self.cross_track_errors)
        speeds = np.array(self.speeds)
        heading_errs = np.array(self.heading_errors_deg)

        rmse_cte = float(np.sqrt(np.mean(ctes ** 2)))
        max_cte = float(np.max(ctes))

        mean_speed = float(np.mean(speeds))
        std_speed = float(np.std(speeds))

        mean_heading_err = float(np.mean(heading_errs))

        target_finish = self.PIPELINE_WAYPOINTS[-1]
        actual_finish = self.gt_positions[-1]
        terminal_error = math.hypot(
            actual_finish[0] - target_finish[0],
            actual_finish[1] - target_finish[1]
        )
        total_time = self.timestamps[-1] if self.timestamps else 0.0

        # estimation vs gt position error
        if self.est_positions and self.gt_positions:
            # Align arrays to the shortest length for 1:1 comparison
            min_len = min(len(self.gt_positions), len(self.est_positions))
            gt_arr = np.array(self.gt_positions[:min_len])
            est_arr = np.array(self.est_positions[:min_len])

            # Calculate point-to-point Euclidean distances
            est_errors = np.linalg.norm(gt_arr - est_arr, axis=1)
            rmse_estimation = float(np.sqrt(np.mean(est_errors ** 2)))
            max_est_error = float(np.max(est_errors))

        # Calculate Latencies
        avg_perception = (
            float(np.mean(self.perception_latencies))
            if self.perception_latencies else 0.0
        )
        avg_pipeline = float(np.mean(self.pipeline_latencies)) if self.pipeline_latencies else 0.0
        avg_planning = max(0.0, avg_pipeline - avg_perception)

        # Build Report String
        report = (
            '================ TRACKING METRICS REPORT ================\n'
            f'1. Cross-Track Error (RMSE)   : {rmse_cte:.3f} m\n'
            f'2. Max Cross-Track Error      : {max_cte:.3f} m\n'
            f'3. Mean Velocity (Momentum)   : {mean_speed:.3f} ± '
            f'{std_speed:.3f} m/s\n'
            f'4. Mean Heading Error         : {mean_heading_err:.2f} deg\n'
            f'5. Terminal Position Error    : {terminal_error:.3f} m\n'
            f'6. Est. vs GT RMSE: {rmse_estimation:.3f} m (Max: {max_est_error:.3f} m)'
            '---------------------------------------------------------\n'
            f'   Avg Perception Latency     : {avg_perception:.1f} ms\n'
            f'   Avg Planning Latency       : {avg_planning:.1f} ms\n'
            f'   Total Pipeline Latency     : {avg_pipeline:.1f} ms\n'
            f'   Mission Execution Time     : {total_time:.1f} s\n'
            '=========================================================\n'
        )

        # Output to console and save to text file
        self.get_logger().info(f'\n{report}')
        report_path = os.path.join(self.results_dir, 'metrics_summary.txt')
        with open(report_path, 'w') as f:
            f.write(report)
        self.get_logger().info(f'📄 Summary saved to: {report_path}')

        # Save time-series data to CSV
        csv_path = os.path.join(self.results_dir, 'time_series_data.csv')
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_s', 'pos_x', 'pos_y', 'cte_m', 'speed_ms',
                'heading_err_deg'
            ])
            for i in range(len(self.timestamps)):
                writer.writerow([
                    self.timestamps[i],
                    self.gt_positions[i][0],
                    self.gt_positions[i][1],
                    self.cross_track_errors[i],
                    self.speeds[i],
                    self.heading_errors_deg[i]
                ])
        self.get_logger().info(f'💾 Raw data saved to: {csv_path}')

        self.generate_academic_plots(rmse_cte, max_cte, terminal_error, mean_speed)

    def generate_academic_plots(self, rmse_cte, max_cte, terminal_err, mean_speed):
        plot_style = (
            'seaborn-v0_8-whitegrid'
            if 'seaborn-v0_8-whitegrid' in plt.style.available
            else 'default'
        )
        plt.style.use(plot_style)
        fig, axes = plt.subplots(1, 3, figsize=(18, 5))

        # --- Panel 1: Top-Down 2D Trajectory ---
        ax1 = axes[0]
        pipe_x, pipe_y = zip(*self.PIPELINE_WAYPOINTS)
        ax1.plot(pipe_x, pipe_y, 'k-', linewidth=3, label='Ground-Truth Pipeline', alpha=0.7)
        ax1.scatter(pipe_x, pipe_y, c='black', s=25, zorder=3)

        gt_x, gt_y = zip(*self.gt_positions)
        ax1.plot(gt_x, gt_y, 'b-', linewidth=2, label='AUV Ground-Truth')

        if self.est_positions:
            est_x, est_y = zip(*self.est_positions)
            ax1.plot(est_x, est_y, 'r--', linewidth=1.5, label='Estimated Odom')

        ax1.scatter([pipe_x[0]], [pipe_y[0]], c='green', s=100, marker='o', label='Start')
        ax1.scatter([pipe_x[-1]], [pipe_y[-1]], c='red', s=120, marker='X', label='Target End')
        ax1.set_title('2D Trajectory Tracking Map', fontsize=12, fontweight='bold')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.legend(loc='best', frameon=True)
        ax1.axis('equal')

        # --- Panel 2: Cross-Track Error Over Time ---
        ax2 = axes[1]
        ax2.plot(self.timestamps, self.cross_track_errors, 'purple', linewidth=2, label='CTE [m]')
        ax2.axhline(rmse_cte, color='crimson', linestyle='--', label=f'RMSE = {rmse_cte:.2f} m')
        ax2.set_title(f'Cross-Track Error (Max: {max_cte:.2f} m)', fontsize=12, fontweight='bold')
        ax2.set_xlabel('Mission Time [s]')
        ax2.set_ylabel('Perpendicular Deviation [m]')
        ax2.legend(loc='upper right', frameon=True)

        # --- Panel 3: Velocity & Heading Error ---
        ax3 = axes[2]
        ax3_twin = ax3.twinx()

        p1 = ax3.plot(
            self.timestamps, self.speeds, 'teal', linewidth=2,
            label='Linear Velocity [m/s]'
        )
        p2 = ax3_twin.plot(
            self.timestamps, self.heading_errors_deg, 'orange', linewidth=1.5,
            linestyle=':', label='Heading Error [°]'
        )

        ax3.set_title('Momentum & Heading Alignment', fontsize=12, fontweight='bold')
        ax3.set_xlabel('Mission Time [s]')
        ax3.set_ylabel('Speed [m/s]', color='teal')
        ax3_twin.set_ylabel('Heading Error [deg]', color='darkorange')

        lines = p1 + p2
        labels = [line.get_label() for line in lines]
        ax3.legend(lines, labels, loc='upper right', frameon=True)

        plt.tight_layout()
        save_path = os.path.join(self.results_dir, 'tracking_benchmark_report.png')
        plt.savefig(save_path, dpi=300)
        self.get_logger().info(f'📊 Benchmark figure saved at: {save_path}')
        plt.close()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected. Computing final metrics...')
        if not node.is_finished:
            node.evaluate_and_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
