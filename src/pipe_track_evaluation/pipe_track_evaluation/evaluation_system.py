import math
import os

import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MetricsNode(Node):

    # Polyline representing the ground-truth pipeline segments
    PIPELINE_WAYPOINTS = [(41,-66.2),(38.4,-68.6),(36.8,-69),(36.4,-76),(31.1,-76),(30.9,-69.3),(23.3,-69.3),(20.3,-65),(13.5,-65)]


    def __init__(self):
        super().__init__('metrics_node')

        self.declare_parameter(
            'results_dir',
            '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/results'
        )
        self.results_dir = self.get_parameter('results_dir').value
        os.makedirs(self.results_dir, exist_ok=True)

        # Topic Subscriptions
        self.sub_gt = self.create_subscription(
            Odometry, 'holocean/odom', self.gt_cb, 10
        )
        self.sub_est = self.create_subscription(
            Odometry, 'auv/estimated_odom', self.est_cb, 10
        )
        self.sub_finish = self.create_subscription(
            Bool, '/movement/finished_execution', self.finish_cb, 10
        )

        # Time series buffers
        self.timestamps = []
        self.gt_positions = []
        self.est_positions = []
        self.speeds = []
        self.cross_track_errors = []
        self.heading_errors_deg = []

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
        """Extract yaw (Euler z) from a geometry_msgs/Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def point_to_segment_projection(p, a, b):
        """
        Calculate perpendicular distance from point p to segment ab,
        and return the segment heading (tangent angle).
        """
        px, py = p
        ax, ay = a
        bx, by = b

        dx = bx - ax
        dy = by - ay
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq == 0.0:
            return math.hypot(px - ax, py - ay), math.atan2(dy, dx)

        # Projection parameter t clamped to [0, 1]
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
        proj_x = ax + t * dx
        proj_y = ay + t * dy

        dist = math.hypot(px - proj_x, py - proj_y)
        seg_heading = math.atan2(dy, dx)
        return dist, seg_heading

    def compute_pipeline_deviation(self, point, auv_yaw):
        """Find minimum cross-track distance and corresponding heading error."""
        min_dist = float('inf')
        target_heading = 0.0

        for i in range(len(self.PIPELINE_WAYPOINTS) - 1):
            seg_a = self.PIPELINE_WAYPOINTS[i]
            seg_b = self.PIPELINE_WAYPOINTS[i + 1]
            dist, heading = self.point_to_segment_projection(point, seg_a, seg_b)
            if dist < min_dist:
                min_dist = dist
                target_heading = heading

        # Angular difference wrapped to [-pi, pi]
        heading_err = math.atan2(
            math.sin(auv_yaw - target_heading),
            math.cos(auv_yaw - target_heading)
        )
        return min_dist, math.degrees(abs(heading_err))

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def gt_cb(self, msg: Odometry):
        if self.is_finished:
            return

        # now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # if self.start_time is None:
        #     self.start_time = now

        self.rel_time += 1.0 / 30.0 # simulation time step
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)

        # Speed magnitude (linear momentum)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.hypot(vx, vy)

        # 2. If Twist is empty (using direct location), derive speed numerically
        if speed == 0.0 and self.prev_pos != (0.0, 0.0):
            dt = 1.0 / 30.0
            dx = px - self.prev_pos[0]
            dy = py - self.prev_pos[1]
            speed = math.hypot(dx, dy) / dt

        # Update previous states for the next tick
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

        # 1. Staying on the Line: RMSE & Max CTE
        rmse_cte = float(np.sqrt(np.mean(ctes ** 2)))
        max_cte = float(np.max(ctes))

        # 2. Momentum & Smoothness: Velocity stats
        mean_speed = float(np.mean(speeds))
        std_speed = float(np.std(speeds))

        # 3. Heading Tracking Error: Mean & Max
        mean_heading_err = float(np.mean(heading_errs))

        # 4. Finishing at Target: Terminal Position Error
        target_finish = self.PIPELINE_WAYPOINTS[-1]
        actual_finish = self.gt_positions[-1]
        terminal_error = math.hypot(
            actual_finish[0] - target_finish[0],
            actual_finish[1] - target_finish[1]
        )
        total_time = self.timestamps[-1] if self.timestamps else 0.0

        # Output to console
        self.get_logger().info('================ TRACKING METRICS REPORT ================')
        self.get_logger().info(f'1. Cross-Track Error (RMSE)   : {rmse_cte:.3f} m')
        self.get_logger().info(f'2. Max Cross-Track Error       : {max_cte:.3f} m')
        self.get_logger().info(f'3. Mean Velocity (Momentum)    : {mean_speed:.3f} ± {std_speed:.3f} m/s')
        self.get_logger().info(f'4. Mean Heading Error          : {mean_heading_err:.2f} deg')
        self.get_logger().info(f'5. Terminal Position Error     : {terminal_error:.3f} m')
        self.get_logger().info(f'   Mission Execution Time      : {total_time:.1f} s')
        self.get_logger().info('=========================================================')

        self.generate_academic_plots(rmse_cte, max_cte, terminal_error, mean_speed)

    def generate_academic_plots(self, rmse_cte, max_cte, terminal_err, mean_speed):
        """Generates a 3-panel publication benchmark figure."""
        plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
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

        p1 = ax3.plot(self.timestamps, self.speeds, 'teal', linewidth=2, label='Linear Velocity [m/s]')
        p2 = ax3_twin.plot(self.timestamps, self.heading_errors_deg, 'orange', linewidth=1.5, linestyle=':', label='Heading Error [°]')

        ax3.set_title('Momentum & Heading Alignment', fontsize=12, fontweight='bold')
        ax3.set_xlabel('Mission Time [s]')
        ax3.set_ylabel('Speed [m/s]', color='teal')
        ax3_twin.set_ylabel('Heading Error [deg]', color='darkorange')

        # Combine twin-axis legends
        lines = p1 + p2
        labels = [l.get_label() for l in lines]
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
        node.evaluate_and_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()