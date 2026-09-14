#!/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/envHolo/bin/python3
"""
holocean_ros2_bridge.py

Run inside the same process as HoloOcean:
with holoocean.make(scenario_cfg=cfg) as env:
    state = env.step(command)

Publishes:
 - /holocean/camera/image_raw         (sensor_msgs/Image)  -- RGB
 - /holocean/camera/camera_info       (sensor_msgs/CameraInfo)
 - /holocean/depth/image_raw_or_value (sensor_msgs/Image OR std_msgs/Float32)
 - /holocean/imu                      (sensor_msgs/Imu)
 - /holocean/mag                      (sensor_msgs/MagneticField)
 - /holocean/dvl                      (geometry_msgs/TwistStamped)
 - /holocean/odom                     (nav_msgs/Odometry)
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import numpy as np
import time
import traceback

from sensor_msgs.msg import Image, CameraInfo, Imu, MagneticField
from std_msgs.msg import Float32, Header, Float32MultiArray
from geometry_msgs.msg import TwistStamped, Vector3, Quaternion, Pose, Point
from nav_msgs.msg import Odometry

# cv_bridge for numpy <-> ROS Image
from cv_bridge import CvBridge

# Helper quaternion conversion
import math
import json


def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - \
        math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + \
        math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - \
        math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + \
        math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    return qx, qy, qz, qw


def vortex_field(location):
    x, y, z = location
    if z > 0:
        return [0, 0, 0]
    strength = 5.0
    r_squared = x**2 + y**2 + 1e-5  # avoid divide by zero
    dx = -y / r_squared * strength
    dy = x / r_squared * strength
    dz = 0.2 * np.cos(0.1 * r_squared)
    return [3*dx, 3*dy, 3*dz]


class HoloOceanROS2Bridge(Node):
    def __init__(self, holoocean_make_fn, step_limit=None):
        """
        holoocean_make_fn: function holoocean.make
        scenario_cfg: config object/str passed to holoocean.make
        command: default command to pass to env.step(command). If None, pass {}.
        step_limit: optional max steps to run (None => run until env finishes or KeyboardInterrupt)
        """
        super().__init__('holocean_ros2_bridge')

        # 1. Declare and get the parameter passed from the launch file
        self.declare_parameter('holoocean_config_path', '')
        config_path = self.get_parameter('holoocean_config_path').value

        # 2. Read the JSON file into a dictionary
        if not config_path:
            self.get_logger().error("No holoocean config path provided!")
            return

        with open(config_path, 'r') as f:
            scenario_cfg = json.load(f)

        self.get_logger().info(
            f"Loaded HoloOcean Config for world: {scenario_cfg['world']}")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # qos = qos_profile_sensor_data

        # Publishers
        self.camera_pub = self.create_publisher(
            Image, 'holocean/camera/image_raw', qos)
        self.cameraDown_pub = self.create_publisher(
            Image, 'holocean/cameraDown/image_raw', qos)
        self.cam_info_pub = self.create_publisher(
            CameraInfo, 'holocean/camera/camera_info', qos)
        self.depth_image_pub = self.create_publisher(
            Image, 'holocean/depth/image_raw', qos)
        self.depth_scalar_pub = self.create_publisher(
            Float32, 'holocean/depth/scalar', qos)
        self.range_finder_pub = self.create_publisher(
            Float32, 'holocean/distance/scalar', qos)
        self.depth_distance_pub = self.create_publisher(
            Float32, 'holocean/depth/distance', qos)
        self.imu_pub = self.create_publisher(Imu, 'holocean/imu', qos)
        self.mag_pub = self.create_publisher(
            MagneticField, 'holocean/mag', qos)
        self.dvl_pub = self.create_publisher(TwistStamped, 'holocean/dvl', qos)
        self.odom_pub = self.create_publisher(Odometry, 'holocean/odom', qos)

        # Bridge and config
        self.bridge = CvBridge()
        self.holoocean_make_fn = holoocean_make_fn
        self.scenario_cfg = scenario_cfg
        self.step_limit = step_limit

        # CameraInfo default (override if you have calibration YAML)
        self.default_cam_info = CameraInfo()
        # fill some reasonable defaults (width/height will be set per-image)
        self.default_cam_info.distortion_model = 'plumb_bob'
        self.default_cam_info.k = [0.0]*9
        self.default_cam_info.d = [0.0]*5
        self.default_cam_info.r = [0.0]*9
        self.default_cam_info.p = [0.0]*12
        self.default_cam_info.binning_x = 0
        self.default_cam_info.binning_y = 0

        self.command = [10, 10, 10, 10, 0, 0, 0, 0]  # default command
        self.sub_cmd = self.create_subscription(
            Float32MultiArray,
            '/holo/cmd',
            self.cmd_callback,
            10
        )

        # run loop in a background timer to keep rclpy spinning responsive
        self.get_logger().info('Starting HoloOcean bridge loop...')
        self._running = True
        # start main loop directly (blocking inside HoloOcean context)
        try:
            self._run_holoocean_loop()
        except Exception as e:
            self.get_logger().error('Exception in holoocean loop: %s' % str(e))
            self.get_logger().error(traceback.format_exc())
            raise

    # ✅ Add this callback:

    def cmd_callback(self, msg):
        self.command = list(msg.data)
        print("Received command:", self.command)

    def _stamp(self):
        # ROS2 builtin time
        return self.get_clock().now().to_msg()

    def _publish_camera(self, np_img):
        """
        np_img: H x W x 3 (uint8) or H x W x 4
        Publishes sensor_msgs/Image and CameraInfo.
        """
        try:
            if np_img is None:
                return
            # Ensure uint8 and 3 channels
            if np_img.dtype != np.uint8:
                # attempt normalization/convert
                np_img = np.clip(np_img, 0, 255).astype(np.uint8)

            # if 4 channels, drop alpha
            if np_img.ndim == 3 and np_img.shape[2] == 4:
                np_img = np_img[:, :, :3]

            # cv_bridge expects BGR by default. HoloOcean likely gives RGB.
            # We'll convert RGB -> BGR for cv2 encoding then specify rgb8 after swapping back.
            # Simpler: tell bridge encoding='rgb8' and pass numpy RGB directly.
            header = Header()
            header.stamp = self._stamp()
            header.frame_id = 'camera_link'

            img_msg = self.bridge.cv2_to_imgmsg(np_img, encoding='rgb8')
            img_msg.header.stamp = header.stamp
            img_msg.header.frame_id = header.frame_id
            self.camera_pub.publish(img_msg)

            # CameraInfo
            cam_info = CameraInfo()
            cam_info.header = img_msg.header
            cam_info.width = np_img.shape[1]
            cam_info.height = np_img.shape[0]
            cam_info.distortion_model = self.default_cam_info.distortion_model
            cam_info.k = self.default_cam_info.k
            cam_info.d = self.default_cam_info.d
            cam_info.r = self.default_cam_info.r
            cam_info.p = self.default_cam_info.p
            cam_info.binning_x = self.default_cam_info.binning_x
            cam_info.binning_y = self.default_cam_info.binning_y
            self.cam_info_pub.publish(cam_info)
        except Exception as e:
            self.get_logger().error("Failed to publish camera: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _publish_camera_down(self, np_img):
        """
        np_img: H x W x 3 (uint8) or H x W x 4
        Publishes sensor_msgs/Image and CameraInfo.
        """
        try:
            if np_img is None:
                return
            # Ensure uint8 and 3 channels
            if np_img.dtype != np.uint8:
                # attempt normalization/convert
                np_img = np.clip(np_img, 0, 255).astype(np.uint8)

            # if 4 channels, drop alpha
            if np_img.ndim == 3 and np_img.shape[2] == 4:
                np_img = np_img[:, :, :3]

            # cv_bridge expects BGR by default. HoloOcean likely gives RGB.
            # We'll convert RGB -> BGR for cv2 encoding then specify rgb8 after swapping back.
            # Simpler: tell bridge encoding='rgb8' and pass numpy RGB directly.
            header = Header()
            header.stamp = self._stamp()
            header.frame_id = 'camera2_link'

            img_msg = self.bridge.cv2_to_imgmsg(np_img, encoding='rgb8')
            img_msg.header.stamp = header.stamp
            img_msg.header.frame_id = header.frame_id
            self.cameraDown_pub.publish(img_msg)
            # self.get_logger().info("Published Down Camera Image")
            # self.get_logger().info(f"Image header: {img_msg.header.frame_id}")

            # CameraInfo
            # cam_info = CameraInfo()
            # cam_info.header = img_msg.header
            # cam_info.width = np_img.shape[1]
            # cam_info.height = np_img.shape[0]
            # cam_info.distortion_model = self.default_cam_info.distortion_model
            # cam_info.k = self.default_cam_info.k
            # cam_info.d = self.default_cam_info.d
            # cam_info.r = self.default_cam_info.r
            # cam_info.p = self.default_cam_info.p
            # cam_info.binning_x = self.default_cam_info.binning_x
            # cam_info.binning_y = self.default_cam_info.binning_y

            # self.cam_info_pub.publish(cam_info)
        except Exception as e:
            self.get_logger().error("Failed to publish camera: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _publish_depth(self, depth_data):
        """
        depth_data can be:
         - 2D numpy array -> publish as sensor_msgs/Image (32FC1 or 16UC1)
         - scalar (float) -> publish as std_msgs/Float32
        """
        try:
            if depth_data is None:
                return
            if isinstance(depth_data, (float, int, np.floating, np.integer)):
                msg = Float32()
                msg.data = float(depth_data)
                self.depth_scalar_pub.publish(msg)
                return

            depth_np = np.array(depth_data)
            if depth_np.ndim == 2:
                # publish as 32-bit float image
                # ensure float32
                depth_f = depth_np.astype(np.float32)
                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                img_msg.header.stamp = self._stamp()
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            elif depth_np.ndim == 3 and depth_np.shape[2] == 1:
                depth_f = depth_np[:, :, 0].astype(np.float32)
                img_msg = self.bridge.cv2_to_imgmsg(depth_f, encoding='32FC1')
                img_msg.header.stamp = self._stamp()
                img_msg.header.frame_id = 'depth_link'
                self.depth_image_pub.publish(img_msg)
            else:
                # fallback: try scalar conversion
                try:
                    val = float(depth_np[0])
                    msg = Float32()
                    msg.data = val
                    self.depth_scalar_pub.publish(msg)
                except Exception:
                    self.get_logger().warn("Unsupported depth shape: %s" % (str(depth_np.shape),))
        except Exception as e:
            self.get_logger().error("Failed to publish depth: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _publish_imu(self, imu_data):
        """imu_data expected as dict/array: ax,ay,az; gx,gy,gz; maybe orientation"""
        try:
            if imu_data is None:
                return
            msg = Imu()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'imu_link'
            # try various formats
            if isinstance(imu_data, dict):
                lin_acc = imu_data.get('linear_acceleration') or imu_data.get(
                    'acc') or imu_data.get('accelerometer')
                ang_vel = imu_data.get(
                    'angular_velocity') or imu_data.get('gyro')
                ori = imu_data.get('orientation')
            else:
                # maybe a numpy array [ax,ay,az, gx,gy,gz] or nested
                try:
                    arr = np.array(imu_data).flatten()
                    if arr.size >= 6:
                        lin_acc = arr[0:3].tolist()
                        ang_vel = arr[3:6].tolist()
                    else:
                        lin_acc = None
                        ang_vel = None
                    ori = None
                except Exception:
                    lin_acc = ang_vel = ori = None

            if lin_acc is not None:
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = [
                    float(x) for x in lin_acc]
            if ang_vel is not None:
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = [
                    float(x) for x in ang_vel]
            if ori is not None:
                # accept list/tuple [x,y,z,w] or dict
                if isinstance(ori, dict):
                    msg.orientation.x = float(ori.get('x', 0.0))
                    msg.orientation.y = float(ori.get('y', 0.0))
                    msg.orientation.z = float(ori.get('z', 0.0))
                    msg.orientation.w = float(ori.get('w', 1.0))
                else:
                    q = list(ori)
                    if len(q) >= 4:
                        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = [
                            float(x) for x in q[:4]]
            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().error("Failed to publish IMU: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _publish_mag(self, mag_data):
        try:
            if mag_data is None:
                return
            msg = MagneticField()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'mag_link'
            # accept sequences
            try:
                v = np.array(mag_data).flatten()
                msg.magnetic_field.x = float(v[0])
                msg.magnetic_field.y = float(v[1])
                msg.magnetic_field.z = float(v[2])
            except Exception:
                # if dict:
                if isinstance(mag_data, dict):
                    msg.magnetic_field.x = float(mag_data.get('x', 0.0))
                    msg.magnetic_field.y = float(mag_data.get('y', 0.0))
                    msg.magnetic_field.z = float(mag_data.get('z', 0.0))
            self.mag_pub.publish(msg)
        except Exception as e:
            self.get_logger().error("Failed to publish magnetometer: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _publish_dvl(self, dvl_data):
        """
        dvl_data: velocity vector or dict with 'vx','vy','vz'
        Publishes geometry_msgs/TwistStamped (linear velocity in m/s)
        """
        try:
            if dvl_data is None:
                return
            msg = TwistStamped()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'dvl_link'
            # try:
            v = np.array(dvl_data)[:3]
            msg.twist.linear.x = float(v[0])
            msg.twist.linear.y = float(v[1]) if v.size > 1 else 0.0
            msg.twist.linear.z = float(v[2]) if v.size > 2 else 0.0
            # except Exception:
            #     if isinstance(dvl_data, dict):
            #         msg.twist.linear.x = float(dvl_data.get('vx', 0.0))
            #         msg.twist.linear.y = float(dvl_data.get('vy', 0.0))
            #         msg.twist.linear.z = float(dvl_data.get('vz', 0.0))
            self.dvl_pub.publish(msg)

            dist = np.average(np.array(dvl_data)[3:])
            self.depth_distance_pub.publish(Float32(data=float(dist)))

        except Exception as e:
            self.get_logger().error("Failed to publish DVL: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _publish_range_finder(self, range_data):
        """
        range_data: scalar distance value (float)
        Publishes std_msgs/Float32
        """
        try:
            if range_data is None:
                return
            msg = Float32()
            msg.data = float(range_data)
            self.range_finder_pub.publish(msg)
        except Exception as e:
            self.get_logger().error("Failed to publish range finder: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _publish_location(self, loc_data):
        """
        loc_data expected like [x,y,z] plus maybe orientation or velocity.
        Publishes nav_msgs/Odometry (pose + optionally twist)
        """
        try:
            if loc_data is None:
                return
            msg = Odometry()
            msg.header.stamp = self._stamp()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            # accept dict or array
            if isinstance(loc_data, dict):
                pos = loc_data.get('position') or loc_data.get(
                    'pos') or loc_data.get('location')
                ori = loc_data.get('orientation')
                vel = loc_data.get('velocity')
            else:
                arr = np.array(loc_data)
                # heuristics
                if arr.size >= 3:
                    pos = arr[0:3].tolist()
                    vel = arr[3:6].tolist() if arr.size >= 6 else None
                    ori = None
                else:
                    pos = None
                    vel = None
                    ori = None

            if pos is not None:
                msg.pose.pose.position.x = float(pos[0])
                msg.pose.pose.position.y = float(pos[1])
                msg.pose.pose.position.z = float(pos[2])
            if ori is not None:
                if isinstance(ori, dict):
                    msg.pose.pose.orientation.x = float(ori.get('x', 0.0))
                    msg.pose.pose.orientation.y = float(ori.get('y', 0.0))
                    msg.pose.pose.orientation.z = float(ori.get('z', 0.0))
                    msg.pose.pose.orientation.w = float(ori.get('w', 1.0))
                else:
                    q = list(ori)
                    if len(q) >= 4:
                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = [
                            float(x) for x in q[:4]]
            if vel is not None:
                msg.twist.twist.linear.x = float(
                    vel[0]) if len(vel) > 0 else 0.0
                msg.twist.twist.linear.y = float(
                    vel[1]) if len(vel) > 1 else 0.0
                msg.twist.twist.linear.z = float(
                    vel[2]) if len(vel) > 2 else 0.0

            self.odom_pub.publish(msg)
        except Exception as e:
            self.get_logger().error("Failed to publish location/odom: %s" % str(e))
            self.get_logger().debug(traceback.format_exc())

    def _run_holoocean_loop(self):
        """
        Run HoloOcean loop inside context manager. Blocking until finished or step_limit reached.
        """
        # Import holoocean lazily to avoid import-time errors if not installed
        holoocean = self.holoocean_make_fn.__self__ if hasattr(
            self.holoocean_make_fn, '__self__') else None
        # But we were passed holoocean.make, so call it directly
        self.get_logger().info("Entering holoocean.make context...")
        steps = 0
        with self.holoocean_make_fn(scenario_cfg=self.scenario_cfg) as env:
            self.get_logger().info("HoloOcean env created.")
            try:
                while self._running:
                    if self.step_limit is not None and steps >= self.step_limit:
                        self.get_logger().info("Reached step limit (%d)." % self.step_limit)
                        break
                    state = env.step(self.command)
                    steps += 1

                    # Each sensor: attempt to read, then publish using helpers
                    try:
                        try:
                            # Camera: expect HxWxC (RGB)
                            cam = state.get("FrontCamera", None) if isinstance(
                                state, dict) else None
                            if cam is None:
                                # maybe state itself is an array? try attribute
                                cam = getattr(state, "FrontCamera", None)
                            if cam is not None:
                                cam_np = np.array(cam)
                                # If sensor returns HxWx4 RGBA or HxWx3
                                self._publish_camera(cam_np)
                        except Exception:
                            self.get_logger().debug("camera publish failed: %s" % traceback.format_exc())

                        try:
                            # Camera: expect HxWxC (RGB)
                            cam = state.get("DownCamera", None) if isinstance(
                                state, dict) else None
                            if cam is None:
                                # maybe state itself is an array? try attribute
                                cam = getattr(state, "DownCamera", None)
                            if cam is not None:
                                cam_np_2 = np.array(cam)
                                # If sensor returns HxWx4 RGBA or HxWx3
                                self._publish_camera_down(cam_np_2)
                        except Exception:
                            self.get_logger().debug("camera publish failed: %s" % traceback.format_exc())

                        # depth = compute_stereo_depth(state.get("DownCamera", None), state.get("FrontCamera", None), focal_length_px=256, baseline_meters=0.5)
                        # self._publish_depth(depth)

                    except Exception:
                        self.get_logger().debug("camera publish failed: %s" % traceback.format_exc())

                    try:
                        range_finder = state.get("RangeFinder", None) if isinstance(
                            state, dict) else getattr(state, "RangeFinder", None)
                        self._publish_range_finder(range_finder)
                    except Exception:
                        self.get_logger().debug("range finder publish failed: %s" % traceback.format_exc())

                    try:
                        dvl = state.get("DVLSensor", None) if isinstance(
                            state, dict) else getattr(state, "DVLSensor", None)
                        # print(dvl)
                        self._publish_dvl(dvl)
                    except Exception:
                        self.get_logger().debug("dvl failed: %s" % traceback.format_exc())

                    try:
                        imu = state.get("IMUSensor", None) if isinstance(
                            state, dict) else getattr(state, "IMUSensor", None)
                        self._publish_imu(imu)
                    except Exception:
                        self.get_logger().debug("imu failed: %s" % traceback.format_exc())

                    try:
                        depth = state.get("DepthSensor", None) if isinstance(
                            state, dict) else getattr(state, "DepthSensor", None)
                        # print(depth)
                        self._publish_depth(depth)
                    except Exception:
                        self.get_logger().debug("depth failed: %s" % traceback.format_exc())

                    try:
                        mag = state.get("MagnetometerSensor", None) if isinstance(
                            state, dict) else getattr(state, "MagnetometerSensor", None)
                        # print(mag)
                        self._publish_mag(mag)
                    except Exception:
                        self.get_logger().debug("mag failed: %s" % traceback.format_exc())

                    try:
                        loc = state.get("LocationSensor", None) if isinstance(
                            state, dict) else getattr(state, "LocationSensor", None)
                        current_velocity = vortex_field(loc)
                        env.set_ocean_currents('auv0', current_velocity)
                        self._publish_location(loc)
                    except Exception:
                        self.get_logger().debug("location failed: %s" % traceback.format_exc())

                    # small sleep is not necessary — env.step controls sim time — but let ROS spin once
                    rclpy.spin_once(self, timeout_sec=0.0)

            except KeyboardInterrupt:
                self.get_logger().info('KeyboardInterrupt - exiting holoocean loop.')
            except Exception as e:
                self.get_logger().error("Exception inside HoloOcean main loop: %s" % str(e))
                self.get_logger().error(traceback.format_exc())
            finally:
                self.get_logger().info("HoloOcean context exited after %d steps." % steps)


def main(args=None):
    import holoocean  # ensure holoocean is available
    rclpy.init(args=args)

    bridge_node = HoloOceanROS2Bridge(holoocean.make,  step_limit=None)

    try:
        # Keep node alive while holoocean loop runs inside constructor
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        bridge_node.get_logger().info("Shutting down via KeyboardInterrupt")
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
