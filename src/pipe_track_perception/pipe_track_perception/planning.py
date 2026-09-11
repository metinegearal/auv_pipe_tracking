
from datetime import datetime, timedelta
import math
import sys

from anyio import sleep

sys.path.append('/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem')

from Tools.explore_2d_dvl import Explore2D
from Tools.Segmentation.pipeMemory import MagnetometerPathFinder
from Tools.Segmentation.newPipeTrack import get_clean_pipe_navigation, get_weighted_pipe_navigation
from Tools.dataManagement.dataStorage import objectDatabase
import sensorSet

import torch

# from Tools.explore2D import get_world_coordinates_from_mask
# from Tools.DecodeSegmentetion import segmentAnomaly
from Tools.mapping2D import point_from_depth, rotate_point_2d
# from ultralytics import YOLO

# Load a pretrained YOLO11n model
#!/usr/bin/env python3
from std_msgs.msg import Float32, Header, Float32MultiArray
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, Imu, MagneticField, FluidPressure
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32, Bool, String
from nav_msgs.msg import Odometry
import numpy as np
import pandas as pd
import cv2
from cv_bridge import CvBridge
# from myMessages.msg import objectDetected2D

from collections import deque


import segmentation_models_pytorch as smp
import torch
from PIL import Image as PILImage
from torchvision import transforms
import json

def maskeImg(img):
    # Recreate the same model structure
    model = smp.Unet(
    encoder_name="resnet34",
    encoder_weights=None,  # Use None when loading weights manually
    in_channels=3,
    classes=1,
    )

    # Load saved weights
    model.load_state_dict(torch.load("models/segment/best_pipe_unet35.pth", map_location="cuda" if torch.cuda.is_available() else "cpu"))
    model.eval()



    # Same transform used during training
    transform = transforms.Compose([
        transforms.Resize((288, 288)),
        transforms.ToTensor(),
    ])

    # Load image
    # img = Image.open(img).convert("RGB")
    img = PILImage.fromarray(img).convert("RGB")
    input_tensor = transform(img).unsqueeze(0)  # Add batch dimension

    # Predict
    with torch.no_grad():
        output = torch.sigmoid(model(input_tensor))  # Sigmoid for binary
    
        # 1. Extract the raw probabilities safely to CPU memory ONCE
        probs = output.squeeze().cpu().numpy()
        
        # 2. Generate the binary mask
        mask = (probs > 0.75).astype(np.uint8) * 255
        
        # 3. Calculate actual confidence: Average probability of the detected pixels
        if np.any(mask): # Make sure we actually found something to avoid dividing by zero!
            confidence = probs[probs > 0.75].mean()
        else:
            confidence = 0.0 # No pipe found, zero confidence
            
    return mask,confidence

# def detectPipes(img, old_center=(144,144), thresholdArea=30, mask=None):
#     if mask is None:
#         mask = maskeImg(img)
#     _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

#     # Find contours
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     if contours:
#         candidates = []
#         for contour in contours:
#             if cv2.contourArea(contour) < thresholdArea:
#                 continue  # skip small noise

#             rect = cv2.minAreaRect(contour)
#             (x, y), (w, h), angle = rect
#             center = (int(x), int(y))

#             # Normalize angle
#             if w < h:
#                 angle += 90

#             candidates.append({
#                 "center": center,
#                 "size": (w, h),
#                 "angle": angle,
#                 "rect": rect,
#                 "distance": np.linalg.norm(np.array(center) - np.array(old_center))
#             })

#         if not candidates:
#             return False, "No valid pipes found!"

#         # Sort by distance to old_center
#         best = min(candidates, key=lambda c: c["distance"])

#         # Draw result
#         result = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#         box = cv2.boxPoints(best["rect"])
#         box = np.intp(box)
#         cv2.drawContours(result, [box], 0, (0, 255, 0), 2)
#         cv2.circle(result, best["center"], 5, (0, 0, 255), -1)

#         print(f"Pipe center: {best['center']}")
#         print(f"Pipe angle: {best['angle']:.2f}°")

#         return True, [best["center"], best["size"], best["angle"], result, best["rect"]]

# def calculate_angle_from_center_to_pixel(px, fov=90, width=288):
#     """
#     Calculate the angle from the center of the image to a given pixel.
    
#     fov: Field of View in degrees (horizontal angle from left to right)
#     width: Width of the image in pixels
#     height: Height of the image in pixels
#     px: x-coordinate of the pixel
#     py: y-coordinate of the pixel
    
#     Returns the angle in degrees from the center of the image to the pixel
#     """
    
#     # Convert FOV to radians (if necessary, otherwise leave as degrees)
#     fov_rad = np.radians(fov)  # We use radians for the calculations
    
#     # Calculate the center of the image
#     center_x = width / 2
    
#     # Calculate the angle per pixel
#     angle_per_pixel = fov_rad / width
    
#     # Calculate the horizontal angle to the pixel
#     angle_to_pixel = (px - center_x) * angle_per_pixel  # In radians
    
#     # Convert the angle back to degrees for user-friendly output
#     # angle_to_pixel_deg = np.degrees(angle_to_pixel)
    
#     return angle_to_pixel

 
# def calculateArea(bbox, depth, nav, angleYaw):
#     x_min, y_min, x_max, y_max = bbox
#     x,y=nav[:2]
#     center_x= (x_min + x_max) / 2
#     center_y= (y_min + y_max) / 2

#     angleX1=calculate_angle_from_center_to_pixel(x_min)
#     angleX2=calculate_angle_from_center_to_pixel(x_max)
#     width= np.tan(angleX2)*depth - np.tan(angleX1)*depth
    
#     angleY1=calculate_angle_from_center_to_pixel(y_min)
#     angleY2=calculate_angle_from_center_to_pixel(y_max)
#     height= np.tan(angleY2)*depth - np.tan(angleY1)*depth

#     x1,y1=rotate_point_2d((np.tan(angleX1)*depth, np.tan(angleY1)*depth),angleYaw)
#     x2,y2=rotate_point_2d((np.tan(angleX2)*depth, np.tan(angleY2)*depth),angleYaw)

#     area=[(x1+x,x2+x),(y1+y,y2+y)]
#     return area


def angle_to_waypoint(angle_rad, distance=5):
    """
    Convert an angle and distance to a waypoint in Cartesian coordinates.
    
    angle_rad: Angle in radians from the forward direction (0 = straight ahead, positive = right, negative = left)
    distance: Distance from the current position to the waypoint
    
    Returns a tuple (x, y) representing the waypoint coordinates relative to the current position.
    """
    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)
    return (x, y)

class objectSegmentation(Node):
    def __init__(self, storage=None):
        super().__init__('object_segmentation_node')

        self.explorer=Explore2D()

        self.bridge = CvBridge()
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.oldCenter=(144,144)
        self.yaw=0

        # --- Base Parameters ---
        self.look_ahead_pixels = 100
        self.angle_weight = 0.5
        # --- Auto-Tuner Subscriber ---
        self.create_subscription(String, '/pipeline/metrics', self.tune_vision_cb, 10)

        self.is_stopped=0
        self.pipe_history = deque(maxlen=150)

        self.movement_history = [] # Store recent movement commands for turn-around detection
        self.current_height=0
        self.startSegment=False
        # self.storage=objectDatabase() if storage is None else storage
        self.finder = MagnetometerPathFinder()
        self.finder.global_arrival_angle_rad = np.deg2rad(180)

        # self.sub_cam = self.create_subscription(Image, 'camera/image_processed', self.cam_callback, 10)
        self.stop_sub = self.create_subscription(Float32MultiArray, '/motion/report', self.stop_callback, 10)
        self.sub_cam = self.create_subscription(Image, 'holocean/cameraDown/image_raw', self.cam_callback, 10)
        self.position_sub = self.create_subscription(Odometry, 'holocean/odom', self.position_callback, 10)
        self.height_sub = self.create_subscription(Float32, 'holocean/depth/distance', self.depth_callback, 10)
        self.sub_mag = self.create_subscription(MagneticField, 'holocean/mag', self.magnetometer_callback, 10)

        self.pinger_angle_sub = self.create_subscription(Float32, '/signal/pingerAngle', self.pinger_callback, 10)
        self.pinger_angle=None

        self.object_pub = self.create_publisher(Image, 'object/mask', 10)
        self.center_pub = self.create_publisher(Float32MultiArray, 'object/center', 10)
        self.waypoint_pubOld = self.create_publisher(Float32MultiArray, 'object/areaWaypoints', 10)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, '/trajectory/waypoint', 10)

        self.record_pub = self.create_publisher(Bool, '/memory/record', 10)
        self.isRecording=False
        self.play_pub = self.create_publisher(String, '/memory/play', 10)

        self.get_logger().info("✅ segmentation_node Started")

    def tune_vision_cb(self, msg):
        metrics = json.loads(msg.data)
        jitter = metrics['jitter']
        cte = metrics['cte']
        
        self.get_logger().info(f"Auto-Tuning Vision... LookAhead: {self.look_ahead_pixels}, AngleWeight: {self.angle_weight:.3f}")

        # 1. Tune for Stability (Jitter)
        if jitter > 10.0:
            # Point is bouncing around. Look closer to the robot and favor straight lines.
            self.look_ahead_pixels -= 10
            self.angle_weight += 0.2
        elif jitter < 2.0:
            # Very stable, we can afford to relax the angle constraint slightly
            self.angle_weight -= 0.05

        # 2. Tune for Curve Tracking (CTE)
        if cte > 40.0 and jitter < 15.0:
            # We are drifting off the pipe, but vision is stable. Look further ahead to anticipate the curve.
            self.look_ahead_pixels += 10

        # 3. CLAMPING (Safety Limits)
        self.look_ahead_pixels = int(np.clip(self.look_ahead_pixels, 40, 200))
        self.angle_weight = float(np.clip(self.angle_weight, 0.1, 3.0))

    def stop_callback(self,msg):
        self.startSegment=True

    def pinger_callback(self, msg):
        angle = msg.data
        self.pinger_angle = np.deg2rad(angle)

    def goByPinger(self):
        if self.pinger_angle is not None:
            self.movement_history.append([self.pinger_angle, datetime.now()])
            waypoint_offset = angle_to_waypoint(self.pinger_angle, distance=5)
            waypoint = waypoint_offset
            self.waypoint_pub.publish(Float32MultiArray(data=[waypoint[0], waypoint[1], -28]))

    def returnBack(self):
        self.record_pub.publish(Bool(data=False))  # Stop recording to save the current path
        self.play_pub.publish(String(data="reverse"))
        self.get_logger().info("⏪ Commanded robot to return back along the path!")

    def position_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_position = np.array([pos.x, pos.y, pos.z])
        # print("Current Position:", pos.x, pos.y, pos.z)

    def depth_callback(self, msg):
        self.current_height = msg.data

    def magnetometer_callback(self, msg):
        self.magnet = [msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z]
        ya=np.arctan2(self.magnet[0], self.magnet[1])-(np.pi/2) 
        if ya < -1*np.pi:
            ya+=np.pi*2  
        self.yaw = ya

    def get_angle_diff(self, target_angle, current_angle):
        """Returns the shortest difference between two angles in degrees"""
        return (current_angle - target_angle + 180) % 360 - 180

    def cam_callback(self, msg):
        if self.is_stopped:
            print("Segmentation stopped, ignoring frame.")
            return # Ignore frames if we already triggered the stop

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # cv2.imwrite(f"/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/results/input_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg", frame)
        frame=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
        


        w,h=(288,288)

        mask, confidence = maskeImg(frame)
        # cv2.imwrite(f"/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/results/mask_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg", mask)
        h, w = mask.shape
        # filness ratio: detected pipe pixels / total pixels in the image
        filness_ratio = np.sum(mask > 0) / (w * h)
        if filness_ratio < 0.0025:  # If less than 1% of the image is detected as pipe, consider it a false positive
            if not self.isRecording:
                self.record_pub.publish(Bool(data=True)) 
                self.isRecording=True
            self.get_logger().warn("Low pipe detection confidence, ignoring frame.")
            self.goByPinger()
            return
        
        # collect all points from mask as (x, y, detected) where detected is 1 if pixel belongs to pipe else 0
        # points = []
        # for y in range(h):
        #     for x in range(w):
        #         detected = 1 if mask[y, x] > 0 else 0
        #         points.append((x, y, detected))
        # self.explorer.add_points(points, img_size=(w, h), depth=self.current_height, position=self.current_position[:2], yaw=self.yaw)
        # self.explorer.update_final_map()

        #get biggest area
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            mask2 = np.zeros_like(mask)
            largest_contour = max(contours, key=cv2.contourArea)
            mask = cv2.drawContours(mask2, [largest_contour], -1, 255, thickness=cv2.FILLED)

        ys_idx, xs_idx = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')  # (h,w)
        detected = (mask > 0).astype(np.uint8)  # 1 where pipe, 0 elsewhere
        points = list(zip(xs_idx.ravel(), ys_idx.ravel(), detected.ravel()))
        # ^ still a list of (x,y,obj) tuples, so add_points API is unchanged

        self.explorer.add_points(
            points,
            img_size=(w, h),
            depth=self.current_height,
            position=self.current_position[:2],
            yaw=self.yaw,
        )

        points_final = self.explorer.get_scan_point()
        point=points_final[0] if points_final else self.current_position[:2]  # Fallback to current position if no points detected
        point=(point[0]*self.explorer.MAP_RESOLUTION, point[1]*self.explorer.MAP_RESOLUTION)



        angle,center,skel_img,weight_map = get_weighted_pipe_navigation(mask, look_ahead_pixels=self.look_ahead_pixels, angle_weight=self.angle_weight)
        # cv2.imwrite(f"/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/results/skel_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg", skel_img)
        
        # angle,center,_ = self.finder.get_target(mask, np.degrees(self.yaw), look_ahead_dist=50)
        # print(f"Calculated Angle: {angle:.2f} degrees")
        print(f"Confidence: {confidence:.2f}")


        cameraPosition=rotate_point_2d(sensorSet.CameraPositions[1][:2],self.yaw) #rotate to world coordinates
        cameraPosition.append(0) #z is 0 for camera position

        # self.storage.add_detected_object({
        #         'center': center,
        #         "angle": self.yaw,
        #         "angles": (self.yaw, -90),
        #         # 'area': area,
        #         'nav_point': (np.array(self.current_position)+np.array(cameraPosition)).tolist(),
        #         'yaw': self.yaw,
        #         'cameraNum': 1,
        #         "color": None,
        #         "tag": "pipe"
        #     }, 1)
        
        self.center_pub.publish(Float32MultiArray(data=[center[0], center[1]]))
        point=point_from_depth(center, self.yaw, 15, True)
        # print(points_final)
        # point=points_final[0] if points_final else self.current_position[:2]  # Fallback to current position if no points detected
        # point=(point[0]*self.explorer.MAP_RESOLUTION, point[1]*self.explorer.MAP_RESOLUTION) # convert back to world coordinates
        print(f"Publishing waypoint: {point[0]:.2f}, {point[1]:.2f}, -29")
        self.waypoint_pub.publish(Float32MultiArray(data=[point[0], point[1], -29]))

        self.pipe_history.append((point[0], point[1]))


        # 3. CHECK FOR TURN-AROUND
        # Only check if we have enough history to form a reliable path vector
        # if len(self.pipe_history) >= 150:
        #     if center[1]>h//2+10: # If the detected center is significantly below the image center, it may indicate a turn-around
        #         self.get_logger().warn("Turn-around detected! Stopping segmentation.")
        #         self.is_stopped += 1  # Set the flag to stop further processing
        #         # if self.is_stopped == 2:
        #         self.returnBack()  # Command the robot to return back along the path
        #         self.pipe_history.clear()  # Clear history to reset detection

        # debug_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        debug_img = cv2.cvtColor(skel_img, cv2.COLOR_GRAY2BGR)

        # Merkezi işaretle (Mavi)
        cv2.circle(debug_img, (w//2, h//2), 5, (255, 0, 0), -1) 
        # Hedef noktayı işaretle (Yeşil)
        cv2.circle(debug_img, center, 5, (0, 255, 0), -1)
        # Yön çizgisini çiz (Sarı)
        cv2.line(debug_img, (w//2, h//2), center, (0, 255, 255), 2) 
        
        cv2.imwrite(f"/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/results/debug_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg", debug_img)
        cv2.imshow("Segmentation Debug", debug_img)
        cv2.waitKey(1)






        mask_img = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_img.header = msg.header  # Preserve the original header for timestamp and frame_id
        self.object_pub.publish(mask_img)

        # if self.startSegment:
        #     mask=segmentAnomaly(frame)
        #     wps=get_world_coordinates_from_mask(mask, navPosition=self.current_position[:2], depth=self.current_height, yaw_angle=self.yaw)
        #     if wps:
        #         self.waypoint_pub.publish(Float32MultiArray(data=wps))
        # ok,[center, size, angle, result, rect] = detectPipes(frame, self.oldCenter, mask=mask)
        # if ok:
        #     cv2.imshow("Mask", result)
        #     cv2.waitKey(1)

        #     box = cv2.boxPoints(rect)
        #     box = np.intp(box)
            
        #     # Calculate the axis-aligned bounding box (bbox) using the rotated box
        #     x_min = np.min(box[:, 0])  # Minimum x-coordinate
        #     x_max = np.max(box[:, 0])  # Maximum x-coordinate
        #     y_min = np.min(box[:, 1])  # Minimum y-coordinate
        #     y_max = np.max(box[:, 1])  # Maximum y-coordinate

        #     # The new bounding box in the same format as the original code
        #     bbox = (x_min, y_min, x_max, y_max)
        #     area=calculateArea(bbox, self.current_height, self.current_position, self.yaw)
        #     self.get_logger().info(f"Detected pipe area waypoints: {area}")
        #     self.waypoint_pub.publish(Float32MultiArray(data=[area[0][0], area[1][0], area[0][1], area[1][1]]))


def main(args=None):
    rclpy.init(args=args)
    node = objectSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
