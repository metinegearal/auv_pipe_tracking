import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
import segmentation_models_pytorch as smp
from sensor_msgs.msg import Image, MagneticField
from std_msgs.msg import Bool, Float32
import torch
import time


class ObjectSegmentation(Node):

    def __init__(self):
        super().__init__('object_segmentation_node')

        # --- 1. SETUP NEURAL NETWORK ONCE ---
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Loading UNet model on: {self.device}')

        self.model = smp.Unet(
            encoder_name='resnet34',
            encoder_weights=None,
            in_channels=3,
            classes=1,
        )

        weight_path = (
            '/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/'
            'pipe_track_ros2/src/pipe_track_perception/'
            'pipe_track_perception/models/segment/best_pipe_unet35.pth'
        )
        self.model.load_state_dict(torch.load(
            weight_path, map_location=self.device))
        self.model.to(self.device)  # Force model to GPU
        self.model.eval()  # Set to evaluation mode

        # Pre-allocate kernel for OpenCV morphology to save CPU cycles
        self.morph_kernel = np.ones((10, 10), np.uint8)

        # --- 2. ROS SETUP ---
        self.bridge = CvBridge()
        self.sub_cam = self.create_subscription(
            Image, 'holocean/cameraDown/image_raw', self.cam_callback, 1)
        self.mask_pub = self.create_publisher(Image, 'object/mask', 10)

        self.magnet = [0, 0, 0]
        self.yaw = 0.0
        self.is_not_started = True  # Flag to check if yaw has been received yet
        self.sub_mag = self.create_subscription(
            MagneticField, 'holocean/mag', self.magnetometer_callback, 10)
        self.pub_mag = self.create_publisher(Float32, 'perception/yaw', 10)

        self.record_pub = self.create_publisher(Bool, '/memory/record', 10)
        self.isRecording = False
        self.is_stopped = False

        self.get_logger().info('✅ segmentation_node Started')

    def magnetometer_callback(self, msg):
        self.magnet = [msg.magnetic_field.x,
                       msg.magnetic_field.y, msg.magnetic_field.z]
        ya = np.arctan2(self.magnet[0], self.magnet[1]) - (np.pi / 2)
        if ya < -1 * np.pi:
            ya += np.pi * 2
        self.yaw = float(ya)
        self.is_not_started = False  # Mark as started once yaw is received

    def maskeImg(self, frame):
        # 1. Fast OpenCV resize
        resized = cv2.resize(frame, (288, 288))

        # 2. Convert directly to Tensor (HWC to CHW), normalize to 0-1, and send to GPU
        tensor = torch.from_numpy(resized).permute(
            2, 0, 1).float().unsqueeze(0) / 255.0
        tensor = tensor.to(self.device)

        # 3. Predict using inference_mode (faster than no_grad)
        with torch.inference_mode():
            output = torch.sigmoid(self.model(tensor))

        # 4. Extract safely to CPU
        probs = output.squeeze().cpu().numpy()
        mask = (probs > 0.75).astype(np.uint8) * 255

        if np.any(mask):
            confidence = probs[probs > 0.75].mean()
        else:
            confidence = 0.0

        return mask, confidence

    def cam_callback(self, msg):
        if self.is_stopped or self.is_not_started:
            print('Segmentation stopped, ignoring frame.')
            return

        # Forward the most recent yaw synchronized with this frame
        self.pub_mag.publish(Float32(data=self.yaw))

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        start_time = time.perf_counter()
        mask, confidence = self.maskeImg(frame)

        torch.cuda.synchronize() 
    
        # 3. Calculate internal latency
        inference_time_ms = (time.perf_counter() - start_time) * 1000.0
        # self.get_logger().info(f'Inference time: {inference_time_ms:.2f} ms')

        h, w = mask.shape
        filness_ratio = np.sum(mask > 0) / (w * h)
        if filness_ratio < 0.0025:
            self.get_logger().warn(
                f'Low pipe detection confidence, ignoring frame. '
                f'filness_ratio={filness_ratio:.5f}'
            )
            return

        # Use pre-allocated kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            mask2 = np.zeros_like(mask)
            largest_contour = max(contours, key=cv2.contourArea)
            mask = cv2.drawContours(
                mask2, [largest_contour], -1, 255, thickness=cv2.FILLED)

        mask_img = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_img.header = msg.header
        self.mask_pub.publish(mask_img)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
