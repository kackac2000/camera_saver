import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import numpy as np


class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')

        # Create an instance of CvBridge
        self.bridge = CvBridge()

        # Declare parameters for topics
        self.camera1_topic = self.declare_parameter('camera1_topic', '/cam_ta1_ws2/color/image_raw/compressed').value
        self.camera2_topic = self.declare_parameter('camera2_topic', '/cam_ta2_ws2/color/image_raw/compressed').value
        self.camera3_topic = self.declare_parameter('camera3_topic', '/cam_ws2_box/color/image_raw/compressed').value

        # Subscribe to three camera topics
        self.camera1_sub = self.create_subscription(CompressedImage, self.camera1_topic, self.camera1_callback, 10)
        self.camera2_sub = self.create_subscription(CompressedImage, self.camera2_topic, self.camera2_callback, 10)
        self.camera3_sub = self.create_subscription(CompressedImage, self.camera3_topic, self.camera3_callback, 10)

        # Initialize variables to hold image data and timestamps
        self.image1 = None
        self.image2 = None
        self.image3 = None
        self.timestamp1 = None
        self.timestamp2 = None
        self.timestamp3 = None

        # Create a directory to save images if it doesn't exist
        self.image_dir = 'saved_images_from_Ultimate_dataset'
        if not os.path.exists(self.image_dir):
            os.mkdir(self.image_dir)

    def camera1_callback(self, msg):
        self.image1 = self.decode_compressed_image(msg)
        self.timestamp1 = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # Use float seconds
        self.get_logger().info(f"Camera_ta1_ws2: Received image at {self.timestamp1}")
        self.save_images_if_ready()

    def camera2_callback(self, msg):
        self.image2 = self.decode_compressed_image(msg)
        self.timestamp2 = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.get_logger().info(f"Camera_ta2_ws2: Received image at {self.timestamp2}")
        self.save_images_if_ready()

    def camera3_callback(self, msg):
        self.image3 = self.decode_compressed_image(msg)
        self.timestamp3 = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.get_logger().info(f"Camera_ws2_box: Received image at {self.timestamp3}")
        self.save_images_if_ready()

    def decode_compressed_image(self, msg):
        # Convert CompressedImage ROS message to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def save_images_if_ready(self):
        if self.image1 is not None and self.image2 is not None and self.image3 is not None:
            min_height = min(self.image1.shape[0], self.image2.shape[0], self.image3.shape[0])
            self.image1 = self.image1[:min_height, :]
            self.image2 = self.image2[:min_height, :]
            self.image3 = self.image3[:min_height, :]
            combined_image = np.hstack((self.image1, self.image2, self.image3))

            if combined_image is not None and combined_image.size > 0:
                # Use timestamp1 (float) for naming the combined image, adjusted by custom timestamp of the first image
                adjusted_timestamp = float(self.timestamp1) - 1730901644.830837 
                filename = os.path.join(self.image_dir, f'{adjusted_timestamp:.6f}.jpg')  # Save with high precision
                cv2.imwrite(filename, combined_image)

                self.get_logger().info(f"Saved combined image to {filename}")
                with open(os.path.join(self.image_dir, 'timestamps.csv'), 'a') as file:
                    file.write(f"{adjusted_timestamp},{self.timestamp1},{self.timestamp2},{self.timestamp3}\n")

                # Reset images after saving
                self.image1 = None
                self.image2 = None
                self.image3 = None
            else:
                self.get_logger().warn("Combined image is empty or invalid.")


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()