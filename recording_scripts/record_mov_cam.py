#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.bridge = CvBridge()

        # Subscription to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/cam_rotation',
            self.image_callback,
            10
        )

        # VideoWriter setup
        self.out = None
        self.frame_width = None
        self.frame_height = None
        self.fps = 30  # Set desired FPS
        self.filename = 'output.mp4'
        self.codec = cv2.VideoWriter_fourcc(*'mp4v')  # Best for MP4

        self.get_logger().info("Video recorder node started. Saving to 'output.mp4'.")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.out is None:
            # Initialize VideoWriter after receiving first frame
            self.frame_height, self.frame_width, _ = cv_image.shape
            self.out = cv2.VideoWriter(
                self.filename,
                self.codec,
                self.fps,
                (self.frame_width, self.frame_height)
            )

        # Write frame to video
        self.out.write(cv_image)

    def destroy_node(self):
        if self.out is not None:
            self.out.release()
            self.get_logger().info("Video saved successfully.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = VideoRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


## ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.25}}"
