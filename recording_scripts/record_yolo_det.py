#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class InferenceVideoRecorder(Node):
    def __init__(self):
        super().__init__('inference_video_recorder')
        self.bridge = CvBridge()

        # Subscribe to YOLO inference output (already annotated with bboxes)
        self.subscription = self.create_subscription(
            Image,
            '/inference_result',
            self.image_callback,
            10
        )

        # VideoWriter setup
        self.out = None
        self.frame_width = None
        self.frame_height = None
        self.fps = 30
        self.filename = 'inference_result.mp4'
        self.codec = cv2.VideoWriter_fourcc(*'mp4v')

        self.get_logger().info("Inference recorder started. Saving to 'inference_result.mp4'.")

    def image_callback(self, msg: Image):
        # Let CvBridge use the original encoding
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if self.out is None:
            self.frame_height, self.frame_width, _ = cv_image.shape
            self.out = cv2.VideoWriter(
                self.filename,
                self.codec,
                self.fps,
                (self.frame_width, self.frame_height)
            )

        # Write annotated frame directly
        self.out.write(cv_image)


    def destroy_node(self):
        if self.out is not None:
            self.out.release()
            self.get_logger().info("Video saved successfully.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = InferenceVideoRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
