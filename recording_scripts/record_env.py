#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class MultiVideoRecorder(Node):
    def __init__(self):
        super().__init__('multi_video_recorder')
        self.bridge = CvBridge()

        # Subscriptions to both camera topics
        self.sub_left = self.create_subscription(
            Image,
            '/cam_left',
            self.left_callback,
            10
        )
        self.sub_gripper = self.create_subscription(
            Image,
            '/cam_gripper',
            self.gripper_callback,
            10
        )

        # Video writer attributes
        self.out_left = None
        self.out_gripper = None

        self.frame_width_left = None
        self.frame_height_left = None
        self.frame_width_gripper = None
        self.frame_height_gripper = None

        self.fps = 30
        self.codec = cv2.VideoWriter_fourcc(*'mp4v')

        self.filename_left = 'cam_left.mp4'
        self.filename_gripper = 'cam_gripper.mp4'

        self.get_logger().info("Multi-camera recorder started. Saving to 'cam_left.mp4' and 'cam_gripper.mp4'.")

    def left_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.out_left is None:
            # Initialize video writer for left camera
            self.frame_height_left, self.frame_width_left, _ = cv_image.shape
            self.out_left = cv2.VideoWriter(
                self.filename_left,
                self.codec,
                self.fps,
                (self.frame_width_left, self.frame_height_left)
            )

        self.out_left.write(cv_image)

    def gripper_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.out_gripper is None:
            # Initialize video writer for gripper camera
            self.frame_height_gripper, self.frame_width_gripper, _ = cv_image.shape
            self.out_gripper = cv2.VideoWriter(
                self.filename_gripper,
                self.codec,
                self.fps,
                (self.frame_width_gripper, self.frame_height_gripper)
            )

        self.out_gripper.write(cv_image)

    def destroy_node(self):
        if self.out_left is not None:
            self.out_left.release()
            self.get_logger().info("Left camera video saved successfully.")
        if self.out_gripper is not None:
            self.out_gripper.release()
            self.get_logger().info("Gripper camera video saved successfully.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = MultiVideoRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down...")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
