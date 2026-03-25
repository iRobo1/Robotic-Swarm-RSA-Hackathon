import rclpy
from rclpy.node import Node 
import requests
import math
from sensor_msgs.msg import Image
import apriltag
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import library.utils as utils

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class Detector:
    def __init__(self, is_mirte_master):
        self.bridge = CvBridge()
        self.april_tag_option = apriltag.DetectorOptions(families="tag36h11")
        self.april_tag_detector = apriltag.Detector(self.april_tag_option)
        self.latest_image = None
        self.topic = "/camera/color/image_raw" if is_mirte_master else "/video1/image_raw"
        self.node = Node("detector")
        self.img_subscriber = self.node.create_subscription(Image,
                                                            self.topic,
                                                            self._on_receive_image,
                                                            10,
                                                            callback_group=MutuallyExclusiveCallbackGroup())

        # self._start_thread()

    def detect_objective_tags(self):
        """
        Detect AprilTags in the provided image.

        Example:
            tags = detector.detect_objective_tags()
            if tags:
                for tag in tags:
                    # tag.tag_id (int): ID of the detected tag
                    # tag.center (tuple[float, float]): X, Y coordinates of tag center
                    # tag.corners (numpy.ndarray): 4x2 array of tag corners
                    # tag.hamming (int): Number of corrected bits
                    # tag.decision_margin (float): Confidence of detection
                    print(f"Tag ID: {tag.tag_id}, Center: {tag.center}")
                    communication.send_objective(utils.get_team_from_tag_id(tag.tag_id), tag.tag_id)

        Returns:
            list:
                - List of detected AprilTag objects
                - Empty list if no image is available
        """
        if self.latest_image is None:
            print("No image available")
            return []

        gray_img = cv2.cvtColor(self.latest_image, cv2.COLOR_RGB2GRAY)
        tags_detected = self.april_tag_detector.detect(gray_img)

        return tags_detected
    

    def _start_thread(self):
        self._thread = threading.Thread(
            target=lambda: self._spin_node(),
            daemon=True
        )
        self._thread.start()


    def _spin_node(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)


    def _on_receive_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        tags = self.detect_objective_tags()
        if tags:
            for tag in tags:
                self.node.get_logger().info(f"Tag {tag.tag_id}, within distance: {utils.is_tag_within_distance(tag)}") # type: ignore