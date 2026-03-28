import time
import rclpy
from rclpy.node import Node 
import math
import numpy as np
from queue import Queue
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range # For sonar
from mirte_msgs.srv import GetRange # For sonar

from library.utils import angle_difference, clamp

MAX_SONAR_RANGE = 4.5
MIN_SONAR_RANGE = 0.05

class Robot:
    def __init__(self):    
        # control parameters
        self.kp_linear = 0.6
        self.kp_angular = 2.5

        self.max_linear_speed = 0.4
        self.max_angular_speed = 0.8

        # Sonar value
        self.range_left = 0
        self.range_right = 0
        
        self.linear_threshold = 0.1
        self.angular_threshold = 0.1

        self.stop_driving_forward_angle = math.pi / 2
        self.slower_driving_forward_angle = 0.4

        self.poll_freq = 0.01
        
        self.node = Node(f"robot")

        # Last integer is keep last 10 messages
        self.pub_motor_left = self.node.create_publisher(Int32, '/io/motor/left/speed', 10)
        self.pub_motor_right = self.node.create_publisher(Int32, '/io/motor/right/speed', 10)

        # /io/distance/left/get_range
        self.sub_sonar_left = self.node.create_subscription(Range, '/io/distance/left', self.on_range_left, 10)
        self.sub_sonar_right = self.node.create_subscription(Range, '/io/distance/right', self.on_range_right, 10)

        # Now use a service for the sonar instead of a subscriber. We can skip spin_once() then
        # NVM didn't work that well
        # self.client_range_left = self.node.create_client(GetRange, '/io/distance/left/get_range')
        # self.client_range_right = self.node.create_client(GetRange, '/io/distance/right/get_range')
        # self.client_range_left.wait_for_service(timeout_sec=5.0)
        # self.client_range_right.wait_for_service(timeout_sec=5.0)
    
    def drive(self, right_motor_speed, left_motor_speed):
        """
        Detect AprilTags in the provided image.

        Example:
            tags = detector.detect_objective_tag()
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

        msg_right = Int32()
        msg_left = Int32()

        msg_right.data = right_motor_speed
        msg_left.data = left_motor_speed
        
        self.pub_motor_left.publish(msg_left)
        self.pub_motor_right.publish(msg_right)
    
    def on_range_left(self, msg):
        # print(msg.range)
        if msg.range > MAX_SONAR_RANGE:
            self.range_left = MAX_SONAR_RANGE
        elif msg.range < MIN_SONAR_RANGE:
            self.range_left = MIN_SONAR_RANGE
        else:
            self.range_left = msg.range

    def on_range_right(self, msg):
        if msg.range > MAX_SONAR_RANGE:
            self.range_right = MAX_SONAR_RANGE
        elif msg.range < MIN_SONAR_RANGE:
            self.range_right = MIN_SONAR_RANGE
        else:
            self.range_right = msg.range

    # def get_range_left(self):
    #     future = self.client_range_left.call_async(GetRange.Request())
    #     rclpy.spin_until_future_complete(self.node, future)
    #     return future.result().range.range  # outer .range = response field, inner .range = float distance

    # def get_range_right(self):
    #     future = self.client_range_right.call_async(GetRange.Request())
    #     rclpy.spin_until_future_complete(self.node, future)
    #     return future.result().range.range

    # def get_range_left(self):
    #     future = self.client_range_left.call_async(GetRange.Request())
    #     while not future.done():
    #         rclpy.spin_once(self.node, timeout_sec=0.1)
    #     return future.result().range.range

    # def get_range_right(self):
    #     future = self.client_range_right.call_async(GetRange.Request())
    #     while not future.done():
    #         rclpy.spin_once(self.node, timeout_sec=0.1)
    #     return future.result().range.range



### Mirte pioneer seems to not work too well
    # def orient_to(self, target_orientation):   
    #     # msg = Twist()
    #     # #     time.sleep(self.poll_freq)

    #     # orient in the desired direction
    #     while True:
    #         rclpy.spin_once(self.node, timeout_sec=0)
    #         self.wait_if_stopped()
            
    #         error = angle_difference(target_orientation, -self.robot_position["angle"])

    #         angular_speed = clamp(error, -self.max_angular_speed, self.max_angular_speed)

    #         if abs(error) < self.angular_threshold:
    #             break
            
    #         # I did not test yet whether this works
    #         if abs(error) < 0.1:
    #             msg.angular.z = angular_speed
    #         else:
    #             msg.angular.z = clamp(error, -self.max_angular_speed, self.max_angular_speed)
            
    #         msg = Twist()
    #         msg.angular.z = angular_speed
    #         msg.linear.x = 0
    #         msg.linear.y = 0
    #         self.cmd_vel_pub.publish(msg)
    #         time.sleep(self.poll_freq)
        
    #     msg = Twist()
    #     self.cmd_vel_pub.publish(msg)
    #     time.sleep(0.2)