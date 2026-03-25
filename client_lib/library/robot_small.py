import time
import rclpy
from rclpy.node import Node 
import math
import numpy as np
from queue import Queue
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist 
from library.utils import angle_difference, clamp

class Robot:
    def __init__(self):    
        # control parameters
        self.kp_linear = 0.6
        self.kp_angular = 2.5

        self.max_linear_speed = 0.4
        self.max_angular_speed = 0.8
        
        self.linear_threshold = 0.1
        self.angular_threshold = 0.1

        self.stop_driving_forward_angle = math.pi / 2
        self.slower_driving_forward_angle = 0.4

        self.poll_freq = 0.01

        
        self.node = Node(f"robot")

        self.pub_motor_left = self.node.create_publisher(Int32, '/io/motor/left/speed', 10)
        self.pub_motor_right = self.node.create_publisher(Int32, '/io/motor/right/speed', 10)
    
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