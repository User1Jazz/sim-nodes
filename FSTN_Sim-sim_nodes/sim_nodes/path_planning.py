import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from fstn_messages_package.msg import Ai2vcu
from fstn_messages_package.msg import ConeArray

import math
import array
import numpy as np

class PathPlanning(Node):

    point = [0,0]

    def __init__(self):
        super().__init__('path_planning')
        self.publisher_ = self.create_publisher(Ai2vcu, '/cmd', 10)     # It must be "/cmd" (I will make it editable at some point...), must be Ai2vcu message type
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.subscription = self.create_subscription(
            ConeArray,                      # It must be ConeArray message type
            '/Sim_Perception',              # It must be "/Sim_Perception" (I will make it editable at some point...)
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    # Send vehicle control data
    def timer_callback(self):
        angle = 0
        angle = self.get_steering(self.point)
        
        msg = Ai2vcu()
        msg.axle_speed_request_rpm = float(500)    # min: 0,   max 4000
        msg.steer_angle_request_deg = angle      # min: -21, max: 21
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing speed: "%f"' % msg.axle_speed_request_rpm)
        self.get_logger().info('Publishing steering: "%f"' % msg.steer_angle_request_deg)
        self.i += 1
    
    # Listen to incoming cone data (this is currently just printing cone data)
    def listener_callback(self, msg):
        self.get_logger().info('Blue Cones:')
        for cone in msg.blue_cones:
            self.get_logger().info('X: "%f", Y: "%f", Z: "%f"' % (cone.x, cone.y, cone.z))
        self.get_logger().info('Yellow Cones:')
        for cone in msg.yellow_cones:
            self.get_logger().info('X: "%f", Y: "%f", Z: "%f"' % (cone.x, cone.y, cone.z))
        
        if len(msg.blue_cones) > 0 and len(msg.yellow_cones) > 0:
            self.point = self.calculate_midpoint(msg.blue_cones[0], msg.yellow_cones[0])
        elif len(msg.blue_cones) == 0:
            self.point = [-1,1]
        elif len(msg.yellow_cones) == 0:
            self.point = [1,1]
        else:
            self.point = [0,1]

        #self.get_logger().info('Orange Cones:')
        #for cone in msg.big_orange_cones:
        #    self.get_logger().info('X: "%f", Y: "%f", Z: "%f"' % (cone.x, cone.y, cone.z))
        #self.get_logger().info('Small Orange Cones:')
        #for cone in msg.orange_cones:
        #    self.get_logger().info('X: "%f", Y: "%f", Z: "%f"' % (cone.x, cone.y, cone.z))

    def calculate_midpoint(self, right_cone, left_cone):
        midpoint = [((right_cone.x + right_cone.x) / 2), ((left_cone.y + left_cone.y) / 2)]
        return midpoint

    def get_steering(self, midpoint):
        if midpoint == [1,1]:
            return float(28)
        elif midpoint == [-1, 1]:
            return float(-28)
        else:
            dot = np.vdot([0,1], midpoint)                                  # Calculate the dot product
            mag = math.sqrt(sum(pow(element, 2) for element in midpoint))   # Caculate the magnitude
            cosA = dot / (mag * 1)                                          # Calculate cosine alpha
            return float(np.arccos(cosA))                                          # Return inverse cosine
        #return math.degrees(math.atan2(midpoint[0], midpoint[1]))


def main(args=None):
    rclpy.init(args=args)

    path_planning = PathPlanning()

    rclpy.spin(path_planning)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_planning.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
