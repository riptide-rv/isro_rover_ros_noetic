#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class EncoderMockNode:
    def __init__(self):
        rospy.init_node('encoder_mock_node', anonymous=True)
        
        # Parameters
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.27)  # Example value, adjust as needed
        self.encoder_ticks_per_meter = 3607.20  # Example value, adjust as needed

        # Publishers
        self.lencoder_pub = rospy.Publisher('/lwheel', Int32, queue_size=10)
        self.rencoder_pub = rospy.Publisher('/rwheel', Int32, queue_size=10)

        # Subscriber
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Timer for updating encoder positions at 100 Hz (every 10 ms)
        self.encoder_update_timer = rospy.Timer(rospy.Duration(0.01), self.update_encoder_positions)

        # Initialize variables
        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.left_encoder = 0
        self.right_encoder = 0

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Calculate wheel velocities
        self.left_velocity = linear - angular * self.wheel_separation / 2
        self.right_velocity = linear + angular * self.wheel_separation / 2

    def update_encoder_positions(self, event=None):
        # Calculate encoder increments
        left_encoder_increment = int(self.left_velocity * self.encoder_ticks_per_meter)
        right_encoder_increment = int(self.right_velocity * self.encoder_ticks_per_meter)

        # Update encoder values
        self.left_encoder += left_encoder_increment
        self.right_encoder += right_encoder_increment

        # Publish encoder values
        self.publish_encoder_positions()

    def publish_encoder_positions(self):
        lencoder_msg = Int32()
        rencoder_msg = Int32()

        lencoder_msg.data = self.left_encoder
        rencoder_msg.data = self.right_encoder
        
        self.lencoder_pub.publish(lencoder_msg)
        self.rencoder_pub.publish(rencoder_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = EncoderMockNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
