#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class ScaleCmdVelNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.sub = rospy.Subscriber("~in_vel", Twist, self.on_cmd_vel)
        self.pub = rospy.Publisher("~out_vel", Twist, queue_size=1)

        self.linear_scale = rospy.get_param("~linear_scale", 0.1)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)

    def on_cmd_vel(self, twist_msg):
        twist_msg.linear.x *= self.linear_scale
        twist_msg.angular.z *= self.angular_scale
        self.pub.publish(twist_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    node = ScaleCmdVelNode("scale_cmd_vel")
    node.run()
