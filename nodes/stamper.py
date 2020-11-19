#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import TwistStamped, Twist

class Stamper(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.subscriber = rospy.Subscriber('/move_base/cmd_vel', Twist, self.update_sub)

        self.twist_message = Twist()
        self.twist_stamped = TwistStamped()

        self.twist_message.linear.x = 0
        self.twist_message.linear.y = 0
        self.twist_message.linear.z = 0
        self.twist_message.angular.x = 0
        self.twist_message.angular.y = 0
        self.twist_message.angular.z = 0

    def run(self):
        i = 0
        while not rospy.is_shutdown():
            rospy.wait_for_message('/move_base/cmd_vel', Twist)
            self.twist_stamped.twist = self.twist_message
            self.twist_stamped.header.stamp = rospy.get_rostime()
            self.twist_stamped.header.seq = i

            self.publisher.publish(self.twist_stamped)
            rospy.loginfo('Message published')

            i += 1

    def update_sub(self, data):
        rospy.loginfo('Message received')
        self.twist_message = data

def main():
    rospy.init_node('stamper')
    x = Stamper()
    x.run()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
