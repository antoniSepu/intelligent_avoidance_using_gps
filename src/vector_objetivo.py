#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, sqrt, sin, cos, atan2

class Marcadores(object):
    def __init__(self):
        self._pub_marker = rospy.Publisher('visualization_marker_obstacles', Marker, queue_size=10)


        self._marker = Marker()
        self.x = 15
        self.y = 5
        #self._marker2 = Marker()
        #self._marker3 = Marker()

    def _generar_marker_objetivo(self, long, lat): #cubo verde

        #pos_x = []
        #pos_y = []
        #ang_to_center = []

        self._marker.header.frame_id = "base_link"
        self._marker.header.stamp = rospy.get_rostime()
        self._marker.type = self._marker.CUBE
        self._marker.action = self._marker.ADD
        self._marker.scale.x = 0.5
        self._marker.scale.y = 0.5
        self._marker.scale.z = 0.5
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 1.0
        self._marker.color.b = 0.0
        self._marker.pose.orientation.x = 0
        self._marker.pose.orientation.y = 0
        self._marker.pose.orientation.z = 0
        self._marker.pose.orientation.w = 0
        self._marker.pose.position.x = long
        self._marker.pose.position.y = lat
        self._marker.pose.position.z = 0

        self._pub_marker.publish(self._marker)

    def _generar_flecha_objetivo(self, long, lat): #Flecha verde
            angle_x = atan2(lat, long)
    
            (x,y,z,w) = quaternion_from_euler(0,angle_x,0,'syxz')
            self._marker.header.frame_id = "base_link"
            self._marker.header.stamp = rospy.get_rostime()
            self._marker.type = self._marker.ARROW
            self._marker.action = self._marker.ADD
            self._marker.scale.x = 0.5
            self._marker.scale.y = 0.1
            self._marker.scale.z = 0.1
            self._marker.color.a = 1.0
            self._marker.color.r = 0.0
            self._marker.color.g = 1.0
            self._marker.color.b = 0.0
            self._marker.pose.orientation.x = x
            self._marker.pose.orientation.y = y
            self._marker.pose.orientation.z = z
            self._marker.pose.orientation.w = w
            self._marker.pose.position.x = 0
            self._marker.pose.position.y = 0
            self._marker.pose.position.z = 0

            self._pub_marker.publish(self._marker)

    def run(self):
        while not rospy.is_shutdown():
            self._generar_marker_objetivo(self.x, self.y)
            self._generar_flecha_objetivo(self.x, self.y)


def main():
    rospy.init_node('dibujante')
    rate = rospy.Rate(5)

    x = Marcadores()

    x.run()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
