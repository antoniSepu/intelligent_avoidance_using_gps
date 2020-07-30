#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

class Marcadores(object):
    def __init__(self):
        self._pub_marker = rospy.Publisher('visualization_marker1', Marker, queue_size=10)
        self._sub_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._update_pose)
        self._sub_mag = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self._update_mag)

        self._marker1 = Marker()
        self._marker2 = Marker()
        self._marker3 = Marker()
        self._pose = PoseStamped()
        self._mag = Float64()

    def _generar_marker_pose(self): #Flecha azul
        while not rospy.is_shutdown():

           self._marker1.header.frame_id = "base_link"
           self._marker1.header.stamp = rospy.get_rostime()
           self._marker1.type = self._marker1.ARROW
           self._marker1.action = self._marker1.ADD
           self._marker1.scale.x = 0.5
           self._marker1.scale.y = 0.1
           self._marker1.scale.z = 0.1
           self._marker1.color.a = 1.0
           self._marker1.color.r = 0.0
           self._marker1.color.g = 0.2
           self._marker1.color.b = 1.0
           self._marker1.pose.orientation.x = self._pose.pose.orientation.x
           self._marker1.pose.orientation.y = self._pose.pose.orientation.y
           self._marker1.pose.orientation.z = self._pose.pose.orientation.z
           self._marker1.pose.orientation.w = self._pose.pose.orientation.w
           self._marker1.pose.position.x = self._pose.pose.position.x
           self._marker1.pose.position.y = self._pose.pose.position.y
           self._marker1.pose.position.z = self._pose.pose.position.z

           self._pub_marker.publish(self._marker1)

    def _generar_marker_mag(self): #Flecha verde
        while not rospy.is_shutdown():

           self._marker2.header.frame_id = "base_link"
           self._marker2.header.stamp = rospy.get_rostime()
           self._marker2.type = self._marker2.ARROW
           self._marker2.action = self._marker2.ADD
           self._marker2.scale.x = 0.5
           self._marker2.scale.y = 0.1
           self._marker2.scale.z = 0.1
           self._marker2.color.a = 1.0
           self._marker2.color.r = 0.0
           self._marker2.color.g = 1.0
           self._marker2.color.b = 0.0
           self._marker2.pose.orientation.x = 0
           self._marker2.pose.orientation.y = 0
           self._marker2.pose.orientation.z = 0
           self._marker2.pose.orientation.w = self._mag.data
           self._marker2.pose.position.x = 0
           self._marker2.pose.position.y = 0
           self._marker2.pose.position.z = 0

           self._pub_marker.publish(self._marker2)

    def _generar_marker_prueba(self): #flecha roja
        while not rospy.is_shutdown():

           self._marker3.header.frame_id = "base_link"
           self._marker3.header.stamp = rospy.get_rostime()
           self._marker3.type = self._marker3.ARROW
           self._marker3.action = self._marker3.ADD
           self._marker3.scale.x = 0.5
           self._marker3.scale.y = 0.1
           self._marker3.scale.z = 0.1
           self._marker3.color.a = 1.0
           self._marker3.color.r = 1.0
           self._marker3.color.g = 0.0
           self._marker3.color.b = 0.0
           self._marker3.pose.orientation.x = 0
           self._marker3.pose.orientation.y = 0
           self._marker3.pose.orientation.z = 0
           self._marker3.pose.orientation.w = 1
           self._marker3.pose.position.x = 0
           self._marker3.pose.position.y = 0
           self._marker3.pose.position.z = 0

           self._pub_marker.publish(self._marke3)

    def _generar_markers(self): #Todos
        while not rospy.is_shutdown():
            self._marker1.header.frame_id = "base_link"
            self._marker1.header.stamp = rospy.get_rostime()
            self._marker1.type = self._marker1.ARROW
            self._marker1.action = self._marker1.ADD
            self._marker1.scale.x = 0.5
            self._marker1.scale.y = 0.1
            self._marker1.scale.z = 0.1
            self._marker1.color.a = 1.0
            self._marker1.color.r = 0.0
            self._marker1.color.g = 0.2
            self._marker1.color.b = 1.0
            self._marker1.pose.orientation.x = self._pose.pose.orientation.x
            self._marker1.pose.orientation.y = self._pose.pose.orientation.y
            self._marker1.pose.orientation.z = self._pose.pose.orientation.z
            self._marker1.pose.orientation.w = self._pose.pose.orientation.w
            self._marker1.pose.position.x = self._pose.pose.position.x
            self._marker1.pose.position.y = self._pose.pose.position.y
            self._marker1.pose.position.z = self._pose.pose.position.z

            self._marker2.header.frame_id = "base_link"
            self._marker2.header.stamp = rospy.get_rostime()
            self._marker2.type = self._marker2.ARROW
            self._marker2.action = self._marker2.ADD
            self._marker2.scale.x = 0.5
            self._marker2.scale.y = 0.1
            self._marker2.scale.z = 0.1
            self._marker2.color.a = 1.0
            self._marker2.color.r = 0.0
            self._marker2.color.g = 1.0
            self._marker2.color.b = 0.0
            self._marker2.pose.orientation.x = 0
            self._marker2.pose.orientation.y = 0
            self._marker2.pose.orientation.z = 0
            self._marker2.pose.orientation.w = self._mag.data
            self._marker2.pose.position.x = 0
            self._marker2.pose.position.y = 0
            self._marker2.pose.position.z = 0

            self._marker3.header.frame_id = "base_link"
            self._marker3.header.stamp = rospy.get_rostime()
            self._marker3.type = self._marker3.ARROW
            self._marker3.action = self._marker3.ADD
            self._marker3.scale.x = 0.5
            self._marker3.scale.y = 0.1
            self._marker3.scale.z = 0.1
            self._marker3.color.a = 1.0
            self._marker3.color.r = 1.0
            self._marker3.color.g = 0.0
            self._marker3.color.b = 0.0
            self._marker3.pose.orientation.x = 0
            self._marker3.pose.orientation.y = 0
            self._marker3.pose.orientation.z = 0
            self._marker3.pose.orientation.w = 1
            self._marker3.pose.position.x = 0
            self._marker3.pose.position.y = 0
            self._marker3.pose.position.z = 0

            self._pub_marker.publish(self._marker1)
            self._pub_marker.publish(self._marker2)
            self._pub_marker.publish(self._marker3)

    def _update_pose(self, data):
        self._pose = data

    def _update_mag(self, data):
        self._mag = data

def main():
    rospy.init_node('register')
    rate = rospy.Rate(5)

    x = Marcadores()

    x._generar_markers()

    #rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
