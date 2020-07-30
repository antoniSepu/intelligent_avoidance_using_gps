#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

class Marcadores(object):
    def __init__(self):
        self._pub_marker1 = rospy.Publisher('visualization_marker1', Marker, queue_size=10)
        self._pub_marker2 = rospy.Publisher('visualization_marker2', Marker, queue_size=10)
        self._pub_marker3 = rospy.Publisher('visualization_marker3', Marker, queue_size=10)
        self._sub_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._update_pose)
        self._sub_mag = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self._update_mag)

        self._marker = Marker()
        self._pose = PoseStamped()
        self._mag = Float64()

    def _generar_marker_pose(self): #Flecha azul
        while not rospy.is_shutdown():

           self._marker.header.frame_id = "base_link"
           self._marker.header.stamp = rospy.get_rostime()
           self._marker.type = self._marker.ARROW
           self._marker.action = self._marker.ADD
           self._marker.scale.x = 0.5
           self._marker.scale.y = 0.1
           self._marker.scale.z = 0.1
           self._marker.color.a = 1.0
           self._marker.color.r = 0.0
           self._marker.color.g = 0.2
           self._marker.color.b = 1.0
           self._marker.pose.orientation.x = self._pose.pose.orientation.x
           self._marker.pose.orientation.y = self._pose.pose.orientation.y
           self._marker.pose.orientation.z = self._pose.pose.orientation.z
           self._marker.pose.orientation.w = self._pose.pose.orientation.w
           self._marker.pose.position.x = self._pose.pose.position.x
           self._marker.pose.position.y = self._pose.pose.position.y
           self._marker.pose.position.z = self._pose.pose.position.z

           self._pub_marker1.publish(self._marker)

    def _generar_marker_mag(self): #Flecha verde
        while not rospy.is_shutdown():

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
           self._marker.pose.orientation.x = 0
           self._marker.pose.orientation.y = 0
           self._marker.pose.orientation.z = 0
           self._marker.pose.orientation.w = self._mag.data
           self._marker.pose.position.x = self._pose.pose.position.x
           self._marker.pose.position.y = self._pose.pose.position.y
           self._marker.pose.position.z = self._pose.pose.position.z

           self._pub_marker2.publish(self._marker)

    def _generar_marker_prueba(self): #flecha roja
        while not rospy.is_shutdown():

           self._marker.header.frame_id = "base_link"
           self._marker.header.stamp = rospy.get_rostime()
           self._marker.type = self._marker.ARROW
           self._marker.action = self._marker.ADD
           self._marker.scale.x = 0.5
           self._marker.scale.y = 0.1
           self._marker.scale.z = 0.1
           self._marker.color.a = 1.0
           self._marker.color.r = 1.0
           self._marker.color.g = 0.0
           self._marker.color.b = 0.0
           self._marker.pose.orientation.x = 0
           self._marker.pose.orientation.y = 0
           self._marker.pose.orientation.z = 0
           self._marker.pose.orientation.w = 1
           self._marker.pose.position.x = 1
           self._marker.pose.position.y = 1
           self._marker.pose.position.z = 1

           self._pub_marker3.publish(self._marker)

    def _update_pose(self, data):
        self._pose = data

    def _update_mag(self, data):
        self._mag = data

def main():
    rospy.init_node('register')
    rate = rospy.Rate(5)

    x = Marcadores()

    x._generar_marker_prueba()
    #x._generar_marker_pose()
    x._generar_marker_mag()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
