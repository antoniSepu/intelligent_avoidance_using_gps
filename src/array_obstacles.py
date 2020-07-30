#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, sqrt, sin, cos

class Camera(object):
    def __init__(self):
        self._sub_laser = rospy.Subscriber('/camera/scan',
                                                LaserScan, self._update_laser)
        self._laser = LaserScan()

    def _get_scan_list(self):
        while True:
            try:
                test = self._laser.ranges[50] #this should raise the IndexError
                measure = self._laser.ranges
                break
            except IndexError:
                print('Waiting for lectures')
                rospy.sleep(0.5)
        return measure

    def _update_laser(self, data):
        self._laser = data

class Marcadores(Camera):
    def __init__(self):
        Camera.__init__(self)
        self._pub_marker = rospy.Publisher('visualization_marker_obstacles', MarkerArray, queue_size=1)


        self._marker_array = MarkerArray()
        self._marker_array.markers = []
        #self._marker2 = Marker()
        #self._marker3 = Marker()

    def _generar_array_marker_obstacle(self, scan): #Flecha roja
        arco = 0.00869565217 #rad entre rayo laser y rayo
        ang_max = 0.5 #rad
        #pos_x = []
        #pos_y = []
        #ang_to_center = []

        for i in range(len(scan)):
            self._marker1 = Marker()

            ang = ang_max - arco*(i+1)
            ang_to_center = 3.14159 + ang
            pos_x = scan[i]*cos(ang)
            pos_y = scan[i]*sin(ang)

            (x,y,z,w) = quaternion_from_euler(ang_to_center,0,0,'syxz')

            self._marker1.header.frame_id = "camera_link"
            self._marker1.header.stamp = rospy.get_rostime()
            self._marker1.type = self._marker1.ARROW
            self._marker1.action = self._marker1.ADD
            self._marker1.scale.x = 0.5
            self._marker1.scale.y = 0.1
            self._marker1.scale.z = 0.1
            self._marker1.color.a = 1.0
            self._marker1.color.r = 1.0
            self._marker1.color.g = 0.0
            self._marker1.color.b = 0.0
            self._marker1.pose.orientation.x = x
            self._marker1.pose.orientation.y = y
            self._marker1.pose.orientation.z = z
            self._marker1.pose.orientation.w = w
            self._marker1.pose.position.x = pos_x
            self._marker1.pose.position.y = pos_y
            self._marker1.pose.position.z = 0

            self._marker_array.markers.append(self._marker1)
            print("Bucle completado : i", )

        self._pub_marker.publish(self._marker_array)

    def run(self):
        while not rospy.is_shutdown():
            scan = self._get_scan_list()
            self._generar_array_marker_obstacle(scan)


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
