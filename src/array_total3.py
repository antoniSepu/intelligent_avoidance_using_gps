#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, sqrt, sin, cos, atan2, pi

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
        self._pub_marker = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)


        self._marker_array = MarkerArray()
        self._marker_array.markers = []

        self.x = 5
        self.y = 1

        self._max_i = 0
        self._average_angle = 0
        self._average_range = 0.01
        self._angle_objective = 0
        self._resultant_angle = 0
        self._resultant_modulus = 0

        #Power of attraction of the objective vector (from 0 to 1)
        self._attraction_force = 0.6

    def _generar_array_marker_obstacle(self, scan): #Flecha roja
        arco = 0.00869565217 #rad entre rayo laser y rayo
        ang_max = 0.5 #rad

        self._max_i = len(scan) - 1

        angle_list = []
        range_list = []

        for i in range(len(scan)):
            if scan[i] != float('inf'):

                self._marker = Marker()

                ang = ang_max - (arco*i)
                ang_to_center = pi - ang
                angle_list.append(ang_to_center)
                range_list.append(scan[i])
                pos_x = scan[i]*cos(-ang)
                pos_y = scan[i]*sin(-ang)

                (x,y,z,w) = quaternion_from_euler(0,0,ang_to_center,'ryxz')

                self._marker.header.frame_id = "camera_link"
                self._marker.header.stamp = rospy.get_rostime()
                self._marker.type = self._marker.ARROW
                self._marker.action = self._marker.ADD
                self._marker.id = i
                self._marker.scale.x = 0.45 / scan[i] #0.45 representa el maximo modulo de 1
                self._marker.scale.y = 0.05
                self._marker.scale.z = 0.05
                self._marker.color.a = 1.0
                self._marker.color.r = 1.0
                self._marker.color.g = 0.0
                self._marker.color.b = 0.0
                self._marker.pose.orientation.x = x
                self._marker.pose.orientation.y = y
                self._marker.pose.orientation.z = z
                self._marker.pose.orientation.w = w
                self._marker.pose.position.x = pos_x
                self._marker.pose.position.y = pos_y
                self._marker.pose.position.z = 0

                self._marker_array.markers.append(self._marker)

        sum_angle_list = sum(angle_list)
        sum_range_list = sum(range_list)
        if len(angle_list) != 0:
            self._average_angle = sum_angle_list / len(angle_list)
            self._average_range = sum_range_list / len(range_list)
        else:
            self._average_angle = 0
            self._average_range = 10

        #print("{0} is an integer while {1} is a string.".format(a,b))

        #print("El rango medio es: {0}".format(self._average_range))

    def _generar_marker_objetivo(self, long, lat): #cubo verde
        self._marker = Marker()

        ang = pi/4
        (x,y,z,w) = quaternion_from_euler(0,ang,ang,'ryxz')
        self._marker.header.frame_id = "camera_link"
        self._marker.header.stamp = rospy.get_rostime()
        self._marker.type = self._marker.CUBE
        self._marker.action = self._marker.ADD
        self._marker.id = self._max_i + 1
        self._marker.scale.x = 0.3
        self._marker.scale.y = 0.3
        self._marker.scale.z = 0.3
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 1.0
        self._marker.color.b = 0.0
        self._marker.pose.orientation.x = x
        self._marker.pose.orientation.y = y
        self._marker.pose.orientation.z = z
        self._marker.pose.orientation.w = w
        self._marker.pose.position.x = long
        self._marker.pose.position.y = lat
        self._marker.pose.position.z = 0

        self._marker_array.markers.append(self._marker)

    def _generar_flecha_objetivo(self, long, lat): #Flecha verde
        self._marker = Marker()
        self._angle_objective = atan2(lat, long)

        (x,y,z,w) = quaternion_from_euler(0,0,self._angle_objective,'ryxz')
        self._marker.header.frame_id = "camera_link"
        self._marker.header.stamp = rospy.get_rostime()
        self._marker.id = self._max_i + 2
        self._marker.type = self._marker.ARROW
        self._marker.action = self._marker.ADD
        self._marker.scale.x = self._attraction_force
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

        self._marker_array.markers.append(self._marker)

    def _generar_flecha_resultante(self): #Flecha azul
        self._marker = Marker()

        #procedemos a sumar el vector obstáculo con el vector _generar_flecha_objetivo
        #Componentes vector obstáculo
        i_obstacle = (0.45/self._average_range) * cos(self._average_angle)
        j_obstacle = (0.45/self._average_range) * sin(self._average_angle)
        obstacle_modulus = sqrt(pow(i_obstacle,2) + pow(j_obstacle,2))
        #Componentes vector objetivo, le pongo un modulo de 0.5 al vector atrayente
        i_objective = self._attraction_force * cos(self._angle_objective)
        j_objective = self._attraction_force * sin(self._angle_objective)

        i_resultant = i_obstacle + i_objective
        j_resultant = j_obstacle + j_objective

        self._resultant_modulus = sqrt(pow(i_resultant,2) + pow(j_resultant,2))
        self._resultant_angle = atan2(j_resultant, i_resultant)

        (x,y,z,w) = quaternion_from_euler(0,0,self._resultant_angle,'ryxz')
        self._marker.header.frame_id = "camera_link"
        self._marker.header.stamp = rospy.get_rostime()
        self._marker.id = self._max_i + 3
        self._marker.type = self._marker.ARROW
        self._marker.action = self._marker.ADD
        self._marker.scale.x = self._resultant_modulus
        self._marker.scale.y = 0.1
        self._marker.scale.z = 0.1
        self._marker.color.a = 1.0
        self._marker.color.r = 0.0
        self._marker.color.g = 0.0
        self._marker.color.b = 1.0
        self._marker.pose.orientation.x = x
        self._marker.pose.orientation.y = y
        self._marker.pose.orientation.z = z
        self._marker.pose.orientation.w = w
        self._marker.pose.position.x = 0
        self._marker.pose.position.y = 0
        self._marker.pose.position.z = 0

        self._marker_array.markers.append(self._marker)

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            scan = self._get_scan_list()
            self._generar_array_marker_obstacle(scan)
            self._generar_marker_objetivo(self.x, self.y)
            self._generar_flecha_objetivo(self.x, self.y)
            self._generar_flecha_resultante()

            self._pub_marker.publish(self._marker_array)
            self._marker_array.markers *= 0

            if self.y <= 5 and self.y >= 0:
                self.y += 0.2
            elif self.y >= -5 and self.y < 0:
                self.y += 0.2
            else:
                self.y = -5

            rate.sleep()


def main():
    rospy.init_node('dibujante')

    x = Marcadores()

    x.run()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
