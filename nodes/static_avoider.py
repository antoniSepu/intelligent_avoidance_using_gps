#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from math import pi, isnan #importing number pi and isnan function

class Camera(object):
    def __init__(self):
        self._sub_laser = rospy.Subscriber('/camera/scan', LaserScan, self._update_laser)
        self._laser = LaserScan()

    def _get_scan_list(self):
        while not rospy.is_shutdown():
            try:
                test = self._laser.ranges[50] #this should raise a IndexError
                measure = self._laser.ranges
                break
            except IndexError:
                rospy.loginfo('Waiting for lectures')
                rospy.sleep(0.5)
        return measure
#We raise the index error because in the first readings, the camera topic does not publish the whole scan array

    def _update_laser(self, data):
        self._laser = data

class Avoider(Camera): #Class which inheritance from Camera
    def __init__(self):
        Camera.__init__(self)
        self._pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=5)
        self._sub_state = rospy.Subscriber('/mavros/state', State, self._update_state)

        self._state = State()

        try: #initialize the server proxy for changing the flight mode with mavros
            rospy.wait_for_service('/mavros/set_mode')
            self._set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        except rospy.ServiceException:
            rospy.loginfo('Service proxy failed')

        self._vel_msg = TwistStamped()
        self._vel_msg.twist.linear.x=0
        self._vel_msg.twist.linear.y=0
        self._vel_msg.twist.linear.z=0
        self._vel_msg.twist.angular.x = 0
        self._vel_msg.twist.angular.y = 0
        self._vel_msg.twist.angular.z = 0
        self._seq = 0

    def run(self): #Main loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("Scanning...")
            scan = self._get_scan_list()
            threat = self._check_front(scan)
            from_where = self._where_2_go(scan)
            if from_where == 3 and self._state.mode == 'AUTO':
                rospy.loginfo("Inminent danger, changing to HOLD mode")
                self._change_mode_to('HOLD')
            elif threat and from_where == 0 and self._state.mode == 'AUTO':
                self._back()
            elif threat and from_where == 1 and self._state.mode == 'AUTO':
                self._avoid_2_right()
            elif threat and from_where == 2 and self._state.mode == 'AUTO':
                self._avoid_2_left()
            elif self._state.mode == 'AUTO' and threat == False:
                rospy.loginfo("No threats on our way")
            elif self._state.mode == 'HOLD' and threat == False:
                rospy.loginfo("Resuming navigation")
                self._change_mode_to('AUTO')
            else:
                rospy.loginfo("Actual flight mode: {0}".format(self._state.mode))

            rate.sleep()

    def _change_mode_to(self, mode): #Method to change the flight mode
        try:
            rospy.loginfo("Changing mode to: {0}".format(mode))
            self._set_mode(0, mode)

        except rospy.ServiceException:
            rospy.loginfo("Couldnt change to mode: {0}".format(mode))

    def _check_front(self, scan): #Return True if there is risk of collision
        threat = False #the scan array shoudl have a length of 640 measures
        for i in range(len(scan)):
            if scan[i] <= 3.0 and isnan(scan[i]) == False and i >= 274 and i <= 366:
                threat = True
            elif scan[i] <= 2.0 and isnan(scan[i]) == False and i >= 251 and i <= 389:
                threat = True
            elif scan[i] <= 1 and isnan(scan[i]) == False and i >= 182 and i <= 458:
                threat = True

        return threat

    def _where_2_go(self, scan):#This method decide where to avoid
        right_ranges_list = []
        left_ranges_list = []
        proximity_alert = False
        for i in range(len(scan)):
            if scan[i] <= 0.6 and scan[i] != float('nan'):
                proximity_alert = True
            elif isnan(scan[i]) == False and i <= 320:
                right_ranges_list.append(scan[i])
            elif isnan(scan[i]) == False and i > 320:
                left_ranges_list.append(scan[i])
            elif isnan(scan[i]) and i <= 320:
                right_ranges_list.append(10.0)
            elif isnan(scan[i]) and i > 320:
                left_ranges_list.append(10.0)

        sum_right_ranges_list = sum(right_ranges_list)
        sum_left_ranges_list = sum(left_ranges_list)
        dif = sum_right_ranges_list - sum_left_ranges_list

        if proximity_alert:
            return 3
        elif dif < 50 and dif > -50:
            #back
            return 0
        elif dif >= 50:
            #right
            return 1
        elif dif <= -50:
            #left
            return 2

    def _avoid_2_right(self): #This is the method for avoiding to the right
        rospy.loginfo('Avoiding to right')

        self._change_mode_to("GUIDED")

        self._vel_msg.twist.linear.x = 0
        self._vel_msg.twist.angular.z = 0

        rate = rospy.Rate(5)
        inic_time = rospy.Time.now()
        while not rospy.is_shutdown():
            actual_time = rospy.Time.now()
       	    self._vel_msg.header.stamp = actual_time
            self._vel_msg.header.seq = self._seq
            self._vel_msg.header.frame_id = "base_link"

            dif = actual_time.secs - inic_time.secs

            if dif < 0.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = 0
            elif dif >= 0.5 and dif < 1.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = -pi / 2
            elif dif >= 1.5 and dif < 3.0:
                self._vel_msg.twist.linear.x = 0.5
                self._vel_msg.twist.angular.z = 0
            elif dif >= 3.0 and dif < 4.0:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = pi / 2
            elif dif >= 4.0 and dif <= 4.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = 0
            else:
                break

            self._pub_vel.publish(self._vel_msg)
            self._seq += 1
            rate.sleep()

        self._change_mode_to("AUTO")


    def _avoid_2_left(self): #This is the method for avoiding to the left
        rospy.loginfo('Avoiding to left')

        self._change_mode_to("GUIDED")

        self._vel_msg.twist.linear.x = 0
        self._vel_msg.twist.angular.z = 0

        rate = rospy.Rate(5)
        inic_time = rospy.Time.now()
        while not rospy.is_shutdown():
            actual_time = rospy.Time.now()
       	    self._vel_msg.header.stamp = actual_time
            self._vel_msg.header.seq = self._seq
            self._vel_msg.header.frame_id = "base_link"

            dif = actual_time.secs - inic_time.secs

            if dif < 0.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = 0
            elif dif >= 0.5 and dif < 1.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = pi / 2
            elif dif >= 1.5 and dif < 3.0:
                self._vel_msg.twist.linear.x = 0.5
                self._vel_msg.twist.angular.z = 0
            elif dif >= 3.0 and dif < 4.0:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = -pi / 2
            elif dif >= 4.0 and dif <= 4.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = 0
            else:
                break

            self._pub_vel.publish(self._vel_msg)
            self._seq += 1
            rate.sleep()

        self._change_mode_to("AUTO")

    def _back(self): #This is the method for avoiding back
        rospy.loginfo("Going back")

        self._change_mode_to("GUIDED")

        self._vel_msg.twist.linear.x = 0
        self._vel_msg.twist.angular.z = 0

        rate = rospy.Rate(5)
        inic_time = rospy.Time.now()
        while not rospy.is_shutdown():
            actual_time = rospy.Time.now()
       	    self._vel_msg.header.stamp = actual_time
            self._vel_msg.header.seq = self._seq
            self._vel_msg.header.frame_id = "base_link"

            dif = actual_time.secs - inic_time.secs

            if dif < 0.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = 0
            elif dif >= 0.5 and dif < 2.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = pi / 2
            elif dif >= 2.5 and dif < 4.0:
                self._vel_msg.twist.linear.x = 0.5
                self._vel_msg.twist.angular.z = 0
            elif dif >= 4.0 and dif <= 4.5:
                self._vel_msg.twist.linear.x = 0
                self._vel_msg.twist.angular.z = 0
            else:
                break


            self._pub_vel.publish(self._vel_msg)
            self._seq += 1
            rate.sleep()

        self._change_mode_to("AUTO")

    def _update_state(self, data):
        self._state = data

def main():
    rospy.init_node('rover_static_avoider')
    avoider = Avoider()
    avoider.run()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
