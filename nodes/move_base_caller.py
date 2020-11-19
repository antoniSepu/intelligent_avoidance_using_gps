#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from mavros_msgs.srv import SetMode
from tf.transformations import euler_from_quaternion
from math import pow, isnan, atan2, sqrt, cos, sin

class Camera(object):
    def __init__(self):
        self._sub_laser = rospy.Subscriber('/camera/scan',
                                                LaserScan, self._update_laser)
        self._laser = LaserScan()

    def _get_scan_list(self):
        while not rospy.is_shutdown():
            try:
                test = self._laser.ranges[50] #this should raise the IndexError
                measure = self._laser.ranges
                break
            except IndexError:
                rospy.loginfo('Waiting for lectures')
                rospy.sleep(0.5)
        return measure

    def _update_laser(self, data):
        self._laser = data

class Avoider(Camera): #inherits from Camera
    def __init__(self):
        Camera.__init__(self)
        self._sub_state = rospy.Subscriber('/mavros/state', State, self._update_state)
        self._sub_result = rospy.Subscriber('/move_base/status', GoalStatusArray, self._update_result)
        self._pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


        self._average_obstacle_range = 10

        self._goal = PoseStamped()
        self._state = State()
        self._result = GoalStatusArray()
        self._listener = tf.TransformListener()
        self._seq = 0

        try:
            rospy.wait_for_service('/mavros/set_mode')
            self._set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        except rospy.ServiceException:
            rospy.loginfo('Service proxy failed')

    def _change_mode_to(self, mode):
        try:
            rospy.loginfo("Changing mode to: {0}".format(mode))
            self._set_mode(0, mode)
        except rospy.ServiceException:
            rospy.loginfo("Couldnt change to mode: {0}".format(mode))

    def _obstacle_detection(self, scan): #this method compute the _average_obstacle_range
        arc = 0.0015625 #radians between the 640 laser rays
        ang_max = 0.5 #max radians (FOV)
        range_list = []
        weight_list = []
        for i in range(len(scan)): #calculate the Weighted average
            if isnan(scan[i]) == False:
                weight = 0
                ang = ang_max - (arc*i)

                weight = 1 - 3.2 * pow(ang,2) #I used a quadratic distribution. Max weight is 1, minimun is 0.2
                weight_list.append(weight)
                range_list.append(scan[i] * weight)

        sum_range_list = sum(range_list)
        sum_weight_list = sum(weight_list)
        if len(range_list) > 0:
            self._average_obstacle_range = sum_range_list / sum_weight_list
        else:
            self._average_obstacle_range = 10


    def _set_goal(self): #This method calculates and publishes the nav goal desired
        now = rospy.Time.now()

        self._goal.header.frame_id = "map"
        self._goal.header.seq = self._seq

        self._seq += 1

        try:
            self._listener.waitForTransform("/map", "/base_link", now, rospy.Duration(0.5))
            #here i get the transform info
            (trans,rot) = self._listener.lookupTransform("/map", "/base_link", now)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException):
            rospy.logerr("Exception raised")

        angular = atan2(trans[1], trans[0])
        linear = sqrt(pow(trans[0], 2) + pow(trans[1], 2))
        (x,y,z) = euler_from_quaternion(rot)
        #here i calculate the position desired in the map frame
        self._goal.pose.position.x = (linear * cos(angular)) + (self._average_obstacle_range + 1) * cos(z)
        self._goal.pose.position.y = (linear * sin(angular)) + (self._average_obstacle_range + 1) * sin(z)
        self._goal.pose.position.z = 0
        self._goal.pose.orientation.w = 1
        self._goal.pose.orientation.x = 0
        self._goal.pose.orientation.y = 0
        self._goal.pose.orientation.z = 0
        self._goal.header.stamp = rospy.Time.now()

        self._pub_goal.publish(self._goal)
        rospy.loginfo("Navigation goal published")

    def run(self): #main loop
        rate = rospy.Rate(10) #10Hz
        while not rospy.is_shutdown():
            scan = self._get_scan_list()
            self._obstacle_detection(scan)

            if len(self._result.status_list) == 0:
                rospy.loginfo("Move_base: Waiting")
                if self._average_obstacle_range < 3.0 and self._state.mode == "AUTO":
                    rospy.loginfo("Obstacle detected. Handing over the controls")
                    self._change_mode_to('GUIDED')
                    self._set_goal()
                elif self._average_obstacle_range >= 3.0 and self._state.mode == "AUTO":
                    rospy.loginfo("Performing mission. No obstacles detected")
                elif self._state.mode != "AUTO":
                    rospy.loginfo("Actual mode is: {0}".format(self._state.mode))

            else:
                if self._result.status_list[-1].status == 1:
                    rospy.loginfo("Move_base: Driving")
                elif self._result.status_list[-1].status == 3:
                    rospy.loginfo("Move_base: Goal reached")
                elif self._result.status_list[-1].status == 4:
                    rospy.loginfo("Move_base: Failed. Aborted")
                elif self._result.status_list[-1].status == 5:
                    rospy.loginfo("Move_base: Failed. Rejected")
                elif self._result.status_list[-1].status == 0:
                    rospy.loginfo("Move_base: Waiting")
                else:
                    rospy.loginfo("Move_base: Unknown status")

                if self._average_obstacle_range < 3.0 and self._state.mode == "AUTO":
                    rospy.loginfo("Obstacle detected. Handing over the controls")
                    self._change_mode_to('GUIDED')
                    self._set_goal()
                elif self._result.status_list[-1].status == 3 and self._average_obstacle_range < 3.0:
                    rospy.loginfo("There are still obstacles. Publishing new goal")
                    self._set_goal()
                elif self._result.status_list[-1].status == 3 and self._average_obstacle_range >= 3.0:
                    rospy.loginfo("Resuming own navigation")
                    self._change_mode_to('AUTO')
                elif self._result.status_list[-1].status == 4 or self._result.status_list[-1].status == 5:
                    self._change_mode_to('HOLD')
                elif self._average_obstacle_range >= 3.0 and self._state.mode == "AUTO":
                    rospy.loginfo("Performing mission. No obstacles detected")
                elif self._average_obstacle_range >= 3.0 and self._state.mode == "HOLD":
                    self._change_mode_to('AUTO')
                elif self ._average_obstacle_range < 3.0 and self._state.mode == "HOLD":
                    rospy.loginfo("Passing to MANUAL")
                    self._change_mode_to('MANUAL')
                elif self._state.mode != "AUTO":
                    rospy.loginfo("Actual mode is: {0}".format(self._state.mode))

            rate.sleep()

    def _update_state(self, data):
        self._state = data

    def _update_result(self, data):
        self._result = data


def main():
    rospy.init_node('intelligent_avoider')

    x = Avoider()

    x.run()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
