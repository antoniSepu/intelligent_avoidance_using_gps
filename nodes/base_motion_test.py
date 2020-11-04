#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import curses

from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from mavros_msgs.msg import State
from math import sqrt, pow

class TextWindow:
    def __init__(self, stdscr):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = 10

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, position, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'line out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = position
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

class RoverInterface:
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self._sub_bat = rospy.Subscriber('/mavros/battery', BatteryState, self._update_battery)
        self._sub_acc = rospy.Subscriber('/mavros/imu/data', Imu, self._update_imu)
        self._sub_state = rospy.Subscriber('/mavros/state', State, self._update_state)
        self._sub_gps = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self._update_gps)
        self._sub_gps_vel = rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, self._update_gps_vel)
        self._sub_gps_mag = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self._update_gps_mag)
        self._sub_mag = rospy.Subscriber('/mavros/imu/data', Imu, self._update_imu)
        self._hz = 5

        self._last_pressed = {}
        self._time_stamp = 0
        self._seq_stamp = 0
        self._angular = 0
        self._linear = 0
        self._pitch = 0.1
        self._battery = BatteryState()
        self._imu = Imu()
        self._state = State()
        self._gps = NavSatFix()
        self._gps_vel = TwistStamped()
        self._gps_vel_module = 0
        self._gps_mag = Float64()

        self.movement_bindings = {
            curses.KEY_UP:    ( 1,  0),
            curses.KEY_DOWN:  (-1,  0),
            curses.KEY_LEFT:  ( 0,  1),
            curses.KEY_RIGHT: ( 0, -1),
            curses.KEY_BACKSPACE: ( 0,  0),
        }

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
                self._seq_stamp += 1
            self._set_velocity()
            self._publish()
            rate.sleep()

    def _get_vel_msg(self, time, seq, linear, angular):
        vel_msg = TwistStamped()
        vel_msg.header.stamp = time
        vel_msg.header.seq = seq
        vel_msg.twist.linear.x = linear
        vel_msg.twist.angular.z = angular
        return vel_msg

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.1:
                keys.append(a)
        for k in keys:
            l, a = self.movement_bindings[k]
            if l == 0 and a == 0:
                self._linear = 0
                self._angular = 0
            else:
                self._linear += l * self._pitch
                self._angular += a * self._pitch
        self._time_stamp = rospy.Time.now()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(0, 1, 'AION robotics rover interface: Use arrow keys to move, backspace to stop, q to exit.')
        self._interface.write_line(1, 1,'Battery voltage: %f' % self._battery.voltage)
        self._interface.write_line(2, 1,'Linear acceleration x: %f' % self._imu.linear_acceleration.x)
        self._interface.write_line(3, 1,'Linear acceleration y: %f' % self._imu.linear_acceleration.y)
        self._interface.write_line(4, 1,'Linear acceleration z: %f' % self._imu.linear_acceleration.z)
        self._interface.write_line(5, 1,'Latitude: %f' % self._gps.latitude)
        self._interface.write_line(6, 1,'Longitude: %f' % self._gps.longitude)
        self._interface.write_line(7, 1,'Altitude: %f' % self._gps.altitude)
        self._interface.write_line(8, 10,'Connected: %r, Armed: %r, Guided: %r, Mode: %s' % (self._state.connected, self._state.armed, self._state.guided, self._state.mode))
        self._interface.write_line(9, 10, 'Linear command: %f, Angular command: %f' % (self._linear, self._angular))
        self._interface.write_line(1, 45,'GNSS velocity module: %f' % self._gps_vel_module)
        self._interface.write_line(2, 45,'GNSS magnetic heading: %f' % self._gps_mag.data)
        self._interface.refresh()

        vel_msg = self._get_vel_msg(self._time_stamp, self._seq_stamp, self._linear, self._angular)
        self._pub_cmd.publish(vel_msg)

    def _update_battery(self, data):
        self._battery = data
        self._battery.voltage = round(self._battery.voltage, 4)

    def _update_imu(self, data):
        self._imu = data
        self._imu.linear_acceleration.x = round(self._imu.linear_acceleration.x, 4)
        self._imu.linear_acceleration.y = round(self._imu.linear_acceleration.y, 4)
        self._imu.linear_acceleration.z = round(self._imu.linear_acceleration.z, 4)

    def _update_state(self, data):
        self._state = data

    def _update_gps(self, data):
        self._gps = data
        self._gps.latitude = round(self._gps.latitude, 4)
        self._gps.longitude = round(self._gps.longitude, 4)
        self._gps.altitude = round(self._gps.altitude, 4)

    def _update_gps_vel(self, data):
        self._gps_vel = data
        self._gps_vel_module = sqrt(pow(self._gps_vel.twist.linear.x, 2) + pow(self._gps_vel.twist.linear.y, 2) + pow(self._gps_vel.twist.linear.z, 2))

    def _update_gps_mag(self, data):
        self._gps_mag = data

def main(stdscr):
    rospy.init_node('rover_interface')
    app = RoverInterface(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
