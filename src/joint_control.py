#!/usr/bin/env python

import rospy
from ros_robot.msg import JointCommand
import serial
from math import pi

PORT = "/dev/ttyACM0"
BAUDRATE = 9600
NUMBYTES = 4
J1CHAN = 1
J2CHAN = 2
J3CHAN = 3
J4CHAN = 4
J5CHAN = 5
J6CHAN = 6
HEADERBYTE = 0x84
MAX = 8000
MIN = 3968
ANGRANGE = pi/2 # radians
def angle_to_pulse(ang):
    value = MIN + int(ang*(MAX-MIN)/ANGRANGE) + (MAX-MIN)/2
    return max(MIN, min(value, MAX))

class JointControl:

    def __init__(self):
        self.j1chan = rospy.get_param("~joint1_channel", J1CHAN)
        self.j2chan = rospy.get_param("~joint2_channel", J2CHAN)
        self.j3chan = rospy.get_param("~joint3_channel", J3CHAN)
        self.j4chan = rospy.get_param("~joint4_channel", J4CHAN)
        self.j5chan = rospy.get_param("~joint5_channel", J5CHAN)
        self.j6chan = rospy.get_param("~joint6_channel", J6CHAN)

        self.comport = None
        self.open_serial()
        if self.comport: rospy.on_shutdown(self.close_serial)

        # create subscribers:
        self.joint_sub = rospy.Subscriber("joint", JointCommand, self.commandcb)
        return
        
    def open_serial(self):
        try:
            self.comport = serial.Serial(rospy.get_param("~port", PORT),
                                    rospy.get_param("~baudrate", BAUDRATE))
        except (serial.SerialException, ValueError):
            rospy.logerr("Could not open serial port!")
            rospy.signal_shutdown("serial port err")
        return

        
    def close_serial(self):
        self.comport.close()
        return

    def commandcb(self, data):
        #rospy.logdebug("data.pan = {0:f} | data.tilt = {1:f}".format(data.pan, data.tilt))
        self.send_command(self.j1chan, angle_to_pulse(data.joint1))
        self.send_command(self.j2chan, angle_to_pulse(data.joint2))
        self.send_command(self.j3chan, angle_to_pulse(data.joint3))
        self.send_command(self.j4chan, angle_to_pulse(data.joint4))
        self.send_command(self.j5chan, angle_to_pulse(data.joint5))
        self.send_command(self.j6chan, angle_to_pulse(data.joint6))   
        return
        
    def send_command(self, channel, value):
        bfr = [0]*NUMBYTES
        bfr[0] = HEADERBYTE
        bfr[1] = channel
        target = value
        bfr[2] = target & 0x7F
        bfr[3] = (target >> 7) &0x7F
        self.comport.write(''.join(map(chr,bfr)))
        return


def main():
    rospy.init_node('joint_controller', log_level=rospy.INFO)

    try:
        controller = JointControl()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
