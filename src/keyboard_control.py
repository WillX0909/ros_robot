#!/usr/bin/env python

import rospy
from ros_robot.msg import JointCommand
import serial
from math import pi
import sys
import termios, atexit
from select import select

DT = 0.01
STEP = pi/40.
MAX = pi/4.0
MIN = -MAX
def clamp(value):
    return max(MIN, min(value, MAX))

# save the terminal settings
fd = sys.stdin.fileno()
new_term = termios.tcgetattr(fd)
old_term = termios.tcgetattr(fd)

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

# switch to normal terminal
def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# switch to unbuffered terminal
def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def getchars():
    return sys.stdin.read(1)

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr <> []

class KeyboardController:

    def __init__(self):
        set_curses_term()
        rospy.on_shutdown(set_normal_term)
        self.joint1_ang = 0
        self.joint2_ang = 0
        self.joint3_ang = 0
        self.joint4_ang = 0
        self.joint5_ang = 0
        self.joint6_ang = 0
        
        # create publishers:
        self.pt_msg = JointCommand()
        self.cont_pub = rospy.Publisher("joint", JointCommand, queue_size=1, latch=True)
        rospy.sleep(1.0)
        self.publish_commands()
        # create a timer for reading keyboard input
        self.key_timer = rospy.Timer(rospy.Duration(DT), self.timercb)
        return

    def timercb(self, data):
        # check for keyboard
        if kbhit():
            k = getchars()
            if k == 'w':
                self.joint1_ang += STEP
                self.joint1_ang = clamp(self.joint1_ang)
            elif k == 's':
                self.joint1_ang -= STEP
                self.joint1_ang = clamp(self.joint1_ang)
            elif k == 'e':
                self.joint2_ang += STEP
                self.joint2_ang = clamp(self.joint2_ang)
            elif k == 'd':
                self.joint2_ang -= STEP
                self.joint2_ang = clamp(self.joint2_ang)
            if k == 'r':
                self.joint3_ang += STEP
                self.joint3_ang = clamp(self.joint3_ang)
            elif k == 'f':
                self.joint3_ang -= STEP
                self.joint3_ang = clamp(self.joint3_ang)
            elif k == 't':
                self.joint4_ang += STEP
                self.joint4_ang = clamp(self.joint4_ang)
            elif k == 'g':
                self.joint4_ang -= STEP
                self.joint4_ang = clamp(self.joint4_ang)
            if k == 'y':
                self.joint5_ang += STEP
                self.joint5_ang = clamp(self.joint5_ang)
            elif k == 'h':
                self.joint5_ang -= STEP
                self.joint5_ang = clamp(self.joint5_ang)
            elif k == 'u':
                self.joint6_ang += STEP
                self.joint6_ang = clamp(self.joint6_ang)
            elif k == 'j':
                self.joint6_ang -= STEP
                self.joint6_ang = clamp(self.joint6_ang)
            elif k == 'q' or k == chr(27):
                print "Quit!"
                rospy.signal_shutdown("User shutdown request")
            sys.stdin.flush()
            self.publish_commands()
        return

    def publish_commands(self):
        self.pt_msg.joint1 = self.joint1_ang
        self.pt_msg.joint2 = self.joint2_ang
        self.pt_msg.joint3 = self.joint3_ang
        self.pt_msg.joint4 = self.joint4_ang
        self.pt_msg.joint5 = self.joint5_ang
        self.pt_msg.joint6 = self.joint6_ang
        self.cont_pub.publish(self.pt_msg)
        return



def main():
    rospy.init_node('keyboard_controller', log_level=rospy.INFO)

    try:
        controller = KeyboardController()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
