#!/usr/bin/env python

# Author: Nick Fragale
# Description: This script converts Joystick commands into Joint Velocity commands
# 
# EBB modified for emergency stop button (button A), prevents further joystick commands and cancels any existing move_base command
# sends zeros to cmd_vel while in ESTOP state, can be toggled by pressing A button after debounce time (2 seconds)
# also checks if joystick lost comms while we were moving
# ramp-up for recent history of motion

# Xbox controller mapping:
#   axes: [l-stick horz,l-stick vert, l-trigger, r-stick horz, r-stick vert, r-trigger]
#   buttons: [a,b,x,y,lb,rb,back,start,xbox,l-stick,r-stick,l-pad,r-pad,u-pad,d-pad]  

import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
import os
import time

cmd = Twist()
last_estop_button=time.time()
last_way_button=time.time()
last_joycb_device_check=time.time()

L_STICK_H_AXES = 0
L_STICK_V_AXES = 1
L_TRIG_AXES = 2
R_STICK_H_AXES = 3 
R_STICK_V_AXES = 4
R_TRIG_AXES = 5

E_STOP=0

USE_XPAD=False
# xboxdrv has only 11 indices
# and the U/D_PAD_BUTTON is the 7th axis
'''
# xpad module uses 2, xboxdrv uses 3
TURN_JOY=3
    U_PAD_BUTTON=7
    D_PAD_BUTTON=7
    U_PAD_BUTTON_VALUE=1
    D_PAD_BUTTON_VALUE=-1
    U_PAD_BUTTON = 13
    D_PAD_BUTTON = 14
    U_PAD_BUTTON_VALUE=1
    D_PAD_BUTTON_VALUE=1
TURN_JOY=2
'''

DRIVE_JOY=1
A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2
Y_BUTTON = 3
LB_BUTTON = 4
RB_BUTTON = 5
BACK_BUTTON = 6
START_BUTTON = 7
L_STICK_BUTTON = 8
R_STICK_BUTTON = 9
L_PAD_BUTTON = 11
R_PAD_BUTTON = 12
U_PAD_BUTTON = 13
D_PAD_BUTTON = 14

MAX_VEL = 2.6
FULL_THROTTLE = 0.8 
ADJ_THROTTLE = True
INC = 0.2 
DEADBAND = 0.2
FWD_ACC_LIM = 0.01
TRN_ACC_LIM = 0.01

prev_fwd = 0
prev_trn = 0

PREV_CMD_TIME = 0
PREV_SEQ_NUM = 0

# cmd_vel publisher  
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
pub_delay = rospy.Publisher('/joystick/delay', Float32, queue_size=3)
pub_cancel_move_base = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)

def limit_acc(fwd,trn):

    fwd_acc = fwd - prev_fwd
    if fwd_acc > FWD_ACC_LIM:
        fwd = prev_fwd + FWD_ACC_LIM
    elif fwd_acc < -FWD_ACC_LIM:
        fwd = prev_fwd - FWD_ACC_LIM

    turn_acc = trn - prev_trn
    if trn_acc > TRN_ACC_LIM:
        trn = prev_trn + TRN_ACC_LIM
    elif trn_acc < -TRN_ACC_LIM:
        trn = prev_trn - TRN_ACC_LIM
    
    prev_fwd = fwd
    prev_trn = trn

    return fwd, trn




def joy_cb(Joy):
    global ADJ_THROTTLE
    global FULL_THROTTLE
    global MAX_VEL
    global INC
    global PREV_CMD_TIME
    global PREV_SEQ_NUM

    global E_STOP
    global cmd
    global pub_cancel_move_base
    global last_estop_button
    global last_way_button
    global last_joycb_device_check
    

    cmd_time = float(Joy.header.stamp.secs) + (float(Joy.header.stamp.nsecs)/1000000000)
    rbt_time = time.time()
    signal_delay = rbt_time - cmd_time

    joy_delay = Float32()
    joy_delay.data = signal_delay
    pub_delay.publish(joy_delay)

    # len==8, 11 for axes,buttons with the xboxdrv
    # len==8, 15 for axes,buttons with the xpad.ko module
    if (len(Joy.axes)==8 and len(Joy.buttons)==11):
        # UP/DOWN buttons are on axes with xpad.ko
        USE_XPAD=True
        TURN_JOY=3
        U_PAD_BUTTON=7
        D_PAD_BUTTON=7
        U_PAD_BUTTON_VALUE=1
        D_PAD_BUTTON_VALUE=-1
    else:
        USE_XPAD=False
        TURN_JOY=2
        U_PAD_BUTTON = 13
        D_PAD_BUTTON = 14
        U_PAD_BUTTON_VALUE=1
        D_PAD_BUTTON_VALUE=1

    #Reject old TCP commands 
    '''
    if (signal_delay > 1.0 and Joy.header.seq == PREV_SEQ_NUM):
        rospy.logwarn('Rejected old TCP /joystick command: ' + str(signal_delay))
        return
    '''
    # the seq num auto-increments regardless of joystick being connected
    #print len(Joy.axes)
    #print len(Joy.buttons)
    #print Joy
    #return

    #Record timestamp and seq for use in next loop
    PREV_CMD_TIME = cmd_time
    PREV_SEQ_NUM = Joy.header.seq

    # check for e-stop button
    if Joy.buttons[A_BUTTON] == 1:
        # debounce 2 seconds
        if (time.time()-last_estop_button > 2.0):
            last_estop_button=time.time()
            # toggle e-stop mode
            if (E_STOP==0):
                E_STOP=1
                cmd.linear.x = 0
                cmd.angular.z = 0
                rospy.loginfo('User indicated E_STOP. Canceling any move_base commands, going to E_STOP state.')
                pub.publish(cmd)
                # cancel any existing move_base command
                nogoal = GoalID()
                pub_cancel_move_base.publish(nogoal)
            else:
                rospy.loginfo('User indicated leaving E_STOP. Going back to regular mode.')
                E_STOP=0
            return
    # check for save_waypoint button
    if Joy.buttons[B_BUTTON] == 1:
        if (time.time()-last_way_button > 5.0):
            last_way_button=time.time()
            rospy.loginfo('User saving a waypoint')
            os.system('/opt/ros/kinetic/bin/rostopic echo /odom/ekf/enc_imu_gps/pose/pose -n 1 >> /tmp/waypoints.txt')

    # stay in E_STOP mode until the button is pressed again
    if (E_STOP==1):
        # keep sending zeros
        cmd.linear.x = 0
        cmd.angular.z = 0
        pub.publish(cmd)
        return
    
    if ADJ_THROTTLE:
        # Increase/Decrease Max Speed
        if (USE_XPAD):
            if int(Joy.axes[U_PAD_BUTTON]) == U_PAD_BUTTON_VALUE:
                FULL_THROTTLE += INC
            if int(Joy.axes[D_PAD_BUTTON]) == D_PAD_BUTTON_VALUE:
                FULL_THROTTLE -= INC
        else:
            if Joy.buttons[U_PAD_BUTTON] == U_PAD_BUTTON_VALUE:
                FULL_THROTTLE += INC
            if Joy.buttons[D_PAD_BUTTON] == D_PAD_BUTTON_VALUE:
                FULL_THROTTLE -= INC
        
        # If the user tries to decrese full throttle to 0
        # Then set it back up to 0.2 m/s
        if FULL_THROTTLE <= 0.001:
            FULL_THROTTLE = INC

        # If the user tries to INCreas the velocity limit when its at max
        # then set velocity limit to max allowed velocity
        if FULL_THROTTLE >= MAX_VEL:
            FULL_THROTTLE = MAX_VEL

        # Report the new FULL_THROTTLE
        #if Joy.buttons[D_PAD_BUTTON] or Joy.buttons[U_PAD_BUTTON]:
        #    rospy.loginfo(FULL_THROTTLE)

        #Update DEADBAND
        DEADBAND = 0.2 * FULL_THROTTLE

    # Drive Forward/Backward commands
    drive_cmd = FULL_THROTTLE * Joy.axes[DRIVE_JOY] #left joystick
    if drive_cmd < DEADBAND and -DEADBAND < drive_cmd:
        drive_cmd = 0 
        
    # Turn left/right commands
    turn_cmd = FULL_THROTTLE * Joy.axes[TURN_JOY] #right joystick
    if turn_cmd < DEADBAND and -DEADBAND < turn_cmd:
        turn_cmd = 0

    #Limit acceleration
    #drive_cmd, turn_cmd = limit_acc(drive_cmd, turn_cmd)

    # update the last time joy_cb was called
    if (drive_cmd != 0) or (turn_cmd != 0):
        last_joycb_device_check=time.time()

    # Publish move commands
    cmd.linear.x = drive_cmd
    cmd.angular.z = turn_cmd
    pub.publish(cmd)
    
# Main Function
def joystick_main():

    # Initialize driver node
    rospy.init_node('joystick_node', anonymous=True)
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Subscribe to the joystick topic
        sub_cmds = rospy.Subscriber("joystick", Joy, joy_cb)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        r.sleep()
    
if __name__ == '__main__':
    try:
        joystick_main()
    except rospy.ROSInterruptException:
        pass

