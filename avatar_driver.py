#!/usr/bin/env python

# Author: Erik Beall
# Description: This script converts /cmd_vel twist messages into 
# the appropriate commands to the Avatar Java SDK over sockets

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import String
import select
import socket
import time
import math

AvosSocket=None
# conversion from old joystick/old robot motor (max speed 2.6) to avatar (max speed 1000)
MOTOR_DRIVE_SCALING=384
last_acc=0.0
timer_clamping_on=False
left_setpoint=0
right_setpoint=0
left_cv=0
right_cv=0
left_incr=0
right_incr=0
enc_pub=None
status_pub=None
cmd_vel_timeout=0.5
curr_cmd_stamp=time.time()

def cmd_vel_cb(cmd):
    global AvosSocket, left_setpoint, right_setpoint, left_cv, right_cv, left_incr, right_incr, curr_cmd_stamp
    global timer_clamping_on


    curr_cmd_stamp = time.time()

    drive_cmd = int(MOTOR_DRIVE_SCALING*cmd.linear.x)
    turn_cmd = int(MOTOR_DRIVE_SCALING*cmd.angular.z)

    # we wish to send the left, right motor velocities from drive_cmd and turn_cmd (each between -1000 and 1000)
    # if abs(drive_cmd)+abs(turn_cmd)>1000, then we will need to adjust accordingly
    max_drive=abs(drive_cmd)+abs(turn_cmd)
    if (max_drive>1000):
        if (drive_cmd>0):
            drive_cmd=drive_cmd-(max_drive-1000)
        else:
            drive_cmd=drive_cmd+(max_drive-1000)
    # clamp motor values - linear motion must be at least 80, rotational motion must be at least 230 for "bigwheels"
    # alternatively, if rotational motion is below 230, add some linear motion
    clamp_val=80
    if (abs(drive_cmd)<clamp_val):
        drive_cmd=0
    #clamp_val=230
    #if (drive_cmd==0 and abs(turn_cmd)<clamp_val):
    #    turn_cmd=0
    left_setpoint=(drive_cmd-turn_cmd)
    right_setpoint=(drive_cmd+turn_cmd)
    clamp_val=40
    # only clamp if they are both below 80
    if (abs(left_setpoint)<2*clamp_val and abs(right_setpoint)<2*clamp_val):
        if (abs(left_setpoint)<clamp_val):
            left_setpoint=0
        if (abs(right_setpoint)<clamp_val):
            right_setpoint=0
    # what is the total difference between our current value and our setpoint
    left_delta=(left_setpoint-left_cv)
    right_delta=(right_setpoint-right_cv)
    # reset the increment values so we reach the setpoint in 0.4 second (increment by 1/4th setpoint every 100msec)
    left_incr=left_delta*0.25
    right_incr=right_delta*0.25
    # however, if delta is less than 1/4 max (or increments thereof), start at the lowest increment
    if (abs(left_delta)<100):
        #left_cv+=left_incr
        left_incr=left_delta*0.5
    if (abs(right_delta)<100):
        #right_cv+=right_incr
        right_incr=right_delta*0.5
    #left_cv+=left_incr
    #right_cv+=right_incr
    #send_motor_cmds_cb(None)
    #left_cv=left_setpoint
    #right_cv=right_setpoint

def parse_avatar_to_ros(buf):
    global enc_pub, status_pub
    mtype=buf.split(':')[0]
    if mtype=='M':
        mvel1=int(buf.split(':')[3].split(',')[0])
        mvel2=int(buf.split(':')[3].split(',')[1])
        menc1=int(buf.split(':')[1].split(',')[0])
        menc2=int(buf.split(':')[1].split(',')[1])
        cmd_msg = Vector3Stamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.vector.x= menc1
        cmd_msg.vector.y= menc2
        enc_pub.publish(cmd_msg)
    elif mtype=='S':
        smtemp1=int(buf.split(':')[1].split(',')[0])
        smtemp2=int(buf.split(':')[1].split(',')[1])
        scharge=int(buf.split(':')[3])
        sbat1=int(buf.split(':')[2].split(',')[0])
        sbat2=int(buf.split(':')[2].split(',')[1])
        cmd_msg = String()
        cmd_msg.data = "BatteryCharge="+str(sbat1)+','+str(sbat2)+":MotorTemperature="+str(smtemp1)+','+str(smtemp2)+":Charge="+str(scharge)
        status_pub.publish(cmd_msg)

def listen_to_avatar_cb(event):
    global AvosSocket, left_cv, right_cv
    # short timeout, less than half the fastest cycle time (100/2 = 50 milliseconds)
    timeout=50
    ready_to_read, ready_to_write, in_error = select.select([AvosSocket],[AvosSocket],[],timeout)
    if len(ready_to_read) != 0:
        buf = AvosSocket.recv(1024)
        if len(buf) != 0:
            # print, parse, send to ROS topics
            # parse into lines (if any)
            for bufline in buf.split('\n'):
                parse_avatar_to_ros(bufline)
    if len(ready_to_write) != 0:
    	avos_cmd='SET='+str(left_cv)+"_"+str(right_cv)+'_'+str(0)+'\n'
    	AvosSocket.sendall(avos_cmd)

def send_motor_cmds_cb(event):
    global AvosSocket, left_setpoint, right_setpoint, left_cv, right_cv, left_incr, right_incr, curr_cmd_stamp, cmd_vel_timeout

    # Check for command timeout - brake if cmd_vel has timout
    dt = time.time() - curr_cmd_stamp
    if( dt > cmd_vel_timeout):
        #rospy.logwarn("/cmd_vel timed out after " + str(dt) + "s , stopping robot")
        left_setpoint = 0
        right_setpoint = 0

    # compute the left_cv (current value) based on last cv and the setpoint
    left_delta=(left_setpoint-left_cv)
    right_delta=(right_setpoint-right_cv)
    if (abs(left_delta)>0):
        left_cv=left_cv+left_incr
    if (abs(right_delta)>0):
        right_cv=right_cv+right_incr
    # clamp to the setpoint if incremented over
    '''
    if (abs(left_cv)>abs(left_setpoint)):
        left_cv=left_setpoint
    if (abs(right_cv)>abs(right_setpoint)):
        right_cv=right_setpoint
    '''
    # only clamp to the setpoint if incremented over, IF the setpoint is greater than +-100
    if (abs(left_cv)>abs(left_setpoint) and abs(left_setpoint)>100):
        left_cv=left_setpoint
    if (abs(right_cv)>abs(right_setpoint) and abs(left_setpoint)>100):
        right_cv=right_setpoint
    left_cv=left_setpoint
    right_cv=right_setpoint
    left_cv=int(left_cv)
    right_cv=int(right_cv)
    TOGGLE_LED=0

    # send move commands down sockets interface
    '''
    avos_cmd='SET='+str(left_cv)+"_"+str(right_cv)+'_'+str(TOGGLE_LED)+'\n'
    AvosSocket.sendall(avos_cmd)
    '''

# Main Function
def avos_motor_driver_main():
    # Initialize driver node

    while not rospy.is_shutdown():

        # Subscribe to the cmd_vel topic - this takes in motor commands and saves them
        sub_cmds = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb)

        # Motor commands timer - here we take the saved motor commands, smooth them every 200 msec and send to the motor
        rospy.Timer(rospy.Duration(0.1), send_motor_cmds_cb)
        
        rospy.Timer(rospy.Duration(0.1), listen_to_avatar_cb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        r.sleep()
    
if __name__ == '__main__':
    # connect to socket for Avos Java SDK program
    AvosSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host=socket.gethostname()
    # set timeout to 1 second
    AvosSocket.settimeout(1)
    # set to nonblocking
    AvosSocket.setblocking(0)
    port=8123
    connection_success=False
    while not connection_success:
        try:
            AvosSocket.connect((host,port))
            connection_success=True
        except:
            #print 'Error connecting, retrying after 1 sec'
            time.sleep(2)
    # send initialization message
    res = AvosSocket.sendall("INIT\n")
    rospy.init_node('avos_motor_driver_node', anonymous=True)
    r = rospy.Rate(20) # 10hz
    # publish the commands actually sent to the motors - in lieu of actual motor encoders
    enc_pub = rospy.Publisher('/cmd_vel_enc', Vector3Stamped, queue_size=3)
    status_pub = rospy.Publisher('/avatar_status', String, queue_size=3, latch=True)

    try:
        avos_motor_driver_main()
    except rospy.ROSInterruptException:
        pass

    AvosSocket.close()

