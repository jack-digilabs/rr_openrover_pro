#!/usr/bin/env python

# Author: Erik Beall and Nick Fragale
# Description: This script converts /cmd_vel twist messages into 
# the appropriate commands to the Avatar Java SDK over sockets

import numpy as np
import rospy
import os
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int32
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
battery1_pub=None
battery2_pub=None
charging_pub=None
cmd_vel_timeout=0.5
curr_cmd_stamp=time.time()
LED_VALUE=0
cur_drive_cmd=0
cur_turn_cmd=0
wheel_diameter = rospy.get_param('default_param', 0.254) # 0.254 meters (10")
wheel_base = rospy.get_param('default_param', 0.3683) # 0.3683 meters (14.5")

bat_array=[]
override=-1
def override_cb(msg):
    global override
    # override == -1, no override, ==1, set to charging, ==0, set to not charging
    override=msg.data

def led_cb(cmd):
    global LED_VALUE
    value=cmd.data
    if value>50:
        value=50
    if value<0:
        value=0
    LED_VALUE=value

def cmd_vel_cb(cmd):
    global AvosSocket, left_setpoint, right_setpoint, left_cv, right_cv, left_incr, right_incr, curr_cmd_stamp
    global timer_clamping_on
    global cur_drive_cmd, cur_turn_cmd


    curr_cmd_stamp = time.time()

    drive_cmd = int(MOTOR_DRIVE_SCALING*cmd.linear.x)
    turn_cmd = int(MOTOR_DRIVE_SCALING*cmd.angular.z)
    cur_drive_cmd = cmd.linear.x
    cur_turn_cmd = cmd.angular.z

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
    global enc_pub, status_pub, battery1_pub, battery2_pub
    global cur_drive_cmd, cur_turn_cmd
    global wheel_diameter, wheel_base
    global charging_pub, bat_array, override
    mtype=buf.split(':')[0]
    if mtype=='M':
        # 1 is Left motor
        # 2 is right motor
        mvel1=int(buf.split(':')[3].split(',')[0])
        mvel2=int(buf.split(':')[3].split(',')[1])
        mrpm1=float(buf.split(':')[2].split(',')[0]) 
        mrpm2=float(buf.split(':')[2].split(',')[1])
        menc1=int(buf.split(':')[1].split(',')[0])
        menc2=int(buf.split(':')[1].split(',')[1])
        cmd_msg = Vector3Stamped()
        odom_msg = Odometry()
        odom_diff_msg = Vector3Stamped()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.vector.x= menc1
        cmd_msg.vector.y= menc2
        # Equations of motion for two wheeled robot
        # https://robotics.stackexchange.com/questions/106/what-is-a-suitable-model-for-two-wheeled-robots
        wheel_circumferance = wheel_diameter * 3.14159
        vel1 = mrpm1 / 60 * wheel_circumferance
        vel2 = mrpm2 / 60 * wheel_circumferance
        friction_factor = 0.95
        forward_velocity = 0.5 * (vel1 + vel2)
        turning_velocity = (1 / wheel_base) * (vel2 - vel1) * friction_factor
        odom_msg.twist.twist.linear.x = forward_velocity
        odom_msg.twist.twist.angular.z = turning_velocity 
        odom_diff_msg.vector.x = cur_drive_cmd - forward_velocity
        odom_diff_msg.vector.z = cur_turn_cmd - turning_velocity
        enc_pub.publish(cmd_msg)
        odom_pub.publish(odom_msg)
        odom_diff_pub.publish(odom_diff_msg)
    elif mtype=='S':
        smtemp1=int(buf.split(':')[1].split(',')[0])
        smtemp2=int(buf.split(':')[1].split(',')[1])
        scharge=int(buf.split(':')[3])
        sbat1=int(buf.split(':')[2].split(',')[0])
        sbat2=int(buf.split(':')[2].split(',')[1])
        with open('/sys/class/thermal/thermal_zone0/temp') as fp:
            ptemp=int(fp.readline().split('\n')[0])
        cmd_msg = String()
        cmd_msg.data = "BatteryCharge="+str(sbat1)+','+str(sbat2)+":MotorTemperature="+str(smtemp1)+','+str(smtemp2)+":Charge="+str(scharge)+":PayloadTemperature="+str(ptemp/1000.0)
        status_pub.publish(cmd_msg)
        cmd1_msg = Int32()
        cmd1_msg.data = sbat1
        battery1_pub.publish(cmd1_msg)
        cmd2_msg = Int32()
        cmd2_msg.data = sbat2
        battery2_pub.publish(cmd2_msg)
        temp1_msg = Int32()
        temp1_msg.data = smtemp1
        motor1temp_pub.publish(temp1_msg)
        temp2_msg = Int32()
        temp2_msg.data = smtemp2
        motor2temp_pub.publish(temp2_msg)
        ptemp_msg = Int32()
        ptemp_msg.data = ptemp
        processortemp_pub.publish(ptemp_msg)
        # do our own charge calculation
        if (sbat1>0 and sbat2>0):
            charge_level=0.5*float(sbat1+sbat2)
        elif (sbat1>0):
            charge_level=float(sbat1)
        elif (sbat2>0):
            charge_level=float(sbat2)
        else:
            # reset bat_array
            bat_array=[]
            return
        # grow bat_array as we get callbacks over time, we only need 2 minutes of data to infer charging state
        bat_array.append(charge_level)
        cmd=Int32()
        # If one cell is above 99 and other is at 99 or higher, we're probably charging
        if charge_level>99.0:
            cmd.data=1
        elif (len(bat_array)<4):
            # not ready to tell if we're charging or not yet
            cmd.data=-1
        else:
            # are we increasing/staying same (1) or decreasing (0) over last two minutes
            # do not use constant level to indicate charging, not meaningful unless the battery is older and drains quickly enough
            if (np.mean(bat_array[-2:])-np.mean(bat_array[-4:-2]))>0:
                cmd.data=1
            elif (np.mean(bat_array[-2:])-np.mean(bat_array[-4:-2]))<0:
                cmd.data=0
            else:
                # can't tell
                #cmd.data=-1
                # or do not send anything
                cmd.data = None
        # done with charging logic, now pull in override logic (if set)
        if (override==0):
            cmd.data=0
        elif (override==1):
            cmd.data=1
        if (cmd.data is not None):
            charging_pub.publish(cmd)
        # limit to last 4 measurements
        bat_array=bat_array[-6:]

def listen_to_avatar_cb(event):
    global AvosSocket, left_cv, right_cv, LED_VALUE
    # short timeout, less than half the fastest cycle time (100/2 = 50 milliseconds)
    timeout=50
    try:
        ready_to_read, ready_to_write, in_error = select.select([AvosSocket],[AvosSocket],[],timeout)
    except:
        print('Socket error in select() in avatar_driver.py, exiting')
        os._exit(1)
    if len(ready_to_read) != 0:
        try:
            buf = AvosSocket.recv(1024)
        except:
            print('Socket error in recv() in avatar_driver.py, exiting')
            os._exit(1)
        if len(buf) != 0:
            # print, parse, send to ROS topics
            # parse into lines (if any)
            for bufline in buf.split('\n'):
                parse_avatar_to_ros(bufline)
    if len(ready_to_write) != 0:
    	avos_cmd='SET='+str(left_cv)+"_"+str(right_cv)+'_'+str(LED_VALUE)+'\n'
        #print('Sending '+avos_cmd)
        try:
    	    AvosSocket.sendall(avos_cmd)
        except:
            print('Socket error in sendall() in avatar_driver.py, exiting')
            os._exit(1)

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
        sub_override = rospy.Subscriber("/avatar/charging_override", Int32, override_cb)

        # Motor commands timer - here we take the saved motor commands, smooth them every 200 msec and send to the motor
        rospy.Timer(rospy.Duration(0.1), send_motor_cmds_cb)
        
        rospy.Timer(rospy.Duration(0.1), listen_to_avatar_cb)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        r.sleep()
    
if __name__ == '__main__':
    # pause 1 sec to allow java SDK node to startup
    time.sleep(1)
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
    enc_pub = rospy.Publisher('/avatar/enc', Vector3Stamped, queue_size=10)
    odom_pub = rospy.Publisher('/avatar/odom', Odometry, queue_size=10)
    odom_diff_pub = rospy.Publisher('/avatar/odom_diff', Vector3Stamped, queue_size=10)
    status_pub = rospy.Publisher('/avatar/status', String, queue_size=1, latch=True)
    battery1_pub = rospy.Publisher('/avatar/battery/cell1/soc', Int32, queue_size=1, latch=True)
    battery2_pub = rospy.Publisher('/avatar/battery/cell2/soc', Int32, queue_size=1, latch=True)
    motor1temp_pub = rospy.Publisher('/avatar/motor1/temp', Int32, queue_size=1, latch=True)
    motor2temp_pub = rospy.Publisher('/avatar/motor2/temp', Int32, queue_size=1, latch=True)
    processortemp_pub = rospy.Publisher('/processor/temp', Int32, queue_size=1, latch=True)
    led_sub = rospy.Subscriber("/avatar/led", Int32, led_cb, queue_size=10)

    charging_pub = rospy.Publisher('/avatar/charging', Int32, queue_size=1, latch=True)
    cmd=Int32()
    cmd.data=-1
    # on startup, assume we're charging
    cmd.data=1
    charging_pub.publish(cmd)

    try:
        avos_motor_driver_main()
    except rospy.ROSInterruptException:
        pass

    AvosSocket.close()

