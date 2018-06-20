#!/bin/bash

homedir=`rospack find rr_openrover_pro`
ret=$?
if [ $ret -ne 0 ] ; then
   echo "rr_openrover_pro not in path, make sure it is in the ROS_PACKAGE_PATH"
   exit -1
fi

cd $homedir/src
progname=RobotROSInterface
if [ ! -f $progname.java ] ; then
    echo "No $progname.java file found, error in installation..."
    exit -1
fi

if [ ! -f $progname.class ] ; then
    echo "No .class file found, compiling java SDK file for first time..."
    javac $progname.java
fi

if [ ! -f $progname.class ] ; then
    echo "No .class file found after compiling, error..."
    #exit -1
fi

ret=`ps -Alf | grep $progname | grep -v grep`
ret=$?
if [ $ret -eq 1 ] ; then
    echo "Starting SDK"
    java $progname > /tmp/avos_ros2sdk_iface.txt 2>&1 
fi

