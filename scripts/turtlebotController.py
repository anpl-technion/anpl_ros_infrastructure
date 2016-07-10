#!/usr/bin/env python


import rospy, os
import cPickle          as pickle 
import rosbag
from sensor_msgs.msg    import LaserScan, Image, CompressedImage, PointCloud2, Imu, NavSatFix, Illuminance
from geometry_msgs.msg import Twist
from datetime           import datetime
from std_msgs.msg       import String
from os.path            import expanduser
from common             import *
from threading          import *



def init():
    print "Init"
    global IMAGE_SUBSCRIBER
    global MOVE_PUBLISHER
    global IMAGE_PUBLISHER

    rospy.init_node("turtlebotController")

    IMAGE_SUBSCRIBER = rospy.Subscriber("/camera/rgb/image_raw/compressed",  CompressedImage, imageCallback, queue_size=1)
    MOVE_PUBLISHER    = rospy.Publisher ("/cmd_vel_mux/input/teleop",           Twist,         queue_size=1)
    IMAGE_PUBLISHER    = rospy.Publisher ("/TurtlebotController/camera/compressed",           CompressedImage,         queue_size=10)
    
global lastImage
lastImage = None

def imageCallback (data):
    global lastImage
    lastImage = data


def finish():
    IMAGE_SUBSCRIBER.unregister();    
    MOVE_PUBLISHER.unregister()
    IMAGE_PUBLISHER.unregister()
    print "\nFinish"

def controll():
    print "Controll"

    moveMeterForward();
    publishCurrentView();


def moveMeterForward():
    twist = Twist()
    twist.linear.x = twist.linear.y = twist.linear.z = 0;
    twist.linear.x = 0.1; # move forward at 0.1 m/s 
    rate = rospy.Rate(10) # 10hz
    count = 0

    while not rospy.is_shutdown() and count < 100:
        MOVE_PUBLISHER.publish(twist);
        rate.sleep();
        count = count + 1;

def publishCurrentView():
    waitUntilAnyImageArrive();

    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = lastImage.format
    msg.data = lastImage.data

    IMAGE_PUBLISHER.publish(msg)


def waitUntilAnyImageArrive():
    while (not lastImage):
        rospy.sleep(0.1);


#main function call init, controll, finish
if __name__=="__main__":
    init()
    controll()
    finish()


