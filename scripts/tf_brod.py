#!/usr/bin/env python

#from http://docs.ros.org/indigo/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html
#from https://github.com/ros-visualization/visualization_tutorials/blob/groovy-devel/rviz_plugin_tutorials/scripts/send_test_msgs.py

import roslib
import rospy, tf
from math               import cos, sin
from sys                import stdout
from sensor_msgs.msg    import Imu


def init():
    global BR
    global COUNTER
    global DOTS
    print "Init"
    rospy.init_node( 'test_imu' )
    BR      = tf.TransformBroadcaster()
    COUNTER = 0
    DOTS    = 1


def broadcast():
    rate    = rospy.Rate(10)
    radius  = 5
    angle   = 0
    dist    = 3
    i       = 0
    while not rospy.is_shutdown():
        BR.sendTransform((radius * cos(angle), radius * sin(angle), 0),
                         tf.transformations.quaternion_from_euler(0, 0, angle),
                         rospy.Time.now(),
                         "imu",
                         "map")
        print_brod()
        rate.sleep()

def print_brod():
    global COUNTER
    global DOTS
    if COUNTER % 5 == 0:        
            str = ''
            for i in range(0,DOTS):
                str += '.'
            for j in range(i,3):
                str += ' '
            stdout.write("\rbroadcasting %s" % str)
            stdout.flush()
            DOTS += 1
            if DOTS % 4 == 0:
                DOTS = 1
            COUNTER = 0
    COUNTER += 1

def finish():
    print "\nFinish"

#main function call init, broadcast, finish
if __name__=="__main__":
    init()
    broadcast()
    finish()
