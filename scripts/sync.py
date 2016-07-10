#!/usr/bin/env python

import rospy 
import cPickle          as pickle
from std_msgs.msg       import String
from common             import *

PRINT_CALL_BACK_ROBOT1  = False
PRINT_READ_DATA         = True
SAVE_TO_FILE            = False

def save_data(sensor):
    FILE_INPUT.write(str(sensor))
    FILE_INPUT.write("\n\n")

def read_data(data):
    sensor = pickle.loads(data.data)
    if PRINT_READ_DATA:
        print (sensor.class_name)
        print (sensor.topic_name)
        print (sensor.data)
    if SAVE_TO_FILE:
        save_data(sensor)

#interapt function to get the robot1 data
def call_back_robot1(data):
    #for debug
    if PRINT_CALL_BACK_ROBOT1:       
        print "read robot1 data at {0}".format(datetime.now().strftime("%H:%M:%S.%f"))
    read_data(data)
    
#function init is called at the begging of main function
#if subscribes to all topics we need    
def init():
    global FILE_INPUT
    global SUB_ROBOT1
    print "Init"
    if SAVE_TO_FILE:
        FILE_INPUT = open('datafile.txt', 'w')
    # from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node("sync", anonymous=True)
    
    
    #For new robot, please add new Subscriber:
    SUB_ROBOT1 = rospy.Subscriber("/WorkST1/robot1", String, call_back_robot1)

def finish():
    SUB_ROBOT1.unregister()
    print "Finish"
    
def snyc():
    print "Sync"
    rospy.spin()

#main function call init and then bug algorithm    
if __name__=="__main__":
    init()
    snyc()
    finish()
