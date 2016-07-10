#!/usr/bin/env python


import rospy, os
import cPickle          as pickle 
import rosbag
from sensor_msgs.msg    import LaserScan, Image, CompressedImage, PointCloud2, Imu, NavSatFix, Illuminance
from datetime           import datetime
from std_msgs.msg       import String
from os.path            import expanduser
from common             import *



PRINT_SEND_DATA                         = True
SAVE                                    = True          # True will create a bag file
BAG_NAME                                = ""            # the name of the Bag file. if set to "" or False will use current time
BAG_PATH                                = "/ANPL/Data/" # the relative path from Home direcory to the bag location. if set to "" will use current location


class SensorListener (object):
    def __init__ (self,topic_name, msg_class, print_callback):
        self.topic_name = topic_name
        self.msg_class  = msg_class
        self.print_callback = print_callback

        self.subscriber = None

    def start (self):
        self.subscriber = rospy.Subscriber(self.topic_name,  self.msg_class, self.topicCallback)

    def topicCallback (self, data):
        #for debug
        data_class_name = data.__class__.__name__
        if self.print_callback:       
            print "read data: [topic = {0}; time = {1}]".format(self.topic_name, datetime.now().strftime("%H:%M:%S.%f"))
            print "data: [{0}]".format(data_class_name)

        send_data(data_class_name, self.topic_name, data);

    def stop (self):
        if (self.subscriber is not None):
            self.subscriber.unregister();
            self.subscriber = None;


#For new sensor, please add new listener:
SENSOR_LISTENERS = (

SensorListener("phone1/android/imu",  Imu, False),
SensorListener("phone1/android/illuminance",  Illuminance, False),
SensorListener("phone1/camera/image/compressed",  CompressedImage, False),

# Gazebo turtlebot image
SensorListener("/camera/rgb/image_raw/compressed",  CompressedImage, False),
# Gazebo turtlebot imu
SensorListener("/mobile_base/sensors/imu_data",  Imu, False),

# Image from our Gazebo turtlebot controller
SensorListener("/TurtlebotController/camera/compressed",  CompressedImage, False)

);



def send_data(class_name, topic_name, data):
    sensor      = Sensor(class_name, topic_name, data)
    string_data = pickle.dumps(sensor)
    strdata        = String(string_data)
    if PRINT_SEND_DATA:       
        print "send data {0}".format(datetime.now().strftime("%H:%M:%S.%f"))
        print "class name {0}".format(class_name)
    if SAVE:
        with BAG_LOCK:
            BAG.write(topic_name, data)
    PUBLISHER_WORKST.publish(strdata)


#  init a bag file , his location and name.
def init_bag():
    global BAG
    global BAG_LOCK
    name_str = format(datetime.now().strftime("%y_%m_%d_%H:%M:%S")) + ".bag"
    name_str = BAG_NAME + name_str;
    if BAG_PATH:                
        home = expanduser("~")
        directory = home + BAG_PATH
        if not os.path.exists(directory):
            os.makedirs(directory)
        name_str = home + BAG_PATH + name_str
    BAG = rosbag.Bag(name_str, 'w')
    print "Bag created to " + name_str


#function init is called at the begging of main function
#if subscribes to all topics we need    
def init():
    print "Init"
    global PUBLISHER_WORKST
    # from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node("collector", anonymous=True)
    rospy.on_shutdown(finish)

    if SAVE:
        init_bag()

    # Start all listeners    
    for i in range(len(SENSOR_LISTENERS)):
        SENSOR_LISTENERS[i].start()

    PUBLISHER_WORKST    = rospy.Publisher ("/WorkST1/robot1",           String,         queue_size=10)
    

def finish():
    # Stop all listeners    
    for i in range(len(SENSOR_LISTENERS)):
        SENSOR_LISTENERS[i].stop()

    if SAVE:
        BAG.close()
    PUBLISHER_WORKST.unregister()
    print "\nFinish"
    rospy.signal_shutdown("Finish")

def collect():
    print "Collect"
    while not rospy.is_shutdown():
        pass


def test_type():
    if type(Sensor) == type:
        print("type")

    else:
        print("not type")

#main function call init, collector, finish
if __name__=="__main__":
    init()
    collect()


