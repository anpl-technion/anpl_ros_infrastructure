#!/usr/bin/env python

#from http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_video_display/py_video_display.html
#from https://github.com/turtlebot/turtlebot_apps/blob/hydro/turtlebot_teleop/scripts/turtlebot_teleop_key
#from http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

import cv2, rospy, sys
import numpy as np
from sensor_msgs.msg        import Image, CompressedImage, CameraInfo
from cv_bridge              import CvBridge


SHOW            = False #if True, show window of image
IS_GRAY         = False #if enable, send/show image at gray scale
IS_COMPRESSED   = False #if enable, send the image at compressed data (jpeg)
MAX_CAMERAS     = 10;   #Max camera to check if availble.

def getAvilableCamera():
    global CAP

    for i in range(0, MAX_CAMERAS):
        CAP = cv2.VideoCapture(i)
        if CAP.isOpened():
            return i
    print "No Avilable Cameras"
    sys.exit(0)

def init():
    print "init"

    global CAP
    global BRIDGE
    global PUBLISHER_CAMERA_COMPRESSED
    global PUBLISHER_CAMERA_RAW
    global PUBLISHER_CAMERA_INFO
    global SETTINGS

    rospy.init_node('web_camera', anonymous=True)
    indexCamera = getAvilableCamera()
    print indexCamera
    BRIDGE = CvBridge()
    prefixname = 'web' + str(indexCamera)
    if IS_COMPRESSED:
        PUBLISHER_CAMERA_COMPRESSED = rospy.Publisher(prefixname + "/webcamera/image/compressed",   CompressedImage,    queue_size=30)
    else:
        PUBLISHER_CAMERA_RAW        = rospy.Publisher(prefixname + "/webcamera/image/raw",          Image,              queue_size=30)
    
    PUBLISHER_CAMERA_INFO           = rospy.Publisher(prefixname + "/webcamera/camera_info",        CameraInfo,         queue_size=30)
    

def send():
    print "Send"
    print "press ctrl-c here or 'q' on the window to stop"
    while not rospy.is_shutdown():
        # Capture frame-by-frame
        ret, frame = CAP.read()
      
        if IS_GRAY:
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            image = frame

        stamp = rospy.Time.now();
        if IS_COMPRESSED:
            #from http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
            #### Create CompressedIamge ####
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
            # Publish new image
            PUBLISHER_CAMERA_COMPRESSED.publish(msg)
        else:
            # convert the resulting frame to ROS image
            ros_image = BRIDGE.cv2_to_imgmsg(image,"bgr8")
            #adding time stamp
            ros_image.header.stamp = stamp
	    ros_image.header.frame_id = "webcamera_optical_frame"
            # publish ROS image
            PUBLISHER_CAMERA_RAW.publish(ros_image)
                
	# mostly copyied from turtulbot gazebo simulation        
	cameraInfo = CameraInfo()
        cameraInfo.header.stamp = stamp
	cameraInfo.header.frame_id = "webcamera_optical_frame"
	cameraInfo.height = 480
	cameraInfo.width = 640
	cameraInfo.distortion_model = "plump_bob"
	cameraInfo.D = [0.0, 0.0, 0.0, 0.0, 0.0]
	cameraInfo.K = [554.25, 0.0, 320.5, 0.0, 554.25, 240.5, 0.0, 0.0, 1.0]
	cameraInfo.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
	cameraInfo.P = [554.25, 0.0, 320.5, -0.0, 0.0, 554.25, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        PUBLISHER_CAMERA_INFO.publish(cameraInfo)
        # Display the resulting frame 
        if SHOW:        
            cv2.imshow('image',image)
        #close the window when you press 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def finish():
    print "\nFinish"
    # When everything done, release the capture
    CAP.release()
    cv2.destroyAllWindows()
    if IS_COMPRESSED:
        PUBLISHER_CAMERA_COMPRESSED.unregister()
    else:
        PUBLISHER_CAMERA_RAW.unregister()

if __name__ =="__main__":
    init()
    send()
    finish()
    
