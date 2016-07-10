#!/usr/bin/env python

import time, os, cv2
import rospy

from geometry_msgs.msg import Twist

MAX_CAMERAS     = 10;   #Max camera to check if availble.


def getAvilableCamera():
    global CAP
    global CASC_PATH
    HOME = os.environ['HOME']
    CASC_PATH = HOME + '/ANPL/infrastructureproject/Data/haarcascade_frontalface_default.xml'

    for i in range(0, MAX_CAMERAS):
        CAP = cv2.VideoCapture(i)
        if CAP.isOpened():
            return i
    print "No Avilable Cameras"
    sys.exit(0)

def init():
    print "init"

    global CAP
    global PUB
    global SETTINGS
    global CENTER
    rospy.init_node('web_camera', anonymous=True)
    indexCamera = getAvilableCamera()
    width = CAP.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)
    CENTER = width / 2
    PUB = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=5)

def finish():
    print "\nFinish"
    # When everything done, release the capture
    stop()    
    CAP.release()
    cv2.destroyAllWindows()

def detect():
    print "press ctrl-c here or 'q' at the window to stop"
    while not rospy.is_shutdown():
        ret, image = CAP.read()

        # Create the haar cascade
        faceCascade = cv2.CascadeClassifier(CASC_PATH)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags = cv2.cv.CV_HAAR_SCALE_IMAGE
        )
        control_speed = 0
        control_turn = 0
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            print "w={0} ,h={0}".format(w,h);        
            control_turn  = (CENTER-(x+w/2))/(CENTER*1.0)
    	    control_speed = (100.0 - w)/(100.0*1.0)
        #print "control={0}".format(control_turn)
        twist = Twist()
        twist.linear.y = 0; twist.linear.z = 0; twist.linear.x = control_speed;
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn;
        PUB.publish(twist)

        
        cv2.imshow("Faces found", image)
        #close the window when you press 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def stop():
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    PUB.publish(twist)
    time.sleep(1)

if __name__ =="__main__":
    init()
    detect()
    finish()
