#!/usr/bin/env python

from gtsam_infrastructure.srv import *
import rospy
from sensor_msgs.msg    import LaserScan, Image, CompressedImage, PointCloud2, Imu, NavSatFix, Illuminance
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf
import math


def handle_request(req):
    print "Handle Request."
    global px, py, ptheta

    rot_m = np.array(req.rot_m);
    rot_m = rot_m.reshape((3, 3));
    euler = tf.transformations.euler_from_matrix(rot_m);
    yaw = euler[2];
    final_dir = ptheta + yaw;
    final_dir = normalizeAngle(final_dir);

    alpha = np.arctan2(req.p_org[1], req.p_org[0]);
    next_coord_dir = ptheta + alpha;
    next_coord_dir = normalizeAngle(next_coord_dir);
    turn(alpha);

    dist = np.sqrt(np.power(req.p_org[0], 2) + np.power(req.p_org[1], 2))
    next_coord_x = px + dist*math.cos(next_coord_dir)
    next_coord_y = py + dist*math.sin(next_coord_dir)
    print "coordinates ", next_coord_x, next_coord_y;

    # Move to origin of next coordinate system
    moveUpToGoal(next_coord_x, next_coord_y);

    # Turn to the right direction
    turnToDirection(final_dir);

    goalDist = euDistance(next_coord_x, next_coord_y, px, py);
    print "goalDist", goalDist

    goalAngleDiff = calcAngleDiff(final_dir, ptheta);
    print "goalAngleDiff", goalAngleDiff

    img, dpt = getCurrentViewAndDepth();

    return TurtlebotControllerServiceResponse(img, dpt, [px, py, ptheta]);

def init():
    print "Init"
    global GT_SUBSCRIBER
    global IMAGE_SUBSCRIBER
    global DEPTH_SUBSCRIBER
    global MOVE_PUBLISHER

    rospy.init_node('turtlebot_controller_service_server')
    s = rospy.Service('turtlebot_controller_service', TurtlebotControllerService, handle_request)

    GT_SUBSCRIBER = rospy.Subscriber("/turtlebot_ground_truth_bc",  Odometry, groundTruthRobotPoseCallback, queue_size=1)
    IMAGE_SUBSCRIBER = rospy.Subscriber("/camera/rgb/image_raw/compressed",  CompressedImage, imageCallback, queue_size=1)
    DEPTH_SUBSCRIBER = rospy.Subscriber("/camera/depth/image_raw",  Image, depthCallback, queue_size=1)
    MOVE_PUBLISHER    = rospy.Publisher ("/cmd_vel_mux/input/teleop",           Twist,         queue_size=1)

    print "TurtlebotControllerService is Ready."
    rospy.spin()

def finish():
    GT_SUBSCRIBER.unregister();    
    IMAGE_SUBSCRIBER.unregister();
    DEPTH_SUBSCRIBER.unregister();
    MOVE_PUBLISHER.unregister();

    print "\nFinish"

global lastImage
lastImage = None

global lastDepth
lastDepth = None

global px, py, ptheta
px = 0
py = 0
ptheta = 0

def groundTruthRobotPoseCallback (data):
    global px, py, ptheta
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y

    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion);
    yaw = euler[2];

    if (yaw > 0):
        ptheta = yaw;
    else:
        ptheta = 2*np.pi + yaw;

def imageCallback (data):
    global lastImage
    lastImage = data

def depthCallback (data):
    global lastDepth
    lastDepth = data

def moveUpToGoal(goalX, goalY):
    global px, py, ptheta
    prevGoalDist = euDistance(goalX, goalY, px, py);
    max_speed = 0.5;
    min_speed = 0.0002;
	
    while (not rospy.is_shutdown()):
        # Calculate current distance to the goal
        goalDist = euDistance(goalX, goalY, px, py);

        # We stop when or goal is reached or we start to go away from it
        if (goalDist < 0.0001 or (goalDist < 0.2 and prevGoalDist < goalDist)):
            stop();
            return;

        # Calculate speed
        if (goalDist > 1):
            xVel = max_speed;
        elif (goalDist > 0.2):
            xVel = max_speed/2;
        elif (goalDist > 0.05):
            xVel = max_speed/4;
        else:
            # In the end decrease speed for more precise movement
            #xVel = (-max_speed * goalDist) + max_speed;
            #xVel = (max_speed * goalDist * goalDist) - (2 * max_speed * goalDist) + max_speed;
            xVel = (max_speed/4) * goalDist / 0.05;
            xVel = max(xVel, min_speed);

        alpha = getGoalDirection(goalX, goalY);

        # Calculate direction of goal
        alpha = getGoalDirection(goalX, goalY);

        # Update our robot's direction in order for it to move to the goal
        angZError = calcAngleDiff(alpha, ptheta);
        if (angZError < -(np.pi / 4)):
            angZError = -(np.pi / 4);
        if (angZError > (np.pi / 4)):
            angZError = (np.pi / 4);

        move(xVel, angZError);
        prevGoalDist = goalDist;

def turnToDirection(direction):
    global px, py, ptheta
    prevGoalDist = calcAngleDiff(direction, ptheta);
    max_speed = np.pi / 4;
    min_speed = np.pi / 100;
	
    while (not rospy.is_shutdown()):
        # Calculate current distance to the goal
        goalDist = calcAngleDiff(direction, ptheta);

        # We stop when or goal is reached or we start to go away from it
        if (goalDist < 0.0001 or prevGoalDist < goalDist):
            stop();
            return;

        # Update our robot's direction in order for it to move to the goal
        angZError = goalDist;
        if (angZError < -np.pi):
            angZError = -np.pi;
        if (angZError > np.pi):
            angZError = np.pi;

        # Calculate angle speed
        diffSign = np.sign(angZError);
        diffValue = math.fabs(angZError);
        if (diffValue > (20 * np.pi / 180)):
            angZError = diffSign * max_speed;
        elif (diffValue > (10 * np.pi / 180)):
            angZError = diffSign * max_speed / 2;
        else:
            # In the end decrease speed for more precise movement
            angZError = (diffSign * max_speed / 2) * (diffValue / (10 * np.pi / 180));
            angZError = max(angZError, min_speed);

        move(0, angZError);
        prevGoalDist = goalDist;

# Calculating in what direction and how much to turn,
# in order to get from direction "ptheta" to direction "alpha"
def calcAngleDiff(alpha, ptheta):
	if (alpha == ptheta):
		return 0.0;
	elif (alpha > ptheta):
		eCC = alpha - ptheta;
		eC = 2*np.pi - (eCC);
		
		if (eCC > eC):
			return -eC;
		else:
			return eCC; 
	else:
		eC = -(alpha - ptheta);
		eCC = 2*np.pi - (eC);
		
		if (eCC > eC):
			return -eC;
		else:
			return eCC; 

def euDistance(start_x, start_y, end_x, end_y):
    d = np.sqrt(np.power(start_x - end_x, 2) + np.power(start_y - end_y, 2));
    return d;

def stop():
	# Stopping turtlebot
	xVel = 0;
	angZVel = 0;
	move(xVel, angZVel, 10);

def move(xVel, angZVel, count = 1):
	# Init direction that turtlebot should go
	twist = Twist()
	twist.linear.x = twist.linear.y = twist.angular.z = 0;
	twist.linear.x = xVel; #m/s	
	twist.angular.z = angZVel; #rad/s
	
	# 10 times, 10 Hz - will move for one sec
	rate = rospy.Rate(10) # 10 Hz
	
	while(not rospy.is_shutdown() and count > 0):
		# "publish" sends the command to turtlebot to keep going
		MOVE_PUBLISHER.publish(twist);
		rate.sleep();
		count = count - 1;

def turn(angular_z):
    if (angular_z == 0):
        return;
    twist = Twist()
    twist.linear.x = twist.linear.y = twist.linear.z = 0;
    twist.angular.z = np.pi/10; # rad's speed
    rate = rospy.Rate(10) # 10 Hz
    count = 0
    countMax = angular_z/twist.angular.z*10;

    while not rospy.is_shutdown() and count < countMax:
        MOVE_PUBLISHER.publish(twist);
        rate.sleep();
        count = count + 1;

def getCurrentViewAndDepth():
    global lastImage
    global lastDepth
    lastImage = None;
    lastDepth = None;
    waitUntilAnyImageAndAnyDepthArrive();

    img = CompressedImage()
    img.header.stamp = rospy.Time.now()
    img.format = lastImage.format
    img.data = lastImage.data

    dpt = Image()
    dpt.header.stamp = rospy.Time.now()
    dpt.height = lastDepth.height
    dpt.width = lastDepth.width
    dpt.encoding = lastDepth.encoding
    dpt.is_bigendian = lastDepth.is_bigendian
    dpt.step = lastDepth.step
    dpt.data = lastDepth.data

    return img, dpt

def waitUntilAnyImageAndAnyDepthArrive():
    while (not lastImage and not lastDepth):
        rospy.sleep(0.1);

def normalizeAngle(angle):
	if (angle < 0):
		angle = angle + 2*np.pi;

	if (angle > 2*np.pi):
		angle = angle - 2*np.pi;

	return angle;

def getGoalDirection(goalX, goalY):
	# Calculate direction of goal
    global px, py, ptheta
    alpha = normalizeAngle(math.atan2(goalY - py, goalX - px));
    return alpha;

if __name__ == "__main__":
    init()
    finish()

