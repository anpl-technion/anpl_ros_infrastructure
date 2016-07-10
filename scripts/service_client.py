#!/usr/bin/env python

import sys
import rospy
from gtsam_infrastructure.srv import *
import numpy as np
import math


def turtlebot_controller_client(p_org, rot_m):
    rospy.wait_for_service('turtlebot_controller_service')
    try:
        turtlebot_controller_service = rospy.ServiceProxy('turtlebot_controller_service', TurtlebotControllerService)
        resp1 = turtlebot_controller_service(p_org, rot_m.flatten());
        return resp1.img, resp1.dpt, resp1.ground_truth
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    p_org = [3, 3, 0];
    theta = 90 * np.pi / 180;
    rot_m = np.array([[math.cos(theta), -math.sin(theta), 0],
                      [math.sin(theta), math.cos(theta),  0],
                      [0,               0,                1]]);

    print "Requesting ", p_org
    (img, dpt, gt) = turtlebot_controller_client(p_org, rot_m)
    print "Response is:"
    print "Image:", img.format
    print "Depth:", dpt.width, dpt.height, dpt.encoding
    print "Ground Truth:", gt
