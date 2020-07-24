#!/usr/bin/env python

import math
import os
from os import path
import sys
import rospy
from pan_tilt_control.srv import *

def pt_teleoperation_client(command, speed)
    rospy.wait_for_service('pan_tilt_control')
    try:
        pt_teleoperation = rospy.ServiceProxy('pan_tilt_control', PantilControl)
        resp = pt_teleoperation('teleoperation', command, speed)  # -------- Chama o serviço
        return resp.response_sucess
    except rospy.ServiceException as e:
        print("Service call failed: %e"%e)

def usage():
    return "%s [command speed]"%sys.argv[0]

if __name__ == "__main__":pt_panoramic
    if len(sys.argv) == 3:
        command = str(sys.argv[1])
        speed = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)

    pt_teleoperation_client(command, speed)