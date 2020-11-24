#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from pantilt_control_ros_node.srv import PantiltControl
from pantilt_control_ros_node.srv import PantiltControlResponse


def pt_panoramic_client(command, new_angle):
    rospy.wait_for_service('pantilt_control')

    try:
        pt_panoramic = rospy.ServiceProxy('pantilt_control', PantiltControl)
        resp = pt_panoramic("set_angle", command, new_angle)

        return resp.response_sucess

    except rospy.ServiceException as e:
        print("Service call failed: %e" % e)


def usage():
    return "%s [command speed]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 3:
        command = str(sys.argv[1])
        new_angle = int(sys.argv[2])

    else:
        print(usage())
        sys.exit(1)

    print("Requesting panoramic --> %s --> %s " % (command, new_angle))
    pt_panoramic_client(command, new_angle)
