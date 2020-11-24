#!/usr/bin/env python

from __future__ import print_function

import math
import os
from os import path
import sys
import rospy
from pantilt_control_ros_node.srv import PantiltControl
from pantilt_control_ros_node.srv import PantiltControlResponse


def take_picture(type_image, name_file):
    if type_image == 0:
        return os.system("sudo ./fluke-ir {0}".format(name_file))
    elif type_image == 1:
        return os.system("sudo timeout 5 ./shoot_an_image_and_get_it.sh"
                         " {0}.jpg".format(name_file))


'''
#H: 90 V: 30 - Sobreposicao=0
tilt = [0, 30, 60, 90]          # Angulo do tilt
pan_fotosH = [4, 4, 4, 1]       # Numero de fotos
pan_steps = [90, 90, 90, 360]   # Passo do Pan
newAngle = 0
'''


def pt_panoramic_client():
    pan_tilt_limit = 360
    # H: 34 V: 25.5 - Sobreposicao=30%
    tilt = [0, 17.85, 35.7, 53.55, 71.4, 90]
    # pan_fotosH = [16, 16, 15, 12, 9, 1]
    pan_steps = [22.5, 22.5, 24, 30, 40, 360]
    returning = False
    name_photos_count = 0

    rospy.wait_for_service('pantilt_control')
    try:
        pt_panoramic = rospy.ServiceProxy('pantilt_control', PantiltControl)

        for idx, tilt_angle in enumerate(tilt):

            if tilt_angle >= pan_tilt_limit:
                tilt_angle = pan_tilt_limit - 1

            resp = pt_panoramic('set_angle', 'tilt', tilt_angle)

            current_step = pan_steps[idx]
            # math.ceil(x): returns the smallest integer not less than x
            steps_count = math.ceil(360 / current_step)
            pan_photos_count = 0

            while pan_photos_count < steps_count:
                if returning:
                    current_angle = 360 - pan_photos_count * current_step

                else:
                    current_angle = pan_photos_count * current_step

                if current_angle >= pan_tilt_limit:
                    current_angle = pan_tilt_limit - 1

                resp = pt_panoramic('set_angle', 'pan', current_angle)

                take_picture(0, name_photos_count)
                print("take_picture", 0, name_photos_count)
                take_picture(1, name_photos_count)
                print("take_picture", 1, name_photos_count)

                # Uncomment the line below when the camera is present
                # if path.exists("{0}.jpg".format(name_photos_count)):
                pan_photos_count = pan_photos_count + 1
                name_photos_count = name_photos_count + 1

            returning = not returning  # Reverse direction of rotation

        return resp.response_sucess

    except rospy.ServiceException as e:
        print("Service call failed: %e" % e)


if __name__ == "__main__":
    pt_panoramic_client()
