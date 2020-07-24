#!/usr/bin/env python

import math
import os
from os import path

import sys
import rospy
from pan_tilt_control.srv import *


def take_picture(type_image, name_file):
    if type_image == 0:
        return os.system("sudo ./fluke-ir {0}".format(name_file))
    elif type_image == 1:
        return os.system("sudo timeout 5 ./shoot_an_image_and_get_it.sh"
                         " {0}.jpg".format(name_file))


'''
#H: 90 V: 30 - Sobreposicao=0
tilt = [0, 30, 60, 90]          # Ângulo do tilt
pan_fotosH = [4, 4, 4, 1]       # Número de fotos
pan_steps = [90, 90, 90, 360]   # Passo do Pan
newAngle = 0
'''

pan_tilt_limit = 360
# H: 34 V: 25.5 - Sobreposicao=30%
tilt = [0, 17.85, 35.7, 53.55, 71.4, 90]
# pan_fotosH = [16, 16, 15, 12, 9, 1]
pan_steps = [22.5, 22.5, 24, 30, 40, 360]
returning = False
name_photos_count = 0

def pt_panoramic_client(self):
    rospy.wait_for_service('pan_tilt_control')
    try:
        pt_panoramic = rospy.ServiceProxy('pan_tilt_control', PantilControl)

        for idx, tilt_angle in enumerate(tilt):
            if tilt_angle >= pan_tilt_limit:
                tilt_angle = pan_tilt_limit - 1
            resp = pt_panoramic('panoramic', 'tilt', tilt_angle) # -------- Chama o serviço
            current_step = pan_steps[idx]
            steps_count = math.ceil(360 / current_step)
            pan_photos_count = 0
            while pan_photos_count < steps_count:
                if returning:
                    current_angle = 360 - pan_photos_count * current_step
                else:
                    current_angle = pan_photos_count * current_step
                if current_angle >= pan_tilt_limit:
                    current_angle = pan_tilt_limit - 1
                resp = pt_panoramic('panoramic', 'pan', current_angle) # ----- Chama o serviço
                take_picture(0, name_photos_count)
                take_picture(1, name_photos_count)
                if path.exists("{0}.jpg".format(name_photos_count)):
                    pan_photos_count = pan_photos_count + 1
                    name_photos_count = name_photos_count + 1
            returning = not returning
        return resp.response_sucess
    except rospy.ServiceException as e:
        print("Service call failed: %e"%e)

if __name__ == "__main__":
    pt_panoramic_client()