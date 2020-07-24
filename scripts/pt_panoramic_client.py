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
pan_passo = [90, 90, 90, 360]   # Passo do Pan
newAngle = 0
'''

limitePanTilt = 360
# H: 34 V: 25.5 - Sobreposicao=30%
tilt = [0, 17.85, 35.7, 53.55, 71.4, 90]
# pan_fotosH = [16, 16, 15, 12, 9, 1]
pan_passo = [22.5, 22.5, 24, 30, 40, 360]
voltando = False
contNamePhotos = 0

def pt_panoramic_client(self):
    rospy.wait_for_service('pan_tilt_control')
    try:
        pt_panoramic = rospy.ServiceProxy('pan_tilt_control', PantilControl)

        for idx, anguloTilt in enumerate(tilt):
            if anguloTilt >= limitePanTilt:
                anguloTilt = limitePanTilt - 1
            resp = pt_panoramic('panoramic', 'tilt', anguloTilt) # -------- Chama o serviço
            passoPanAtual = pan_passo[idx]
            numeroDePassos = math.ceil(360 / passoPanAtual)
            contPhotosPan = 0
            while contPhotosPan < numeroDePassos:
                if voltando:
                    anguloAtual = 360 - contPhotosPan * passoPanAtual
                else:
                    anguloAtual = contPhotosPan * passoPanAtual
                if anguloAtual >= limitePanTilt:
                    anguloAtual = limitePanTilt - 1
                resp = pt_panoramic('panoramic', 'pan', anguloAtual) # ----- Chama o serviço
                take_picture(0, contNamePhotos)
                take_picture(1, contNamePhotos)
                statusFile = path.exists("{0}.jpg".format(contNamePhotos))
                if statusFile:
                    contPhotosPan = contPhotosPan + 1
                    contNamePhotos = contNamePhotos + 1
            voltando = not voltando
        return resp.response_sucess
    except rospy.ServiceException as e:
        print("Service call failed: %e"%e)

if __name__ == "__main__":
    pt_panoramic_client()