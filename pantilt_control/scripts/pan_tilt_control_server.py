#!/usr/bin/env python

from pantilt_control_code import StandardCommands as set_pantilt
from pantilt_control_code import AdvancedCommands as teleop_pantilt
from pan_tilt_control.srv import PantilControl, PantilControlResponse
import rospy

serial_port = '/dev/ttyUSB0'

def handle_pan_tilt_control(req):
    print("%s order for %s " % (req.operation_type, req.operation_specification,
          req.required_value))
    if req.operation_type == 'panoramic':
        panomaric = set_pantilt(serial_port)
        panoramic.set_angle(req.operation_specification, req.required_value)
        return PantilControlResponse(True)
    elif req.operation_type == 'teleoperation':
        teleop = teleop_pantilt(serial_port)
        teleop.teleoperation(req.operation_specification, req.required_value)
        return PantilControlResponse(True)
    else:
        return PantilControlResponse(False)

def pan_tilt_control_server():
    rospy.init_node('pan_tilt_control_server')
    service = rospy.Service('pan_tilt_control',
                            PantilControl, handle_pan_tilt_control)
    print("Ready to control pantilt")
    rospy.spin()


if __name__ == "__main__":
    pan_tilt_control_server()