#!/usr/bin/env python3

from __future__ import print_function

''' Import service '''
from pantilt_control_ros_node.srv import PantiltControl
from pantilt_control_ros_node.srv import PantiltControlResponse
import rospy

''' Uncomment to simulate '''
from pantilt_control_code_test import StandardCommands as teleop_pantilt
from pantilt_control_code_test import AdvancedCommands as set_angle_pantilt

''' Uncomment when the pantilt is on '''
# from pantilt_control_code import StandardCommands as teleop_pantilt
# from pantilt_control_code import AdvancedCommands as set_pantilt

serial_port = '/dev/ttyUSB0'

''' Fazer um objeto para abrir a serial e iniciar o serviço '''
# class Service():

#     def __init__(self):
#         self.reset_service = rospy.Servic
#         serial_port



def handle_pantilt_control(req):

    print("operation_type = ", req.operation_type)
    print("operation_specification | required_value = %s | %s" % (
        req.operation_specification, req.required_value))

    if req.operation_type == "set_angle":
        panomaric = set_angle_pantilt(serial_port)
        panomaric.set_angle(req.operation_specification, req.required_value)
        return PantiltControlResponse(True)

    elif req.operation_type == "teleoperation":
        teleop = teleop_pantilt(serial_port)
        print("req.operation_type == teleoperation")
        teleop.teleoperation(req.operation_specification, req.required_value)
        return PantiltControlResponse(True)

    else:
        return PantiltControlResponse(False)


def pantilt_control_server():
    rospy.init_node('pantilt_control_server')
    service = rospy.Service('pantilt_control',
                            PantiltControl, handle_pantilt_control)
    print("Ready to control pantilt\n\n")
    rospy.spin()


if __name__ == "__main__":
    pantilt_control_server()
