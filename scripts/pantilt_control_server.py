#!/usr/bin/env python3

from __future__ import print_function
import serial

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

''' Fazer um objeto para abrir a serial e iniciar o serviço '''
class PantiltControlService:

    def __init__(self):


        ''' Uncomment to simulate '''
        self._ser = 'serial_simulator'

        ''' Uncomment when the pantilt is on '''
        #  self._serial_port = '/dev/ttyUSB0'
        # self._ser = serial.Serial(self._serial_port, 2400, timeout=1) # init Serial

        self._service = rospy.Service('pantilt_control',
                                PantiltControl, self.handle_pantilt_control)
        print("Ready to control pantilt\n\n")

    def handle_pantilt_control(self, req):

        print("operation_type = ", req.operation_type)
        print("operation_specification | required_value = %s | %s" % (
            req.operation_specification, req.required_value))

        if req.operation_type == "set_angle":
            panomaric = set_angle_pantilt(self._ser)
            panomaric.set_angle(req.operation_specification, req.required_value)
            return PantiltControlResponse(True)

        elif req.operation_type == "teleoperation":
            teleop = teleop_pantilt(self._ser)
            print("req.operation_type == teleoperation")
            teleop.teleoperation(req.operation_specification, req.required_value)
            return PantiltControlResponse(True)

        else:
            return PantiltControlResponse(False)


if __name__ == "__main__":
    rospy.init_node('pantilt_control_server')
    PantiltControlService()
    rospy.spin()