#!/usr/bin/env python3

from sebot_vision.srv import PantiltControl
from sebot_vision.srv import PantiltControlResponse
from sebot_vision.srv import Panoramic
import rospy

from pantilt_control_code_test import StandardCommands as teleop_pantilt_simulated
from pantilt_control_code_test import AdvancedCommands as set_angle_pantilt_simulated

from pantilt_control_code import StandardCommands as teleop_pantilt
from pantilt_control_code import AdvancedCommands as set_angle_pantilt

class PanoramicManager:

    def __init__(self):
        print("Initializing Panoramic Manager...")
        self.load_params()
        self.setup_pantilt()
        self.setup_cameras()
        
        print("Ready to control pantilt\n\n")

    def load_params(self):
        self._use_sim_time = rospy.get_param('/use_sim_time')
        self._serial_frequency = 2400
        self._serial_time_out = 1
        self._serial_port = '/dev/ttyUSB0'

    def setup_pantilt(self):
        if(self._use_sim_time):
            self._pantilt_serial = 'serial_simulator'
        else:
            self._pantilt_serial = serial.Serial(self._serial_port, self._serial_frequency, timeout=self._serial_time_out)

        self._service = rospy.Service('sebot_vision/pantilt_control',
                                PantiltControl, self.handle_pantilt_control)

    def setup_cameras(self):
        self._service = rospy.Service('sebot_vision/create_panoramic',
                                Panoramic, self.handle_create_panoramic)

    def handle_pantilt_control(self, req):
        operation_successful = False

        print("operation_type = ", req.operation_type)
        print("operation_specification | required_value = %s | %s" % (
            req.operation_specification, req.required_value))
            
        if req.operation_type == "set_angle":
            if self._use_sim_time:
                panomaric = set_angle_pantilt_simulated(self._pantilt_serial)
            else:
                panomaric = set_angle_pantilt(self._pantilt_serial)
            panomaric.set_angle(req.operation_specification, req.required_value)
            operation_successful = True

        elif req.operation_type == "teleoperation":
            if self._use_sim_time:
                teleop = teleop_pantilt_simulated(self._pantilt_serial)
            else:
                teleop = teleop_pantilt(self._pantilt_serial)
            print("req.operation_type == teleoperation")
            teleop.teleoperation(req.operation_specification, req.required_value)
            operation_successful = True

        return PantiltControlResponse(operation_successful)


    def handle_create_panoramic(operation_succesful):
        pass



if __name__ == "__main__":
    rospy.init_node('panoramic_manager_node')
    PanoramicManager()
    rospy.spin()