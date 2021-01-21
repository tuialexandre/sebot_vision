from sebot_vision.srv import PantiltControl
from sebot_vision.srv import PantiltControlResponse
from sebot_vision.srv import Panoramic
import rospy

class PanoramicManager:

    def __init__(self):
        load_configuration()
        setup_pantilt()
        setup_cameras()
        
        print("Ready to control pantilt\n\n")

    
    def setup_pantilt(self):

        # Uncomment to simulate
        self._ser = 'serial_simulator'

        # Uncomment when the pantilt is on
        # self._serial_port = '/dev/ttyUSB0'
        # self._ser = serial.Serial(self._serial_port, 2400, timeout=1) # init Serial

        self._service = rospy.Service('pantilt_control',
                                PantiltControl, self.handle_pantilt_control)

    def setup_cameras(self):
        self._service = rospy.Service('create_panoramic',
                                Panoramic, self.handle_create_panoramic)


    def load_configuration(self):
        pass


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
            
    def handle_create_panoramic(self):
        pass



if __name__ == "__main__":
    rospy.init_node('panoramic_manager_node')
    PanoramicManager()
    rospy.spin()