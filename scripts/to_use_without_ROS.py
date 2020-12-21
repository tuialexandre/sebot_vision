import serial

from pantilt_control_code import StandardCommands as teleop_pantilt
from pantilt_control_code import AdvancedCommands as set_angle_pantilt

# serial_port = '/dev/ttyUSB0'
serial_port = 'COM7'
ser = serial.Serial(serial_port, 2400, timeout=1) # init Serial

set_angle = set_angle_pantilt(ser)
# teleop = teleop_pantilt(ser)

# set_angle.set_angle('pan',50.5)
# set_angle.set_angle('tilt',60)
# set_angle.set_angle('pan',60)
# set_angle.set_angle('tilt',30)
# set_angle.set_angle('pan',10)
# set_angle.set_angle('tilt',160)
# set_angle.set_angle('pan',200)
# set_angle.set_angle('tilt',63)

set_angle.set_angle('pan',10)
set_angle.set_angle('tilt',10)

print(set_angle.get_pan_angle())
print(set_angle.get_tilt_angle())