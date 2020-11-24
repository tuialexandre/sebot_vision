''' 
This code only simulates a communication with a pantilt, so see that the 
communication commands are comments.
It will only be analyzed if the desired command is correct.
-----------------------------------------------------------------------------
To call this code only
import pantilt_control_code as pt
pantil = pt.PanTilt('/dev/ttyUSB0')  # Enter with the pantilt USB port
# For choose string == 'pan' or 'tilt' in the entry
pantilt.set_angle('tilt or pan', angle)
pantilt.set_pan_angle(angle)
pantilt.set_tilt_angle(angle)
pantilt.get_pan_angle()
pantilt.get_tilt_angle()
del pantilt

Structure of the pelco-D frame
[synchronism, addres, command 1, command 2, data 1, data 2, checksun]
'''

# import serial
import time


class PanTilt():

    def __init__(self, port):
        self._data_write = [255, 1, 0, 0, 0, 0, 0]  # Pelco-D frame
        self._data_read = [255, 1, 0, 0, 0, 0, 0]
        # self._ser = serial.Serial(port, 2400, timeout=1)
        self._byte_query_pan = 81  # Query Pan Position
        self._byte_query_tilt = 83  # Query Tilt position
        # pdb.set_trace()

    def stop(self):
        print('Stop \n')
        self._data_write = [255, 1, 0, 0, 0, 0, 0]
        # self._ser.write(self._data_write)  # Write in the serial port
        print(self._data_write)

    def get_pan_angle(self):
        return self.read_serial_port_and_returns_angle(self._byte_query_pan)

    def get_tilt_angle(self):
        return self.read_serial_port_and_returns_angle(self._byte_query_tilt)

    def read_serial_port_and_returns_angle(self, byte_query):
        self._cont_byte = 0
        self._ser.write([255, 1, 0, byte_query, 0, 0, byte_query + 1])
        while self._cont_byte < 7:
            self._data_str = self._ser.read(1)
            self._byte = int.from_bytes(self._data_str, byteorder='big')
            self._data_read.insert(self._cont_byte, self._byte)
            self._cont_byte = self._cont_byte + 1
            angle = self._data_read[4]*256/100 + self._data_read[5]/100
        return angle

    def __del__(self):
        # self._ser.close()
        print('Destructor...')


#  S. C. is used to teleoperated the pan tilt
class StandardCommands(PanTilt):
    def __init__(self, port):
        super().__init__(port)
        ''' These values are part of the pelco D protocol '''
        self._pan_right_byte = 2
        self._pan_left_byte = 4
        self._tilt_up_byte = 16  # 8 in the pelco D table
        self._tilt_down_byte = 8  # 16 in the pelco D table
        self._pan_right_and_tilt_up_byte = 18  # 10 in the pelco D table
        self._pan_right_and_tilt_down_byte = 10  # 12 in the pelco D table
        self._pan_left_and_tilt_up_byte = 20 # 18 in the pelco D table
        self._pan_left_and_tilt_down_byte = 12 # 20 in the pelco D table

    def teleoperation(self, operation_specification,
                      required_value):
        movement_orientation = operation_specification
        valid_command = False
        #  print('valid_command initializes --> ', valid_command)

        try:
            self._speed = int(required_value)  # 0 <= speed <= 60
            # print('self._speed = ', self._speed)

            if (self._speed >= 0 and self._speed <= 60):
                # print('Speed is ok!')

                if movement_orientation == 'right':
                    self._data_write[3] = self._pan_right_byte
                    self._data_write[4] = self._speed
                    self._data_write[5] = 0
                    valid_command = True

                elif movement_orientation == 'left':
                    self._data_write[3] = self._pan_left_byte
                    self._data_write[4] = self._speed
                    self._data_write[5] = 0
                    valid_command = True

                elif movement_orientation == 'up':
                    self._data_write[3] = self._tilt_up_byte
                    self._data_write[4] = 0
                    self._data_write[5] = self._speed
                    valid_command = True

                elif movement_orientation == 'down':
                    self._data_write[3] = self._tilt_down_byte
                    self._data_write[4] = 0
                    self._data_write[5] = self._speed
                    valid_command = True

                elif movement_orientation == 'right_and_up':
                    self._data_write[3] = self._pan_right_and_tilt_up_byte
                    self._data_write[4] = self._speed
                    self._data_write[5] = self._speed
                    valid_command = True

                elif movement_orientation == 'right_and_down':
                    self._data_write[3] = self._pan_right_and_tilt_down_byte
                    self._data_write[4] = self._speed
                    self._data_write[5] = self._speed
                    valid_command = True

                elif movement_orientation == 'left_and_up':
                    self._data_write[3] = self._pan_left_and_tilt_up_byte
                    self._data_write[4] = self._speed
                    self._data_write[5] = self._speed
                    valid_command = True

                elif movement_orientation == 'left_and_down':
                    self._data_write[3] = self._pan_left_and_tilt_down_byte
                    self._data_write[4] = self._speed
                    self._data_write[5] = self._speed
                    valid_command = True
                else:
                    raise ValueError('The operation specification is not valid'
                                     '\n\t\t     Enter with a string')
            else:
                raise ValueError('Speed ranges from zero to sixty')

            # Send to pantilt

            if valid_command is True:
                print("Command is ok!\n")
                # self._ser.write(self._data_write)
                print("Send to pantilt >> ", self._data_write)
                print("\n\n")

        except ZeroDivisionError as error:
            print('ZeroDivisionError')
            print("Description error:  ", error)
        except ValueError as error:
            print('ValueError')
            print("Description error:  ", error)
        except TypeError as error:
            print('TypeError')
            print("Description error:  ", error)
        except Exception as error:
            print('Exception')
            print("Description error:  ", error)


#  A. C. is used to send the pan tilt to a certain angle
class AdvancedCommands(PanTilt):
    def __init__(self, port):
        super().__init__(port)
        self._byte_set_pan = 75  # Set Pan Position
        self._byte_set_tilt = 77  # Set Tilt position
        self._setter = None
        self._getter = None
        self._angle_difference_calculator = None

    def set_angle(self, command, new_angle):

        try:
            # Defines the getter and setter in the function below
            self.type_range_and_command_checker(command, new_angle)

            # Send command to pantilt
            self._setter(new_angle)

        except ZeroDivisionError as error:
            print('ZeroDivisionError')
            print("Description error:  ", error)
        except ValueError as error:
            print('ValueError')
            print("Description error:  ", error)
        except TypeError as error:
            print('TypeError')
            print("Description error:  ", error)
        except Exception as error:
            print('Exception')
            print("Description error:  ", error)

    # This function check the type, the range and the command
    def type_range_and_command_checker(self, command, new_angle):

        # Type Checker
        if not (type(new_angle) == int or type(new_angle) == float):
            raise TypeError('The new_angle variable must be either'
                            ' int or float')
        # Type command
        if command == 'pan':
            # print('command == pan')

            if not (new_angle >= 0 and new_angle <= 360):
                print("new_angle = " , new_angle)
                raise ValueError('Desired pan angle out of range'
                                 'Enter with 0< angle <360')

            self._getter = self.get_pan_angle
            self._setter = self.set_pan_angle

        elif command == 'tilt':
            if (new_angle > 90) and (new_angle < 311):
                raise Exception('Desired tilt angle in dead zone')
            elif (new_angle < -49) or (new_angle >= 360):
                raise ValueError('Desired tilt angle out of range')
            elif (new_angle >= -49) and (new_angle < 0):
                new_angle = new_angle + 360  # For negative angle

            self._getter = self.get_tilt_angle
            self._setter = self.set_tilt_angle

        print('Command and angle input are OK!\n')

    def set_pan_angle(self, angle):
        angle = float(angle)
        self.frame_byte_calculator(angle, self._byte_set_pan)
        # self._ser.write(self._data_write)
        print('Send to pan >> ', self._data_write)
        print("\n\n")

    def set_tilt_angle(self, angle):
        angle = float(angle)
        self.frame_byte_calculator(angle, self._byte_set_tilt)
        # self._ser.write(self._data_write)
        print('Send to tilt >> ', self._data_write)
        print("\n\n")

    # Frame byte calculator from angle and direction --> (byte3)
    def frame_byte_calculator(self, angle, byte3):
        self._data_write[3] = byte3
        self._data_write[4] = int(angle*100/256)
        # print(self._data_write[4])
        self._data_write[5] = int(angle*100) - self._data_write[4]*256

        if self._data_write[5] != 255:
            self._data_write[5] = self._data_write[5]
        else:
            self._data_write[5] = 254

        # print(self._data_write[5])
        # print('Command sent:  ', self._data_write)
        self._data_write[6] = 10  # Can be any number
        """The correct thing is to put the checksum,
        but don't work in this hardware.
        self._data_write[6] = ( 1
                            + byte3
                            + self._data_write[4]
                            + self._data_write[5])%256
        """
        return self._data_write
