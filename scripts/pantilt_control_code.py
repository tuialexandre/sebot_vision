"""
Author: Wesley Rodrigues and Gabriel Haverot
Email: wesley.rodrigues@lactec.org.br
"""

import serial
import time


class PanTilt():

    def __init__(self, serial_connection):
        self._data_write = [255, 1, 0, 0, 0, 0, 0]  # Pelco-D frame
        self._data_read = [255, 1, 0, 0, 0, 0, 0]
        self._ser = serial_connection
        self._query_pan_position = 81  # Query Pan Position
        self._query_tilt_position = 83  # Query Tilt position
        # pdb.set_trace()


    def stop(self):
        """
        This function send Pelco-D command to pantilt stop
        """
        print('Stop \n')
        self._data_write = [255, 1, 0, 0, 0, 0, 0]
        self._ser.write(self._data_write)  # Write in the serial port


    def get_pan_angle(self):
        """
        This funcion send Pelco-D command to get pan angle
        """
        return self.read_serial_port_and_returns_angle(self._query_pan_position)


    def get_tilt_angle(self):
        """
        This funcion send Pelco-D command to get tilt angle
        """
        return self.read_serial_port_and_returns_angle(self._query_tilt_position)


    def read_serial_port_and_returns_angle(self, query_position):
        """
        Send command for query position. See Advanced command table that is
        [255 001 000 081/083 000 000 (081/083+1)]
        query_pan_position = 081
        query_tilt_position = 083
        """

        cont_byte = 0
        self._ser.write([255, 1, 0, query_position, 0, 0, query_position + 1])

        while cont_byte < 7:
            # Read serial port
            data_str = self._ser.read(1)

            # Return the integer represented by the given array of bytes
            byte = int.from_bytes(data_str, byteorder='big')

            # Inserts the elements at the given index
            self._data_read.insert(cont_byte, byte)

            cont_byte = cont_byte + 1

        # Convert the bytes in angle
        # byte4 = int(angle*100/256) = mmm -> Mais significativo
        # byte5 = int(angle*100) - byte4*256 = lll -> Menos significativo
        # As a isolate angle at equations, we have,
        angle = self._data_read[4]*256/100 + self._data_read[5]/100

        return angle


    def __del__(self):
        # Close serial connection
        self._ser.close()
        print('Destructor...')





class StandardCommands(PanTilt):
    """
    Standard commands are used to teleoperate the pantilt. The user choose
    that if want right, left, up, down, etc, ...
    """
    def __init__(self, port):
        super().__init__(port)

        # These values are part of the pelco D protocol
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
        """
        This funciont receive the operation_specification which can be
        one of the movement directions and the required_value which is
        the speed.
        """

        movement_orientation = operation_specification
        valid_command = False

        try:
            self._speed = int(required_value)  # 0 <= speed <= 60

            # Check the speed range
            if (self._speed >= 0 and self._speed <= 60):
                """
                Here the code will assemble the list according to
                the  required movement and speed.
                """

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
                print('Command is ok!')
                self._ser.write(self._data_write)

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




class AdvancedCommands(PanTilt):
    """
    Advanced commands are used to send the pantilt to a specific angle
    """

    def __init__(self, port):
        super().__init__(port)
        self._set_pan_position = 75  # Set Pan Position
        self._set_tilt_position = 77  # Set Tilt position

        # These variables wil then be defined as functions
        self._setter = None
        self._getter = None
        self._angle_difference_calculator = None
        self._tolerance = 0.4



    def set_angle(self, command, new_angle):
        """
        This function receive command = pan or tilt and
        new_angle = 0 < float < 360
        """

        try:
            # Defines the getter and setter
            self.type_range_and_command_checker(command, new_angle)

            old_angle = self._getter()
            number_of_attempts = 3
            arrived_in_position = False

            for _ in range(number_of_attempts):

                # Send command
                self._setter(new_angle)
  
                # Start counting the time
                start_time = time.time()

                # If you want to know how long it takes to arrive
                # time_variation = self.time_calculator(old_angle,
                #     new_angle, start_time)

                # Wait here until the pantilt arrive
                self.time_calculator(old_angle, new_angle, start_time) # BLocking

                # Calculate how far to reach the desired position
                angle_difference = self._angle_difference_calculator(
                                            self._getter(), new_angle)

                # If arrived in position, exit the loop
                arrived_in_position = angle_difference <= self._tolerance
                if arrived_in_position is True:
                    break

            if not arrived_in_position:
                raise Exception("Failed to move to desired pan tilt position, "
                                "necessary maintenance")

            new_angle = self._getter()

            ''' This piece is used to calculate the average speed '''
            # displacement = self._angle_difference_calculator(old_angle,
            #                                                 new_angle)
            # speed = displacement/time_variation
            # print('\nold_angle, new_angle, displacement,'
            #       ' time_variation, speed')
            # print(f'{old_angle:.2f}   , {new_angle:.2f}   ,'
            #      f'{displacement:.2f}     , {time_variation:.2f}
            #      f,{speed:.2f}')

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




    def type_range_and_command_checker(self, command, new_angle):
        """
        This function check the type, the range and command, after
        she difines the getter and setter functions
        """

        # Type Checker
        if not (type(new_angle) == int or type(new_angle) == float):
            raise TypeError('The new_angle variable must be either'
                            ' int or float')

        # Command and range checker for 'pan'
        if command == 'pan':

            if not (new_angle > 0 and new_angle < 360):
                raise ValueError('Desired pan angle out of range'
                                 'Enter with 0< angle <360')

            # Define getter and setter
            self._getter = self.get_pan_angle
            self._setter = self.set_pan_angle
            self._angle_difference_calculator = self.tilt_displacement_calculator


        # Command and range checker for 'filt'
        elif command == 'tilt':

            if (new_angle > 90) and (new_angle < 311):
                raise Exception('Desired tilt angle in dead zone')

            elif (new_angle < -49) or (new_angle >= 360):
                raise ValueError('Desired tilt angle out of range')

            elif (new_angle >= -49) and (new_angle < 0):
                new_angle = new_angle + 360  # For negative angle

            # Define getter and setter
            self._getter = self.get_tilt_angle
            self._setter = self.set_tilt_angle
            self._angle_difference_calculator = self.pan_displacement_calculator

        print('Command and angle input are OK!\n\n')




    def set_pan_angle(self, angle):
        """
        This function call the byte calculator and write in serial port
        """

        angle = float(angle)

        # Mount the byte packet in the _data_write variable
        self.frame_byte_calculator(angle, self._set_pan_position)
        self._ser.write(self._data_write)




    def set_tilt_angle(self, angle):
        """
        This function call the byte calculator and write in serial port
        """

        angle = float(angle)

        # Mount the byte packet in the _data_write variable
        self.frame_byte_calculator(angle, self._set_tilt_position)
        self._ser.write(self._data_write)




    def frame_byte_calculator(self, angle, byte3):
        """
        This function calculates the 4 and 5 bytes from the
        angle provided
        """

        self._data_write[3] = byte3
        self._data_write[4] = int(angle*100/256)
        # print(self._data_write[4])
        self._data_write[5] = int(angle*100) - self._data_write[4]*256

        if self._data_write[5] != 255:
            self._data_write[5] = self._data_write[5]
        else:
            self._data_write[5] = 254

        # print(self._data_write[5])
        print('Command sent:  ', self._data_write)
        self._data_write[6] = 10  # Can be any number
        """The correct thing is to put the checksum,
        but don't work in this hardware.
        self._data_write[6] = ( 1
                            + byte3
                            + self._data_write[4]
                            + self._data_write[5])%256
        """
        return self._data_write




    def time_calculator(self, old_angle, new_angle, start_time):
        """
        In this function, the process will stay inside the loop until the
        pantilt reaches the position or the time runs out.
        Iterations is number of correct iterations accumulated until you
        accept that Pan Tilt has entered the desired range.
        """

        iterations = 3
        cont = 1
        sample_time = 0.5
        previous_time = 0
        maximum_pantilt_speed = 10  # degrees/seconds, Estimated based on tests

        angle_difference = self._angle_difference_calculator(old_angle,
                                                             new_angle)

        # Estimate the maximum time for the pantilt to reach its destination
        maximum_time = max(angle_difference / maximum_pantilt_speed, 2)

        # Stay inside the loop until the pantil reaches the position or
        # does not arrive after threee attempts
        while cont <= iterations:

            if time.time() - previous_time >= sample_time:

                # Update the time
                previous_time = time.time()
                current_time_variation = previous_time - start_time

                # Take the current angle
                current_angle = self._getter() 
 
                # Exits the loop if time runs out
                if current_time_variation > maximum_time:
                    break

                print('Current angle: ', current_angle)
                print('Counter: ', cont)

                # Calculates the difference between the desired angle and the
                # currente one.
                angle_difference = self._angle_difference_calculator(
                            current_angle, new_angle)

                # Increment one if it came close
                if angle_difference <= self._tolerance:
                    sample_time = 0.01 # Shorten the semple time
                    cont = cont + 1

            time.sleep(0.1) # Test and see if you can withdraw

        time_variation = time.time() - start_time

        return time_variation




    def tilt_displacement_calculator(self, old_or_current_angle, new_angle):
        """
        Calculate the displacement module in degrees for tilt
        """
        angle_difference = abs(new_angle - old_or_current_angle)
        return angle_difference



    def pan_displacement_calculator(self, old_or_current_angle, new_angle):
        """
        Calculate the displacement module in degrees for pan
        """
        angle_difference = new_angle - old_or_current_angle
        angle_difference = abs((angle_difference+180) % 360 - 180)
        return angle_difference
