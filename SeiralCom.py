import serial
import subprocess
import time
import re
import math


class SerialCommunication():

    def __init__(self):
        self.connection = None
        self.connection_opened = False
        self.currentMainboardReturnData = ""

    def open(self):

        try:
            ports = subprocess.check_output('ls /dev/ttyACM1', shell=True).split('\n')[:-1]

        except:
            print('mainboard: /dev/ttyACM empty')
            return False
        self.connection_opened = False
        for port in ports:  # analyze serial ports
            try:
                while not self.connection_opened:
                    self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=True)
                    self.connection_opened = self.connection.isOpen()
                    time.sleep(0.5)
                self.connection.flush()
                print("mainboard: Port opened successfully")
            except Exception as e:
                print(e)
                continue
        return self.connection_opened


    def write(self, comm):
        if self.connection is not None:
            try:
                self.connection.write(comm + '\n')
            except:
                print('mainboard: err write ' + comm)


    def set_PID(self, P1, S1, P2, S2, P3, S3, P4, S4, P5, S5, P6, S6):
        if self.connection_opened:
            PID_command = "ps:{}:{}:{}:{}:{}:{}:{}:{}:{}:{}:{}:{}".format(int(P1), int(S1), int(P2), int(S2), int(P3), int(S3), int(P4), int(S4), int(P5), int(S5), int(P6), int(S6))
            # print(P1, S1, P2, S2, P3, S3)
            self.write(PID_command)
        else:
            print('Mainboard opened failed')


    def read(self):
        while self.connection is not None and self.connection.isOpen() and self.connection.in_waiting > 0:
            character = self.connection.read()  # read the mainboard feedback character by character
            if character == '\n':
                fullCommand = self.currentMainboardReturnData  # save all characters in full command
                self.currentMainboardReturnData = ""   # clean the 'currentMainboardReturnData' after receive all data
                print(fullCommand)
                return fullCommand

            self.currentMainboardReturnData += character
        return ""

    def write_mainboard(self, position1, speed1, position2, speed2):
        self.set_PID(
            position1, speed1,
            -position1, speed1,
            position1, speed1,
            -position2, speed2,
            position2, speed2,
            -position2, speed2
        )


    def extract_poistion(self, text):
        pattern = 'Pos0:([-\d]+), Pos1:([-\d]+), Pos2:([-\d]+), Pos3:([-\d]+), Pos4:([-\d]+), Pos5:([-\d]+)'
        m = re.search(pattern, text)

        if m is not None:
            return (int(m.group(1)), -int(m.group(2)), int (m.group(3)), -int(m.group(4)), int(m.group(5)), -int(m.group(6)))

        else:
            print('no match')
            return

    def calculate_position_and_speed(self, pos1, pos2, pos3, pos4, pos5, pos6):
        p1, s1 = self.calculate_position_and_speed_for_set(pos1, pos2, pos3)
        p2, s2 = self.calculate_position_and_speed_for_set(pos4, pos5, pos6)

        position1, speed1, position2, speed2 = self.synchronous(pos1, pos4, p1, s1, p2, s2)

        #return (p1, s1, p2, s2)
        return (position1, speed1, position2, speed2)


    def calculate_position_and_speed_for_set(self, pos1, pos2, pos3):
        counts_per_rotation = 6533.0
        leg1_angle = pos1 % counts_per_rotation

        leg_speed_upper = 40
        leg_speed_lower = 10
        leave_ground_p = 6233
        touch_ground_p = 300

        position = 0
        speed = 0

        # if leg1_angle >= touch_ground_p and leg1_angle < leave_ground_p:
        #     speed = leg_speed_upper
        # else:
        #     speed = leg_speed_lower

        if leg1_angle >= touch_ground_p - 20 and leg1_angle < leave_ground_p - 20:
            position = math.floor(pos1 / counts_per_rotation) * counts_per_rotation + leave_ground_p
            speed = leg_speed_upper
        elif leg1_angle < touch_ground_p - 20:
            position = math.floor(pos1 / counts_per_rotation) * counts_per_rotation + touch_ground_p
            speed = leg_speed_lower
        else:
            position = math.ceil(pos1 / counts_per_rotation) * counts_per_rotation + touch_ground_p
            speed = leg_speed_lower

        # position = (math.ceil(pos1 / counts_per_rotation) + 1) * counts_per_rotation

        return position, speed

    def synchronous(self, posSet1, posSet2, positionSet1, speedSet1, positionSet2, speedSet2):

        counts_per_rotation = 6533.0
        set1_angle = posSet1 % posSet2ounts_per_rotation
        set2_angle = posSet2 % counts_per_rotation

        print(set1_angle, set2_angle)

        leave_ground_p = 6233
        touch_ground_p = 300

        position1 = positionSet1
        speed1 = speedSet1
        position2 = positionSet2
        speed2 = speedSet2

        if set1_angle > touch_ground_p and set2_angle < leave_ground_p:
            position1 = posSet1
        elif set1_angle > leave_ground_p and set2_angle < touch_ground_p:
            position1 = posSet1
        elif set2_angle > leave_ground_p and set1_angle < leave_ground_p:
            position2 = posSet2
        elif set2_angle > leave_ground_p and set1_angle < touch_ground_p:
            position2 = posSet2

        return position1, speed1, position2, speed2



if __name__ == '__main__':
    serial_communication = SerialCommunication()
    serial_communication.open()

    serial_communication.write_mainboard(0, 0, 0, 0)

    while True:
        positions = serial_communication.extract_poistion(serial_communication.read())

        if positions is not None:
            print(positions)
            (pos1, pos2, pos3, pos4, pos5, pos6) = positions
            #print(positions)
            (position1, speed1, position2, speed2) = serial_communication.calculate_position_and_speed(pos1, pos2, pos3, pos4, pos5, pos6)
            serial_communication.write_mainboard(position1, speed1, position2, speed2)

        time.sleep(0.01)



