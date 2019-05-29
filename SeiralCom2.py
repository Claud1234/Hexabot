import serial
import subprocess
import time
import re
import math
import keyboard
from enum import Enum
import sys

class Side(Enum):
    LEFT = 1
    RIGHT = 2


class Set(Enum):
    LEFT = 1
    RIGHT = 2


def interpolate(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class Leg:
    def __init__(self, side, set):
        self.side = side
        self.set = set
        self.position = 0
        self.leg_angle = 0
        self.positionSetpoint = 0
        self.is_move_forward = True
        self.counts_per_rotation = 6533
        self.rear_robot_angle = 500
        self.front_robot_angle = 6033
        self.upper_speed = 150
        self.speed_ramp_counts = 2000

    def isAtTouchGround(self):
        touch_ground_angle = self.front_robot_angle if self.is_move_forward else self.rear_robot_angle

        return abs(touch_ground_angle - self.leg_angle) <= 20

    def isAtLeaveGround(self):
        leave_ground_angle = self.rear_robot_angle if self.is_move_forward else self.front_robot_angle

        return abs(leave_ground_angle - self.leg_angle) <= 20

    def is_at_position(self):
        return abs(self.positionSetpoint - self.position) <= 20

    def goToTouchGround(self):
        if self.isAtTouchGround():
            return

        touch_ground_p = self.front_robot_angle if self.is_move_forward else self.rear_robot_angle
        fullRotationCount = math.floor(self.position / self.counts_per_rotation)

        self.positionSetpoint = fullRotationCount * self.counts_per_rotation + touch_ground_p

        if not self.is_move_forward and self.positionSetpoint > self.position:
            self.positionSetpoint -= self.counts_per_rotation
        elif self.is_move_forward and self.positionSetpoint < self.position:
            self.positionSetpoint += self.counts_per_rotation

    def goToLeaveGround(self):
        if self.isAtLeaveGround():
            return

        leave_ground_p = self.rear_robot_angle if self.is_move_forward else self.front_robot_angle
        fullRotationCount = math.floor(self.position / self.counts_per_rotation)

        self.positionSetpoint = fullRotationCount * self.counts_per_rotation + leave_ground_p

        if not self.is_move_forward and self.positionSetpoint > self.position:
            self.positionSetpoint -= self.counts_per_rotation
        elif self.is_move_forward and self.positionSetpoint < self.position:
            self.positionSetpoint += self.counts_per_rotation

    def update(self, forward_speed, rotate_speed, position):
        self.position = position
        self.leg_angle = position % self.counts_per_rotation

        lower_speed = forward_speed + (rotate_speed if self.side == Side.LEFT else -rotate_speed)
        self.is_move_forward = lower_speed >= 0

        lower_speed = abs(lower_speed)

        if self.leg_angle < self.rear_robot_angle or self.leg_angle > self.front_robot_angle:
            speed = lower_speed
        # elif self.rear_robot_angle < self.leg_angle < self.rear_robot_angle + self.speed_ramp_counts:
        #     speed = interpolate(
        #         self.leg_angle,
        #         self.rear_robot_angle,
        #         self.rear_robot_angle + self.speed_ramp_counts,
        #         lower_speed,
        #         self.upper_speed
        #     )
        # elif self.front_robot_angle > self.leg_angle > self.front_robot_angle - self.speed_ramp_counts:
        #     speed = interpolate(
        #         self.leg_angle,
        #         self.front_robot_angle - self.speed_ramp_counts,
        #         self.front_robot_angle,
        #         self.upper_speed,
        #         lower_speed
        #     )
        else:
            speed = self.upper_speed

        return self.positionSetpoint, abs(speed)


class Motion:
    def __init__(self):
        self.leg1 = Leg(Side.LEFT, Set.LEFT)
        self.leg2 = Leg(Side.RIGHT, Set.RIGHT)
        self.leg3 = Leg(Side.LEFT, Set.RIGHT)
        self.leg4 = Leg(Side.RIGHT, Set.LEFT)
        self.leg5 = Leg(Side.LEFT, Set.LEFT)
        self.leg6 = Leg(Side.RIGHT, Set.RIGHT)

        self.legs = [self.leg1, self.leg2, self.leg3, self.leg4, self.leg5, self.leg6]

        self.active_set = Set.LEFT

        self.set_go_to_leave_ground(Set.LEFT)
        self.set_go_to_touch_ground(Set.RIGHT)

        self.prevLeg1Position = 0

        self.forward_speed = 0
        self.rotate_speed = 0

    def is_set_at_touch_ground(self, leg_set):
        for leg in self.legs:
            if leg.set == leg_set and not leg.isAtTouchGround():
                return False

        return True

    def is_set_at_leave_ground(self, leg_set):
        for leg in self.legs:
            if leg.set == leg_set and not leg.isAtLeaveGround():
                return False

        return True

    def set_go_to_touch_ground(self, leg_set):
        for leg in self.legs:
            if leg.set == leg_set:
                leg.goToTouchGround()

    def set_go_to_leave_ground(self, leg_set):
        for leg in self.legs:
            if leg.set == leg_set:
                leg.goToLeaveGround()

    def update(self, positions):
        (pos1, pos2, pos3, pos4, pos5, pos6) = positions

        (leg1Position, leg1Speed) = self.leg1.update(self.forward_speed, self.rotate_speed, pos1)
        (leg2Position, leg2Speed) = self.leg2.update(self.forward_speed, self.rotate_speed, pos2)
        (leg3Position, leg3Speed) = self.leg3.update(self.forward_speed, self.rotate_speed, pos3)
        (leg4Position, leg4Speed) = self.leg4.update(self.forward_speed, self.rotate_speed, pos4)
        (leg5Position, leg5Speed) = self.leg5.update(self.forward_speed, self.rotate_speed, pos5)
        (leg6Position, leg6Speed) = self.leg6.update(self.forward_speed, self.rotate_speed, pos6)

        print(leg2Position, leg2Speed)

        # if self.is_set_at_leave_ground(Set.LEFT) and self.is_set_at_touch_ground(Set.RIGHT):
        #     self.set_go_to_touch_ground(Set.LEFT)
        #     self.set_go_to_leave_ground(Set.RIGHT)
        #
        # elif self.is_set_at_touch_ground(Set.LEFT) and self.is_set_at_leave_ground(Set.RIGHT):
        #     self.set_go_to_leave_ground(Set.LEFT)
        #     self.set_go_to_touch_ground(Set.RIGHT)

        if self.is_set_at_leave_ground(Set.LEFT) and self.is_set_at_touch_ground(Set.RIGHT):
            self.active_set = Set.RIGHT
        elif self.is_set_at_touch_ground(Set.LEFT) and self.is_set_at_leave_ground(Set.RIGHT):
            self.active_set = Set.LEFT

        if self.active_set == Set.LEFT:
            self.set_go_to_leave_ground(Set.LEFT)
            self.set_go_to_touch_ground(Set.RIGHT)
        elif self.active_set == Set.RIGHT:
            self.set_go_to_touch_ground(Set.LEFT)
            self.set_go_to_leave_ground(Set.RIGHT)

        return (leg1Position, leg1Speed,
                leg2Position, leg2Speed,
                leg3Position, leg3Speed,
                leg4Position, leg4Speed,
                leg5Position, leg5Speed,
                leg6Position, leg6Speed)


class SerialCommunication:
    def __init__(self):
        self.connection = None
        self.connection_opened = False
        self.currentMainboardReturnData = ""

    def open(self):

        try:
            ports = subprocess.check_output('ls /dev/ttyACM1', shell=True).decode("utf-8").split('\n')[:-1]

            self.connection_opened = False

            for port in ports:  # analyze serial ports
                try:
                    while not self.connection_opened:
                        self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=True)
                        self.connection_opened = self.connection.isOpen()
                        print('self.connection_opened', self.connection_opened)
                        time.sleep(0.5)
                    self.connection.flush()
                    print("mainboard: Port opened successfully")
                except Exception as e:
                    print(e)
                    continue

        except TypeError as err:
            print(err)
        except:
            print('mainboard: /dev/ttyACM empty', sys.exc_info()[0])
            return False

        return self.connection_opened

    def write(self, comm):
        if self.connection is not None:
            try:
                self.connection.write((comm + '\n').encode('utf-8'))
            except TypeError as err:
                print('mainboard: err write ' + comm, err)
            except:
                print('mainboard: err write ' + comm, sys.exc_info()[0])

    def set_PID(self, P1, S1, P2, S2, P3, S3, P4, S4, P5, S5, P6, S6):
        if self.connection_opened:
            PID_command = "ps:{}:{}:{}:{}:{}:{}:{}:{}:{}:{}:{}:{}".format(int(P1), int(S1),
                                                                          int(P2), int(S2),
                                                                          int(P3), int(S3),
                                                                          int(P4), int(S4),
                                                                          int(P5), int(S5),
                                                                          int(P6), int(S6))
            # print(P1, S1, P2, S2, P3, S3)
            self.write(PID_command)
        else:
            print('Mainboard opened failed')

    def read(self):
        while self.connection is not None and self.connection.isOpen() and self.connection.in_waiting > 0:
            character = self.connection.read().decode('utf-8')  # read the mainboard feedback character by character
            if character == '\n':
                fullCommand = self.currentMainboardReturnData  # save all characters in full command
                self.currentMainboardReturnData = ""
                # print(fullCommand)
                return fullCommand

            self.currentMainboardReturnData += character
        return ""

    def write_mainboard(self, position1, speed1,
                        position2, speed2,
                        position3, speed3,
                        position4, speed4,
                        position5, speed5,
                        position6, speed6):
        self.set_PID(
            position1, speed1,
            -position2, speed2,
            position3, speed3,
            -position4, speed4,
            position5, speed5,
            -position6, speed6
        )

    def extract_poistion(self, text):
        pattern = 'Pos0:([-\d]+), Pos1:([-\d]+), Pos2:([-\d]+), Pos3:([-\d]+), Pos4:([-\d]+), Pos5:([-\d]+)'
        m = re.search(pattern, text)

        if m is not None:
            return (int(m.group(1)),
                    -int(m.group(2)),
                    int(m.group(3)),
                    -int(m.group(4)),
                    int(m.group(5)),
                    -int(m.group(6)))
        else:
            #print('no match')
            return


if __name__ == '__main__':
    motion = Motion()

    serial_communication = SerialCommunication()
    serial_communication.open()

    serial_communication.write_mainboard(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    while True:

        positions = serial_communication.extract_poistion(serial_communication.read())

        if positions is not None:
            print(positions)
            (
                position1, speed1,
                position2, speed2,
                position3, speed3,
                position4, speed4,
                position5, speed5,
                position6, speed6
            ) = motion.update(positions)

            serial_communication.write_mainboard(position1, speed1,
                                                 position2, speed2,
                                                 position3, speed3,
                                                 position4, speed4,
                                                 position5, speed5,
                                                 position6, speed6)

        if keyboard.is_pressed('up arrow'):
            motion.forward_speed = 10
        # break
        elif keyboard.is_pressed('down arrow'):
            motion.forward_speed = -10
        # break
        elif keyboard.is_pressed('left arrow'):
            motion.rotate_speed = -10
        # break
        elif keyboard.is_pressed('right arrow'):
            motion.rotate_speed = 10
        else:
            motion.forward_speed = 0
            motion.rotate_speed = 0

        time.sleep(0.01)
