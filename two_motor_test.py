import sys
import time
import numpy as np
from canmotorlib import CanMotorController


def setZeroPosition(motor, initPos):

    pos = initPos

    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                curr))


# Motor ID
motor_id1 = 0x01
motor_id2 = 0x02

if len(sys.argv) != 2:
    print('Provide CAN device name (can0, slcan0 etc.)')
    sys.exit(0)

print("Using Socket {} for can communucation".format(sys.argv[1],))

motor_1 = CanMotorController(sys.argv[1], motor_id1)
motor_2 = CanMotorController(sys.argv[1], motor_id2)

print("Enabling Motors..")

pos1, vel1, curr1 = motor_1.enable_motor()

print("Motor 1 Status: Pos: {}, Vel: {}, Torque: {}".format(pos1, vel1, curr1))

pos2, vel2, curr2 = motor_2.enable_motor()

print("Motor 2 Status: Pos: {}, Vel: {}, Torque: {}".format(pos2, vel2, curr2))

# print("Setting Motor 1 to Zero Position...")

# setZeroPosition(motor_1, pos1)

# print("Setting Motor 2 to Zero Position...")

# setZeroPosition(motor_2, pos2)


# # Rotation Test. Uncommnent to rotate the motors for almost 4 revolutions.
# angularVelDeg = 720
# sleepTime = 2

# pos1, vel1, curr1 = motor_1.send_deg_command(0, angularVelDeg, 0, 5, 0)
# pos2, vel2, curr2 = motor_2.send_deg_command(0, angularVelDeg / 2, 0, 5, 0)

# time.sleep(sleepTime)

# pos1, vel1, curr1 = motor_1.send_deg_command(0, 0, 0, 5, 0)
# pos2, vel2, curr2 = motor_2.send_deg_command(0, 0, 0, 5, 0)

# time.sleep(0.5)


print("Disabling Motors...")

pos1, vel1, curr1 = motor_1.disable_motor()

print("Motor 1 Status: Pos: {}, Vel: {}, Torque: {}".format(pos1, vel1, curr1))

pos2, vel2, curr2 = motor_2.disable_motor()

print("Motor 2 Status: Pos: {}, Vel: {}, Torque: {}".format(pos2, vel2, curr2))
