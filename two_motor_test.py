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
motor_id_shoulder = 0x01
motor_id_elbow = 0x03

if len(sys.argv) != 2:
    print('Provide CAN device name (can0, slcan0 etc.)')
    sys.exit(0)

print("Using Socket {} for can communucation".format(sys.argv[1],))

motor_shoulder = CanMotorController(sys.argv[1], motor_id_shoulder)
motor_elbow = CanMotorController(sys.argv[1], motor_id_shoulder)

print("Enabling Motors..")

pos_shoulder, vel_shoulder, curr_shoulder = motor_shoulder.enable_motor()

print("Shoulder Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_shoulder, vel_shoulder,
                                                            curr_shoulder))

pos_elbow, vel_elbow, curr_elbow = motor_elbow.enable_motor()

print("Elbow Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_elbow, vel_elbow, curr_elbow))

print("Setting Shoulder Motor to Zero Position...")

setZeroPosition(motor_shoulder, pos_shoulder)

print("Setting Elbow Motor to Zero Position...")

setZeroPosition(motor_elbow, pos_elbow)


# # Rotation Test. Uncommnent to rotate the motors for almost 4 revolutions.
# angularVelDeg = 720
# sleepTime = 2

# pos_shoulder, vel_shoulder, curr_shoulder = motor_1.send_deg_command(0, angularVelDeg, 0, 5, 0)
# pos_elbow, vel_elbow, curr_elbow = motor_2.send_deg_command(0, angularVelDeg / 2, 0, 5, 0)

# time.sleep(sleepTime)

# pos_shoulder, vel_shoulder, curr_shoulder = motor_1.send_deg_command(0, 0, 0, 5, 0)
# pos_elbow, vel_elbow, curr_elbow = motor_2.send_deg_command(0, 0, 0, 5, 0)

# time.sleep(0.5)


print("Disabling Motors...")

pos_shoulder, vel_shoulder, curr_shoulder = motor_elbow.disable_motor()

print("Shoulder Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_shoulder, vel_shoulder,
                                                                    curr_shoulder))

pos_elbow, vel_elbow, curr_elbow = motor_elbow.disable_motor()

print("Elbow Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_elbow, vel_elbow, curr_elbow))
