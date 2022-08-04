import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController


def setZeroPosition(motor, initPos):

    pos = initPos

    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                curr))


# Motor ID
motor_id_shoulder = 0x08
motor_id_shoulder2 = 0x09
motor_id_elbow = 0x05
motor_id_elbow2 = 0x03

if len(sys.argv) != 3:
    print('Provide CAN device names (can0, slcan0 etc.)')
    sys.exit(0)

print("Using Socket {} for can communication".format(sys.argv[1],))

motor_shoulder = CanMotorController(sys.argv[1], motor_id_shoulder)
motor_shoulder2 = CanMotorController(sys.argv[1], motor_id_shoulder2)
motor_elbow = CanMotorController(sys.argv[2], motor_id_elbow)
motor_elbow2 = CanMotorController(sys.argv[2], motor_id_elbow2)

print("Enabling Motors..")

pos_shoulder, vel_shoulder, curr_shoulder = motor_shoulder.enable_motor()

print("Shoulder Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_shoulder, vel_shoulder,
                                                            curr_shoulder))


pos_shoulder2, vel_shoulder, curr_shoulder = motor_shoulder2.enable_motor()

print("Shoulder Motor 2 Status: Pos: {}, Vel: {}, Torque: {}".format(pos_shoulder, vel_shoulder,
                                                            curr_shoulder))

pos_elbow, vel_elbow, curr_elbow = motor_elbow.enable_motor()

print("Elbow Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_elbow, vel_elbow, curr_elbow))

pos_elbow2, vel_elbow, curr_elbow = motor_elbow2.enable_motor()

print("Elbow Motor 2 Status: Pos: {}, Vel: {}, Torque: {}".format(pos_elbow, vel_elbow, curr_elbow))


print("Setting Shoulder Motor to Zero Position...")

setZeroPosition(motor_shoulder, pos_shoulder)

print("Setting Shoulder 2 Motor to Zero Position...")

setZeroPosition(motor_shoulder2, pos_shoulder2)

print("Setting Elbow Motor to Zero Position...")

setZeroPosition(motor_elbow, pos_elbow)

print("Setting Elbow 2 Motor to Zero Position...")

setZeroPosition(motor_elbow2, pos_elbow2)

time.sleep(1)

print("Moving 180 degrees...")

pos, vel, curr = motor_shoulder.send_deg_command(0, 90, 0, 2, 0)
print("Shoulder Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
pos, vel, curr = motor_shoulder2.send_deg_command(0, 90, 0, 2, 0)
print("Shoulder 2 Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
pos, vel, curr = motor_elbow.send_deg_command(0, 90, 0, 2, 0)
print("Elbow Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
pos, vel, curr = motor_elbow2.send_deg_command(0, 90, 0, 2, 0)
print("Elbow 2 Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
time.sleep(2)

print("Sending Zero Command")

pos, vel, curr = motor_shoulder.send_deg_command(0, 0, 0, 0, 0)
print("Shoulder Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
pos, vel, curr = motor_shoulder2.send_deg_command(0, 0, 0, 0, 0)
print("Shoulder 2 Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
pos, vel, curr = motor_elbow.send_deg_command(0, 0, 0, 0, 0)
print("Elbow Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
pos, vel, curr = motor_elbow2.send_deg_command(0, 0, 0, 0, 0)
print("Elbow 2 Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))

print("Disabling Motors...")

pos_shoulder, vel_shoulder, curr_shoulder = motor_shoulder.disable_motor()

print("Shoulder Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_shoulder, vel_shoulder,
                                                                    curr_shoulder))

pos_shoulder, vel_shoulder, curr_shoulder = motor_shoulder2.disable_motor()

print("Shoulder 2 Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_shoulder, vel_shoulder,
                                                                    curr_shoulder))

pos_elbow, vel_elbow, curr_elbow = motor_elbow.disable_motor()

print("Elbow Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_elbow, vel_elbow, curr_elbow))

pos_elbow, vel_elbow, curr_elbow = motor_elbow2.disable_motor()

print("Elbow 2 Motor Status: Pos: {}, Vel: {}, Torque: {}".format(pos_elbow, vel_elbow, curr_elbow))
