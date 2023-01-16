import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController


def setZeroPosition(motor):
    pos, _, _ = motor.set_zero_position()
    while abs(np.rad2deg(pos)) > 0.5:
        pos, vel, curr = motor.set_zero_position()
        print("Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))


def main():

    if len(sys.argv) < 3:
        print("Provide CAN device name (can0, slcan0 etc.) and motor IDs. E.g. python3 can_motorlib_test.py can0 2 3")
        sys.exit(0)

    print(
        "Using Socket {} for can communication".format(
            sys.argv[1],
        )
    )
    motor_controller_dict = {}
    for i in range(2, len(sys.argv)):
        motor_controller_dict[int(sys.argv[i])] = CanMotorController(
            sys.argv[1], int(sys.argv[i]), motor_type="AK80_6_V1p1"
        )

    print("Enabling Motors..")
    for motor_id, motor_controller in motor_controller_dict.items():
        pos, vel, curr = motor_controller.enable_motor()
        print("Motor {} Status: Pos: {}, Vel: {}, Torque: {}".format(motor_id, pos, vel, curr))

    print("Setting Shoulder Motor to Zero Position...")
    for motor_id, motor_controller in motor_controller_dict.items():
        print("Setting Motor {} to Zero Position...".format(motor_id))
        setZeroPosition(motor_controller)

    time.sleep(1)
    print("Moving Motors...")
    # Moving 180 degrees
    for motor_id, motor_controller in motor_controller_dict.items():
        pos, vel, curr = motor_controller.send_deg_command(0, 90, 0, 2, 0)
        print("Moving Motor {} Position: {}, Velocity: {}, Torque: {}".format(motor_id, pos, vel, curr))
    time.sleep(2)

    for motor_id, motor_controller in motor_controller_dict.items():
        pos, vel, curr = motor_controller.send_deg_command(0, 0, 0, 0, 0)
        print("Reached Motor {} Position: {}, Velocity: {}, Torque: {}".format(motor_id, pos, vel, curr))
    time.sleep(1)

    print("Disabling Motors...")
    for motor_id, motor_controller in motor_controller_dict.items():
        pos, vel, curr = motor_controller.disable_motor()
        time.sleep(0.2)
        print("Motor {} Status: Pos: {}, Vel: {}, Torque: {}".format(motor_id, pos, vel, curr))


if __name__ == "__main__":
    main()
