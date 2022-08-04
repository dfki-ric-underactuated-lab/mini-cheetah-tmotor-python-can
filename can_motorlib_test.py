import sys
import time
import numpy as np
from src.motor_driver.canmotorlib import CanMotorController

# Motor ID
motor_id = 0x05

if len(sys.argv) != 2:
    print('Provide CAN device name (can0, slcan0 etc.)')
    sys.exit(0)

print("Using Socket {} for can communication".format(sys.argv[1],))
# print(type((sys.argv[1],)))

motor_controller = CanMotorController(sys.argv[1], motor_id)

startTime = time.perf_counter()

pos, vel, curr = motor_controller.enable_motor()

# pos, vel, curr = motor_controller.send_deg_command(0, 0, 0, 0, 0)
print("Initial Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel),
                                                                curr))

endTime = time.perf_counter()

print("Time for One Command: {}".format(endTime - startTime))

time.sleep(1)

while abs(np.rad2deg(pos)) > 0.5:
    pos, vel, curr = motor_controller.set_zero_position()
    print("Zero Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))

time.sleep(1)

# Moving 180 degrees
pos, vel, curr = motor_controller.send_deg_command(0, 90, 0, 2, 0)
print("Moving Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
time.sleep(2)
# Send zeros
pos, vel, curr = motor_controller.send_deg_command(0, 0, 0, 0, 0)
print("Reached Position: {}, Velocity: {}, Torque: {}".format(pos, vel, curr))
time.sleep(1)

pos, vel, curr = motor_controller.disable_motor()
print("Final Position: {}, Velocity: {}, Torque: {}".format(np.rad2deg(pos), np.rad2deg(vel), curr))
