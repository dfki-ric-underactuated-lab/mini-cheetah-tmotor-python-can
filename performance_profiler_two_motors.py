import cProfile
import time
import numpy as np
from canmotorlib import CanMotorController

# Motor ID
motor_id1 = 0x01
motor_id2 = 0x02
can_port = 'can0'
motor_1 = CanMotorController(can_port, motor_id1)
motor_2 = CanMotorController(can_port, motor_id2)


def print_pofile_func(to_print, numTimes, sleepTime):
    for i in range(numTimes):
        print(to_print)
        time.sleep(sleepTime)


def print_10():
    print_pofile_func("test print!!", 10, 0.1)


currTime = np.zeros(10000)


def motor_send_n_commands(numTimes):
    for i in range(numTimes):
        pos1, vel1, curr1 = motor_1.send_deg_command(0, 0, 0, 0, 0)
        pos2, vel2, curr2 = motor_2.send_deg_command(0, 0, 0, 0, 0)
        currTime[i] = time.time()


if __name__ == "__main__":
    # cProfile.run('print_10()')
    # cProfile.run("print_pofile_func('test', 10, 0.01)")
    motor_1.enable_motor()
    motor_2.enable_motor()
    steps_array = np.linspace(10000, 10000, 1)
    startdtTest = time.time()
    # steps_array = np.linspace(0, 5, 6)
    for i in steps_array:
        print("Starting Profiler for {} Commands".format(i))
        cmdString ="motor_send_n_commands({})".format(int(i))
        # cmdString = "print_pofile_func('test', {}, 0.1)".format(int(i))
        cProfile.run(cmdString)

    enddtTest = time.time()
    motor_1.disable_motor()
    motor_2.disable_motor()
    dt = (enddtTest - startdtTest) / 10000
    cmd_freq = 1 / dt
    print("Dt = {}".format(dt))
    print("Command Frequency: {} Hz".format(cmd_freq))
