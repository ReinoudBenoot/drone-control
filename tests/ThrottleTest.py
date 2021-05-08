from matplotlib import pyplot as plt
from src.Drone import *
import time

connection_string = "COM4"
drone = Drone(connection_string)

drone.arm()
flightmode = "STABILIZE"
drone.mode(flightmode, 10)

servo1 = []
servo2 = []
servo3 = []
servo4 = []
servo5 = []
servo6 = []
t = []
h = []
ref_alt = drone.get_altitude(1)
start_time = time.monotonic()
rc_l = []
while True:
    drone.multiplexer_add('throttle_control', 20, priority=5)
    thr = drone.get_message('SERVO_OUTPUT_RAW', True, priority=4)
    rc = drone.get_message('RC_CHANNELS', True, priority=4)
    print("Input1:", rc['chan3_raw'])
    print("Motor1:", thr['servo1_raw'])
    print("Motor3:", thr['servo3_raw'])
    print("Motor5:", thr['servo5_raw'])
    press = drone.get_altitude(1)
    alt_diff = ((ref_alt - press) * 100) / (1.29 * 9.81)
    servo1.append(thr['servo1_raw'])
    servo2.append(thr['servo2_raw'])
    servo3.append(thr['servo3_raw'])
    servo4.append(thr['servo4_raw'])
    servo5.append(thr['servo5_raw'])
    servo6.append(thr['servo6_raw'])
    t.append(time.monotonic() - start_time)
    h.append(alt_diff)
    rc_l.append(rc['chan3_raw'])

    if time.monotonic() - start_time > 3:
        drone.multiplexer_add('throttle_control', 10, priority=6)
        drone.multiplexer_add('throttle_control', 0, priority=7)
        break

drone.vehicle.arducopter_disarm()
plt.figure()
plt.plot(t, servo1, color="red", label="Motor 1")
plt.plot(t, servo2, "--",color="red", label="Motor 2")
plt.plot(t, servo3, color="yellow", label="Motor 3")
plt.plot(t, servo4, "--",color="yellow", label="Motor 4")
plt.plot(t, servo5, color="black", label="Motor 5")
plt.plot(t, servo6, "--",color="black", label="Motor 6")
plt.plot(t, rc_l, color="green", label="rc input")
plt.legend()
plt.grid(b=True)
plt.show()

plt.figure()
plt.plot(t, h, color="black", label="Altitude [m]")
plt.legend()
plt.grid(b=True)
plt.show()
