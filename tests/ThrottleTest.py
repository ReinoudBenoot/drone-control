from src.Drone import *
import time

connection_string = "udp:127.0.0.1:14550"
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
    drone.set_throttle(20, priority=5)
    thr = drone.servo_output_raw[-1]
    rc = drone.rc_channels[-1]
    print("Input1:", rc['chan3_raw'])
    print("Motor1:", thr['servo1_raw'])
    print("Motor3:", thr['servo3_raw'])
    print("Motor5:", thr['servo5_raw'])
    alt = drone.get_altitude(1)
    servo1.append(thr['servo1_raw'])
    servo2.append(thr['servo2_raw'])
    servo3.append(thr['servo3_raw'])
    servo4.append(thr['servo4_raw'])
    servo5.append(thr['servo5_raw'])
    servo6.append(thr['servo6_raw'])
    t.append(time.monotonic() - start_time)
    h.append(alt)
    rc_l.append(rc['chan3_raw'])

    if time.monotonic() - start_time > 3:
        drone.set_throttle(0, priority=6)
        break

drone.disarm()
plt.figure(1)
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

plt.figure(2)
plt.plot(t, h, color="black", label="Altitude [m]")
plt.legend()
plt.grid(b=True)
plt.show()
