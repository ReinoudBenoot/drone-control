from src.Drone import *
import matplotlib.pyplot as plt
import time

connection_string = 'com3'

drone = Drone(connection_string)
vz = []
duration = 10
start_time = time.monotonic()
delta_t = time.monotonic() - start_time
t_list = []
while delta_t < duration:
    imu = drone.get_message("RAW_IMU", True, priority=5)
    imu = imu.to_dict()
    print(imu['zacc'])
    vz.append(imu['zacc'])
    t_list.append(delta_t)
    delta_t = time.monotonic() - start_time

plt.plot(t_list, vz)
plt.xlabel("Tijd [s]")
plt.ylabel("snelheid [m/s]")
plt.title("Verticale snelheid")
plt.show()