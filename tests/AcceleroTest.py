from src.Drone import *
import matplotlib.pyplot as plt
import time

connection_string = 'udp:127.0.0.1:14550'

drone = Drone(connection_string)
za = []
duration = 10
start_time = time.monotonic()
delta_t = time.monotonic() - start_time
t_list = []
while delta_t < duration:
    if not drone.scaled_imu2:
        continue
    imu = drone.scaled_imu2[-1]
    print(imu['zacc'])
    za.append(imu['zacc'])
    t_list.append(delta_t)
    delta_t = time.monotonic() - start_time

plt.plot(t_list, za)
plt.xlabel("Tijd [s]")
plt.ylabel("snelheid [m/s]")
plt.title("Verticale snelheid")
plt.show()