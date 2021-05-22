from src.Drone import *
import matplotlib.pyplot as plt
import time

connection_string = 'udp:127.0.0.1:14550'

drone = Drone(connection_string, alt_type='barometer')
altitude = []
duration = 10
start_time = time.monotonic()
delta_t = time.monotonic() - start_time
t_list = []
while delta_t < duration:
    alt = drone.get_altitude(5)
    print("{0}: {1}".format(round(delta_t), round(alt, 2)))
    altitude.append(alt)
    t_list.append(delta_t)
    delta_t = time.monotonic() - start_time

plt.plot(t_list, altitude)
plt.xlabel("Tijd [s]")
plt.ylabel("Hoogte [m]")
plt.title("Barometrische hoogte")
plt.ylim([-0.5, 0.5])
plt.show()