from src.Drone import *

connection_string = "udp:127.0.0.1:14550"
drone = Drone(connection_string)

drone.arm()
drone.change_alt(2, kp=15, ki=4, kd=0, l_limit=30, u_limit=70, debug=True)