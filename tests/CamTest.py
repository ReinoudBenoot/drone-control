from src.Drone import *

connection_string = 'udp:127.0.0.1:14550'

drone = Drone(connection_string, alt_type="barometer")
drone.scan = True
while True:
    print(drone.target[-1])