from src.Drone import *

connection_string = 'udp:127.0.0.1:14550'

drone = Drone(connection_string, alt_type="barometer")

drone.arm_and_takeoff(2)
drone.go_to_target(5)
drone.land()
drone.disarm()