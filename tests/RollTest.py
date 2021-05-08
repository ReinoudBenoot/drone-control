from src.Drone import *

connection_string = 'udp:127.0.0.1:14550'

drone = Drone(connection_string, alt_type="barometer")
drone.arm_and_takeoff(0.25)
drone.roll(5)
time.sleep(1)
drone.roll(0)
drone.land()
drone.disarm()