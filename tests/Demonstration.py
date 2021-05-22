from src.Drone import *

connection_string = 'udp:127.0.0.1:14550'

drone = Drone(connection_string, alt_type="barometer")

drone.arm_and_takeoff(2)
drone.yaw(20)
drone.pitch(10)
time.sleep(2)
while True:
    vx = drone.local_position_ned[-1]['vx']
    angle = 3 * (0 + vx)
    drone.pitch(angle)
    if -0.2 < vx < 0.5:
        drone.pitch(0)
        break
time.sleep(0.5)
drone.land()
drone.disarm()