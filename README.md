# drone-control
| Information  | Content |
| ------------- | ------------- |
| Author  | Reinoud Benoot  |
| Contact  | reinoud.benoot@student.kuleuven.be  |

In this repository you will find a library for controlling a multicopter using your computer. The library uses ArduPilot and MAVLink. The library is meant to be used for indoor flight. It also contains functionalities which require a camera.

## Table of contents
- [Installation](#Installation)
- [Features](#Features)
- [Useful software](#Useful-software)
- [Usage](#Usage)
- [Code Example](#Code-Example)
- [Short Demonstration](#Short-Demonstration)

## Installation
In order to use this library you will need to clone this repository to your computer. You will also need to [install](https://mavlink.io/en/getting_started/installation.html) the MAVLink library for Python. Note: Certain functionalities also require the drone-object-detection-using-haar-cascades library which can be found [here](https://github.com/thomassabbe/drone-object-detection-using-haar-cascades). The library also uses [Matplotlib](https://matplotlib.org/stable/users/installing.html), [NumPy](https://numpy.org/install/) and [pynput](https://pypi.org/project/pynput/).

Note: MAVLink's Python library also requires a library called serial. This library is not always automatically installed when installing MAVLink. In this case serial can be manually installed via this [link](https://pypi.org/project/serial/).

## Features
- Basic controls for indoor flight
- Navigation to a target detected using the drone-object-detection-using-haar-cascades library

## Useful software
ArduPilot's SITL simulator was used for testing and developing the code. This simulator does not require any hardware and can be used to accurately simulate the behaviour of the drone. For further info and installation see following [link](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).  To start the simulator following commands must be executed in cygwin, assuming you installed the simulator following the steps in the link above:
```
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter --map --console --out 127.0.0.1:14550
```
For further information, a tutorial on how to use the simulator is provided at [ArduPilot SITL simulator Tutorial](https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html).

Two other useful programs that were used are [MissionPlanner](https://ardupilot.org/planner/docs/mission-planner-overview.html) and [QGroundControl](http://qgroundcontrol.com/). Both these programs are Ground Control Software (GCS) that can be used to plan, analyse and setup both the drone in the simulator and the real drone. For installation see [MissionPlanner](https://ardupilot.org/planner/docs/mission-planner-installation.html) and [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).

## Usage
Using the library is fairly straightforward. The library consists of three classes: Drone, Controller and PID. The Drone class contains many functions which a user can use to control the drone. The Controller and PID classes on the otherhand contain functions that are used in the Drone class but are not supposed to be used on their own. The main purpose of these two classes is to create structure in the library and to make it clearer for the user which functions are meant to be used.\
Inside the Drone class you will find a lot of documentation in the code itself explaining the purpose of each function. The most important functions for a user are listed here:
| Function  | Description |
| ------------- | ------------- |
| arm  | Arms the drone.  |
| arm_and_takeoff  | Arms the drone and starts taking off to the desired altitude. The program continues after reaching the desired altitude.  |
| mode  | Sets the mode of the drone. Only Stabilize mode and Altitude Hold mode are available when flying inside.  |
| change_alt  | Orders the drone to fly to the desired altitude. The program continues after reaching the desired altitude.  |
| set_throttle  | Sets the throttle of the drone. Input is in %. <br /> Note: This function should only be used for debugging and testing purposes. For changing altitude, using change_alt is recommended.|
| land  | Lands the drone.  |
| go_to_target  | Can be used to order the drone to fly to the detected target. This only causes the drone to move horizontally, so the drone should already be airborne.  |
| pitch  | Causes the drone to pitch to the desired angle.  |
| roll  | Causes the drone to roll to the desired angle.  |
| yaw  | Causes the drone to yaw to the desired angle relative to its current heading.  |
| disarm  | Disarms the drone. Only works when the drone is no longer airborne.  |

The library also contains a killswitch to instantly shut down the drone and the program. This killswitch is bound to 'k'.

Note: This library was developed for indoor flight. As a result you will not find any functions that use a GPS module for navigation.

## Code Example
The following lines of code are an example on how to use the library. 
```
from src.Drone import *

connection_string = 'udp:127.0.0.1:14550'

drone = Drone(connection_string, alt_type="barometer")

drone.arm_and_takeoff(2)
time.sleep(1)
drone.land()
drone.disarm()
```
When an object of the Drone class is created, a connection is also made with the drone or the simulator. To connect to the SITL simulator the connection string should be `'udp:127.0.0.1:14550'`. To connect to the drone, the connection string should be the COM port used by the telemetry radio, e.g., `'COM3'`. Further, this example orders the drone to takeoff to an altitude of 2 meters, wait there for one second and then land and disarm. In this example, the barometer is used to measure the altitude. `alt_type` can also be set to `"sonar"` or `"gps"`. The GPS option should only be used for the simulator since the simulator does not suffer from bad GPS signal. When flying a real drone indoors, using this option is not recommended. 

## Short Demonstration
In following video a short demonstration in the simulator is shown. The drone takes off to an altitude of approx. 2m after which it rotates through an angle of 20°. Then it pitches forward and flies a distance until it slows down again. The demonstration ends with the drone landing.
Note: This demonstration does not include the object detection. The drone simply flies a certain distance forward.
[![Short Demonstration](http://img.youtube.com/vi/LFrgGkENEBE/0.jpg)](http://www.youtube.com/watch?v=LFrgGkENEBE)
