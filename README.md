# drone-control
| Information  | Content |
| ------------- | ------------- |
| Author  | Reinoud Benoot  |
| Contact  | reinoud.benoot@student.kuleuven.be  |

In this repository you will find a library for controlling a multicopter using your computer. The library uses ArduPilot and MAVLink. The library is meant to be used for indoor flight. It also contains functionalities which require a camera.

## Table of contents
- [Installation](#Installation)
- [Features](#Features)
- [Simulator](#Simulator)
- [Usage](#Usage)

## Installation
In order to use this library you will need to clone this repository to your computer. You will also need to [install](https://mavlink.io/en/getting_started/installation.html) the MAVLink library for Python. Note: Certain functionalities also require the drone-object-detection-using-haar-cascades library which can be found [here](https://github.com/thomassabbe/drone-object-detection-using-haar-cascades). The library also uses [Matplotlib](https://matplotlib.org/stable/users/installing.html), [NumPy](https://numpy.org/install/) and [pynput](https://pypi.org/project/pynput/).

Note: MAVLink's Python library also requires a library called serial. This library is not always automatically installed when installing MAVLink. In this case serial can be manually installed via this [link](https://pypi.org/project/serial/).

## Features
- Basic controls for indoor flight
- Navigation to a target detected using the drone-object-detection-using-haar-cascades library

## Useful software
ArduPilot's SITL simulator was used for testing and developing the code. This simulator does not require any hardware and can be used to accurately simulate the behaviour of the drone. For further info and installation see following [link](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html). A tutorial on how to use the simulator is provided at [ArduPilot SITL simulator Tutorial](https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html).

Two other useful programs that were used are [MissionPlanner](https://ardupilot.org/planner/docs/mission-planner-overview.html) and [QGroundControl](http://qgroundcontrol.com/). Both these programs are can be used to plan, analyse and setup both the drone in the simulator and the real drone. For installation see [MissionPlanner](https://ardupilot.org/planner/docs/mission-planner-installation.html) and [QGroundControl](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html).

## Usage
Using the library is fairly straightforward. The library consists of three classes: Drone, Controller and PID. The Drone class contains many functions which a user can use to control the drone. The Controller and PID classes on the otherhand contain functions that are used in the Drone class but are not supposed to be used on their own. The main purpose of these two classes is to create structure in the library and to make it clearer for the user which functions are meant to be used.\
Inside the Drone class you will find a lot of documentation in the code itself explaining the purpose of each function. The most important functions for a user are listed here:
| Function  | Description |
| ------------- | ------------- |
| arm  | Reinoud Benoot  |
| Contact  | reinoud.benoot@student.kuleuven.be  |

Note: This library was developed for indoor flight. As a result you will not find any functions that use a GPS module for navigation.
