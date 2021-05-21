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
ArduPilot's SITL simulator was used for testing and developing the code. This simulator does not require any hardware and can be used to accurately simulate the behaviour of the drone to the code. For further info and installation see the following [link](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

## Usage
ArduPilot's SITL simulator was used for testing and developing the code.
