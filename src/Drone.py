import time
from pymavlink import mavutil
import matplotlib.pyplot as plt
import threading
import math
from pynput import keyboard
import os
from src.dronevision_library import visual_algorithm
import numpy

class Drone:

    def __init__(self, connection_string, alt_type="barometer", scantime=0, color='white'):
        print("Connecting...")
        try:
            self.vehicle = mavutil.mavlink_connection(connection_string, force_connected=True, baud=57600)
            self.vehicle.wait_heartbeat()
            print("Successfully connected to system", self.vehicle.target_system)
        except TimeoutError:
            print("Failed to connect")

        self.interrupt()
        self.lock = threading.Lock()
        self.commands = {}
        self.controller = Controller(self)
        self.functions = {'set_throttle': self.set_throttle, 'set_mode': self.set_mode,
                          'recv_msg': self.recv_msg, 'param_request_read': self.param_request_read,
                          'manual_control': self.controller.manual_control}
        self.output = {}
        self.execute(func=self.multiplexer, args=(), daemon=True)
        self.execute(func=self.read_message, args=(), daemon=True)
        # waits until the script starts receiving messages
        while not self.heartbeat:
            time.sleep(0.1)

        self.hold_alt = False
        self.alt_hold_throttle = 50

        self.target = []
        self.scan = False

        self.start_pressure = 0

        self.alt_type = alt_type
        self.flightmode = self.get_mode(1)

        self.disable_gps()

        self.execute(func=self.alt_hold, args=(), daemon=True)
        self.execute(func=self.get_target, args=(scantime, color), daemon=True)
        self.hover_throttle = None

    def arm(self):
        """Arms the drone."""
        self.controller.load_parameters()
        self.hover_throttle = self.get_parameter(b'MOT_THST_HOVER', 1)['param_value']

        #Sets the mode to stabilize before arming since the drone is not armable in some modes
        self.mode('STABILIZE', 10)
        self.vehicle.arducopter_arm()
        print("Arming drone...")
        #Wait for the drone to be armed
        while True:
            if self.armed:
                break
            time.sleep(0.5)
        print("Drone is armed")

    @property
    def armed(self):
        """Returns whether the drone is armed or not."""
        heartbeat = self.heartbeat[-1]
        if heartbeat['base_mode'] == 209:
            return True
        elif heartbeat['base_mode'] == 81:
            return False
        else:
            print("Unknown state")

    def disarm(self):
        """Disarms the drone. Can only be done when not airborne."""
        time.sleep(1)
        while True:
            self.vehicle.arducopter_disarm()
            if not self.armed:
                break
            time.sleep(0.5)

    def arm_and_takeoff(self, alt):
        """Arms and requests the drone to take off."""
        self.arm()
        self.change_alt(alt, kp=15, ki=4, kd=0, l_limit=30, u_limit=70)
        print("Altitude reached")

    def disable_gps(self):
        """Changes certain parameters to disable the usage of the gps."""
        self.vehicle.param_set_send('ATT_ACC_COMP', 0, mavutil.mavlink.MAVLINK_TYPE_INT32_T)
        self.vehicle.param_set_send('ATT_MAG_DECL_A', 0, mavutil.mavlink.MAVLINK_TYPE_INT32_T)
        self.vehicle.param_set_send('EKF2_AID_MASK', 5.60519385729926828369491833316E-45,
                                    mavutil.mavlink.MAVLINK_TYPE_INT32_T)

    def mode(self, flightmode, priority):
        """Changes the flightmode of the drone."""
        print("Setting flightmode...")
        self.multiplexer_add('set_mode', param1=flightmode, priority=priority)
        while self.flightmode != ('COPTER_MODE_' + flightmode):
            time.sleep(0.2)
            self.flightmode = self.get_mode(1)

    def set_mode(self, flightmode):
        """Sends the command to change the flightmode."""
        self.vehicle.set_mode(flightmode)

    def get_mode(self, priority):
        """Returns the mode of the drone."""
        heartbeat = self.heartbeat[-1]
        custom_mode = heartbeat['custom_mode']
        return mavutil.mavlink.enums['COPTER_MODE'][custom_mode].name

    def ack_command(self, priority):
        """Provides feedback about the command that was sent.

        Outputs either that the command was received and executed, received but still waiting to execute or was not
        executed.
        """
        ack_msg = self.get_message('COMMAND_ACK', True, priority=priority)
        return mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description

    def baro_altitude(self, priority):
        """Returns the altitude of the drone based on the barometer."""
        # sets the pressure of the initial position if it has not been done before
        if self.start_pressure == 0:
            self.start_pressure = self.get_pressure(priority)

        # calculates the pressure using the formula used by ArduPilot itself
        # see https://discuss.ardupilot.org/t/estimating-gps-vs-baro-height-some-thoughts/25952/5 for more information
        pressure = self.get_pressure(priority)
        scaling = pressure / self.start_pressure
        temp = 293.15
        altitude = 153.8462 * temp * (1.0 - math.exp(0.190259 * math.log(scaling)))

        return altitude

    def get_pressure(self, priority):
        """Returns the pressure measured by the barometer."""
        scaled_pressure = self.scaled_pressure[-1]
        return scaled_pressure['press_abs']

    def get_message(self, msg, blocking, priority=0):
        """Sends commands to request messages from the drone."""
        message = self.recv_msg(msg, blocking)
        message = message.to_dict()
        return message

    def read_message(self):
        """Reads every message sent by the drone and stores it in the corresponding list."""
        # Defining all the required lists
        self.heartbeat = []
        self.attitude = []
        self.scaled_pressure = []
        self.gps_raw_int = []
        self.param_value = []
        self.scaled_imu2 = []
        self.servo_output_raw = []
        self.ack_comm = []
        self.local_position_ned = []
        self.rc_channels = []

        while True:
            message = self.vehicle.recv_match()
            if not message:
                continue
            # checks which message it received and adds it to the corresponding list
            if message.name == 'HEARTBEAT':
                self.heartbeat.append(message.to_dict())
            elif message.name == 'ATTITUDE':
                self.attitude.append(message.to_dict())
            elif message.name == 'SCALED_PRESSURE':
                self.scaled_pressure.append(message.to_dict())
            elif message.name == 'GPS_RAW_INT':
                self.gps_raw_int.append(message.to_dict())
            elif message.name == 'PARAM_VALUE':
                self.param_value.append(message.to_dict())
            elif message.name == 'SCALED_IMU2':
                self.scaled_imu2.append(message.to_dict())
            elif message.name == 'SERVO_OUTPUT_RAW':
                self.servo_output_raw.append(message.to_dict())
            elif message.name == 'ACK_COMMAND':
                self.ack_comm.append(message.to_dict())
            elif message.name == 'LOCAL_POSITION_NED':
                self.local_position_ned.append(message.to_dict())
            elif message.name == 'RC_CHANNELS':
                self.rc_channels.append(message.to_dict())


    def change_alt(self, alt, kp=1, ki=1, kd=1, l_limit=40, u_limit=60, epsilon=0.05, debug=False):
        """Commands the drone to fly to the desired altitude without GPS.

            Parameters:
                alt (float): target altitude
                kp (int): proportional gain
                ki (int): integral gain
                kd (int): derivative gain
                l_limit (float): minimum output value of the PID
                u_limit (float): maximum output value of the PID
                epsilon (float): allowed error for the altitude
                debug (bool): True to print and plot output for debugging
        """
        timer = 0

        # variables for plotting
        height = []
        setpoint = []
        time_l = []
        p_l = []
        i_l = []
        d_l = []
        thr_l = []
        start_time = time.monotonic()

        pid = PID(kp, ki, kd, setpoint=alt, output_limits=(l_limit, u_limit))
        self.mode('ALT_HOLD', 10)

        while True:
            # calculate and request a new throttle setting
            curr_alt = self.get_altitude(2) #/1000
            throttle = pid(curr_alt)
            self.set_throttle(throttle, priority=5)

            # used for debugging
            if debug:
                time_l.append(time.monotonic() - start_time)
                p, i, d = pid.components
                height.append(curr_alt)
                setpoint.append(alt)
                p_l.append(p)
                i_l.append(i)
                d_l.append(d)
                thr_l.append(throttle)

            # the drone holds its altitude after staying around the target altitude for a certain amount of time
            if alt - epsilon < curr_alt < alt + epsilon:
                timer += 1
                if timer == 3 * (0.1 / 0.01):
                    if self.flightmode == 'COPTER_MODE_STABILIZE':
                        self.alt_hold_throttle = self.hover_throttle
                    elif self.flightmode =='COPTER_MODE_ALT_HOLD':
                        self.alt_hold_throttle = 50
                    self.hold_alt = True
                    break
            else:
                timer = 0

        # used for debugging
        if debug:
            # Plot the altitude in function of time
            plt.figure(1)
            plt.plot(time_l, height)
            plt.plot(time_l, setpoint, "r--")
            plt.xlabel("time [s]")
            plt.ylabel("height [m]")
            plt.title("takeoff behaviour")

            # Plot the output from the PID in function of time
            plt.figure(2)
            plt.plot(time_l, p_l, color="red", label="Proportional")
            plt.plot(time_l, i_l, color="green", label="Integral")
            plt.plot(time_l, d_l, color="blue", label="Differential")
            plt.plot(time_l, thr_l, color="black", label="Throttle")
            plt.legend()
            plt.xlabel("time [s]")
            plt.ylabel("Throttle [%]")
            plt.title("PID behaviour")
            plt.show()

    def set_throttle(self, throttle, priority):
        """Requests the desired throttle setting.

            Parameters:
                throttle (float): throttle setting in %
        """
        self.controller.throttle = throttle
        self.controller(priority=priority)

    def multiplexer(self):
        """Selects which command to send to the drone based on priority."""
        while True:
            with self.lock:
                no_commands = len(self.commands)
                # starts looping as soon as a command is requested
                if no_commands > 0:
                    # determine the command with the highest priority
                    next_prio = 0
                    next_command = None
                    commands = self.commands
                    for c in commands:
                        if commands[c][0] > next_prio:
                            next_prio = commands[c][0]
                            next_command = c
                    if next_command is not None:
                        p = commands[next_command]

                        if next_prio != 0:
                            # in case of a command requesting output from the drone, the output is saved to an attribute
                            if next_command == "recv_msg":
                                msg = self.functions[next_command](p[1], p[2])
                                if msg is not None:
                                    self.output[p[1]] = msg
                            else:
                                # determines the number of parameters of the command
                                no_param = 0
                                while p[no_param] is not None:
                                    no_param += 1
                                no_param -= 1 # subtract 1 since the priority is not a relevant parameter

                                # executes the command based on the number of parameters
                                if no_param == 0:
                                    self.functions[next_command]()
                                elif no_param == 1:
                                    self.functions[next_command](p[1])
                                elif no_param == 2:
                                    self.functions[next_command](p[1], p[2])
                                elif no_param == 3:
                                    self.functions[next_command](p[1], p[2], p[3])
                                elif no_param == 4:
                                    self.functions[next_command](p[1], p[2], p[3], p[4])
                                elif no_param == 5:
                                    self.functions[next_command](p[1], p[2], p[3], p[4], p[5])
                                elif no_param == 6:
                                    self.functions[next_command](p[1], p[2], p[3], p[4], p[5], p[6])

                        self.commands[next_command][0] = 0 # sets the priority of the executed command to 0
                        # increments the priority of every requested command, to prevent commands with a high priority
                        # to only be executed
                        for command in self.commands:
                            if self.commands[command][0] != 0:
                                self.commands[command][0] += 1

    def multiplexer_add(self, comm, param1=None, param2=None, param3=None, param4=None, param5=None, param6=None,
                        priority=0):
        """Adds the requested command to the commands dictionary."""
        with self.lock:
            # the order of the parameters must be identical to the order of the arguments of the requested function
            self.commands[comm] = [priority, param1, param2, param3, param4, param5, param6]

    @staticmethod
    def execute(func, args, daemon=False):
        """"Executes the function in a new thread.

            Parameters:
                func (function): the function to be run in the new thread
                args (tuple): contains the required arguments
                daemon (bool): determines whether the program waits for the completion of the thread before finishing
                or not, false or true respectively
        """
        thread = threading.Thread(target=func, args=args, daemon=daemon)
        thread.start()

    def get_parameter(self, parameter, priority):
        """Returns the requested parameter."""
        self.multiplexer_add('param_request_read', parameter, priority=priority)
        return self.get_message('PARAM_VALUE', True, priority=priority)

    def param_request_read(self, parameter):
        """Sends a request to the drone to send the information of a parameter."""
        self.vehicle.mav.param_request_read_send(self.vehicle.target_system, self.vehicle.target_component, parameter,
                                                 -1)

    def land(self, throttle_decrement=5):
        """"Causes the drone to land without GPS."""
        self.hold_alt = True
        if self.flightmode == 'COPTER_MODE_STABILIZE':
            self.alt_hold_throttle = self.hover_throttle - throttle_decrement
        elif self.flightmode == 'COPTER_MODE_ALT_HOLD':
            self.alt_hold_throttle = 50 - throttle_decrement
        az = []
        start_time = time.monotonic()
        while True:
            # reads and stores the vertical acceleration
            imu = self.scaled_imu2[-1]
            az.append(imu['zacc'])

            # when a spike in acceleration is measured, the drone has touched the ground and the drone can disarm
            if az[-1] < -1010:
                break
        self.hold_alt = False

    def recv_msg(self, msg_type, blocking):
        """Receive a message from the drone."""
        return self.vehicle.recv_match(type=msg_type, blocking=blocking)

    def alt_hold(self):
        """Repeatedly sends commands to hold altitude."""
        while True:
            if self.hold_alt:
                self.set_throttle(self.alt_hold_throttle, priority=1)
            else:
                time.sleep(0.5)

    def sonar_altitude(self, priority):
        """Returns the altitude measured by the sonar, assuming the sonar is pointing downwards."""
        distance_sensor = self.distance_sensor[-1]
        return distance_sensor['current_distance']

    def gps_altitude(self, priority):
        """Returns the altitude based on the GPS."""
        local_position = self.local_position_ned[-1]
        return local_position['z']

    def get_altitude(self, priority):
        """Returns the altitude.

        The type of sensor used to measure altitude is defined when creating a Drone object.
        """
        if self.alt_type.lower() == "barometer":
            return self.baro_altitude(priority)
        elif self.alt_type.lower() == "sonar":
            return self.sonar_altitude(priority)
        elif self.alt_type.lower() == "gps":
            return self.gps_altitude(priority)
        else:
            print("Invalid method of measuring altitude")

    def get_target(self, scantime, color, debug=False, source=''):
        """Stores the current distance and the current angle relative to the drone

            Parameters:
                scantime (float): the time to scan for objects after which it returns the output
                color (str): the color to be detected
                debug (bool): True will show white bounding around detected object every 2 seconds
                source (str): the source image
        """
        # when enabled, will continuously detect the object
        while True:
            if self.scan:
                distance, angle = visual_algorithm(scantime, safezonesize, color, debug, source)
                self.target.append((distance, angle))
            else:
                time.sleep(0.5)

    def yaw_to_target(self, angle):
        """Orders the drone to rotate to the target.

        The drone will rotate so it can either fly towards the target by pitching or so it can fly towards the target
        by rolling, whichever requires the least amount of rotation.
        """
        # checks in which quadrant the target is located and rotates acoordingly
        if -45 <= angle <= 45:
            self.yaw(angle)
            self._x_target = True
        elif 45 < angle < 135:
            self.yaw(90 - angle)
            self._x_target = False
        elif 135 <= angle <= -135:
            self.yaw(180 - angle)
            self._x_target = True
        elif -135 < angle < -45:
            self.yaw(-90 - angle)
            self._x_target = False

    def go_to_target(self, kp, f = 0.05, p_w = 1*10**-6, p_h = 1*10**-6):
        """Commands the drone to fly towards the detected target.

            Based on Image Based Visual Servoing, the required angle to fly towards the target is calculated to realise
            speed control.

            Parameters:
                kp (int): proportional term
                f (float): focal distance of the camera
                p_w (float): pixel width
                p_h (float): pixel height
        """
        z = self.get_altitude(2)
        self.yaw_to_target(self.target[-1][1])
        # continously calculates the required angle needed to fly towards the target
        while True:
            u = 0
            v = 0
            pe = []
            # checks whether the target is aligned along the roll or pitch axis, respectively
            if self._x_target:
                u = self.target[-1][0]
                pe = [[-u], [0]]
            elif not self._x_target:
                v = self.target[-1][0]
                pe = [[0],[-v]]

            # jacobian matrix
            jp = [[-f/(p_w * z), 0, u/z, (p_w * u * v)/f, -(f**2 + (p_w*u)**2) / (p_w*f), v],
                  [0, -f/(p_h * z), 0, 0, -(p_h * u * v) / f, -u]]

            i_jp = numpy.linalg.pinv(jp) # inverse of the jacobian matrix
            velocity = kp * numpy.dot(i_jp, pe)
            angle = 4000 * velocity
            if self._x_target:
                self.pitch(angle[0][0])
            elif not self._x_target:
                self.roll(angle[1][0])

    def interrupt(self):
        """Starts a listener to act as a killswitch for the drone and the script."""
        def on_press(key):
            if key == keyboard.KeyCode.from_char('k'):
                self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                                   mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, 0, 1, 0, 0, 0, 0, 0, 0)
                os._exit(1)
        listener = keyboard.Listener(on_press=on_press)
        listener.start()

    def pitch(self, angle):
        """Commands the drone to pitch to the desired angle."""
        self.controller.pitch = angle

    def roll(self, angle):
        """Commands the drone to roll to the desired angle."""
        self.controller.roll = angle

    def yaw(self, angle):
        """Commands the drone to yaw to the desired angle relative to its current heading."""
        self.controller.yaw = angle
        # waits until the drone has reached its correct heading
        while True:
            time.sleep(0.5)
            if not self.controller.rotating:
                break


class Controller():
    """The class that is used to control the movement of the drone."""

    def __init__(self, drone):
        self.drone = drone
        self.throttle = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.rotating = False # attribute used to indicate whether the drone is rotating or not
        self.error = 0
        self._timer = 0

    def __call__(self, priority=1):
        """Calls the controller to send a request for controlling the drone using the attitude and throttle settings
        defined in the arguments of the object."""
        # only calculates the yaw rate when the desired yaw has changed or when the drone is still rotating
        if self.yaw != 0 or self.rotating:
            yaw = self.yaw_rate(self.yaw)
        else:
            yaw = 0
        self.drone.multiplexer_add('manual_control', self.pitch, self.roll, self.throttle, int(yaw), priority=priority)

    def load_parameters(self):
        """Loads all the necessary parameters."""
        self.angle_max = (self.drone.get_parameter(b'ANGLE_MAX', 1)['param_value'] / 100)
        self.yaw_p = 15

    def manual_control(self, pitch=0, roll=0, throttle=0, yaw=0, buttons=0):
        """Sends the desired pitch, roll, yaw rate and throttle to the drone."""
        pitch = int((pitch / self.angle_max) * 1000)
        roll = int((roll / self.angle_max) * 1000)
        thr = int((throttle / 100) * 1000)

        self.drone.vehicle.mav.manual_control_send(self.drone.vehicle.target_system, pitch, roll, thr, yaw, buttons)

    def yaw_rate(self, yaw):
        """Is used to determine the needed yaw rate to rotate to the desired angle."""
        hdg_rad = self.drone.attitude[-1]['yaw']
        hdg_deg = hdg_rad * (180 / math.pi)
        # sets the yaw attribute to zero when calculating the yaw rate for the first time
        if yaw != 0:
            self._des_hdg_deg = hdg_deg + yaw
            self.yaw = 0
            self.rotating = True

        self.error = self._des_hdg_deg - hdg_deg
        # if the drone stays within a certain margin for a certain period of time, the rotation is completed
        if -5 < self.error < 5:
            self._timer += 1
            if self._timer == 50:
                self.rotating = False
                self._timer = 0
        else:
            self._timer = 0
        return self.yaw_p * self.error

class PID(object):
    """Simple PID controller."""
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0, sample_time=0.01, output_limits=(None, None)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        self._min_output, self._max_output = output_limits

    def __call__(self, input_):
        """
        Call the PID controller with *input_* and calculate and return a control output if sample_time seconds has
        passed since the last update. If no new output is calculated, return the previous output instead (or None if
        no value has been calculated yet).
        :param dt: If set, uses this value for timestep instead of real time. This can be used in simulations when
                   simulation time is different from real time.
        """
        now = time.monotonic()
        try:
            dt = now - self._last_time
        except AttributeError:
            dt = 1e-16

        try:
            if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
                # only update every sample_time seconds
                return self._last_output
        except AttributeError:
            pass

        # compute error terms
        error = self.setpoint - input_
        try:
            d_input = input_ - (self._last_input if self._last_input is not None else input_)
        except AttributeError:
            d_input = 0

        # compute the proportional term
        self._proportional = self.Kp * error

        # compute integral and derivative terms
        try:
            self._integral += self.Ki * error * dt
        except AttributeError:
            self._integral = self.Ki * error * dt
        self._integral = self.clamp(self._integral, self.output_limits)  # avoid integral windup

        self._derivative = -self.Kd * d_input / dt

        # compute final output
        output = self._proportional + self._integral + self._derivative
        output = self.clamp(output, self.output_limits)

        # keep track of state
        self._last_output = output
        self._last_input = input_
        self._last_time = now

        return output

    def clamp(self, value, limits):
        lower, upper = limits
        if value is None:
            return None
        elif upper is not None and value > upper:
            return upper
        elif lower is not None and value < lower:
            return lower
        return value

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful for visualizing
        what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper). See also the *output_limts* parameter in
        :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output
