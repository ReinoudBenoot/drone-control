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

        self.hold_alt = False
        self.alt_hold_throttle = 50

        self.pos = PosEstimator(self)
        self.hold_pos = False

        self.target = []
        self.scan = False

        self.start_pressure = 0
        #self._baro_alt = [(0, 0)]
        #self.baro = Barometer(self)

        self.alt_type = alt_type
        self.flightmode = self.get_mode(1)

        self._plot = {}

        self.disable_gps()

        self.execute(func=self.alt_hold, args=(), daemon=True)
        self.execute(func=self.get_target, args=(scantime, color), daemon=True)
        self.execute(func=self.dead_reckoning, args=(), daemon=True)
        self.hover_throttle = None

    def arm(self):
        self.controller.load_parameters()
        self.hover_throttle = self.get_parameter(b'MOT_THST_HOVER', 1)['param_value']
        self.mode('STABILIZE', 10)
        self.vehicle.arducopter_arm()
        print("Arming drone...")
        while True:
            if self.armed:
                break
            time.sleep(0.5)
        print("Drone is armed")

    @property
    def armed(self):
        heartbeat = self.get_message('HEARTBEAT', True, priority=1)
        if heartbeat['base_mode'] == 209:
            return True
        elif heartbeat['base_mode'] == 81:
            return False
        else:
            print("Unknown state")

    def disarm(self):
        time.sleep(1)
        while True:
            self.vehicle.arducopter_disarm()
            if not self.armed:
                break
            time.sleep(0.5)

    def arm_and_takeoff(self, alt):
        """Arms and requests the drone to take off."""
        self.arm()
        self.indoor_takeoff(alt)

    def disable_gps(self):
        """Changes certain parameters to disable the usage of the gps."""
        self.vehicle.param_set_send('ATT_ACC_COMP', 0, mavutil.mavlink.MAVLINK_TYPE_INT32_T)
        self.vehicle.param_set_send('ATT_MAG_DECL_A', 0, mavutil.mavlink.MAVLINK_TYPE_INT32_T)
        self.vehicle.param_set_send('EKF2_AID_MASK', 5.60519385729926828369491833316E-45, mavutil.mavlink.MAVLINK_TYPE_INT32_T)

    def indoor_takeoff(self, alt):
        """"Commands the drone to take off indoors."""
        self.change_alt(alt, kp=15, ki=4, kd=0, l_limit=30, u_limit=70)
        print("Altitude reached")

    def mode(self, flightmode, priority):
        """Changes the flightmode of the drone."""
        print("Setting flightmode...")
        self.multiplexer_add('set_mode', param1=flightmode, priority=priority)
        print(self.ack_command(1))
        while self.flightmode != ('COPTER_MODE_' + flightmode):
            time.sleep(0.2)
            self.flightmode = self.get_mode(1)

    def set_mode(self, flightmode):
        """Sends the command to change the flightmode."""
        self.vehicle.set_mode(flightmode)

    def get_mode(self, priority):
        heartbeat = self.get_message('HEARTBEAT', True, priority=priority)
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
        """Returns the altitude of the drone based the barometer."""
        if self.start_pressure == 0:
            self.start_pressure = self.get_pressure(priority)

        pressure = self.get_pressure(priority)
        now = time.time()
        scaling = pressure / self.start_pressure
        temp = 293.15
        altitude = 153.8462 * temp * (1.0 - math.exp(0.190259 * math.log(scaling)))
        """self._baro_alt.append((altitude, now))

        dx = self._baro_alt[-1][0] - self._baro_alt[-2][0]
        dt = self._baro_alt[-1][1] - self._baro_alt[-2][1]
        try:
            if dx / dt > 2:
                self._baro_alt[-1][0] = self._baro_alt[-2][0]
        except ZeroDivisionError:
            print(self._baro_alt[-1][1], self._baro_alt[-2][1])

        if len(self._baro_alt) > 2:
            self._baro_alt.pop(0)
        total = 0
        for i in self._baro_alt:
            total += i[0]
        f_altitude = total / len(self._baro_alt)"""

        return altitude

    def get_pressure(self, priority):
        scaled_pressure = self.get_message('SCALED_PRESSURE', True, priority=priority)
        return scaled_pressure['press_abs']

    def get_message(self, msg, blocking, priority=0):
        """Sends commands to request messages from the drone."""
        self.multiplexer_add('recv_msg', msg, blocking, priority=priority)
        while True:
            if msg in self.output:
                message = self.output[msg]
                message = message.to_dict()
                return message

    def change_alt(self, alt, kp=1, ki=1, kd=1, l_limit=40, u_limit=60, epsilon=0.05):
        """Commands the drone to fly to the desired altitude without GPS.

            Parameters:
                alt (float): target altitude
                kp (int): proportional gain
                ki (int): integral gain
                kd (int): derivative gain
                l_limit (float): minimum output value of the PID
                u_limit (float): maximum output value of the PID
                epsilon (float): allowed error for the altitude
        """
        timer = 0

        # lists for plotting
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
            time_l.append(time.monotonic() - start_time)

            # calculate and request a new throttle setting
            curr_alt = self.get_altitude(2) #/1000
            throttle = pid(curr_alt)
            self.set_throttle(throttle, priority=5)

            p, i, d = pid.components
            #print("P: {0}%, I: {1}%, D: {2}%".format(round(p, 1), round(i, 1), round(d, 1)))
            #print("Throttle: {0}%".format(round(throttle,1)))
            #print("Altitude:", curr_alt)

            #height.append(curr_alt)
            #setpoint.append(alt)
            #p_l.append(p)
            #i_l.append(i)
            #d_l.append(d)
            #thr_l.append(throttle)

            debug = {'p': p, 'i': i, 'd': d, 'throttle': throttle, 'alt': alt, 'curr_alt': curr_alt, 'height': height,
                     'start_time': start_time, 'looping': True}
            self.debug('change_alt', debug)

            # the drone holds its altitude after staying around the target altitude for a certain amount of time
            if alt - epsilon < curr_alt < alt + epsilon:
                timer += 1
                if timer == 3 * (0.1 / 0.01):
                    if self.flightmode == 'COPTER_MODE_STABILIZE':
                        self.alt_hold_throttle = self.hover_throttle
                    elif self.flightmode =='COPTER_MODE_ALT_HOLD':
                        self.alt_hold_throttle = 50
                    self.hold_alt = True
                    debug['looping'] = False
                    break
            else:
                timer = 0

        self.debug('change_alt', debug)

        # Plot the altitude in function of time
        """plt.figure(1)
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
        plt.show()"""

    def set_throttle(self, throttle, priority):
        """Requests the desired throttle setting.

            Parameters:
                throttle (float): throttle setting in %
        """
        self.controller.throttle = throttle
        self.controller(priority=priority)

    def manual_control(self, pitch=32767, roll=32767, throttle=32767, yaw=32767, buttons=0):
        thr = int((throttle / 100) * 1000)
        self.vehicle.mav.manual_control_send(self.vehicle.target_system, pitch, roll, thr, yaw, buttons)

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
                args (dict): contains the required arguments
                daemon (bool): determines whether the program waits for the completion of the thread before finishing
                or not, false or true respectively
        """
        thread = threading.Thread(target=func, args=args, daemon=daemon)
        thread.start()

    def get_parameter(self, parameter, priority):
        self.multiplexer_add('param_request_read', parameter, priority=priority)
        return self.get_message('PARAM_VALUE', True, priority=priority)

    def param_request_read(self, parameter):
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
            imu = self.get_message("SCALED_IMU2", True, priority=5)
            print(imu['zacc'])
            az.append(imu['zacc'])

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

    def pos_hold(self):
        pass

    def throttle_rc(self, channel, throttle):
        if channel < 1 or channel > 18:
            print("Channel does not exist.")
            return
        pwm = 10 * throttle + 1000
        rc_channel_values = [0 for _ in range(18)]
        rc_channel_values[channel - 1] = pwm
        self.vehicle.mav.rc_channels_override_send(self.vehicle.target_system, self.vehicle.target_component,
                                                   *rc_channel_values)

    def sonar_altitude(self, priority):
        distance_sensor = self.get_message('DISTANCE_SENSOR', True, priority=priority)
        return distance_sensor['current_distance']

    def gps_altitude(self, priority):
        local_position = self.get_message('LOCAL_POSITION_NED', True, priority=priority)
        return local_position['z']

    def ekf_altitude(self, priority):
        global_position_int = self.get_message('AHRS2', True, priority=priority)
        return global_position_int['altitude']

    def get_altitude(self, priority):
        if self.alt_type.lower() == "barometer":
            return self.baro_altitude(priority)
        elif self.alt_type.lower() == "sonar":
            return self.sonar_altitude(priority)
        elif self.alt_type.lower() == "gps":
            return self.gps_altitude(priority)
        else:
            print("Invalid method of measuring altitude")

    def get_target(self, scantime, color, debug=False, source=''):
        while True:
            if self.scan:
                distance, angle = visual_algorithm(scantime, safezonesize, color, debug, source)
                self.target.append((distance, angle))
            else:
                time.sleep(0.5)

    def yaw_to_target(self, angle):
        if -45 < angle < 45:
            self.yaw(angle)
            self._x_target = True
        elif 45 < angle < 135:
            self.yaw(90 - angle)
            self._x_target = False
        elif 135 < angle < -135:
            self.yaw(180 - angle)
            self._x_target = True
        elif -135 < angle < -45:
            self.yaw(-90 - angle)
            self._x_target = False

    def go_to_target(self, kp):
        p_w = 0
        p_h = 0
        f = 0
        z = self.get_altitude(2)
        while True:
            u = 0
            v = 0
            if self._x_target:
                u = self.target[-1][0]
            elif not self._x_target:
                v = self.target[-1][0]

            jp = [[-f/(p_w * z), 0, u/z, (p_w * u * v)/f, -(f**2 + (p_w*u)**2) / (p_w*f), v],
                  [0, -f/(p_h * z), 0, 0, -(p_h * u * v) / f, -u]]
            i_jp = numpy.linalg.pinv(jp)
            pe = -u
            velocity = kp * i_jp * pe
            angle = 6 * velocity
            if self._x_target:
                self.pitch(angle)
            elif not self._x_target:
                self.roll(angle)

    def simulated_target(self, distance, angle):
        self.target.append((distance, angle))
        start_pos = self.pos_gps()
        start_hdg = self.drone.get_message('ATTITUDE', blocking=True, priority=1)['yaw'] * (180 / math.pi)
        while True:
            new_hdg = self.drone.get_message('ATTITUDE', blocking=True, priority=1)['yaw'] * (180 / math.pi)
            d_hdg = start_hdg - new_hdg
            new_angle = angle - d_hdg

            new_pos = self.pos_gps()
            d_pos = [start_pos[0] - new_pos[0], start_pos[1] - new_pos[1]]
            d_pos_m = [d_pos[0] * 110574, d_pos[1] * 111320 * math.cos(new_pos[0])]

            self.target.append(, new_angle)

    def pos_gps(self):
        raw_gps = self.get_message('RAW_GPS', True, priority=1)
        return [raw_gps[x], raw_gps[y]]

    def dead_reckoning(self):
        while True:
            if self.hold_pos:
                x, y = self.pos.pos_calc()
                print("x:", x, "y:", y)
                time.sleep(0.5)
            else:
                time.sleep(0.5)

    def kill(self):
        self.vehicle.mav.command_long_send(self.vehicle.target_system, self.vehicle.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, 0, 1, 0, 0, 0, 0, 0, 0)
        os._exit(1)

    def interrupt(self):
        def on_press(key):
            if key == keyboard.KeyCode.from_char('k'):
                self.kill()
        listener = keyboard.Listener(on_press=on_press)
        listener.start()

    def debug(self, function, vars):
        if function == 'change_alt':
            if not self._plot:
                self._plot = [[], [], [], [], [], [], []]
                #height = []
                #setpoint = []
                #p_l = []
                #i_l = []
                #d_l = []
                #thr_l = []
                #time_l = []

            if vars['looping']:
                print("P: {0}%, I: {1}%, D: {2}%".format(round(vars['p'], 1), round(vars['i'], 1), round(vars['d'],
                                                                                                           1))) #debug = {'p': p, 'i': i, 'd': d, 'throttle': throttle, 'alt': alt, 'curr_alt': curr_alt, 'height': height, 'start_time': start_time, 'looping': True}
                print("Throttle: {0}%".format(round(vars['throttle'], 1)))
                print("Altitude:", vars['curr_alt'])

                self._plot[0].append(vars['curr_alt'])
                self._plot[1].append(vars['alt'])
                self._plot[2].append(vars['p'])
                self._plot[3].append(vars['i'])
                self._plot[4].append(vars['d'])
                self._plot[5].append(vars['throttle'])
                self._plot[6].append(time.monotonic() - vars['start_time'])

            elif not vars['looping']:
                # Plot the altitude in function of time
                plt.figure(1)
                plt.plot(self._plot[6], self._plot[0])
                plt.plot(self._plot[6], self._plot[1], "r--")
                plt.xlabel("time [s]")
                plt.ylabel("height [m]")
                plt.title("takeoff behaviour")

                # Plot the output from the PID in function of time
                plt.figure(2)
                plt.plot(self._plot[6], self._plot[2], color="red", label="Proportional")
                plt.plot(self._plot[6], self._plot[3], color="green", label="Integral")
                plt.plot(self._plot[6], self._plot[4], color="blue", label="Differential")
                plt.plot(self._plot[6], self._plot[5], color="black", label="Throttle")
                plt.legend()
                plt.xlabel("time [s]")
                plt.ylabel("Throttle [%]")
                plt.title("PID behaviour")
                plt.show()

    def pitch(self, angle):
        self.controller.pitch = angle

    def roll(self, angle):
        self.controller.roll = angle

    def yaw(self, angle):
        self.controller.yaw = angle
        while 1 < self.controller.error or self.controller.error < -1:
            time.sleep(0.5)

class Controller():

    def __init__(self, drone, rate=2): #minimum 1 Hz
        self.drone = drone
        self.throttle = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self._rotating = False
        self.error = 0
        self._timer = 0

    def __call__(self, priority=1):
        self.run(priority)

    def load_parameters(self):
        self.angle_max = (self.drone.get_parameter(b'ANGLE_MAX', 1)['param_value'] / 100)
        self.yaw_p = 4.5 #self.drone.get_parameter(b'ATC_ANG_YAW_P', 1)['param_value']

    def run(self, priority):
        if self.yaw != 0 or self._rotating:
            yaw = self.yaw_rate(self.yaw)
        else:
            yaw = 0
        self.drone.multiplexer_add('manual_control', self.pitch, self.roll, self.throttle, int(yaw), priority=priority)

    def manual_control(self, pitch=0, roll=0, throttle=0, yaw=0, buttons=0):
        pitch = int((pitch / self.angle_max) * 1000)
        roll = int((roll / self.angle_max) * 1000)
        thr = int((throttle / 100) * 1000)

        self.drone.vehicle.mav.manual_control_send(self.drone.vehicle.target_system, pitch, roll, thr, yaw, buttons)

    def yaw_rate(self, yaw):
            hdg_rad = self.drone.get_message('ATTITUDE', blocking=True, priority=1)['yaw']
            hdg_deg = hdg_rad * (180 / math.pi)
            if yaw != 0:
                self._des_hdg_deg = hdg_deg + yaw
                self.yaw = 0
                self._rotating = True
            self.error = self._des_hdg_deg - hdg_deg
            if -1 < self.error < 1:
                self._timer += 1
                if self._timer == 5:
                    self._rotating = False
                    self._timer = 0
            else:
                self._timer = 0
            return self.yaw_p * self.error

class PosEstimator():

    def __init__(self, drone):
        self.drone = drone
        self._time = []
        self._x_a = []
        self._y_a = []
        self._v_x = []
        self._v_y = []
        self._spd_x = [0]
        self._spd_y = [0]
        self._x = []
        self._y = []

    def imu(self):
        attitude = self.drone.get_message('ATTITUDE', blocking=True, priority=1)
        pitch, roll = round(attitude['pitch'], 2), round(attitude['roll'], 2)
        if -0.01 < pitch < 0.01: pitch = 0
        if -0.01 < roll < 0.01: roll = 0
        scaled_imu = self.drone.get_message('SCALED_IMU2', blocking=True, priority=1)
        xacc, yacc, zacc = scaled_imu['xacc'] * (9.81/1000), scaled_imu['yacc'] * (9.81/1000),\
                           scaled_imu['zacc'] * (9.81/1000)
        return pitch, roll, xacc, yacc, zacc

    def horizontal_acc(self, pitch, roll, xacc, yacc, zacc):
        x_a = xacc * math.cos(pitch) - zacc * math.sin(pitch)
        y_a = yacc * math.cos(roll) - zacc * math.sin(roll)
        return x_a, y_a

    def pos_calc(self):
        pitch, roll, xacc, yacc, zacc = self.imu()
        x_a, y_a = self.horizontal_acc(pitch, roll, xacc, yacc, zacc)
        self._x_a.append(x_a)
        self._y_a.append(y_a)
        self._time.append(time.monotonic())

        if len(self._time) >= 2:
            dt = self._time[-1] - self._time[-2]
            dx_a = self._x_a[-1] - self._x_a[-2]
            v_x = (self._x_a[-2] + dx_a/2) * dt
            self._v_x.append(v_x)
            vtot_x = sum(self._v_x)

            self._spd_x.append(vtot_x)
            dspd_x = self._spd_x[-1] - self._spd_x[-2]
            x = (self._spd_x[-2] + dspd_x/2) * dt
            self._x.append(x)
            pos_x = sum(self._x)

            dy_a = self._y_a[-1] - self._y_a[-2]
            v_y = (self._y_a[-2] + dy_a / 2) * dt
            self._v_y.append(v_y)
            vtot_y = sum(self._v_y)

            self._spd_y.append(vtot_y)
            dspd_y = self._spd_y[-1] - self._spd_y[-2]
            y = (self._spd_y[-2] + dspd_y / 2) * dt
            self._y.append(y)
            pos_y = sum(self._y)

            return pos_x, pos_x
        else:
            return 0, 0

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

"""class Barometer():

    def __init__(self, drone):
        self.start_pressure = 0
        self.alt = [[0, 0]]
        self.drone = drone

    def __call__(self):
        if self.start_pressure == 0:
            self.start_pressure = self.drone.get_pressure(1)

        pressure = self.drone.get_pressure(priority)
        now = time.monotonic()
        scaling = pressure / self.start_pressure
        temp = 293.15
        altitude = 153.8462 * temp * (1.0 - math.exp(0.190259 * math.log(scaling)))

        self.alt.append([altitude, now])

        dx = self.alt[-1][0] - self._baro_alt[-2][0] alleen wanneer nieuwe waarden zijn opgenomen
        dt = self.alt[-1][1] - self._baro_alt[-2][1]
        if dx / dt > 2:
            self.alt[-1][0] = self._baro_alt[-2][0]

        if len(self.alt) > 2:
            self.alt.pop(0)
        total = 0
        for i in self._baro_alt:
            total += i[0]
        f_altitude = total / len(self._baro_alt)

        return f_altitude

    def filter(self):
        dx = self.alt[-1][0] - self._baro_alt[-2][0]
        dt = self.alt[-1][1] - self._baro_alt[-2][1]
        if dx / dt > 2:
            self.alt[-1][0] = self._baro_alt[-2][0]"""