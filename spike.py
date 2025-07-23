"""
Motor Control Module for LEGO Robotics
Author: Denise Kilburg on behalf of the TPS Computer Science Department
Email: dmkilburg@gmail.com

This module provides object-oriented abstractions for controlling single and paired motors
using the LEGO robotics hardware API. It includes high-level utilities for basic motion,
tank-style movement, yaw-based PID correction, arc turns, and line following.

Classes
-------
Motor
    A wrapper around a single motor with configurable speed, acceleration, and rotation parameters.
    Provides methods for timed movement, absolute positioning, and attribute resetting.

MotorPair
    A controller for coordinating two motors as a differential drive unit (e.g., left and right wheels).
    Supports forward/reverse motion, PID-based straight driving and turning, arc-based turning,
    and sensor-guided line following.

Usage
-----
These classes are designed to simplify robotics control logic and make higher-level behaviors
(such as precise turning or stable line following) easy to implement with readable syntax.

Example
-------
>>> left_motor = Motor("A")
>>> right_motor = Motor("B")
>>> robot = MotorPair(1, ("A", "B"))

>>> robot.forward(time=2000)                # Move forward for 2 seconds
>>> robot.right_turn(theta=90, PID=True)    # Perform a precise 90-degree right turn using PID
>>> robot.follow_line_distance(50, port="C")  # Follow a line for 50 cm using a color sensor
>>> robot.arc_turn(r=20, theta=180)         # Execute an arc turn with 20 cm radius over 180 degrees
>>> robot.stop()                            # Stop the robot
"""

from hub import light_matrix, motion_sensor
import motor
import runloop
import motor_pair
import color_sensor
import math
import time
import asyncio


PORT_DICT = {
    'A': 0,
    'B': 1,
    'C': 2,
    'D': 3,
    'E': 4,
    'F': 5
}

class Timer:
    """
    A simple timer class for managing timed events, with optional callback and repeat behavior.

    This class uses `time.ticks_ms()` to track elapsed time (compatible with MicroPython), 
    and can optionally call a function when the timer ends and restart automatically.

    Parameters
    ----------
    duration : int
        The duration of the timer in milliseconds.
    func : callable, optional
        A function to call when the timer ends. Defaults to None.
    repeat : bool, optional
        If True, the timer will automatically restart after deactivation. Defaults to None.
    autostart : bool, optional
        If True, the timer will start immediately upon creation. Defaults to False.

    Attributes
    ----------
    duration : int
        Timer duration in milliseconds.
    start_time : int
        The time (in ms) when the timer was started.
    active : bool
        Whether the timer is currently active.
    func : callable or None
        Function to be executed when the timer ends.
    repeat : bool or None
        Whether the timer restarts automatically after ending.

    Methods
    -------
    activate()
        Starts or restarts the timer.
    
    deactivate()
        Stops the timer. If `repeat` is True, the timer is reactivated.
    
    __bool__()
        Returns True if the timer is active, allowing usage in boolean expressions.

    Examples
    --------
    >>> def callback():
    ...     print("Timer finished!")

    >>> t = Timer(1000, func=callback, repeat=True, autostart=True)
    >>> if t:
    ...     print("Timer is active.")
    """
    def __init__(self, duration, func = None, repeat = None, autostart = False ):
        self.duration = duration
        self.start_time = 0
        self.active = False
        self.func = func
        self.repeat = repeat

        if autostart:
            self.activate()

    def __bool__(self):
        return self.active
    
    def activate(self):
        self.active = True
        self.start_time = time.ticks_ms()

    def deactivate(self):
        self.active = False
        self.start_time = 0
        if self.repeat:
            self.activate()

    def update(self):
        if time.ticks_ms() - self.start_time >= self.duration:
            if self.func and self.start_time != 0:
                self.func()
            self.deactivate()

class Motor:
    """
    Async-compatible controller for a single motor with support for time-based,
    degree-based, and position-based movement. Designed for LEGO-style robotics.

    Parameters
    ----------
    port : str
        Name of the port (e.g., "A") to control, mapped from PORT_DICT.

    Attributes
    ----------
    port : Any
        Hardware-specific port value.
    speed : int
        Speed setting for the motor.
    acceleration : int
        Acceleration setting for the motor.
    deceleration : int
        Deceleration setting for the motor.
    stop_command : constant
        Stop behavior (e.g., motor.BRAKE).

    Methods
    -------
    run(time=0, degrees=0, rotations=0, reverse=False)
        Runs the motor for a time, degrees, or rotations (only one should be non-zero).

    reset(position=0)
        Moves the motor to an absolute encoder position.

    reset_attr()
        Resets motor speed, acceleration, deceleration, and stop behavior to defaults.

    stop()
        Stops the motor using the current stop behavior.
    """

    DEFAULT_SPEED = 1000
    DEFAULT_ACCEL = 1000
    DEFAULT_DECEL = 1000

    def __init__(self, port):
        self.port = PORT_DICT[port]
        self.speed = self.DEFAULT_SPEED
        self.acceleration = self.DEFAULT_ACCEL
        self.deceleration = self.DEFAULT_DECEL
        self.stop_command = motor.BRAKE

        print("Default values: velocity:{}, acceleration: {}, deceleration: {}, rotation: clockwise".format(
            self.speed, self.acceleration, self.deceleration))

    async def run(self, time=0, degrees=0, rotations=0, reverse=False):
        if rotations:
            degrees = int(rotations * 360)
        speed = -abs(self.speed) if reverse else abs(self.speed)

        if time:
            motor.run_for_time(self.port, time, speed,
                stop=self.stop_command,
                acceleration=self.acceleration,
                deceleration=self.deceleration)
            await asyncio.sleep(time / 1000)
        elif degrees:
            motor.run_for_degrees(self.port, degrees, speed,
                stop=self.stop_command,
                acceleration=self.acceleration,
                deceleration=self.deceleration)
            await asyncio.sleep(0.01)
        else:
            motor.run(self.port, speed, acceleration=self.acceleration)
            await asyncio.sleep(0.01)

    async def reset(self, position=0):
        motor.run_to_absolute_position(
            self.port, position, self.speed,
            direction=motor.SHORTEST_PATH,
            stop=self.stop_command,
            acceleration=self.acceleration,
            deceleration=self.deceleration
        )
        await asyncio.sleep(0.01)

    def reset_attr(self):
        self.speed = self.DEFAULT_SPEED
        self.acceleration = self.DEFAULT_ACCEL
        self.deceleration = self.DEFAULT_DECEL
        self.stop_command = motor.BRAKE

    def stop(self):
        motor.stop(self.port, stop=self.stop_command)

class MotorPair:
    """
    motor pair controller for LEGO-style robots with support for
    forward and reverse motion, PID-controlled driving, tank turns, arc turns,
    and line following.

    This class abstracts control over a left/right motor pair using paired ports,
    with additional support for fine-tuned PID yaw correction and motion duration tracking
    using a Timer class. It is intended to work within an `async def main()` loop.

    Parameters
    ----------
    pair : int
        Index of the motor pair (1-based, mapped to 0–2 internally).
    ports : tuple of str
        Tuple of port names corresponding to the left and right motors, e.g., ("A", "B").

    Attributes
    ----------
    port1 : Any
        The mapped hardware port for the first motor.
    port2 : Any
        The mapped hardware port for the second motor.
    index : int
        Internal index used for the motor_pair system.
    speed : int
        Default speed for both motors (in device-specific units).
    left_speed : int
        Speed for the left motor (can differ from right_speed).
    right_speed : int
        Speed for the right motor.
    acceleration : int
        Acceleration value used during ramp-up.
    deceleration : int
        Deceleration value used during ramp-down.
    stop_command : constant
        Stop behavior for motors (e.g., BRAKE or COAST).
    wheel_circumference : float
        Circumference of the wheels (cm), used for distance calculations.
    wheelbase : float
        Distance between the left and right wheels (cm), used for turning and arc motion.
    Kp, Ki, Kd : float
        PID control constants used for yaw correction.

    Methods
    -------
    forward(steering=0, time=0, rotations=0, distance=0, PID=False)
        Moves the robot forward using time, rotations, or distance. Optional PID correction.
    
    reverse(**kwargs)
        Reverses the motion using the same options as forward().
    
    right_turn(theta, PID=False)
        Turns the robot to the right by `theta` degrees (PID or basic mode).
    
    left_turn(theta, PID=False)
        Turns the robot to the left by `theta` degrees (PID or basic mode).
    
    arc_turn(r, theta, reverse=False)
        Performs an arc-based turn using radius `r` and angle `theta`.
    
    follow_line(duration, port, reverse=False)
        Follows a line using reflection from a color sensor for a given time.
    
    follow_line_distance(distance, port, reverse=False)
        Follows a line for a given distance (in cm) using reflection input.
    
    reset_motors(position=0)
        Resets both motors to an absolute encoder position.
    
    reset_attr()
        Resets speed, acceleration, deceleration, and stop_command to defaults.
    
    stop()
        Stops the robot using the configured stop behavior.
    
    wait_until_stopped(timeout=5.0)
        Waits until the motors stop moving, based on position change detection.

    
    """

    DEFAULT_SPEED = 360
    DEFAULT_ACCEL = 1000
    DEFAULT_DECEL = 1000

    def __init__(self, pair, ports):
        self.port1 = PORT_DICT[ports[0]]
        self.port2 = PORT_DICT[ports[1]]
        self.index = pair - 1 if 1 <= pair <= 3 else 0

        self.speed = self.left_speed = self.right_speed = self.DEFAULT_SPEED
        self.acceleration = self.DEFAULT_ACCEL
        self.deceleration = self.DEFAULT_DECEL
        self.stop_command = motor.BRAKE

        self.wheel_circumference = 17.5
        self.wheelbase = 11.2

        ## Odometery
        self.x = 0.0 ## current x-pos in cm
        self.y = 0.0 ## current y-pos in cm
        self.heading = 0 ## current heading in rads
        self.last_left_pos = motor.relative_position(self.port1)
        self.last_right_pos = motor.relative_position(self.port2)

        self.left_reversed = False
        self.right_reversed = True  # or False, depending on your robot


        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.0

        motor_pair.pair(self.index, self.port1, self.port2)

    def _update_odometry(self):
        current_left = motor.relative_position(self.port1)
        current_right = motor.relative_position(self.port2)


        if self.left_reversed:
            current_left *= -1
        if self.right_reversed:
            current_right *= -1


        print("[ODOM] current_left: {}, last_left: {}".format(current_left, self.last_left_pos))
        print("[ODOM] current_right: {}, last_right: {}".format(current_right, self.last_right_pos))

        delta_left_degrees = current_left - self.last_left_pos
        delta_right_degrees = current_right - self.last_right_pos

        # convert degrees to distance (cm)
        distance_left = (delta_left_degrees / 360) * self.wheel_circumference
        distance_right = (delta_right_degrees / 360) * self.wheel_circumference

        print("Δleft_deg: {}, Δright_deg: {}".format(delta_left_degrees, delta_right_degrees))
        print("Δleft_cm: {:.2f}, Δright_cm: {:.2f}".format(distance_left, distance_right))


        delta_distance = (distance_left + distance_right) / 2.0
        delta_heading = (distance_right - distance_left) / self.wheelbase  # radians

        self.heading += delta_heading
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi
    

        self.x += delta_distance * math.cos(self.heading)
        self.y += delta_distance * math.sin(self.heading)

        print("[ODOM] Δdist: {:.2f}, Δθ: {:.2f} → (x: {:.2f}, y: {:.2f}, θ: {:.2f})".format(
        delta_distance, delta_heading, self.x, self.y, self.heading))

        # Only now update the last positions
        self.last_left_pos = current_left
        self.last_right_pos = current_right


    def reset_attr(self):
        self.speed = self.left_speed = self.right_speed = self.DEFAULT_SPEED
        self.acceleration = self.DEFAULT_ACCEL
        self.deceleration = self.DEFAULT_DECEL
        self.stop_command = motor.BRAKE

    def reset_motors(self, position=0):
        for port in (self.port1, self.port2):
            motor.run_to_absolute_position(
                port, position, self.speed,
                direction=motor.SHORTEST_PATH,
                stop=self.stop_command,
                acceleration=self.acceleration,
                deceleration=self.deceleration
            )

    def _angle_diff(self, target, current):
        return (target - current + 1800) % 3600 - 1800

    def forward(self, steering=0, time=0, rotations=0, distance=0, PID=False):
        self._record_odometry_snapshot()
        degrees = int(rotations * 360) if rotations else 0
        if distance:
            degrees = int(distance * (360 / self.wheel_circumference))

        if PID:
            print("using PID based movement")
            if not any([time, rotations, distance]):
                print("PID mode requires a time, distance, or rotation target.")
                return
            self._run_pid_loop(degrees, time)
        elif time:
            print("using time based movement")
            if self.left_speed != self.right_speed:
                self.left_speed = self.right_speed

            motor_pair.move_tank_for_time(
            self.index, self.left_speed, self.right_speed, time,
            stop=self.stop_command,
            acceleration=self.acceleration,
            deceleration=self.deceleration
            )

            timer = Timer(time, autostart=True)
            while timer.active:
                timer.update()
            self._update_odometry()
        elif degrees:
            print("using degree/distnace based movement")
            motor_pair.move_tank_for_degrees(
                self.index, degrees, self.left_speed, self.right_speed,
                stop=self.stop_command,
                acceleration=self.acceleration,
                deceleration=self.deceleration
            )
            self._wait_until_stopped()
            self._update_odometry()
            
        else:
            print("using indefinite movement")
            motor_pair.move(self.index, steering, velocity=self.speed)
            
            

    def _run_pid_loop(self, degrees, time):
        target_yaw = 0
        integral = last_error = 0
        dt = 0.05

        if time:
            timer = Timer(time, autostart=True)
            while timer.active:
                error = self._angle_diff(target_yaw, motion_sensor.tilt_angles()[0])
                integral += error * dt
                derivative = (error - last_error) / dt
                output = self.Kp * error + self.Ki * integral + self.Kd * derivative
                motor_pair.move(self.index, int(-max(min(output, 100), -100)), velocity=self.speed)
                last_error = error
                timer.update()
        elif degrees:
            start = motor.absolute_position(self.port1)
            while abs(motor.relative_position(self.port1) - start) <= degrees:
                error = self._angle_diff(target_yaw, motion_sensor.tilt_angles()[0])
                integral += error * dt
                derivative = (error - last_error) / dt
                output = self.Kp * error + self.Ki * integral + self.Kd * derivative
                motor_pair.move(self.index, int(-max(min(output, 100), -100)), velocity=self.speed)
                last_error = error
                
        self.stop()
        

    def reverse(self, **kwargs):
        self.left_speed *= -1
        self.right_speed *= -1
        self.speed *= -1
        self.forward(**kwargs)
        self.reset_attr()

    def stop(self):
        motor_pair.stop(self.index, stop=self.stop_command)


    def right_turn(self, theta, PID=False):
        self._turn(theta, left_active=True, PID=PID)

    def left_turn(self, theta, PID=False):
        self._turn(theta, left_active=False, PID=PID)

    def _turn(self, theta, left_active=True, PID=False):
        circumference = 2 * math.pi * self.wheelbase
        arc_len = circumference * theta / 360
        degrees = int(arc_len * (360 / self.wheel_circumference))
        target_yaw = (-theta if left_active else theta) * 10
        self._record_odometry_snapshot()

        if not PID:
        
            left = self.left_speed if left_active else 0
            right = 0 if left_active else self.right_speed
            motor_pair.move_tank_for_degrees(
                self.index, degrees, left, right,
                stop=self.stop_command,
                acceleration=self.acceleration,
                deceleration=self.deceleration
            )
            
            self._wait_until_stopped()
            self._update_odometry()
        else:
            self._run_pid_turn(target_yaw)

    def _run_pid_turn(self, target_yaw):
        integral = last_error = 0
        dt = 0.05
        while True:
            current_yaw = motion_sensor.tilt_angles()[0]
            error = self._angle_diff(target_yaw, current_yaw)
            if abs(error) < 10:
                break
            integral += error * dt
            derivative = (error - last_error) / dt
            output = self.Kp * error + self.Ki * integral + self.Kd * derivative
            motor_pair.move(self.index, int(-max(min(output, 100), -100)), velocity=self.speed)
            last_error = error
            
        self.stop()
        self._update_odometry()
        

    def follow_line(self, duration, port, reverse=False):
        timer = Timer(duration, autostart=True)
        while timer.active:
            reflection = color_sensor.reflection(PORT_DICT[port])
            steering = int(-3 / 5 * reflection + 30)
            if reverse:
                self.reverse(steering=steering)
            else:
                self.forward(steering=steering)
            timer.update()
            
        self._update_odometry()

    def follow_line_distance(self, distance, port, reverse=False):
        duration = int(distance * (360 / self.wheel_circumference) * (1000 / self.speed))
        self.follow_line(duration, port, reverse)

    def move(self, left_speed, right_speed):
        motor_pair.move_tank(self.index, left_speed, right_speed)
        

    def arc_turn(self, r, theta, reverse=False):
        degrees = int(abs(2 * math.pi * r * theta / self.wheel_circumference))

        if theta < 0:
            right_speed = self.right_speed
            left_speed = int(self.right_speed * (r - self.wheelbase) / r)
        else:
            left_speed = self.left_speed
            right_speed = int(self.left_speed * (r - self.wheelbase) / r)

        if reverse:
            left_speed *= -1
            right_speed *= -1

        self._record_odometry_snapshot()
        motor_pair.move_tank_for_degrees(
            self.index, degrees, left_speed, right_speed,
            stop=self.stop_command,
            acceleration=self.acceleration,
            deceleration=self.deceleration
        )
        self._update_odometry()

    def get_position(self):
        """
        Returns the current estimated position and heading of the robot
        
        Returns
        -------
        tuple of (float, float float)
            The (x, y, heading) coordinates:
            - x: x-position in cm
            - y: y-position in cm
            - heading: heading in radians
        """
        return self.x, self.y, self.heading

    def _wait_until_stopped(self, timeout=5.0):
        """Waits until motors stop based on encoder readings."""
        print("Waiting for robot to stop...")

        time.sleep(0.05)  # allow motors to begin moving

        start_time = time.time()
        prev_pos1 = motor.relative_position(self.port1)
        prev_pos2 = motor.relative_position(self.port2)

        stable_count = 0
        STABLE_THRESHOLD = 3  # require 3 consecutive stable readings

        while True:
            time.sleep(0.05)

            curr_pos1 = motor.relative_position(self.port1)
            curr_pos2 = motor.relative_position(self.port2)

            delta1 = abs(curr_pos1 - prev_pos1)
            delta2 = abs(curr_pos2 - prev_pos2)

            if delta1 < 1 and delta2 < 1:
                stable_count += 1
                if stable_count >= STABLE_THRESHOLD:
                    print("Motors appear to have stopped.")
                    break
            else:
                stable_count = 0  # reset if motion resumes

            if time.time() - start_time > timeout:
                print("Timeout waiting for motors to stop.")
                break

            prev_pos1, prev_pos2 = curr_pos1, curr_pos2

    def _record_odometry_snapshot(self):
        self.last_left_pos = motor.relative_position(self.port1)
        self.last_right_pos = motor.relative_position(self.port2)

        print("[SNAPSHOT] last_left: {}, last_right: {}".format(self.last_left_pos, self.last_right_pos))

    
    def go_to(self, x_target, y_target):
        x_current, y_current, heading = self.get_position()

        dx = x_target - x_current
        dy = y_target - y_current

        desired_theta = math.atan2(dy, dx)
        dtheta = (desired_theta - heading + math.pi) % (2 * math.pi) - math.pi

        print("Turning {:.1f} degrees".format(math.degrees(dtheta)))
        self.right_turn(math.degrees(-dtheta))

        distance = int(math.sqrt(dx**2 + dy**2))
        print("moving {:.2f} forward".format(distance))
        self.forward(distance=distance)
