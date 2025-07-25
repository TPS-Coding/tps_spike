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
    controller for a single motor with support for time-based,
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

    def run(self, time=0, degrees=0, rotations=0, reverse=False):
        if rotations:
            degrees = int(rotations * 360)
        speed = -abs(self.speed) if reverse else abs(self.speed)

        if time:
            motor.run_for_time(self.port, time, speed,
                stop=self.stop_command,
                acceleration=self.acceleration,
                deceleration=self.deceleration)
            self._wait_until_stopped()
            
        elif degrees:
            motor.run_for_degrees(self.port, degrees, speed,
                stop=self.stop_command,
                acceleration=self.acceleration,
                deceleration=self.deceleration)
            self._wait_until_stopped()
            
        else:
            motor.run(self.port, speed, acceleration=self.acceleration)
            

    def reset(self, position=0):
        motor.run_to_absolute_position(
            self.port, position, self.speed,
            direction=motor.SHORTEST_PATH,
            stop=self.stop_command,
            acceleration=self.acceleration,
            deceleration=self.deceleration
        )
        

    def reset_attr(self):
        self.speed = self.DEFAULT_SPEED
        self.acceleration = self.DEFAULT_ACCEL
        self.deceleration = self.DEFAULT_DECEL
        self.stop_command = motor.BRAKE

    def stop(self):
        motor.stop(self.port, stop=self.stop_command)

    def _wait_until_stopped(self, timeout=5.0):
        """Waits until the motor stops based on encoder readings."""
        print("Waiting for motor to stop...")

        time.sleep(0.05)  # allow motors to begin moving

        start_time = time.time()
        prev_pos = motor.relative_position(self.port)
        

        stable_count = 0
        STABLE_THRESHOLD = 3  # require 3 consecutive stable readings

        while True:
            time.sleep(0.05)

            curr_pos = motor.relative_position(self.port)
            

            delta1 = abs(curr_pos - prev_pos)

            if delta1 < 1:
                stable_count += 1
                if stable_count >= STABLE_THRESHOLD:
                    print("Motor appears to have stopped.")
                    break
            else:
                stable_count = 0  # reset if motion resumes

            if time.time() - start_time > timeout:
                print("Timeout waiting for motors to stop.")
                break

            prev_pos = curr_pos


class MotorPair():
    """Revised MotorPair class where each function controls the motors independently as opposed to using
    LEGO motor_pair module. Will update documentation in a bit. I'm rewritting it this way so that I can add
    more advanced functionality without having to fit into the confines of LEGO motor_pair

    Parameters
    ----------
    port1 : str
        Name of the 1st port (e.g., "A") to control, mapped from PORT_DICT.
    port2 : str
        Name of the 2nd port (e.g., "B) to control, mapped from PORT_DICT

    These ports will be considered "paired"


    """
    def __init__(self, port1, port2):
        self.lt_motor = Motor(port1) ## for forward motion, needs to be reversed as motors run clockwise by default
        self.rt_motor = Motor(port2)

        self.speed = 360 
        self.wheel_circumference = 17.5 ## cm, small wheel from LEGO docs
        self.wheelbase = 11.2 ## cm

        ## Odometry
        self.x = 0.0
        self.y = 0.0
        self.heading = 0

        ## PID constants
        KP = 1.0 ## Proportional
        KI = 0.0 ## Integral
        KD = 0.0 ## Derivative
        

        self._update_speed()

    def _update_speed(self):
        self.lt_motor.speed = self.rt_motor.speed = self.speed

    def _await_stop(self):
        self.rt_motor._wait_until_stopped()
        self.lt_motor._wait_until_stopped()

    def _distance_to_degrees(self, distance):
        return int(distance * 360 / self.wheel_circumference)

    def forward(self, distance=0, time=0, steering=0, PID=False):
        if distance != 0 and time == 0:
            degrees = self._distance_to_degrees(distance)
            if not PID:
                self.lt_motor.run(degrees=degrees, reverse=True)
                self.rt_motor.run(degrees=degrees)
            else:
                self._run_pid_loop(degrees, time, target_yaw=0)
        elif distance == 0 and time != 0:
            if not PID:
                self.lt_motor.run(time=time, reverse=True)
                self.rt_motor.run(time=time)
            else:
                print("PID requires a distance at this time. Please set a distance")
        else:
            self.lt_motor.run(reverse=True)
            self.rt_motor.run()
        self._await_stop()
  

    def reverse(self, distance=0, time=0, steering=0):
        if distance != 0 and time == 0:
            degrees = self._distance_to_degrees(distance)
            self.lt_motor.run(degrees= degrees)
            self.rt_motor.run(degrees=degrees, reverse=True)
        elif distance == 0 and time != 0:
            self.lt_motor.run(time=time)
            self.rt_motor.run(time=time, reverse=True)
        else:
            self.lt_motor.run()
            self.rt_motor.run()
        self._await_stop()
        self._update_speed()

    def tank_turn(self, theta):
        degrees = self._distance_to_degrees(self._arc_length(abs(theta)))
        if theta >= 0:
            self.rt_motor.run(degrees=degrees)
        elif theta < 0:
            self.lt_motor.run(degrees=degrees, reverse=True)
        self._await_stop()

    def pivot_turn(self, theta):
        degrees = int(self._distance_to_degrees(self._arc_length(abs(theta)))/ 2)
        if theta >= 0:
            self.rt_motor.run(degrees=degrees)
            self.lt_motor.run(degrees=degrees)
        elif theta < 0:
            self.rt_motor.run(degrees=degrees, reverse=True)
            self.lt_motor.run(degrees=degrees, reverse=True)

        self._await_stop()

    def _run_pid_loop(self, target_degrees, time, target_yaw=0):
        """
        PID loop that adjusts motor speeds to keep the robot heading straight
        (or toward a target yaw angle) while driving forward a specified distance.
        """

        KP = 2.0  

        # Reset motor positions to 0
        motor.reset_relative_position(self.lt_motor.port, 0)
        motor.reset_relative_position(self.rt_motor.port, 0)

        # Loop until the average of both motors has reached the target degrees
        while True:
            # Get current yaw from motion sensor
            current_yaw = motion_sensor.tilt_angles()[0]  # Assuming X axis

            # Calculate yaw error
            yaw_error = target_yaw - current_yaw

            # Apply P-control to adjust speed
            correction = KP * yaw_error

            # Clamp correction if needed
            max_correction = self.speed * 0.5
            correction = max(-max_correction, min(correction, max_correction))

            # Adjust motor speeds
            left_speed = self.speed - correction
            right_speed = self.speed + correction

            motor.run(self.lt_motor.port, -int(left_speed))  # Reversed left motor
            motor.run(self.rt_motor.port, int(right_speed))

            # Calculate average position
            left_pos = abs(motor.relative_position(self.lt_motor.port))
            right_pos = abs(motor.relative_position(self.rt_motor.port))
            avg_pos = (left_pos + right_pos) / 2

            if avg_pos >= target_degrees:
                break

        # Stop both motors
        self.lt_motor.stop()
        self.rt_motor.stop()


    

        