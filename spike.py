""" Python 3
    Author: Denise Kilburg on behalf of the TPS Computer Science Department
    Classes for Lego Spike. API is more kid friendly than default. This is quick, 
    non professional code written only to help the kids ease into python coding 
    for lego robotics. Documentation and example code
    will be added to the Tuxedo Park School github: www.github.com/tps-coding
    This library must be written to the /flash directory 
    of the hub in order to import to the lego prime spike app. """

from hub import light_matrix, motion_sensor
import motor
import runloop
import motor_pair
import color_sensor
import math
import time



PORT_DICT = {
    'A': 0,
    'B': 1,
    'C': 2,
    'D': 3,
    'E': 4,
    'F': 5
}

class Timer:
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

class Motor():
    def __init__(self, port):
        self.port = PORT_DICT[port]
        self.speed = 1000
        self.acceleration = 1000
        self.deceleration = 1000
        self.stop_command = motor.BRAKE

        print("Default values: velocity:{}, acceleration: {}, deceleration: {}, rotation: clockwise".format(self.speed, 
            self.acceleration, self.deceleration))
        

    def run(self, time=0, degrees = 0, rotations=0, reverse=False):
        ### time (ms)
        if rotations != 0:
            degrees = int(rotations*360)
        if reverse:
            if self.speed > 0:
                self.speed *= -1
        if time != 0:
            return motor.run_for_time(self.port, time, self.speed, stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration)
        elif degrees != 0:
            return motor.run_for_degrees(self.port, degrees, self.speed,stop=self.stop_command,acceleration=self.acceleration,deceleration=self.deceleration)
        else:
            motor.run(self.port, self.speed, acceleration=self.acceleration)

    def reset(self, position=0):
        ##sets to default position 
        return motor.run_to_absolute_position(self.port,position, self.speed, direction=motor.SHORTEST_PATH, 
            stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration)
        
    def reset_attr(self):
        ##sets to default attr 
        self.speed = 1000
        self.acceleration = 1000
        self.deceleration = 1000
        self.stop_command = motor.BRAKE

    def stop(self):
        return motor.stop(self.port,stop=self.stop_command)

class MotorPair():
    def __init__(self, pair, ports):
        self.port1 = PORT_DICT[ports[0]]
        self.port2 = PORT_DICT[ports[1]]
        self.index = 0
        self.speed = 360
        self.right_speed = 360
        self.left_speed = 360
        self.acceleration = 1000
        self.deceleration = 1000
        self.stop_command = motor.BRAKE
        self.wheel_circumference = 17.6 ## small wheel. 27.6 --> big wheel
        self.wheelbase = 11.2  ## cm 

        ## For PID controller
        self.Kp = 1.0
        self.Ki = 0
        self.Kd = 0
        
        if pair == 1:
            self.index = 0
        elif pair == 2:
            self.index = 1
        elif pair == 3:
            self.index = 2

        motor_pair.pair(self.index, self.port1, self.port2)

    def angle_diff(self, target, current):
        diff = (target - current + 1800) % 3600 - 1800
        return diff

    
    def reset_attr(self):
        self.speed = self.right_speed = self.left_speed = 360
        self.acceleration = self.deceleration = 1000
        self.stop_command = motor.BRAKE

    def reset_motors(self, position=0):
        motor.run_to_absolute_position(self.port2,position, self.speed, direction=motor.SHORTEST_PATH, 
            stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration)

        motor.run_to_absolute_position(self.port1,position, self.speed, direction=motor.SHORTEST_PATH, 
            stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration)

    def forward(self,steering=0, time=0, rotations=0, distance=0, PID=False):
        degrees = int(rotations * 360)
        if PID and time == rotations == distance == 0:
            print('Cannot use PID without specifying a time or distance or rotations')
            return
        if distance != 0:
            degrees = int(distance *(360/self.wheel_circumference))
        if not PID:
            if time != 0 and rotations == 0 and distance == 0:
                if self.left_speed != self.right_speed:
                    self.left_speed = self.right_speed  
                return motor_pair.move_tank_for_time(self.index, self.left_speed, self.right_speed, time, 
                    stop=self.stop_command, acceleration=self.acceleration,deceleration=self.deceleration)
            elif  degrees != 0 and time == 0:
                return motor_pair.move_tank_for_degrees(self.index, degrees, self.left_speed, self.right_speed, 
                    stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration )
            elif time==rotations==distance==0:
                motor_pair.move(self.index, steering, velocity=self.speed)
        else:
            # Target yaw angle (counterclockwise is +, clockwise is -)
            target_yaw = 0 # deciDegrees

            # Initialize PID variables
            integral = 0
            last_error = 0
            dt = 0.05  # Loop time

            if time != 0:
                timer = Timer(time, autostart=True)
                while timer.active:
                    current_yaw = motion_sensor.tilt_angles()[0]
                   
                    error = self.angle_diff(target_yaw, current_yaw)

                    integral += error * dt
                    derivative = (error - last_error) / dt

                    output = self.Kp * error + self.Ki * integral + self.Kd * derivative
                    output = max(min(output, 100), -100)  # Clamp steering

                    steering=int(-output)
                    motor_pair.move(self.index, steering, velocity=self.speed)
                    timer.update()
                self.stop()

            elif degrees != 0:
                start = motor.absolute_position(self.port1)
                while int(abs(motor.relative_position(self.port1) - start)) <= degrees:
                    current_yaw = motion_sensor.tilt_angles()[0]
                    
                    error = self.angle_diff(target_yaw, current_yaw)

                    integral += error * dt
                    derivative = (error - last_error) / dt

                    output = self.Kp * error + self.Ki * integral + self.Kd * derivative
                    output = max(min(output, 100), -100)  # Clamp steering

                    steering=int(-output)
                    motor_pair.move(self.index, steering, velocity=self.speed)
                    
                self.stop()
                    
    def reverse(self,steering=0, time=0, rotations=0, distance=0, PID=False):
       self.left_speed *= -1
       self.right_speed *= -1
       self.speed *= -1
       self.forward(steering, time, rotations, distance, PID)
       self.reset_attr()

    def stop(self):
        motor_pair.stop(self.index, stop=self.stop_command)

    def right_turn(self,theta, PID=False):
        circumference = 2 * math.pi * self.wheelbase
        distance = circumference * theta/360
        degrees = int(distance * (360/self.wheel_circumference))
        if not PID:
            return motor_pair.move_tank_for_degrees(self.index, degrees, self.left_speed, 0, 
                stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration)
        else:
            # Target yaw angle (counterclockwise is +, clockwise is -)
            target_yaw = -theta * 10 # deciDegrees

            # Initialize PID variables
            integral = 0
            last_error = 0
            dt = 0.05  # Loop time

            while True:
                current_yaw = motion_sensor.tilt_angles()[0]
                    
                error = self.angle_diff(target_yaw, current_yaw)

                if abs(error) < 10.0:
                    break  # Stop if close enough

                integral += error * dt
                derivative = (error - last_error) / dt

                output = self.Kp * error + self.Ki * integral + self.Kd * derivative
                output = max(min(output, 100), -100)  # Clamp steering

                steering=int(-output)
                motor_pair.move(self.index, steering, velocity=self.speed)
            self.stop()

    def left_turn(self,theta, PID=False):
        circumference = 2 * math.pi * self.wheelbase
        distance = circumference * theta/360
        degrees = int(distance * (360/self.wheel_circumference))
        if not PID:
            return motor_pair.move_tank_for_degrees(self.index, degrees, 0, self.right_speed, 
                stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration)
        else:
             # Target yaw angle (counterclockwise is +, clockwise is -)
            target_yaw = theta * 10 # deciDegrees

            # Initialize PID variables
            integral = 0
            last_error = 0
            dt = 0.05  # Loop time

            while True:
                current_yaw = motion_sensor.tilt_angles()[0]
                    
                error = self.angle_diff(target_yaw, current_yaw)

                if abs(error) < 10.0:
                    break  # Stop if close enough

                integral += error * dt
                derivative = (error - last_error) / dt

                output = self.Kp * error + self.Ki * integral + self.Kd * derivative
                output = max(min(output, 100), -100)  # Clamp steering

                steering=int(-output)
                motor_pair.move(self.index, steering, velocity=self.speed)
            self.stop()

    def follow_line(self, duration, port, reverse=False):
        follow_timer = Timer(duration, autostart=True)
        while follow_timer.active:
            Steering = int(math.floor(-3/5)*color_sensor.reflection(PORT_DICT[port])+30)
            self.reverse(steering=Steering) if reverse else self.forward(steering=Steering)
            follow_timer.update()

    def follow_line_distance(self, distance, port, reverse = False):
        ## distance (cm); port = color sensor port
        duration = int(distance * (360/self.wheel_circumference)*(1000/self.speed))
        self.follow_line(duration, port, reverse)

    def move(self, left_speed, right_speed):
        motor_pair.move_tank(self.index, left_speed, right_speed)

    def arc_turn(self, r, theta, reverse=False):
        """
        r2 =  27 cm = r  *This is the turn radius of outer wheel
        arc len = 2*pi*r*theta/360
        arc len in wheel deg = 2*pi*r*theta/wheel circumference
        v2/v1 = r2/r1
        
        """
        degrees = int(abs(2 * math.pi * r * theta / self.wheel_circumference))
        
        if theta < 0:
            #counterclockwise
            # v2 -> right wheel
            right_speed = self.right_speed
            left_speed = int(self.right_speed * (r- self.wheelbase) / r)
            
        else:
            #clockwise
            # v2 -> left wheel
            left_speed = self.left_speed
            right_speed = int(self.left_speed * (r - self.wheelbase) / r)
        
        if not reverse:
            return motor_pair.move_tank_for_degrees(self.index, degrees, left_speed, right_speed, 
                    stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration )
        else:
            return motor_pair.move_tank_for_degrees(self.index, degrees, -left_speed, -right_speed, 
                    stop=self.stop_command, acceleration=self.acceleration, deceleration=self.deceleration )
            
            
    
