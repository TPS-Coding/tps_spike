# LEGO Robotics Python Classes for SPIKE Prime

Welcome to the custom Python classes designed for the Tuxedo Park School FIRST LEGO League (FLL) robotics team! 
This codebase provides an easy, powerful way to control LEGO SPIKE Prime robots using Python and async programming.

## What's Included

### `Motor`
A simple, flexible controller for a single motor.

- Run a motor for time, degrees, or rotations
- Reset its position
- Customize speed, acceleration, and stop behavior
- Now fully `async` for use in `async def main()` functions

### `MotorPair`
A high-level controller for a pair of motors (left and right wheels).

- Drive forward using time, rotations, or distance
- Turn left or right using tank-style or PID-controlled turns
- Perform arc turns and follow lines with a color sensor
- Stop smoothly or instantly with customizable braking
- Fully compatible with `async def main()` (awaitable methods)
- Includes helper logic for timed movement and PID yaw correction

## How to Use

In your LEGO SPIKE program, define and control your robot like this:

```python
async def main():
    robot = MotorPair(1, ("A", "B"))
    await robot.forward(time=2000)
    await robot.right_turn(90)
    robot.stop()
```

You can also use the `Motor` class directly for mechanisms like arms, claws, or tools.

```python
motor = Motor("C")
await motor.run(degrees=180)
```

## Purpose

These classes:
- Make it easier to write clear, easy to understand code
- Help students learn real programming concepts without having to navigate LEGO's confusing API


## Next Steps

I'm currently working on:
- Odometry (robot position tracking)
- Obstacle detection and avoidance
- Sensor fusion (gyro + encoder for better heading)



---
