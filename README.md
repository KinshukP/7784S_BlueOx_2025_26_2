# 7784S_BlueOx_2025_26_2

A VEX V5 robot built for competitive autonomous navigation using odometry and PID control.

## Tools & Libraries
- **PROS** — real-time operating system for VEX V5
- **LemLib** — motion and odometry library for PROS
- **C++**

## Hardware
- 6-motor tank drive (blue cartridge)
- V5 IMU (inertial sensor)
- Rotation sensor tracking wheel
- Lift, intake, and pneumatic mechanisms

## What I Learned

**Odometry** — using a free-spinning tracking wheel + IMU to track the robot's (x, y, heading) position on the field at all times, even when the drive wheels slip.

**PID Control** — tuning proportional and derivative gains to make the robot move to a target smoothly without overshooting. Separate PID loops for linear (forward/back) and angular (turning) movement.

**Path Planning** — chaining together moveToPoint, turnToHeading, and moveToPose calls to build a full autonomous routine that navigates across the field and scores multiple times.

**Arcade Drive** — mapping two joystick axes (forward + turn) into left/right motor commands for driver control.
