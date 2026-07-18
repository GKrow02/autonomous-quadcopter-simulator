# Autonomous Quadcopter Flight Simulator

A Python-based quadcopter simulation developed to study flight dynamics, feedback control, autonomous waypoint navigation, and three-dimensional path planning.

The simulator models the translational and rotational motion of a quadcopter, uses PID-based controllers to stabilize and navigate the vehicle, and generates collision-free paths around static obstacles using a 3D A* path-planning algorithm.

<img width="1917" height="1137" alt="image" src="https://github.com/user-attachments/assets/f836b630-6559-4dea-a885-53d65666f1ba" />


<img width="1917" height="1137" alt="image" src="https://github.com/user-attachments/assets/621b7cd8-65cb-4768-96b8-fecd413a58a8" />

## Development Process

This project was developed iteratively with AI-assisted debugging and implementation support. I developed the quadcopter dynamics simulation, PID-based flight control, waypoint navigation, visualization, and initial reactive obstacle-avoidance system.

After testing revealed limitations in the reactive approach, AI assistance was used to diagnose the failure cases and support the implementation of the 3D A* planner, segment collision checking, and path smoothing. I integrated the planner into the existing simulation, tested it across multiple environments, and added numerical collision-clearance validation.

## Project Overview

The project began as a basic quadcopter dynamics and control simulation. It was gradually expanded to include:

- Attitude stabilization
- Altitude and position control
- Autonomous waypoint navigation
- Simulated sensor noise
- Aerodynamic drag and angular damping
- Static obstacle representation
- Three-dimensional A* path planning
- Collision checking and path smoothing
- Mission-performance measurements
- 2D plots and 3D flight animation

The current simulator is a standalone Python prototype. It is intended to validate control and autonomy concepts before integrating the navigation system with PX4 SITL.


## Current Capabilities

### Quadcopter Dynamics

The simulation models:

- X, Y, and Z translational motion
- Roll, pitch, and yaw rotational motion
- Individual motor thrust
- Motor-generated roll, pitch, and yaw torque
- Gravity
- Linear aerodynamic drag
- Angular damping
- Ground collision prevention

The model is intentionally simplified and is not intended to replace a high-fidelity flight-dynamics simulator.

### Flight Control

The drone uses multiple feedback-control loops:

- PID-based altitude control
- Roll, pitch, and yaw stabilization
- Position-based pitch and roll commands
- Motor mixing for four individual motors
- Controller-output and motor-command limits
- Integral windup limiting for altitude control

The position controller converts waypoint error into desired pitch and roll angles. The attitude controller then adjusts the individual motor commands to move the drone toward the selected target.

### Autonomous Waypoint Navigation

The vehicle can autonomously navigate through a sequence of three-dimensional mission waypoints.

The navigation system:

- Tracks the active mission waypoint
- Calculates three-dimensional position error
- Advances through the mission automatically
- Uses separate tolerances for mission and planner waypoints
- Stops the simulation after the final waypoint is reached

### 3D A* Path Planning

When the direct route to a mission waypoint is blocked, the simulator uses a three-dimensional A* planner to calculate a safe path.

The planner:

- Represents the environment as a three-dimensional search grid
- Rejects positions inside obstacle safety regions
- Searches for a low-cost route to the destination
- Reconstructs the route as intermediate waypoints
- Checks complete path segments for collisions
- Smooths the planned route when longer safe segments are available
- Sends the resulting waypoint sequence to the flight controller

This replaced an earlier reactive avoidance system that generated only one temporary waypoint. Testing showed that the original method could create safe-looking intermediate points while still producing unsafe flight segments.

### Obstacle Modeling

Obstacles are currently:

- Static
- Predefined before the simulation
- Represented using spherical collision regions
- Expanded by an adjustable safety margin

The simulator numerically measures the drone’s minimum distance from each obstacle to verify that the planned trajectory remains collision-free.

### Sensor and Disturbance Modeling

The simulation currently includes:

- Random roll, pitch, and yaw measurement noise
- Random X, Y, and Z position measurement noise
- Linear aerodynamic drag
- Rotational damping

The obstacles are currently known by the planner rather than detected through a simulated camera, LiDAR, or depth sensor.

## Visualization and Mission Analysis

The simulator produces:

- Roll, pitch, and yaw response plots
- Individual motor-command plots
- Altitude-response plots
- Two-dimensional waypoint and obstacle plots
- Three-dimensional flight animation
- Full flight-path visualization

It also calculates:

- Total mission time
- Total path length
- Maximum simulated speed
- Final position error
- Minimum distance from each obstacle
- Minimum physical obstacle clearance
- Collision status

These measurements make it possible to evaluate the system numerically instead of relying only on visual inspection.

## Technologies Used

- Python
- NumPy
- Matplotlib
- Matplotlib Animation
