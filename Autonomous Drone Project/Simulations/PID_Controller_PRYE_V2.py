import numpy as np
import matplotlib.pyplot as plt



# Physics


mass = 1.5                      # kg
gravity = 9.81                 # m/s^2
weight = mass * gravity        # N

max_motor_thrust = 8           # N per motor

hover_thrust_per_motor = weight / 4

hover_percentage = (
    hover_thrust_per_motor / max_motor_thrust
) * 100

throttle = hover_percentage



# Drone constants


arm_length = 0.25              # m
inertia = 2
yaw_torque_coefficient = 0.1
dt = 0.01



# Initial motor commands


motor_1_command = 0
motor_2_command = 0
motor_3_command = 0
motor_4_command = 0



# Initial orientation


roll = 0
pitch = 0
yaw = 0

roll_angular_velocity = 0
pitch_angular_velocity = 0
yaw_angular_velocity = 0



# Initial position


x = 0
y = 0
z = 0

x_velocity = 0
y_velocity = 0
z_velocity = 0



# Desired orientation


desired_roll = 0
desired_pitch = 0
desired_yaw = 0



# Desired position


waypoints = [
    (0, 0, 10),
    (10, 0, 10),
    (10, 10, 10),
    (0, 10, 10),
    (0, 0, 10)
]

waypoint_index = 0
waypoint_tolerance = .5

desired_x, desired_y, desired_z = waypoints[waypoint_index]


# Integral values


roll_integral = 0
pitch_integral = 0
yaw_integral = 0
z_integral = 0



# PID gains


Kp_angle = 4
Kd_angle = 5
Ki_angle = 0

Kp_position = .008
Kd_position = 0.25

Kp_z = 2
Kd_z = 4
Ki_z = 0.05



# Lists for graphs


times = []

roll_values = []
pitch_values = []
yaw_values = []

motor_1_values = []
motor_2_values = []
motor_3_values = []
motor_4_values = []

z_values = []
x_values = []
y_values = []

waypoint_indices = []




# Simulation loop


for i in range(60000):

    # Disturbances


    
    # Simulated sensor readings
    

    measured_roll = roll + np.random.uniform(-0.3, 0.3)
    measured_pitch = pitch + np.random.uniform(-0.3, 0.3)
    measured_yaw = yaw + np.random.uniform(-0.3, 0.3)
    measured_z = z + np.random.uniform(-0.3, 0.3)
    measured_x = x + np.random.uniform(-0.3, 0.3)
    measured_y = y + np.random.uniform(-0.3, 0.3)

    
    # X and Y error
    

    x_error = desired_x - measured_x
    y_error = desired_y - measured_y

    
    # Adjusted desired pitch and roll based on desired x and y positions
    


    desired_pitch = (
    Kp_position * x_error
    - Kd_position * x_velocity
)
    desired_roll = (
    Kp_position * y_error
    - Kd_position * y_velocity
)
    desired_pitch = np.clip(desired_pitch, -15, 15)
    desired_roll = np.clip(desired_roll, -15, 15)


    
    # Errors roll pitch yaw and z
    

    roll_error = desired_roll - measured_roll
    pitch_error = desired_pitch - measured_pitch
    yaw_error = desired_yaw - measured_yaw
    z_error = desired_z - measured_z


    
    # Integral updates
    

    roll_integral += roll_error * dt
    pitch_integral += pitch_error * dt
    yaw_integral += yaw_error * dt
    z_integral += z_error * dt

    z_integral = np.clip(z_integral, -20, 20)


    
    # PID controller commands
    

    roll_command = (
        Kp_angle * roll_error
        - Kd_angle * roll_angular_velocity
        + Ki_angle * roll_integral
    )

    pitch_command = (
        Kp_angle * pitch_error
        - Kd_angle * pitch_angular_velocity
        + Ki_angle * pitch_integral
    )

    yaw_command = (
        Kp_angle * yaw_error
        - Kd_angle * yaw_angular_velocity
        + Ki_angle * yaw_integral
    )

    z_command = (
        Kp_z * z_error
        - Kd_z * z_velocity
        + Ki_z * z_integral
    )


    # Limit controller outputs

    roll_command = np.clip(roll_command, -10, 10)
    pitch_command = np.clip(pitch_command, -10, 10)
    yaw_command = np.clip(yaw_command, -10, 10)
    z_command = np.clip(z_command, -10, 10)


    
    # Altitude controller
    

    throttle = hover_percentage + z_command
    throttle = np.clip(throttle, 0, 100)


    
    # Motor mixer
    

    motor_1_command = (
        throttle
        - roll_command
        - pitch_command
        + yaw_command
    )

    motor_2_command = (
        throttle
        + roll_command
        - pitch_command
        - yaw_command
    )

    motor_3_command = (
        throttle
        + roll_command
        + pitch_command
        + yaw_command
    )

    motor_4_command = (
        throttle
        - roll_command
        + pitch_command
        - yaw_command
    )


    # Limit motor commands

    motor_1_command = np.clip(motor_1_command, 0, 100)
    motor_2_command = np.clip(motor_2_command, 0, 100)
    motor_3_command = np.clip(motor_3_command, 0, 100)
    motor_4_command = np.clip(motor_4_command, 0, 100)


    
    # Convert motor commands to thrust
    

    motor_1_thrust = (
        motor_1_command / 100
    ) * max_motor_thrust

    motor_2_thrust = (
        motor_2_command / 100
    ) * max_motor_thrust

    motor_3_thrust = (
        motor_3_command / 100
    ) * max_motor_thrust

    motor_4_thrust = (
        motor_4_command / 100
    ) * max_motor_thrust


    
    # Actual torques from motors
    

    roll_torque = (
        -motor_1_thrust
        + motor_2_thrust
        + motor_3_thrust
        - motor_4_thrust
    ) * arm_length

    pitch_torque = (
        -motor_1_thrust
        - motor_2_thrust
        + motor_3_thrust
        + motor_4_thrust
    ) * arm_length

    yaw_torque = (
        motor_1_thrust
        - motor_2_thrust
        + motor_3_thrust
        - motor_4_thrust
    ) * yaw_torque_coefficient


    
    # Angular accelerations
    

    roll_angular_acceleration = roll_torque / inertia
    pitch_angular_acceleration = pitch_torque / inertia
    yaw_angular_acceleration = yaw_torque / inertia


    
    # Update angular velocities
    

    roll_angular_velocity += (
        roll_angular_acceleration * dt
    )

    pitch_angular_velocity += (
        pitch_angular_acceleration * dt
    )

    yaw_angular_velocity += (
        yaw_angular_acceleration * dt
    )


    
    # Update orientation
    

    roll += roll_angular_velocity * dt
    pitch += pitch_angular_velocity * dt
    yaw += yaw_angular_velocity * dt


    
    # Total thrust
    

    total_thrust = (
        motor_1_thrust
        + motor_2_thrust
        + motor_3_thrust
        + motor_4_thrust
    )

    
    # XY-axis physics
    

    x_thrust = total_thrust * np.sin(np.radians(pitch)) * np.cos(np.radians(roll))

    y_thrust = total_thrust * np.sin(np.radians(roll)) * np.cos(np.radians(pitch))

    x_acceleration = x_thrust / mass
    y_acceleration = y_thrust / mass

    x_velocity += x_acceleration * dt
    y_velocity += y_acceleration * dt

    x += x_velocity * dt
    y += y_velocity * dt

    
    # Z-axis physics
    

    z_thrust = (
        total_thrust
        * np.cos(np.radians(roll))
        * np.cos(np.radians(pitch))
    )

    z_force = z_thrust - weight

    z_acceleration = z_force / mass

    z_velocity += z_acceleration * dt
    z += z_velocity * dt


    # Ground collision

    if z < 0:
        z = 0
        z_velocity = 0


    distance_to_waypoint = np.sqrt(
    (desired_x - x) ** 2
    + (desired_y - y) ** 2
    + (desired_z - z) ** 2
)
    
    if distance_to_waypoint < waypoint_tolerance:

        if waypoint_index < len(waypoints) - 1:

            waypoint_index += 1
            desired_x, desired_y, desired_z = waypoints[waypoint_index]

        else:

            print("Mission Complete")
            break


    
    # Save values
    

    x_values.append(x)
    y_values.append(y)
    z_values.append(z)

    times.append(i * dt)

    roll_values.append(roll)
    pitch_values.append(pitch)
    yaw_values.append(yaw)

    motor_1_values.append(motor_1_command)
    motor_2_values.append(motor_2_command)
    motor_3_values.append(motor_3_command)
    motor_4_values.append(motor_4_command)

    waypoint_indices.append(waypoint_index)




# Graphs


plt.close("all")


# Orientation graph

plt.figure()

plt.plot(times, roll_values, label="Roll")
plt.plot(times, pitch_values, label="Pitch")
plt.plot(times, yaw_values, label="Yaw")

plt.xlabel("Time (s)")
plt.ylabel("Angle (degrees)")
plt.title("Drone Orientation")
plt.grid()
plt.legend()


# Motor graph

plt.figure()

plt.plot(times, motor_1_values, label="Motor 1")
plt.plot(times, motor_2_values, label="Motor 2")
plt.plot(times, motor_3_values, label="Motor 3")
plt.plot(times, motor_4_values, label="Motor 4")

plt.xlabel("Time (s)")
plt.ylabel("Motor Command (%)")
plt.title("Motor Commands")
plt.grid()
plt.legend()


# Z-position graph

plt.figure()

plt.plot(times, z_values, label="Z Position")
plt.axhline(
    desired_z,
    linestyle="--",
    label="Desired Z"
)

plt.xlabel("Time (s)")
plt.ylabel("Z Position (m)")
plt.title("Drone Z Position")
plt.grid()
plt.legend()

# XY-position graph
waypoint_x = [point[0] for point in waypoints]
waypoint_y = [point[1] for point in waypoints]

plt.figure()

plt.plot(x_values, y_values, label="Drone Path")
plt.plot(waypoint_x, waypoint_y, "ro--", label="Waypoints")

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Waypoint Navigation")
plt.axis("equal")
plt.grid()
plt.legend()

plt.show()

