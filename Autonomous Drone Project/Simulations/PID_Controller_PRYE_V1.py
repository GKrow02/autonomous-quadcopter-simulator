import numpy as np 
import matplotlib.pyplot as plt 

#Physics
mass = 1.5 #kg
g = 9.81 #m/s^2
weight = mass * g #N
hover_thrust_per_motor = weight / 4

max_motor_thrust = 8
hover_percentage = (
    hover_thrust_per_motor / max_motor_thrust
) * 100

throttle = hover_percentage

#Drone Constants

arm_length = 0.25  # meters from center to each motor



# Theoretical Values

motor_1 = 0
motor_2 = 0
motor_3 = 0
motor_4 = 0

pitch = 20
roll = -15
yaw = 30
altitude = 0

roll_angular_v = 0
pitch_angular_v = 0
yaw_angular_v = 0
vertical_v = 0

desired_roll = 0
desired_pitch = 0
desired_yaw = 0
desired_altitude = 10

integral_roll = 0
integral_pitch = 0
integral_yaw = 0
integral_altitude = 0



inertia = 2
dt = .01


Kp_angle = 4  # How aggresive the change will be
Kd_angle = 5
Ki_angle = 0

Kp_altitude = 2  # How aggresive the change will be
Kd_altitude = 4
Ki_altitude = 0.05

yaw_torque_coefficient = 0.1  # Adjust this value based on your drone's characteristics
# Lists for what curent angle and time are

roll_angles = []
pitch_angles = []
yaw_angles = []
times = []
motor_1_values = []
motor_2_values = []
motor_3_values = []
motor_4_values = []
altitudes = []

# Actual PID controller

for i in range(7000):

# Distrubances to see how system reacts to them

    if i == 500:
      roll = roll + 20

    if i == 1500:
        pitch = pitch + 20

    if i == 3000:
        yaw = yaw + 20

    if i == 4000:
        altitude = altitude + 5


    measured_roll = roll + np.random.uniform(-0.3, 0.3) #Less precise instrument
    roll_error = desired_roll - measured_roll
    
    measured_pitch = pitch + np.random.uniform(-0.3, 0.3) #Less precise instrument
    pitch_error = desired_pitch - measured_pitch

    measured_yaw = yaw + np.random.uniform(-0.3, 0.3) #Less precise instrument
    yaw_error = desired_yaw - measured_yaw

    measured_altitude = altitude + np.random.uniform(-0.3, 0.3)  # Less precise instrument
    altitude_error = desired_altitude - measured_altitude

    integral_roll += roll_error * dt
    integral_pitch += pitch_error * dt
    integral_yaw += yaw_error * dt
    integral_altitude += altitude_error * dt
    integral_altitude = np.clip(integral_altitude, -20, 20)

    roll_command = Kp_angle*roll_error - Kd_angle*roll_angular_v + Ki_angle * integral_roll
    pitch_command = Kp_angle*pitch_error - Kd_angle*pitch_angular_v + Ki_angle * integral_pitch
    yaw_command = Kp_angle*yaw_error - Kd_angle*yaw_angular_v + Ki_angle * integral_yaw
    altitude_command = Kp_altitude*altitude_error - Kd_altitude*vertical_v + Ki_altitude * integral_altitude

    roll_command = np.clip(roll_command, -10, 10) #Limits torque to something more realistic
    pitch_command = np.clip(pitch_command, -10, 10) #Limits torque to something more realistic
    yaw_command = np.clip(yaw_command, -10, 10) #Limits torque to something more realistic
    altitude_command = np.clip(altitude_command, -10, 10) #Limits torque to something more realistic

    throttle = hover_percentage + altitude_command
    throttle = np.clip(throttle, 0, 100)

# Motor thrust calculations based on roll, pitch, and yaw adjustments

    motor_1 = throttle - roll_command - pitch_command + yaw_command
    motor_2 = throttle + roll_command - pitch_command - yaw_command
    motor_3 = throttle + roll_command + pitch_command + yaw_command
    motor_4 = throttle - roll_command + pitch_command - yaw_command

    motor_1 = np.clip(motor_1, 0, 100)
    motor_2 = np.clip(motor_2, 0, 100)
    motor_3 = np.clip(motor_3, 0, 100)
    motor_4 = np.clip(motor_4, 0, 100)

    motor_1_thrust = (motor_1 / 100) * max_motor_thrust
    motor_2_thrust = (motor_2 / 100) * max_motor_thrust
    motor_3_thrust = (motor_3 / 100) * max_motor_thrust
    motor_4_thrust = (motor_4 / 100) * max_motor_thrust

    actual_roll_torque = (
        -motor_1_thrust + motor_2_thrust + motor_3_thrust - motor_4_thrust
    ) * arm_length

    actual_pitch_torque = (
        -motor_1_thrust - motor_2_thrust + motor_3_thrust + motor_4_thrust
    ) * arm_length

    actual_yaw_torque = (
        motor_1_thrust - motor_2_thrust + motor_3_thrust - motor_4_thrust
    ) * yaw_torque_coefficient

    roll_angular_a = actual_roll_torque/inertia
    pitch_angular_a = actual_pitch_torque/inertia
    yaw_angular_a = actual_yaw_torque/inertia

    roll_angular_v += roll_angular_a * dt
    pitch_angular_v += pitch_angular_a * dt
    yaw_angular_v += yaw_angular_a * dt

    roll += roll_angular_v * dt
    pitch += pitch_angular_v * dt
    yaw += yaw_angular_v * dt

    total_thrust = motor_1_thrust + motor_2_thrust + motor_3_thrust + motor_4_thrust
    vertical_thrust = (
    total_thrust
    * np.cos(np.radians(roll))
    * np.cos(np.radians(pitch))
)

    vertical_force = vertical_thrust - weight
    vertical_a = vertical_force / mass

    vertical_v += vertical_a * dt
    altitude += vertical_v * dt

    # Drone cannot go below the ground

    if altitude < 0:
        altitude = 0
        vertical_v = 0

    altitudes.append(altitude)
    motor_1_values.append(motor_1)
    motor_2_values.append(motor_2)
    motor_3_values.append(motor_3)
    motor_4_values.append(motor_4)
    roll_angles.append(roll)
    pitch_angles.append(pitch)
    yaw_angles.append(yaw)
    times.append(i*dt)

plt.close("all")

# Graph 1: Angles
plt.figure()
plt.plot(times, pitch_angles, label="Pitch")
plt.plot(times, roll_angles, label="Roll")
plt.plot(times, yaw_angles, label="Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (degrees)")
plt.title("Drone Orientation")
plt.grid()
plt.legend()

# Graph 2: Motors
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

# Graph 3: Altitude
plt.figure()
plt.plot(times, altitudes)
plt.xlabel("Time (s)")
plt.ylabel("Altitude (m)")
plt.title("Drone Altitude")
plt.grid()

plt.show()