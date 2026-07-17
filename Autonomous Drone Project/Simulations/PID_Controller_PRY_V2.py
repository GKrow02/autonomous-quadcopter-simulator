import numpy as np 
import matplotlib.pyplot as plt 

# Theoretical Values 
Base_Thrust = 50

motor_1 = 0
motor_2 = 0
motor_3 = 0
motor_4 = 0

pitch = 20
roll = -15
yaw = 30

roll_angular_v = 0
pitch_angular_v = 0
yaw_angular_v = 0

desired_roll = 0
desired_pitch = 0
desired_yaw = 0

integral_roll = 0
integral_pitch = 0
integral_yaw = 0



inertia = 2
dt = .01


Kp = 2  # How aggresive the change will be
Kd = 2
Ki= .05

# Lists for what curent angle and time are

roll_angles = []
pitch_angles = []
yaw_angles = []
times = []
motor_1_values = []
motor_2_values = []
motor_3_values = []
motor_4_values = []

# Actual PID controller

for i in range(7000):

# Distrubances to see how system reacts to them

    if i == 500:
      roll = roll + 20

    if i == 1500:
        pitch = pitch + 20

    if i == 3000:
        yaw = yaw + 20

    measured_roll = roll + np.random.uniform(-0.3, 0.3) #Less precise instrument
    roll_error = desired_roll - measured_roll
    

    measured_pitch = pitch + np.random.uniform(-0.3, 0.3) #Less precise instrument
    pitch_error = desired_pitch - measured_pitch

    measured_yaw = yaw + np.random.uniform(-0.3, 0.3) #Less precise instrument
    yaw_error = desired_yaw - measured_yaw

    integral_roll += roll_error * dt
    integral_pitch += pitch_error * dt
    integral_yaw += yaw_error * dt

    torque_roll = Kp*roll_error - Kd*roll_angular_v + Ki * integral_roll
    torque_pitch = Kp*pitch_error - Kd*pitch_angular_v + Ki * integral_pitch
    torque_yaw = Kp*yaw_error - Kd*yaw_angular_v + Ki * integral_yaw

    torque_roll = np.clip(torque_roll, -10, 10) #Limits torque to something more realistic
    torque_pitch = np.clip(torque_pitch, -10, 10) #Limits torque to something more realistic
    torque_yaw = np.clip(torque_yaw, -10, 10) #Limits torque to something more realistic

# Motor thrust calculations based on roll, pitch, and yaw adjustments

    motor_1 = Base_Thrust - torque_roll - torque_pitch + torque_yaw
    motor_2 = Base_Thrust + torque_roll - torque_pitch - torque_yaw
    motor_3 = Base_Thrust + torque_roll + torque_pitch + torque_yaw
    motor_4 = Base_Thrust - torque_roll + torque_pitch - torque_yaw

    motor_1 = np.clip(motor_1, 30, 80)
    motor_2 = np.clip(motor_2, 30, 80)
    motor_3 = np.clip(motor_3, 30, 80)
    motor_4 = np.clip(motor_4, 30, 80)

    actual_roll_torque = (
        -motor_1 + motor_2 + motor_3 - motor_4
    ) / 4

    actual_pitch_torque = (
        -motor_1 - motor_2 + motor_3 + motor_4
    ) / 4

    actual_yaw_torque = (
        motor_1 - motor_2 + motor_3 - motor_4
    ) / 4

    roll_angular_a = actual_roll_torque/inertia
    pitch_angular_a = actual_pitch_torque/inertia
    yaw_angular_a = actual_yaw_torque/inertia

    roll_angular_v += roll_angular_a * dt
    pitch_angular_v += pitch_angular_a * dt
    yaw_angular_v += yaw_angular_a * dt

    roll += roll_angular_v * dt
    pitch += pitch_angular_v * dt
    yaw += yaw_angular_v * dt

    motor_1_values.append(motor_1)
    motor_2_values.append(motor_2)
    motor_3_values.append(motor_3)
    motor_4_values.append(motor_4)
    roll_angles.append(roll)
    pitch_angles.append(pitch)
    yaw_angles.append(yaw)
    times.append(i*dt)

plt.close('all')



plt.plot(times, pitch_angles, label="Pitch")
plt.plot(times, roll_angles, label="Roll")
plt.plot(times, yaw_angles, label="Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (degrees)")
plt.grid()
plt.legend()

plt.figure()
plt.plot(times, motor_1_values, label="Motor 1")
plt.plot(times, motor_2_values, label="Motor 2")
plt.plot(times, motor_3_values, label="Motor 3")
plt.plot(times, motor_4_values, label="Motor 4")
plt.xlabel("Time (s)")
plt.ylabel("Motor Output")
plt.title("Motor Thrust Commands")
plt.grid()
plt.legend()

plt.show()
