import numpy as np 
import matplotlib.pyplot as plt 

# Theoretical Values 

angle = 20
angular_v = 0
desired_angle = 0

inertia = 2
dt = .01
integral = 0

Kp = 2  # How aggresive the change will be
Kd = 2
Ki= .05

# Lists for what curent angle and time are

angles = []
times = []

# Actual PID controller

for i in range(7000):

# Distrubances to see how system reacts to them

    if i == 500:
      angle = angle + np.random.normal(-15, 15)

    if i == 1500:
        angle = angle + np.random.normal(-15, 15)

    if i == 3000:
        angle = angle + np.random.normal(-15, 15)

    measured_angle = angle + np.random.normal(0, 0.3) #Less precise instrument
    error = desired_angle - measured_angle


    integral += error * dt

    torque = Kp*error - Kd*angular_v + Ki * integral

    torque = np.clip(torque, -10, 10) #Limits torque to something more realistic


    angular_a = torque/inertia

    angular_v += angular_a * dt
    angle += angular_v *dt

    angles.append(angle)
    times.append(i*dt)

plt.close('all')

plt.plot(times, angles)
plt.xlabel("Time (s)")
plt.ylabel("Angle (degrees)")
plt.grid()
plt.show()

