import numpy as np
import heapq
from itertools import product
import matplotlib.pyplot as plt

np.random.seed(7)
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D



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

#Moments of inertia

Ixx = 0.04                     # kg*m^2
Iyy = 0.04                    # kg*m^2
Izz = 0.08                   # kg*m^2

yaw_torque_coefficient = 0.1
dt = 0.01


# Linear drag coefficients

drag_x = 0.25
drag_y = 0.25
drag_z = 0.15

# Angular damping coefficients

angular_damping_roll = 0.04
angular_damping_pitch = 0.04
angular_damping_yaw = 0.06

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
    (0, 0, 4),
    (10, 0, 6),
    (10, 10, 8),
    (0, 10, 10),
    (0, 0, 12)
]

Obstacles = [
    (5, 0, 5),
    (10, 5, 7),
    (5, 10, 9),
    (0, 5, 11)
]

# Physical obstacle radius and extra planning buffer.
obstacle_radius = 1.5
avoidance_margin = 1.5
safety_radius = obstacle_radius + avoidance_margin

# A* grid settings.
grid_resolution = 1.0
vertical_cost_weight = 1.2


def point_to_segment_distance(point, segment_start, segment_end):
    """Shortest 3D distance from a point to a finite line segment."""
    point = np.asarray(point, dtype=float)
    segment_start = np.asarray(segment_start, dtype=float)
    segment_end = np.asarray(segment_end, dtype=float)

    segment = segment_end - segment_start
    segment_length_squared = np.dot(segment, segment)

    if segment_length_squared < 1e-12:
        return np.linalg.norm(point - segment_start)

    projection = np.dot(point - segment_start, segment) / segment_length_squared
    projection = np.clip(projection, 0.0, 1.0)
    closest_point = segment_start + projection * segment

    return np.linalg.norm(point - closest_point)


def segment_is_clear(segment_start, segment_end, clearance):
    """True only when the complete segment clears every obstacle."""
    for obstacle in Obstacles:
        if point_to_segment_distance(
            obstacle,
            segment_start,
            segment_end
        ) < clearance:
            return False

    return True


def plan_safe_path(start_position, goal_position):
    """Plan and smooth a collision-free 3D path using A*."""
    start_position = np.asarray(start_position, dtype=float)
    goal_position = np.asarray(goal_position, dtype=float)

    # Do not use A* when the direct route is already safe.
    if segment_is_clear(start_position, goal_position, safety_radius):
        return [tuple(goal_position)]

    obstacle_array = np.asarray(Obstacles, dtype=float)

    for obstacle in obstacle_array:
        if np.linalg.norm(goal_position - obstacle) < safety_radius:
            raise ValueError(
                "A mission waypoint is inside an obstacle safety radius."
            )

    all_points = np.vstack((start_position, goal_position, obstacle_array))

    padding = safety_radius + 2 * grid_resolution
    lower_bound = np.floor(
        (np.min(all_points, axis=0) - padding) / grid_resolution
    ) * grid_resolution
    upper_bound = np.ceil(
        (np.max(all_points, axis=0) + padding) / grid_resolution
    ) * grid_resolution

    # The drone cannot plan below the ground.
    lower_bound[2] = max(0.0, lower_bound[2])

    grid_shape = (
        np.round((upper_bound - lower_bound) / grid_resolution)
        .astype(int)
        + 1
    )

    def position_to_index(position):
        return tuple(
            np.round((position - lower_bound) / grid_resolution)
            .astype(int)
        )

    def index_to_position(index):
        return lower_bound + grid_resolution * np.asarray(index, dtype=float)

    start_index = position_to_index(start_position)
    goal_index = position_to_index(goal_position)

    blocked_cache = {}

    def index_is_blocked(index):
        if index == start_index or index == goal_index:
            return False

        if index in blocked_cache:
            return blocked_cache[index]

        position = index_to_position(index)

        blocked = position[2] < 0.0

        if not blocked:
            for obstacle in Obstacles:
                if np.linalg.norm(position - np.asarray(obstacle)) < safety_radius:
                    blocked = True
                    break

        blocked_cache[index] = blocked
        return blocked

    neighbor_steps = []

    for offset in product((-1, 0, 1), repeat=3):
        if offset == (0, 0, 0):
            continue

        horizontal_distance = np.sqrt(offset[0] ** 2 + offset[1] ** 2)
        vertical_distance = abs(offset[2])

        movement_cost = grid_resolution * np.sqrt(
            horizontal_distance ** 2
            + (vertical_cost_weight * vertical_distance) ** 2
        )

        neighbor_steps.append((offset, movement_cost))

    open_set = []
    start_heuristic = np.linalg.norm(goal_position - start_position)
    heapq.heappush(open_set, (start_heuristic, 0.0, start_index))

    came_from = {}
    cost_from_start = {start_index: 0.0}
    closed_set = set()

    while open_set:
        _, current_cost, current_index = heapq.heappop(open_set)

        if current_index in closed_set:
            continue

        if current_index == goal_index:
            break

        closed_set.add(current_index)
        current_position = index_to_position(current_index)

        for offset, movement_cost in neighbor_steps:
            neighbor_index = tuple(
                current_index[axis] + offset[axis]
                for axis in range(3)
            )

            if any(
                neighbor_index[axis] < 0
                or neighbor_index[axis] >= grid_shape[axis]
                for axis in range(3)
            ):
                continue

            if index_is_blocked(neighbor_index):
                continue

            neighbor_position = index_to_position(neighbor_index)

            # Node checks alone are not enough: a diagonal edge can cut
            # through an obstacle even if both endpoints are outside it.
            if not segment_is_clear(
                current_position,
                neighbor_position,
                safety_radius
            ):
                continue

            new_cost = current_cost + movement_cost

            if new_cost >= cost_from_start.get(neighbor_index, np.inf):
                continue

            cost_from_start[neighbor_index] = new_cost
            came_from[neighbor_index] = current_index

            displacement = goal_position - neighbor_position
            heuristic = np.sqrt(
                displacement[0] ** 2
                + displacement[1] ** 2
                + (vertical_cost_weight * displacement[2]) ** 2
            )

            heapq.heappush(
                open_set,
                (new_cost + heuristic, new_cost, neighbor_index)
            )

    if goal_index != start_index and goal_index not in came_from:
        raise RuntimeError("A* could not find a collision-free path.")

    grid_path = [goal_index]

    while grid_path[-1] != start_index:
        grid_path.append(came_from[grid_path[-1]])

    grid_path.reverse()

    path_positions = [index_to_position(index) for index in grid_path]
    path_positions[0] = start_position
    path_positions[-1] = goal_position

    # Greedy line-of-sight smoothing removes unnecessary A* grid points
    # while rechecking every resulting segment for collisions.
    smoothed_path = [path_positions[0]]
    current_path_index = 0

    while current_path_index < len(path_positions) - 1:
        safe_next_index = None

        for next_path_index in range(
            len(path_positions) - 1,
            current_path_index,
            -1
        ):
            if segment_is_clear(
                path_positions[current_path_index],
                path_positions[next_path_index],
                safety_radius
            ):
                safe_next_index = next_path_index
                break

        if safe_next_index is None:
            raise RuntimeError(
                "A* path smoothing could not connect two safe points."
            )

        smoothed_path.append(path_positions[safe_next_index])
        current_path_index = safe_next_index

    return [tuple(point) for point in smoothed_path[1:]]


# Mission/path-following state.
waypoint_index = 0
waypoint_tolerance = 0.5
planner_waypoint_tolerance = 0.45
planner_speed_tolerance = 0.8
mission_speed_tolerance = 1.0

path_queue = [waypoints[0]]
planned_route_points = [tuple((x, y, z)), waypoints[0]]

desired_x, desired_y, desired_z = path_queue.pop(0)
current_target_is_mission_waypoint = True


# Integral values


roll_integral = 0
pitch_integral = 0
yaw_integral = 0
z_integral = 0



# PID gains


Kp_angle = 4
Kd_angle = 2
Ki_angle = 0

Kp_position = .03
Kd_position = 0.1

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
minimum_obstacle_distances = [np.inf] * len(Obstacles)




# Simulation loop


for i in range(60000):

    # Disturbances

    if current_target_is_mission_waypoint:
        current_tolerance = waypoint_tolerance
    else:
        current_tolerance = planner_waypoint_tolerance


    
    # Simulated sensor readings
    

    angle_sensor_noise = np.radians(0.3)

    measured_roll = roll + np.random.uniform(
        -angle_sensor_noise,
        angle_sensor_noise
    )

    measured_pitch = pitch + np.random.uniform(
        -angle_sensor_noise,
        angle_sensor_noise
    )

    measured_yaw = yaw + np.random.uniform(
        -angle_sensor_noise,
        angle_sensor_noise
    )
  
    measured_z = z + np.random.uniform(-0.1, 0.1)
    measured_x = x + np.random.uniform(-0.1, 0.1)
    measured_y = y + np.random.uniform(-0.1, 0.1)

    
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
   
    max_tilt_angle = np.radians(12)
    
    desired_pitch = np.clip(
        desired_pitch,
        -max_tilt_angle,
        max_tilt_angle
    )

    desired_roll = np.clip(
        desired_roll,
        -max_tilt_angle,
        max_tilt_angle
    )


    
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
  
  # Angular damping opposes rotation

    roll_torque -= (
        angular_damping_roll
        * roll_angular_velocity
    )

    pitch_torque -= (
        angular_damping_pitch
        * pitch_angular_velocity
    )

    yaw_torque -= (
        angular_damping_yaw
        * yaw_angular_velocity
)


    
    # Angular accelerations
    

    roll_angular_acceleration = roll_torque / Ixx
    pitch_angular_acceleration = pitch_torque / Iyy
    yaw_angular_acceleration = yaw_torque / Izz


    
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

    x_thrust = (
        total_thrust
        * np.sin(pitch)
        * np.cos(roll)
    )

    y_thrust = (
        total_thrust
        * np.sin(roll)
        * np.cos(pitch)
    )

    # Aerodynamic drag caused by relative air velocity

    x_drag_force = (
        -drag_x * x_velocity
    )

    y_drag_force = (
        -drag_y * y_velocity
    )


    # Total horizontal forces

    x_force = x_thrust + x_drag_force
    y_force = y_thrust + y_drag_force


    # Horizontal acceleration and motion

    x_acceleration = x_force / mass
    y_acceleration = y_force / mass

    x_velocity += x_acceleration * dt
    y_velocity += y_acceleration * dt

    x += x_velocity * dt
    y += y_velocity * dt


    # Z-axis physics

    z_thrust = (
        total_thrust
        * np.cos(roll)
        * np.cos(pitch)
    )

    z_drag_force = (
        -drag_z * z_velocity
    )

    z_force = (
        z_thrust
        - weight
        + z_drag_force
    )

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
    
    
    current_speed = np.sqrt(
        x_velocity ** 2
        + y_velocity ** 2
        + z_velocity ** 2
    )

    if current_target_is_mission_waypoint:
        current_speed_tolerance = mission_speed_tolerance
    else:
        current_speed_tolerance = planner_speed_tolerance

    target_reached = (
        distance_to_waypoint < current_tolerance
        and current_speed < current_speed_tolerance
    )

    if target_reached:

        # Continue through the intermediate waypoints returned by A*.
        if path_queue:
            desired_x, desired_y, desired_z = path_queue.pop(0)
            current_target_is_mission_waypoint = len(path_queue) == 0

        # The current mission waypoint is complete. Plan to the next one.
        elif waypoint_index < len(waypoints) - 1:
            waypoint_index += 1

            new_path = plan_safe_path(
                (x, y, z),
                waypoints[waypoint_index]
            )

            planned_route_points.extend(new_path)
            path_queue = list(new_path)

            desired_x, desired_y, desired_z = path_queue.pop(0)
            current_target_is_mission_waypoint = len(path_queue) == 0

        else:
            print("Mission Complete")
            break

    for obstacle_index, obstacle in enumerate(Obstacles):
        obstacle_distance = np.linalg.norm(
            np.asarray((x, y, z)) - np.asarray(obstacle)
        )
        minimum_obstacle_distances[obstacle_index] = min(
            minimum_obstacle_distances[obstacle_index],
            obstacle_distance
        )

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

plt.plot(
    times,
    np.degrees(roll_values),
    label="Roll"
)

plt.plot(
    times,
    np.degrees(pitch_values),
    label="Pitch"
)

plt.plot(
    times,
    np.degrees(yaw_values),
    label="Yaw"
)

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

for obstacle_index, obstacle in enumerate(Obstacles):

    plt.plot(
        obstacle[0],
        obstacle[1],
        marker="x",
        markersize=12,
        label="Obstacle" if obstacle_index == 0 else None
    )

    obstacle_circle = plt.Circle(
        (obstacle[0], obstacle[1]),
        obstacle_radius + avoidance_margin,
        fill=False,
        linestyle="--",
        label="Safety Radius" if obstacle_index == 0 else None
    )

    plt.gca().add_patch(obstacle_circle)

plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Waypoint Navigation")
plt.axis("equal")
plt.grid()
plt.legend()

mission_time = times[-1]

path_length = 0

max_speed = 0

for i in range(1, len(x_values)):
    if i == 0:
        continue
    dx = x_values[i] - x_values[i - 1]
    dy = y_values[i] - y_values[i - 1]
    dz = z_values[i] - z_values[i - 1]

    speed = np.sqrt(
        dx**2 +
        dy**2 +
        dz**2
    ) / dt

    if speed > max_speed:
            max_speed = speed
    
    path_length += np.sqrt(
        dx**2 + dy**2 + dz**2
    )
    

print("Mission time:", mission_time, "seconds")
print("Total path length:", path_length, "meters")
print("Maximum Speed:", max_speed, "m/s")

final_error = np.sqrt(
    (desired_x - x)**2
    + (desired_y - y)**2
    + (desired_z - z)**2
)

print("Final position error:", final_error, "meters")
minimum_distance_report = [
    round(float(distance), 3)
    for distance in minimum_obstacle_distances
]

minimum_physical_clearance = (
    min(minimum_obstacle_distances) - obstacle_radius
)

print("Minimum distance to each obstacle:", minimum_distance_report)
print(
    "Minimum physical clearance:",
    round(float(minimum_physical_clearance), 3),
    "meters"
)

if min(minimum_obstacle_distances) <= obstacle_radius:
    print("Collision detected")
else:
    print("No obstacle collisions")


# 3D Animation Figure

fig = plt.figure()

ax = fig.add_subplot(111, projection="3d")

ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_zlabel("Altitude (m)")

ax.set_title("Drone Flight Animation")


# Plot waypoints in 3D

waypoint_z = [point[2] for point in waypoints]

ax.plot(
    waypoint_x,
    waypoint_y,
    waypoint_z,
    "ro--",
    label="Waypoints"
)

planned_x = [point[0] for point in planned_route_points]
planned_y = [point[1] for point in planned_route_points]
planned_z = [point[2] for point in planned_route_points]

ax.plot(
    planned_x,
    planned_y,
    planned_z,
    linestyle=":",
    label="A* Planned Path"
)

# Fit the 3D axes to the entire mission, including A* detours that may
# temporarily leave the original waypoint square.
axis_x_values = x_values + waypoint_x + planned_x + [o[0] for o in Obstacles]
axis_y_values = y_values + waypoint_y + planned_y + [o[1] for o in Obstacles]
axis_z_values = z_values + waypoint_z + planned_z + [o[2] for o in Obstacles]

axis_padding = 2.0
x_min, x_max = min(axis_x_values) - axis_padding, max(axis_x_values) + axis_padding
y_min, y_max = min(axis_y_values) - axis_padding, max(axis_y_values) + axis_padding
z_min = max(0.0, min(axis_z_values) - axis_padding)
z_max = max(axis_z_values) + axis_padding

ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_zlim(z_min, z_max)
ax.set_box_aspect((x_max - x_min, y_max - y_min, z_max - z_min))

for obstacle_index, obstacle in enumerate(Obstacles):

    ax.scatter(
        obstacle[0],
        obstacle[1],
        obstacle[2],
        marker="x",
        s=100,
        label="Obstacle" if obstacle_index == 0 else None
    )



# Create the drone marker at its initial position

drone_marker, = ax.plot(
    [x_values[0]],
    [y_values[0]],
    [z_values[0]],
    marker="o",
    markersize=8,
    label="Drone"
)

# Create an empty path trail

flight_trail, = ax.plot(
    [],
    [],
    [],
    label="Flight Path"
)

ax.legend()

def update(frame):
    drone_marker.set_data(
        [x_values[frame]],
        [y_values[frame]]
    )

    drone_marker.set_3d_properties(
        [z_values[frame]]
    )

    flight_trail.set_data(
        x_values[:frame + 1],
        y_values[:frame + 1]
    )

    flight_trail.set_3d_properties(
        z_values[:frame + 1]
    )

    return drone_marker, flight_trail

animation = FuncAnimation(
    fig,
    update,
    frames=range(0, len(x_values), 10),
    interval=20
)


plt.show()