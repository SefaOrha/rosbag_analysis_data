import csv
from datetime import datetime
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

import rosbag
bag_name = 'C:/Users/sefa_/PycharmProjects/adastec/rosbag/example_simulation_data_1.bag'
bag = rosbag.Bag(bag_name)
topics = bag.get_type_and_topic_info().topics.keys()
for topic, info in bag.get_type_and_topic_info().topics.items():
    print("{}, {} messages".format(topic, info.message_count))

start_time = bag.get_start_time()
end_time = bag.get_end_time()
duration = end_time - start_time
print("Total duration of the rosbag: {} seconds".format(duration))

csv_name1 = 'control.csv'

csv_name2 = 'pose.csv'

topic1 = '/control/control_cmd'
topic2 = '/current_pose'

mode = 'w'

headers1 = ['timestamp', 'steering_angle', 'steering_angle_velocity', 'velocity', 'acceleration']
headers2 = ['timestamp', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z',
            'orientation_w']

with open(csv_name1, mode) as file1, open(csv_name2, mode) as file2:
    writer1 = csv.writer(file1)
    writer2 = csv.writer(file2)
    writer1.writerow(headers1)
    writer2.writerow(headers2)

prev_x, prev_y, prev_z = None, None, None

total_distance = 0.0

with rosbag.Bag(bag_name) as bag:
    for topic, msg, t in bag.read_messages(topics=[topic1, topic2]):
        timestamp = datetime.fromtimestamp(t.to_sec())
        if topic == topic1:
            row = [timestamp, msg.control.steering_angle, msg.control.steering_angle_velocity,
                   msg.control.velocity, msg.control.acceleration]
            with open(csv_name1, 'a') as file:
                writer = csv.writer(file)
                writer.writerow(row)
        elif topic == topic2:
            row = [timestamp, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                   msg.pose.orientation.x, msg.pose.orientation.y,
                   msg.pose.orientation.z, msg.pose.orientation.w]
            with open(csv_name2, 'a') as file:
                writer = csv.writer(file)
                writer.writerow(row)

            if prev_x is not None and prev_y is not None and prev_z is not None:
                dx = msg.pose.position.x - prev_x
                dy = msg.pose.position.y - prev_y
                dz = msg.pose.position.z - prev_z
                distance = ((dx ** 2) + (dy ** 2) + (dz ** 2)) ** 0.5
                total_distance += distance
                print(f"Mesafe: {distance:.2f} metre")
            prev_x = msg.pose.position.x
            prev_y = msg.pose.position.y
            prev_z = msg.pose.position.z
print(f"Total distance traveled calculated from position data : {total_distance:.2f} metre")


df1 = pd.read_csv(csv_name1)
df2 = pd.read_csv(csv_name2)

distances = np.sqrt(np.sum(np.diff(df2[['position_x', 'position_y', 'position_z']], axis=0)**2, axis=1))
cumulative_distances = np.cumsum(distances)



# Steering Angle
plt.figure(figsize=(8, 5))
plt.plot(df1['timestamp'], df1['steering_angle'])
plt.title('Steering Angle')
plt.xlabel('Time')
plt.ylabel('Angle (rad)')
plt.savefig('steering_angle.png')

# Steering Angle Velocity
plt.figure(figsize=(8, 5))
plt.plot(df1['timestamp'], df1['steering_angle_velocity'])
plt.title('Steering Angle Velocity')
plt.xlabel('Time')
plt.ylabel('Angular Velocity (rad/s)')
plt.savefig('steering_angle_velocity.png')

# Velocity
plt.figure(figsize=(8, 5))
plt.plot(df1['timestamp'], df1['velocity'])
plt.title('Velocity')
plt.xlabel('Time')
plt.ylabel('Velocity (m/s)')
plt.savefig('velocity.png')

# Acceleration
plt.figure(figsize=(8, 5))
plt.plot(df1['timestamp'], df1['acceleration'])
plt.title('Acceleration')
plt.xlabel('Time')
plt.ylabel('Acceleration (m/s^2)')
plt.savefig('acceleration.png')



# Position X
fig, ax = plt.subplots(figsize=(8,6))
ax.plot(df2['timestamp'], df2['position_x'])
ax.set_title('Position X')
ax.set_xlabel('Time')
ax.set_ylabel('Position (m)')
plt.savefig("position_x.png")

# Position Y
fig1, ax = plt.subplots(figsize=(8,6))
ax.plot(df2['timestamp'], df2['position_y'])
ax.set_title('Position Y')
ax.set_xlabel('Time')
ax.set_ylabel('Position (m)')
plt.savefig("position_y.png")

# Position Z
fig2, ax = plt.subplots(figsize=(8,6))
ax.plot(df2['timestamp'], df2['position_z'])
ax.set_title('Position Z')
ax.set_xlabel('Time')
ax.set_ylabel('Position (m)')
plt.savefig("position_z.png")

# Cumulative Distance
fig3, ax = plt.subplots(figsize=(8,6))
ax.plot(df2['timestamp'].iloc[1:], cumulative_distances)
ax.set_title('Cumulative Distance')
ax.set_xlabel('Time')
ax.set_ylabel('Distance (m)')
plt.savefig("calculated_distance_from_position.png")


def calculate_distance(accelerations, velocities, steering_angles, steering_angle_velocities, delta_t):

    distance = 0.0
    prev_velocity = 0.0
    prev_steering_angle = 0.0
    prev_steering_angle_vel = steering_angle_velocities.iloc[0]

    for i in range(1, len(accelerations)):
        avg_acceleration = (accelerations[i] + accelerations[i - 1]) / 2
        avg_velocity = (prev_velocity + ((avg_acceleration * delta_t) / 2) + velocities[i]) / 2
        avg_steering_angle = (prev_steering_angle + steering_angles[i]) / 2
        avg_steering_angle_vel = (steering_angle_velocities[i] + prev_steering_angle_vel) / 2

        displacement = avg_velocity * delta_t

        angle_displacement = avg_velocity * delta_t * math.tan(avg_steering_angle) / 2 / math.sin(math.radians(45))
        additional_displacement = angle_displacement * math.sin(
            math.radians(45 - abs(avg_steering_angle_vel) * delta_t))

        distance += displacement + additional_displacement

        prev_velocity = avg_velocity
        prev_steering_angle = steering_angles[i]
        prev_steering_angle_vel = steering_angle_velocities[i]

    return distance


data = pd.read_csv('C:/Users/sefa_/PycharmProjects/adastec/control.csv')
data['timestamp'] = pd.to_datetime(data['timestamp'])

# Extract relevant columns
timestamps = data['timestamp']
steering_angles = data['steering_angle']
steering_angle_velocities = data['steering_angle_velocity']
velocities = data['velocity']
accelerations = data['acceleration']

delta_t = (timestamps.iloc[-1] - timestamps.iloc[0]).total_seconds() / len(timestamps)

distance = calculate_distance(accelerations, velocities, steering_angles,steering_angle_velocities, delta_t)

print(f' Total distance traveled calculated from control data: {distance:.2f} meters')

# Calculate cumulative distances at each time step
cumulative_distances = []
prev_distance = 0.0
prev_velocity = 0.0

for i in range(len(velocities)):
    avg_velocity = (velocities[i] + prev_velocity) / 2
    cumulative_distance = prev_distance + avg_velocity * delta_t
    cumulative_distances.append(cumulative_distance)
    prev_distance = cumulative_distance
    prev_velocity = velocities[i]

# Plot cumulative distances
plt.figure(figsize=(12, 6))
plt.plot(timestamps, cumulative_distances)
plt.xlabel('Time')
plt.ylabel('Distance (m)')
plt.title('Cumulative Distance Travelled Over Time')
plt.savefig("calculated_distance_from_control.png")


