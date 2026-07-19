import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
import os

bag_path = 'rosbag2_2026_05_22-15_34_37'
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

target_topic = '/planner/wrench_planner/goal_current_odom'
if target_topic not in type_map:
    print(f"Topic {target_topic} not found in bag.")
    exit(1)
    
msg_type = get_message(type_map[target_topic])

times = []
targets_x = []
targets_y = []
targets_yaw = []
master_x = []
master_y = []
master_yaw = []

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == target_topic:
        msg = deserialize_message(data, msg_type)
        times.append(t)
        targets_x.append(msg.targets.x)
        targets_y.append(msg.targets.y)
        targets_yaw.append(msg.targets.yaw)
        master_x.append(msg.master.x)
        master_y.append(msg.master.y)
        master_yaw.append(msg.master.yaw)

if len(times) == 0:
    print("No messages found.")
    exit(1)

times = np.array(times)
times = (times - times[0]) / 1e9 # seconds

targets_x = np.array(targets_x)
targets_y = np.array(targets_y)
targets_yaw = np.array(targets_yaw)
master_x = np.array(master_x)
master_y = np.array(master_y)
master_yaw = np.array(master_yaw)

# calculate yaw error properly considering wrap-around
yaw_error = targets_yaw - master_yaw
yaw_error = (yaw_error + 180) % 360 - 180

plt.figure(figsize=(15, 12))

plt.subplot(3, 1, 1)
plt.plot(times, targets_x, label='Target X', linestyle='--', color='blue')
plt.plot(times, master_x, label='Master X', color='cyan')
plt.plot(times, targets_x - master_x, label='Error X', color='red', alpha=0.6)
plt.axhline(0, color='black', linewidth=0.5)
plt.legend()
plt.ylabel('X [m]')
plt.title('WrenchPlan: Target vs Master Position (X/Y) and Yaw')
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(times, targets_y, label='Target Y', linestyle='--', color='green')
plt.plot(times, master_y, label='Master Y', color='lime')
plt.plot(times, targets_y - master_y, label='Error Y', color='red', alpha=0.6)
plt.axhline(0, color='black', linewidth=0.5)
plt.legend()
plt.ylabel('Y [m]')
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(times, targets_yaw, label='Target Yaw', linestyle='--', color='purple')
plt.plot(times, master_yaw, label='Master Yaw', color='magenta')
plt.plot(times, yaw_error, label='Error Yaw', color='red', alpha=0.6)
plt.axhline(0, color='black', linewidth=0.5)
plt.legend()
plt.ylabel('Yaw [deg]')
plt.xlabel('Time [s]')
plt.grid(True)

plt.tight_layout()
output_path = os.path.join(os.path.dirname(__file__), 'bag_error_plot.png')
os.makedirs(os.path.dirname(output_path), exist_ok=True)
plt.savefig(output_path)
print(f'Plot saved as {output_path}')
