import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import numpy as np

bag_path = 'rosbag2_2026_05_22-14_29_34'
storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options)
topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

target_topic = '/rosout'
if target_topic not in type_map:
    print(f"Topic {target_topic} not found in bag.")
    exit(1)
    
msg_type = get_message(type_map[target_topic])

print("==========================================================")
print("  Extracting CSV path from rosout ")
print("==========================================================")

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == target_topic:
        msg = deserialize_message(data, msg_type)
        if "CSV path" in msg.msg:
            print(f"[{msg.name}]: {msg.msg}")
            break


