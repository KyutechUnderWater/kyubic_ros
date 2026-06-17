import csv
import sys
import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def extract_bag(bag_path, topic_name, output_csv):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'  # or 'sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        # Fallback to sqlite3 if mcap fails
        print(f"Failed to open as mcap: {e}. Trying sqlite3...")
        storage_options.storage_id = 'sqlite3'
        reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_map:
        print(f"Topic {topic_name} not found in the bag.")
        print(f"Available topics: {list(type_map.keys())}")
        return
        
    msg_type = type_map[topic_name]
    msg_class = get_message(msg_type)
    
    # Filter by topic
    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)
    
    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'z_depth'])
        
        count = 0
        t0 = None
        while reader.has_next():
            topic, data, t = reader.read_next()
            msg = deserialize_message(data, msg_class)
            ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t0 is None:
                t0 = ts_sec
            rel_ts = ts_sec - t0
            depth = msg.pose.position.z_depth
            writer.writerow([rel_ts, depth])
            count += 1
            
    print(f"Saved {count} rows to {output_csv}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract depth from ros2 bag to CSV')
    parser.add_argument('bag_path', type=str, help='Path to the bag directory')
    parser.add_argument('--topic', type=str, default='/localization/depth_odom', help='Topic name')
    parser.add_argument('--output', type=str, default='depth_data.csv', help='Output CSV file name')
    args = parser.parse_args()
    
    extract_bag(args.bag_path, args.topic, args.output)
