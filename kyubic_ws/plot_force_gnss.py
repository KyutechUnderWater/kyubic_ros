import matplotlib.pyplot as plt
import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def plot_bag(bag_path, output_png):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        storage_options.storage_id = 'sqlite3'
        reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    topic_force = '/driver/actuator_rp2040_driver/robot_force'
    topic_gnss = '/localization/gnss/odom'
    
    if topic_force not in type_map or topic_gnss not in type_map:
        print("Required topics not found!")
        return
        
    msg_class_force = get_message(type_map[topic_force])
    msg_class_gnss = get_message(type_map[topic_gnss])
    
    storage_filter = rosbag2_py.StorageFilter(topics=[topic_force, topic_gnss])
    reader.set_filter(storage_filter)
    
    force_t, force_x, force_y = [], [], []
    gnss_t, gnss_x, gnss_y = [], [], []
    
    t0 = None
    
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == topic_force:
            msg = deserialize_message(data, msg_class_force)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t0 is None: t0 = ts
            force_t.append(ts - t0)
            force_x.append(msg.wrench.force.x)
            force_y.append(msg.wrench.force.y)
        elif topic == topic_gnss:
            msg = deserialize_message(data, msg_class_gnss)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if t0 is None: t0 = ts
            gnss_t.append(ts - t0)
            gnss_x.append(msg.pose.position.x)
            gnss_y.append(msg.pose.position.y)
            
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    
    ax1.plot(gnss_t, gnss_x, label='GNSS X', color='blue')
    ax1.plot(gnss_t, gnss_y, label='GNSS Y', color='orange')
    ax1.set_ylabel('GNSS Position (m)')
    ax1.set_title('GNSS Position vs Time')
    ax1.grid(True)
    ax1.legend()
    
    ax2.plot(force_t, force_x, label='Force X', color='red', alpha=0.7)
    ax2.plot(force_t, force_y, label='Force Y', color='green', alpha=0.7)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Robot Force (N)')
    ax2.set_title('Robot Force vs Time')
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.savefig(output_png)
    print(f"Plot saved to {output_png}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path')
    parser.add_argument('output_png')
    args = parser.parse_args()
    plot_bag(args.bag_path, args.output_png)
