import csv
import matplotlib.pyplot as plt

timestamps = []
depths = []

with open('/home/ros/kyubic_ros/tools/depth_data.csv', 'r') as f:
    reader = csv.reader(f)
    next(reader) # skip header
    for row in reader:
        timestamps.append(float(row[0]))
        depths.append(float(row[1]))

# Make time relative to start
t0 = timestamps[0]
time_rel = [t - t0 for t in timestamps]

plt.figure(figsize=(10, 6))
plt.plot(time_rel, depths, label='Z Depth', color='blue')
plt.xlabel('Time (seconds)')
plt.ylabel('Depth (meters)')
plt.title('Depth vs Time')
plt.grid(True)
plt.legend()
# Invert Y axis so downward is positive
plt.gca().invert_yaxis()

plt.savefig('/home/ros/kyubic_ros/tools/depth_plot.png')
print("Plot saved to depth_plot.png")
