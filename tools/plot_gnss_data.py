import csv
import matplotlib.pyplot as plt
import argparse

def plot_gnss(csv_path, output_png):
    latitudes = []
    longitudes = []

    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        next(reader) # skip header
        for row in reader:
            lat = float(row[1])
            lon = float(row[2])
            # filter out exactly 0.0 values which mean no fix
            if lat != 0.0 and lon != 0.0:
                latitudes.append(lat)
                longitudes.append(lon)

    if not latitudes:
        print("No valid GNSS data to plot.")
        return

    plt.figure(figsize=(10, 8))
    plt.plot(longitudes, latitudes, marker='o', markersize=2, linestyle='-', color='blue', alpha=0.6)
    
    # Mark start and end
    plt.plot(longitudes[0], latitudes[0], marker='s', color='green', markersize=8, label='Start')
    plt.plot(longitudes[-1], latitudes[-1], marker='x', color='red', markersize=8, label='End')
    
    # Format axes so they have equal scaling, converting degrees appropriately would be better but this is a rough plot
    # To prevent severe stretching, we can set aspect ratio. At latitude 34, cos(34) ~ 0.83
    # A quick approximation:
    import math
    mean_lat = sum(latitudes)/len(latitudes)
    plt.gca().set_aspect(1.0 / math.cos(math.radians(mean_lat)))

    plt.xlabel('Longitude (degrees)')
    plt.ylabel('Latitude (degrees)')
    plt.title('GNSS Trajectory (Longitude vs Latitude)')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(output_png, dpi=150)
    print(f"Plot saved to {output_png}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='/home/ros/kyubic_ros/tools/gnss_data.csv')
    parser.add_argument('--output', default='/home/ros/kyubic_ros/tools/gnss_trajectory_plot.png')
    args = parser.parse_args()
    plot_gnss(args.input, args.output)
