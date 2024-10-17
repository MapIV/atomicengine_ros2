import os
import argparse
from argparse import RawTextHelpFormatter
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.transforms as transforms
from transforms3d import _gohlketransforms as transformations
from rich import print
from rich.table import Table
from rich.console import Console
console = Console()
font = {'size'   : 18}
matplotlib.rc('font', **font)
plt.tight_layout()

parser = argparse.ArgumentParser(
                    prog='CSV Tracks Processor',
                    description='Loads some data and plots either \'pcd\', \'heatmap\', or \'count\'',
                    formatter_class=RawTextHelpFormatter)
parser.add_argument('--input', type=str, required=True, help='input csv file, e.g.: /home/map4/output.csv')
parser.add_argument('--output', type=str, required=False, help='base path for output folder, e.g.: /home/map4/output/')
parser.add_argument('--area', type=str, default=[-1e9, 1e9, -1e9, 1e9], nargs='+', help='area for counting objects')
parser.add_argument('--exclusion_area', type=str, default=[], nargs='+', help='area for excluding objects')
parser.add_argument('--target_id', type=str, default=None, help='generate a velocity profile for a specify target object_id such as \'Car:2\' or \'Bicycle:12\' or \'Pedestrian:6\'')
parser.add_argument('--rate', type=float, help='rate matching the rosbag playback rate, e.g. --rate 0.2')
parser.add_argument('--min_distance', type=float, default=1, help='minimum distance to consider a track valid, e.g. 5 meters')
parser.add_argument('--max_velocity', type=float, default=35, help='maximum velocity to consider a track valid, e.g. 35m/s -> 126 km/h')
parser.add_argument('--velocity_threshold', type=float, default=50, help='velocity safety threshold, e.g. speed limit in km/h')
parser.add_argument('--acceleration_threshold', type=float, default=4, help='acceleration threshold to consider dangerous acceleration m/s^2')
parser.add_argument('--debug', action='store_true', help='show filtering process')
args = parser.parse_args()
input_fp = os.path.dirname(os.path.abspath(args.input))
output_directory = args.output if args.output is not None else input_fp
os.makedirs(output_directory, exist_ok=True)
if args.debug: console.print(f"Output directory : {output_directory} ", style="bold green")

def check_objects(dataframe):
    if len(dataframe) == 0:
        console.print("No objects found in the given area. Please check your input .csv and filtering settings", style="bold red")
        exit(1)

# Load the CSV into a dataframe
df = pd.read_csv(args.input)
# The CSV header looks like this:
# timestamp	sequence_number	lidar_frame_id	lidar_x	lidar_y	lidar_z	object_frame_id	object_id	object_class	x_position	y_position	z_position	x_dimension	y_dimension	z_dimension	quaternion_x	quaternion_y	quaternion_z	quaternion_w	velocity_x	velocity_y	velocity_z

# Counting Area
if len(args.area) % 4 != 0:
    console.print("The detection --area must be a multiple of 4.", style="bold red")
    exit(1)
x_min, x_max, y_min, y_max = float(args.area[0]), float(args.area[1]), float(args.area[2]), float(args.area[3])

# Velocity and calibration parameters
min_distance = args.min_distance
max_velocity = args.max_velocity
dangerous_velocity = args.velocity_threshold*1000/3600  # km/h to m/s
dangerous_acceleration = args.acceleration_threshold

## Object filtering
class_name_map = {
             1: 'Car',
             2: 'Truck',
             3: 'Bus',
             5: 'Bike',
             7: 'Ped'}

# Scale timestamps based on the playback rate
if args.rate is not None:
    df['timestamp'] -= df['timestamp'].iloc[0]
    df['timestamp'] = df['timestamp'].diff().fillna(0) * args.rate
    df['timestamp'] = df['timestamp'].cumsum()

# Group by object_id and find the mode of the object_class for each group
object_class_mode = df.groupby('object_id')['object_class'].apply(lambda x: x.mode().iloc[0]).reset_index()
object_class_mode.columns = ['object_id', 'most_common_class']

# Merge the mode results back to the main dataframe
df = pd.merge(df, object_class_mode, on='object_id', how='left')

# Update the object_class for each track based on the mode
df['object_class'] = df['most_common_class']
df.drop(columns=['most_common_class'], inplace=True)

# Position filtering
if args.debug: console.print(f"Filtering by detection --area, フィルタリングの前: {len(df)} 個", style='bold yellow')
df = df[(df['x_position'] >= x_min) & (df['x_position'] <= x_max) & (df['y_position'] >= y_min) & (df['y_position'] <= y_max)]
if args.debug: console.print(f"Filtered by detection --area, フィルタリングの後: {len(df)} 個", style='bold orange1')
check_objects(df)

# Area masks, remove objects inside this zone
if len(args.exclusion_area) % 4 != 0:
    console.print("The --exclusion_area list must be a multiple of 4.", style="bold red")
    exit(1)

# Apply exclusion area filtering
if args.debug: console.print(f"Filtering by --exclusion_area, フィルタリングの前: {len(df)} 個", style='bold yellow')
for i in range(0, len(args.exclusion_area), 4):
    x_min, x_max, y_min, y_max = map(float, args.exclusion_area[i:i+4])
    df = df[~((df['x_position'] >= x_min) & (df['x_position'] <= x_max) & (df['y_position'] >= y_min) & (df['y_position'] <= y_max))]
if args.debug: console.print(f"Filtered by --exclusion_area, フィルタリングの後: {len(df)} 個", style='bold orange1')
check_objects(df)

# Relabel class_id 3 (bus) to car.
# df.loc[df['object_class'] == 3, 'object_class'] = 1  # Assuming class_id 1 is car

# Adjusting labels based on dimentions
min_length = 0.2  
min_width = 0.2  
df = df[~((df['x_dimension'] < min_length) & (df['y_dimension'] < min_width))]
max_bike_or_ped_length = 2.0  
max_bike_or_ped_width = 2.0  
big_bike_or_ped_condition = (
    ((df['object_class'] == 5) | (df['object_class'] == 7)) &  
    ((df['x_dimension'] > max_bike_or_ped_length) | (df['y_dimension'] > max_bike_or_ped_width))
)
df.loc[big_bike_or_ped_condition, 'object_class'] = 1  #

# Filter by track length
def filter_tracks_by_span(track_df, distance_threshold=min_distance):
    x_span = track_df['x_position'].max() - track_df['x_position'].min()
    y_span = track_df['y_position'].max() - track_df['y_position'].min()
    track_length = np.sqrt(x_span**2 + y_span**2)
    return track_length >= distance_threshold

if args.debug: console.print(f"Filtering by track length --min_distance, フィルタリングの前: {len(df)} 個", style='bold yellow')
filtered_tracks = df.groupby('object_id').filter(lambda x: filter_tracks_by_span(x))
if args.debug: console.print(f"Filtered by track length --min_distance, フィルタリングの後: {len(filtered_tracks)} 個", style='bold orange1')
check_objects(filtered_tracks)


# filtering by velocity
def filter_tracks_by_velocity(track_df, velocity_threshold=max_velocity):
    return (track_df['velocity_x'].abs().max() <= velocity_threshold and track_df['velocity_y'].abs().max() <= velocity_threshold)


if args.debug: console.print(f"Filtering by max velocity, フィルタリングの前: {len(filtered_tracks)} 個", style='bold yellow')
filtered_tracks = filtered_tracks.groupby('object_id').filter(lambda x: filter_tracks_by_velocity(x))
if args.debug: console.print(f"Filtered by max velocity, フィルタリングの後: {len(filtered_tracks)} 個", style='bold orange1')
check_objects(filtered_tracks)

# ordered timestamps
timestamps_ordered = sorted(filtered_tracks['timestamp'].unique())

# get uuid to readable
uuid_to_readable = {}
uuid_to_color = {}
for class_id, name in class_name_map.items():
    # Sort the UUIDs based on the first occurrence in the dataframe
    class_uuids = filtered_tracks[filtered_tracks['object_class'] == class_id]
    ordered_uuids = class_uuids.drop_duplicates(subset="object_id", keep="first").sort_values(by="timestamp")["object_id"].values
    for idx, uuid in enumerate(ordered_uuids, start=1):
        uuid_to_readable[uuid] = f"{name}:{idx}"
        uuid_to_color[uuid] = list(np.random.choice(range(256), size=3)/255)

# class count
unique_uuid_per_class = filtered_tracks.groupby('object_class')['object_id'].nunique()

# check is there's 0 objects
if len(unique_uuid_per_class) == 0:
    console.print("No objects found in the given area. Please check your input .csv and filtering settings", style="bold red")
    exit(1)

# Results report
def calculate_object_metrics(group):
    # Calculate various metrics for the object
    uuid = group['object_id'].iloc[0]
    object_class = class_name_map[group['object_class'].iloc[0]]
    count = len(group)
    lifetime = group['timestamp'].max() - group['timestamp'].min()
    length = ((group['x_position'].max() - group['x_position'].min())**2 + (group['y_position'].max() - group['y_position'].min())**2)**0.5
    average_velocity = group[['velocity_x', 'velocity_y']].abs().mean().mean()
    max_velocity = group[['velocity_x', 'velocity_y']].abs().max().max()
    max_acceleration = group[['velocity_x', 'velocity_y']].diff().abs().max().max()  # This is a simple approximation

    return pd.Series([uuid, object_class, count, lifetime, length, average_velocity, max_velocity, max_acceleration],
                     index=['UUID', 'Class', 'Matching Count', 'Lifetime', 'Track Length', 'Average Velocity', 'Max Velocity', 'Max Acceleration'])

# Apply the function to each group and export to CSV
trajectory_metrics = filtered_tracks.groupby('object_id').apply(calculate_object_metrics)
trajectory_metrics.to_csv(os.path.join(output_directory, 'trajectory_metrics.csv'), index=False)

# Summary report
table = Table(title="\n Summary Report")

table.add_column("Metric", justify="left", style="bold", no_wrap=True)
table.add_column("Value", style="cyan")

class_counts = trajectory_metrics['Class'].value_counts()
class_counts_str = '\n'.join([f"{cls}: {count}" for cls, count in class_counts.items()])
table.add_row("Number of objects per class", class_counts_str)
table.add_row("Average track length", f"{trajectory_metrics['Track Length'].mean():.2f} meters")
table.add_row("Average lifetime", f"{trajectory_metrics['Lifetime'].mean():.2f} seconds")
table.add_row(f"Dangerous objects (Velocity > {args.velocity_threshold} km/h)", str(sum(trajectory_metrics['Max Velocity'] > dangerous_velocity)))
table.add_row(f"Dangerous objects (Acceleration > {args.acceleration_threshold} m/s^2)", str(sum(trajectory_metrics['Max Acceleration'] > args.acceleration_threshold)))
table.add_row("Length of the dataset", f"{(filtered_tracks['timestamp'].max()-filtered_tracks['timestamp'].min()):.2f} seconds")
console.print(table)

# Class count
class_counts = trajectory_metrics['Class'].value_counts()
classes = class_counts.index
counts = class_counts.values

# Generate a color for each class
colors = plt.cm.viridis(np.linspace(0, 1, len(classes)))

# Create the bar chart
plt.figure(figsize=(10, 6))
for i, cls in enumerate(classes):
    plt.bar(cls, counts[i], color=colors[i])

plt.xlabel('Object Class')
plt.ylabel('Count')
plt.title('Object Counts by Class')
plt.xticks(rotation=45)

# Save the plot
output_file_path = os.path.join(output_directory, 'object_counts.png')
plt.savefig(output_file_path)
plt.close()

# Velocity profile
def plot_velocity_profile(uuid):
    track = filtered_tracks[filtered_tracks['object_id'] == uuid]
    if track.empty:
        print(f"No track found for UUID: {uuid}")
        return
    plt.figure(figsize=(10, 6))
    plt.plot(track['timestamp']-track['timestamp'][0], np.sqrt(track['velocity_x']**2 + track['velocity_y']**2)*3600/1000, label='Velocity')
    plt.axhline(y=args.velocity_threshold, color='b', linestyle='-.', label='Velocity Threshold')
    plt.xlabel('Relative timestamp (seconds)')
    plt.ylabel('Velocity (km/h))')
    plt.title(f'Velocity Profile for {uuid}')
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_directory, f'velocity/{uuid}.png'))
    plt.close()

if args.target_id is not None: 
    os.makedirs(os.path.join(output_directory, 'velocity'), exist_ok=True)
    if isinstance(args.target_id, list):
        for uuid in args.target_id:
            plot_velocity_profile(uuid)
    elif args.target_id == 'all':
        for uuid in uuid_to_readable.keys():
            plot_velocity_profile(uuid)
    else:
        plot_velocity_profile(args.target_id)
    