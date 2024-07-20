import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime
import pytz

# Define the log directory 
log_dir = '2024_6_27_12_38_13/'

# Extract year, month, day, hour, minute, and second from log directory
log_components = log_dir.rstrip('/').split('_')
year_str = log_components[0]
month_str = log_components[1]
day_str = log_components[2]
hour_str = log_components[3]
minute_str = log_components[4]
second_str = log_components[5]

# Construct start_timestamp at 12:43:00 in your local time zone (GMT+01:00 with DST)
start_timestamp_local = datetime.strptime(f"{year_str}-{month_str}-{day_str} {hour_str}:{minute_str}:{second_str}", "%Y-%m-%d %H:%M:%S")
start_timezone = pytz.timezone('Europe/London')  # Adjust to your specific timezone
start_timestamp_local = start_timezone.localize(start_timestamp_local)

print(f"Start timestamp in local time zone: {start_timestamp_local}")

# Define the list of CSV filenames
current_land_state_file = log_dir + "current_land_state.csv"
velocity_setpoint_file = log_dir + "trajectory_setpoint.csv"
vehicle_odometry_file = log_dir + "vehicle_odometry.csv"

# Function to check if files exist
def check_file(file_path):
    if not os.path.isfile(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")

# Check all files
check_file(current_land_state_file)
check_file(velocity_setpoint_file)
check_file(vehicle_odometry_file)

# Read the CSV files into DataFrames
df_land_state = pd.read_csv(current_land_state_file)
df_velocity_setpoint = pd.read_csv(velocity_setpoint_file)
df_vehicle_odometry = pd.read_csv(vehicle_odometry_file)

# Extract relevant columns from vehicle_odometry and convert to numpy arrays
df_vehicle_odometry = df_vehicle_odometry[['timestamp', 'velocity_x', 'velocity_y', 'velocity_z']]
df_vehicle_odometry['timestamp'] = pd.to_datetime(df_vehicle_odometry['timestamp'], unit='us', utc=True)
timestamps_vehicle_odometry = df_vehicle_odometry['timestamp'].dt.tz_convert(start_timezone).to_numpy()
velocities_vehicle_odometry = df_vehicle_odometry[['velocity_x', 'velocity_y', 'velocity_z']].values

# Extract relevant columns from velocity_setpoint and convert to numpy arrays
df_velocity_setpoint = df_velocity_setpoint[['timestamp', 'velocity_x', 'velocity_y', 'velocity_z']]
df_velocity_setpoint['timestamp'] = pd.to_datetime(df_velocity_setpoint['timestamp'], unit='us', utc=True)
timestamps_velocity_setpoint = df_velocity_setpoint['timestamp'].dt.tz_convert(start_timezone).to_numpy()
velocities_velocity_setpoint = df_velocity_setpoint[['velocity_x', 'velocity_y', 'velocity_z']].values

# Extract relevant columns from land_state and convert to numpy arrays
df_land_state = df_land_state[['timestamp', 'state']]
df_land_state['timestamp'] = pd.to_datetime(df_land_state['timestamp'], unit='us', utc=True)
timestamps_land_state = df_land_state['timestamp'].dt.tz_convert(start_timezone).to_numpy()
states_land_state = df_land_state['state'].values

# Create the plot
fig, ax = plt.subplots(3, 1, figsize=(15, 10), sharex=True)

# Define the state colors
state_colors = {
    'Correction': 'yellow',
    'Descent': 'blue',
    'Landing': 'orange',
    'Landed': 'green'
}

# Add background colors for each state
for state in df_land_state['state'].unique():
    state_df = df_land_state[df_land_state['state'] == state]
    for _, row in state_df.iterrows():
        for axis in ax:
            axis.axvspan(row['timestamp'], row['timestamp'], color=state_colors[state], alpha=0.1)

# Plot velocity setpoint
ax[0].plot(timestamps_velocity_setpoint, velocities_velocity_setpoint[:, 0], label='Velocity Setpoint X', color='r')
ax[1].plot(timestamps_velocity_setpoint, velocities_velocity_setpoint[:, 1], label='Velocity Setpoint Y', color='g')
ax[2].plot(timestamps_velocity_setpoint, velocities_velocity_setpoint[:, 2], label='Velocity Setpoint Z', color='b')

# Plot vehicle odometry
ax[0].plot(timestamps_vehicle_odometry, velocities_vehicle_odometry[:, 0], label='Velocity Odometry X', color='r', linestyle='dashed')
ax[1].plot(timestamps_vehicle_odometry, velocities_vehicle_odometry[:, 1], label='Velocity Odometry Y', color='g', linestyle='dashed')
ax[2].plot(timestamps_vehicle_odometry, velocities_vehicle_odometry[:, 2], label='Velocity Odometry Z', color='b', linestyle='dashed')

# Add labels and legends
for i in range(3):
    ax[i].set_ylabel('Velocity (m/s)')
    ax[i].legend()

# Format the x-axis to show dates in local time zone
ax[2].xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m-%d %H:%M:%S', tz=start_timezone))
ax[2].set_xlabel('Timestamp (s)')

# Set the x-axis limits to start from the specified start_timestamp
end_timestamp = max(timestamps_velocity_setpoint.max(), timestamps_vehicle_odometry.max())
print(f"End timestamp: {end_timestamp}")
for axis in ax:
    axis.set_xlim(start_timestamp_local, end_timestamp)

# Rotate the x-axis labels for better readability
plt.setp(ax[2].xaxis.get_majorticklabels(), rotation=25, ha='right')

# Create custom legend with state labels and colors
legend_handles = []
for state, color in state_colors.items():
    legend_handles.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=color, markersize=10, label=state))

fig.legend(handles=legend_handles, loc='lower center', bbox_to_anchor=(0.5, -0.02), ncol=len(state_colors), fancybox=True, shadow=True)


plt.tight_layout()
plt.show()
