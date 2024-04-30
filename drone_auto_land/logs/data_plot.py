import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the lists of CSV filenames and starting timestamps
local_position_files = [
    "log_5_2024-4-24-16-40-54_vehicle_local_position_0.csv",
    "log_7_2024-4-24-16-55-02_vehicle_local_position_0.csv",
    "log_8_2024-4-24-17-13-58_vehicle_local_position_0.csv",
    "log_10_2024-4-24-17-29-24_vehicle_local_position_0.csv",
    "log_11_2024-4-24-17-45-44_vehicle_local_position_0.csv"
]

trajectory_setpoint_files = [
    "log_5_2024-4-24-16-40-54_trajectory_setpoint_0.csv",
    "log_7_2024-4-24-16-55-02_trajectory_setpoint_0.csv",
    "log_8_2024-4-24-17-13-58_trajectory_setpoint_0.csv",
    "log_10_2024-4-24-17-29-24_trajectory_setpoint_0.csv",
    "log_11_2024-4-24-17-45-44_trajectory_setpoint_0.csv"
]

starting_timestamps = [1130876623, 2001347158, 476337047, 226758392, 333733449]

# Create a new figure
fig = plt.figure(figsize=(15, 10))

# Iterate over each pair of CSV files and starting timestamp
for i, (local_file, setpoint_file, start_timestamp) in enumerate(zip(local_position_files, trajectory_setpoint_files, starting_timestamps), 1):
    # Read local position CSV file into a DataFrame
    df_local = pd.read_csv(local_file)
    
    # Read trajectory setpoint CSV file into a DataFrame
    df_setpoint = pd.read_csv(setpoint_file)
    
    # Filter out rows with timestamps greater than or equal to the starting timestamp
    df_local_filtered = df_local[df_local['timestamp'] >= start_timestamp]
    df_setpoint_filtered = df_setpoint[df_setpoint['timestamp'] >= start_timestamp]
    
    # Convert DataFrame columns to NumPy arrays
    x_vehicle = df_local_filtered['x'].to_numpy()
    y_vehicle = df_local_filtered['y'].to_numpy()
    z_vehicle = df_local_filtered['z'].to_numpy()

    x_setpoint = df_setpoint_filtered['position[0]'].to_numpy()
    y_setpoint = df_setpoint_filtered['position[1]'].to_numpy()
    z_setpoint = df_setpoint_filtered['position[2]'].to_numpy()

    # Create subplot
    ax = fig.add_subplot(2, 3, i, projection='3d')

    # Plot x, y, and z from the local position file
    ax.plot(x_vehicle, y_vehicle, z_vehicle, label='Vehicle Position', color='blue')

    # Plot x, y, and z from the trajectory setpoint file
    ax.scatter(x_setpoint, y_setpoint, z_setpoint, label='Setpoint', color='red', marker='o')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Comparison {i}: Vehicle Position vs Setpoint')
    ax.legend()

    # Set equal aspect ratio
    ax.set_box_aspect([np.ptp(x_vehicle), np.ptp(y_vehicle), np.ptp(z_vehicle)])

    # Invert z-axis
    ax.invert_zaxis()


# Adjust layout
plt.tight_layout()

# Show plot
plt.show()
