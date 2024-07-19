import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the log directory 
log_dir = '2024_7_19_10_24_4/'
# Define the list of CSV filenames
aruco_pose_drone_file = log_dir + "aruco_pose_drone.csv"
aruco_pose_local_file = log_dir + "aruco_pose_drone.csv"
vehicle_odometry_file = log_dir + "vehicle_odometry.csv"

# Create a new figure with two subplots
fig = plt.figure(figsize=(15, 8))

# Subplot for Aruco pose drone data
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title('Aruco Pose Drone')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# Read Aruco pose drone CSV file into a DataFrame
df_aruco_drone = pd.read_csv(aruco_pose_drone_file)
x_aruco_drone = df_aruco_drone['position_x'].to_numpy()
y_aruco_drone = df_aruco_drone['position_y'].to_numpy()
z_aruco_drone = df_aruco_drone['position_z'].to_numpy()
ax1.scatter(x_aruco_drone, y_aruco_drone, z_aruco_drone, color='r', s=10, label='Aruco Pose Drone')

# Connect the points with lines in subplot 1
ax1.plot(x_aruco_drone, y_aruco_drone, z_aruco_drone, color='r', linewidth=0.5, label='_nolegend_')

# Highlight first and last points in subplot 1
ax1.scatter(x_aruco_drone[0], y_aruco_drone[0], z_aruco_drone[0], color='black', s=50, label='First Point Drone')
ax1.scatter(x_aruco_drone[-1], y_aruco_drone[-1], z_aruco_drone[-1], color='orange', s=50, label='Last Point Drone')

# Set equal aspect ratio for subplot 1
ax1.set_box_aspect([max(x_aruco_drone)-min(x_aruco_drone), max(y_aruco_drone)-min(y_aruco_drone), max(z_aruco_drone)-min(z_aruco_drone)])

ax1.legend()

# Subplot for Aruco pose local and vehicle odometry data
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_title('Aruco Pose Local & Vehicle Odometry')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

# Read Aruco pose local CSV file into a DataFrame
df_aruco_local = pd.read_csv(aruco_pose_local_file)
x_aruco_local = df_aruco_local['position_x'].to_numpy()
y_aruco_local = df_aruco_local['position_y'].to_numpy()
z_aruco_local = df_aruco_local['position_z'].to_numpy()
ax2.scatter(x_aruco_local, y_aruco_local, z_aruco_local, color='g', s=5, label='Aruco Pose Local')

# Read vehicle odometry CSV file into a DataFrame
df_vehicle_odometry = pd.read_csv(vehicle_odometry_file)
x_vehicle_odometry = df_vehicle_odometry['position_x'].to_numpy()
y_vehicle_odometry = df_vehicle_odometry['position_y'].to_numpy()
z_vehicle_odometry = df_vehicle_odometry['position_z'].to_numpy()
ax2.scatter(x_vehicle_odometry, y_vehicle_odometry, z_vehicle_odometry, color='b', s=5, label='Vehicle Odometry')
ax2.invert_zaxis()

# Highlight first and last points in subplot 2
ax2.scatter(x_aruco_local[0], y_aruco_local[0], z_aruco_local[0], color='black', s=50, label='First Point Aruco Local')
ax2.scatter(x_aruco_local[-1], y_aruco_local[-1], z_aruco_local[-1], color='orange', s=50, label='Last Point Aruco Local')
ax2.scatter(x_vehicle_odometry[0], y_vehicle_odometry[0], z_vehicle_odometry[0], color='brown', s=50, label='First Point Odometry')
ax2.scatter(x_vehicle_odometry[-1], y_vehicle_odometry[-1], z_vehicle_odometry[-1], color='pink', s=50, label='Last Point Odometry')

# Connect points with lines in subplot 2
ax2.plot(x_aruco_local, y_aruco_local, z_aruco_local, color='g', linewidth=0.5, label='_nolegend_')
ax2.plot(x_vehicle_odometry, y_vehicle_odometry, z_vehicle_odometry, color='b', linewidth=0.5, label='_nolegend_')

# Set equal aspect ratio for subplot 2
ax2.set_box_aspect([max(x_aruco_local)-min(x_aruco_local), max(y_aruco_local)-min(y_aruco_local), max(z_aruco_local)-min(z_aruco_local)])

# Set legend for subplot 2
ax2.legend()

# Adjust layout
plt.tight_layout()

# Show plot
plt.show()