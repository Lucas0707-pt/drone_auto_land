import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the log directory 
log_dir = '2024_4_30_18_57_36/'
# Define the list of CSV filenames
aruco_pose_camera_file = log_dir + "aruco_pose_camera.csv"
aruco_pose_local_file = log_dir + "aruco_pose_local.csv"
vehicle_odometry_file = log_dir + "vehicle_odometry.csv"

# Create a new figure with two subplots
fig = plt.figure(figsize=(15, 8))

# Subplot for Aruco pose camera data
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title('Aruco Pose Camera')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# Read Aruco pose camera CSV file into a DataFrame
df_aruco_camera = pd.read_csv(aruco_pose_camera_file)
x_aruco_camera = df_aruco_camera['x'].to_numpy()
y_aruco_camera = df_aruco_camera['y'].to_numpy()
z_aruco_camera = df_aruco_camera['z'].to_numpy()
ax1.scatter(x_aruco_camera, y_aruco_camera, z_aruco_camera, color='r', s=10, label='Aruco Pose Camera')

# Connect the points with lines in subplot 1
ax1.plot(x_aruco_camera, y_aruco_camera, z_aruco_camera, color='r', linewidth=0.5, label='_nolegend_')

# Highlight first and last points in subplot 1
ax1.scatter(x_aruco_camera[0], y_aruco_camera[0], z_aruco_camera[0], color='black', s=50, label='First Point Camera')
ax1.scatter(x_aruco_camera[-1], y_aruco_camera[-1], z_aruco_camera[-1], color='orange', s=50, label='Last Point Camera')

# Set equal aspect ratio for subplot 1
ax1.set_box_aspect([max(x_aruco_camera)-min(x_aruco_camera), max(y_aruco_camera)-min(y_aruco_camera), max(z_aruco_camera)-min(z_aruco_camera)])

ax1.legend()

# Subplot for Aruco pose local and vehicle odometry data
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_title('Aruco Pose Local & Vehicle Odometry')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

# Read Aruco pose local CSV file into a DataFrame
df_aruco_local = pd.read_csv(aruco_pose_local_file)
x_aruco_local = df_aruco_local['x'].to_numpy()
y_aruco_local = df_aruco_local['y'].to_numpy()
z_aruco_local = df_aruco_local['z'].to_numpy()
ax2.scatter(x_aruco_local, y_aruco_local, z_aruco_local, color='g', s=5, label='Aruco Pose Local')

# Read vehicle odometry CSV file into a DataFrame
df_vehicle_odometry = pd.read_csv(vehicle_odometry_file)
x_vehicle_odometry = df_vehicle_odometry['x'].to_numpy()
y_vehicle_odometry = df_vehicle_odometry['y'].to_numpy()
z_vehicle_odometry = df_vehicle_odometry['z'].to_numpy()
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