import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rplidar import RPLidar
import serial
import time
import rplidar
import threading

# LIDAR and IMU setup
LIDAR_PORT = '/dev/ttyUSB0'
IMU_PORT = '/dev/ttyACM0'
IMU_BAUDRATE = 9600

lidar = RPLidar(LIDAR_PORT)
imu_serial = serial.Serial(IMU_PORT, IMU_BAUDRATE, timeout=1)

# Parameters
DMAX = 4000  # Maximum distance in mm to include points
POINT_SIZE = 1  # Size of points in the plot

# Global map for 3D points
global_map = []

# Functions
def polar_to_cartesian(angle_deg, distance_mm):
    """Convert polar coordinates to Cartesian coordinates."""
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y

def get_tilt_angle():
    """Fetch tilt angle (pitch) from IMU."""
    try:
        line = imu_serial.readline().decode('utf-8').strip()
        if line.startswith("Accel:"):  # Clean up prefix if present
            line = line.replace("Accel: ", "").strip()
        if line.count(",") == 2:  # Ensure correct format
            ax, ay, gz = map(float, line.split(","))
        else:
            return 0.0  # Default to no tilt on malformed data
        # Use gyroscope Z-axis (gz) for tilt angle (pitch)
        time.sleep(0.05)  # Adjust delay as needed
        return gz  # In degrees or radians as needed
    except Exception as e:
        print(f"Error reading IMU data: {e}")
        return 0.0

def transform_to_3d(local_points, tilt_angle):
    """Transform 2D LIDAR points to 3D based on tilt angle."""
    global_points = []
    tilt_rad = math.radians(tilt_angle)
    for x, y in local_points:
        z = y * math.sin(tilt_rad)  # Project y-axis distance to z-axis
        y_proj = y * math.cos(tilt_rad)  # Adjust y-axis for tilt
        global_points.append((x, y_proj, z))
    return global_points

def update_map(lidar, ax, scatter):
    global global_map
    for scan in lidar.iter_scans():
        start_time = time.time()

        # LIDAR data collection
        local_points = []
        for _, angle, distance in scan:
            if distance > 0 and distance <= DMAX:  # Filter out invalid or out-of-range points
                x, y = polar_to_cartesian(angle, distance)
                local_points.append((x, y))

        # IMU data collection
        tilt_angle = get_tilt_angle()

        # Transform to 3D points
        global_points = transform_to_3d(local_points, tilt_angle)
        global_map.extend(global_points)

        # Update scatter plot
        if global_map:
            x_coords = [p[0] for p in global_map]
            y_coords = [p[1] for p in global_map]
            z_coords = [p[2] for p in global_map]
            scatter._offsets3d = (x_coords, y_coords, z_coords)

        # Maintain consistent timing
        elapsed = time.time() - start_time
        time.sleep(max(0, 0.1 - elapsed))  # Adjust interval as needed
        plt.pause(0.01)  # Adjust for smoother animation

# Main execution
def main():
    try:
        print("Starting LIDAR and IMU...")
        
        # Clear LIDAR buffer
        lidar.stop()
        lidar.disconnect()
        lidar.connect()
        lidar.clean_input()

        lidar._serial.reset_input_buffer()  # Clear any old data
        imu_serial.reset_input_buffer()


        # Clear IMU buffer
        imu_serial.reset_input_buffer()  # Clear IMU input buffer
        imu_serial.reset_output_buffer()  # Clear IMU output buffer

        # Start LIDAR motor
        lidar.start_motor()
        lidar._set_pwm(60)
        lidar.clean_input
        print(lidar.motor_speed)# Ensure the motor is running
  # Reduce from default 660 RPM to 200 RPM

        # Set up the plot
        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("3D Mapping with Tilted LIDAR")
        ax.set_xlim(-DMAX, DMAX)
        ax.set_ylim(-DMAX, DMAX)
        ax.set_zlim(-DMAX, DMAX)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        scatter = ax.scatter([], [], [], s=POINT_SIZE)

        # Begin dynamic mapping
        update_map(lidar, ax, scatter)

    except KeyboardInterrupt:
        print("Stopping LIDAR and IMU...")

    finally:
        lidar.stop()
        lidar.disconnect()
        imu_serial.close()
        print("LIDAR and IMU stopped")


if __name__ == "__main__":
    main()
