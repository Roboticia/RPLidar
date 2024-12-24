import math
import matplotlib.pyplot as plt
from rplidar import RPLidar

# LIDAR setup
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

# Parameters for dynamic mapping
DMAX = 4000  # Maximum distance in mm to include points
POINT_SIZE = 1  # Size of points in the plot

# Functions
def polar_to_cartesian(angle_deg, distance_mm):
    """Convert polar coordinates to Cartesian coordinates."""
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y

def update_map(lidar, ax, scatter):
    """Fetch data from LIDAR and update the map dynamically."""
    points = []
    for scan in lidar.iter_scans():
        for _, angle, distance in scan:
            if distance > 0 and distance <= DMAX:  # Filter out invalid or out-of-range points
                x, y = polar_to_cartesian(angle, distance)
                points.append((x, y))

        # Update scatter plot
        if points:
            x_coords = [p[0] for p in points]
            y_coords = [p[1] for p in points]
            scatter.set_offsets(list(zip(x_coords, y_coords)))
            plt.pause(0.01)  # Adjust for smoother animation

        points.clear()  # Clear points for the next frame

# Main execution
def main():
    try:
        print("Starting LIDAR...")
        lidar.connect()
        
        # Set up the plot
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_title("Dynamic 2D Mapping")
        ax.set_xlim(-DMAX, DMAX)
        ax.set_ylim(-DMAX, DMAX)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.grid(True)
        scatter = ax.scatter([], [], s=POINT_SIZE)

        # Dynamic mapping
        update_map(lidar, ax, scatter)

    except KeyboardInterrupt:
        print("Stopping LIDAR...")

    finally:
        lidar.stop()
        lidar.disconnect()
        print("LIDAR stopped.")

if __name__ == "__main__":
    main()
