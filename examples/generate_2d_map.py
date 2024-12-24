import math
import matplotlib.pyplot as plt

# File containing raw data (output from record_measurements.py)
DATA_FILE = "output.txt"

def polar_to_cartesian(angle_deg, distance_mm):
    """Convert polar coordinates to Cartesian coordinates."""
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y

def load_data(file_path):
    """Load raw LIDAR data from the file."""
    points = []
    with open(file_path, "r") as f:
        for line in f:
            parts = line.split()
            if len(parts) != 4:
                continue  # Skip invalid lines
            
            _, quality, angle, distance = parts
            quality = int(quality)
            angle = float(angle)
            distance = float(distance)

            # Filter out invalid or low-quality data
            if distance > 0 and quality > 0:
                x, y = polar_to_cartesian(angle, distance)
                points.append((x, y))
    return points

def plot_map(points):
    """Plot the 2D map using the processed points."""
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]
    plt.figure(figsize=(10, 10))
    plt.scatter(x_coords, y_coords, s=1)
    plt.title("2D Map from LIDAR Data")
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # Load and process LIDAR data
    points = load_data(DATA_FILE)
    
    # Plot the 2D map
    plot_map(points)
