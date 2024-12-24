import serial

def read_imu_data(port='/dev/ttyACM0', baudrate=9600):
    """Reads IMU data from Arduino via Serial."""
    ser = serial.Serial(port, baudrate, timeout=1)
    try:
        while True:
            line = ser.readline().decode('utf-8').strip()  # Read and clean data
            print(f"Raw data: {line}")  # Debug: Print raw data
            if line.startswith("Accel:"):  # Handle lines with 'Accel:' prefix
                line = line.replace("Accel: ", "").strip()  # Remove the prefix

            if line.count(",") == 2:  # Ensure correct format (two commas)
                try:
                    ax, ay, gz = map(float, line.split(","))
                    print(f"AccelX: {ax}, AccelY: {ay}, GyroZ: {gz}")
                except ValueError:
                    print("Malformed data, skipping...")
            else:
                print("Incomplete or malformed data, skipping...")
    except KeyboardInterrupt:
        print("Stopping IMU read...")
    finally:
        ser.close()



if __name__ == "__main__":
    read_imu_data()
