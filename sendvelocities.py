import json
import numpy as np
import socket
import time
from scipy.signal import butter, filtfilt

ACCEL_SENSITIVITY = 2048.0  # counts per g
GRAVITY = 9.81  # m/s^2

UDP_IP = "127.0.0.1"
UDP_PORT = 51912

# Unity scaling factor
METERS_TO_UNITY = 0.13350

def gps_to_unity(latitude, longitude):
    """
    Converts GPS coordinates to Unity coordinates using the mapping equations.
    """
    LAT_REF = 37.778369
    LON_REF = -122.390004
    C = 3915.367
    F = 1287.471
    A = 0.13350
    B = -0.000166
    D = -0.000174
    E = 0.13450
    METERS_PER_DEG_LAT = 111000
    AVG_LAT_RAD = np.radians(37.896032)
    METERS_PER_DEG_LON = 111000 * np.cos(AVG_LAT_RAD)

    delta_lat = latitude - LAT_REF
    delta_lon = longitude - LON_REF
    delta_n = delta_lat * METERS_PER_DEG_LAT
    delta_e = delta_lon * METERS_PER_DEG_LON
    x_unity = A * delta_e + B * delta_n + C
    z_unity = D * delta_e + E * delta_n + F
    return x_unity, z_unity

def main():
    json_file = input("Enter the path to the drone log JSON file: ")
    takeoff_lat = float(input("Enter the takeoff latitude: "))
    takeoff_lon = float(input("Enter the takeoff longitude: "))

    with open(json_file, 'r') as f:
        data = json.load(f)

    acc_list = []
    timestamps = []
    for sample in data:
        ax_g = sample['accelerometer']['x'] / ACCEL_SENSITIVITY
        ay_g = sample['accelerometer']['y'] / ACCEL_SENSITIVITY
        az_g = sample['accelerometer']['z'] / ACCEL_SENSITIVITY

        acc_mps2 = np.array([ax_g * GRAVITY, ay_g * GRAVITY, az_g * GRAVITY])
        sample['accel_mps2'] = {
            'x': acc_mps2[0],
            'y': acc_mps2[1],
            'z': acc_mps2[2]
        }
        acc_list.append(acc_mps2)
        timestamps.append(sample['timestamp'])

    timestamps = np.array(timestamps)

    # Calculate sampling frequency
    dt_list = np.diff(timestamps)
    fs_list = 1 / dt_list
    fs = np.median(fs_list)  # Use median sampling frequency

    # Low-pass filter for accelerations
    def low_pass_filter(data, cutoff_frequency, fs):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff_frequency / nyquist
        b, a = butter(N=1, Wn=normal_cutoff, btype='low', analog=False)
        filtered_data = filtfilt(b, a, data, axis=0)
        return filtered_data

    cutoff_freq = 5  # in Hz
    acc_array = np.array(acc_list)
    acc_filtered = low_pass_filter(acc_array, cutoff_freq, fs)

    velocity = np.zeros(3)

    velocities = []

    # Gravity vector - assuming drone z-axis is up
    gravity_vector = np.array([0, 0, GRAVITY])

    for i in range(len(data)):
        if i == 0:
            dt = 0
        else:
            dt = timestamps[i] - timestamps[i - 1]

        acc = acc_filtered[i]

        # Subtract gravity
        acc_corrected = acc - gravity_vector

        velocity += acc_corrected * dt

        velocities.append(velocity.copy())

    # Velocities -> Unity units
    velocities_unity = [v * METERS_TO_UNITY for v in velocities]

    # Compute Unity coordinates for takeoff location
    x_unity_takeoff, z_unity_takeoff = gps_to_unity(takeoff_lat, takeoff_lon)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Sending velocities over UDP...")

    for i in range(len(data)):
        # Compute dt for Unity
        if i == 0:
            dt = 0  # For first data point, dt is 0
        else:
            dt = timestamps[i] - timestamps[i - 1]

        packet = {
            'timestamp': data[i]['timestamp'],
            'dt': dt,
            'velocity': {
                'x': float(velocities_unity[i][0]),
                'y': float(velocities_unity[i][1]),
                'z': float(velocities_unity[i][2])
            },
            'position_offset': {
                'x': x_unity_takeoff,
                'z': z_unity_takeoff
            }
        }

        message = json.dumps(packet)
        
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

        # Wait for the next timestamp (simulate real-time)
        if i < len(data) - 1:
            dt_sleep = timestamps[i + 1] - timestamps[i]
            if dt_sleep > 0:
                time.sleep(dt_sleep)

    print("Data transmission completed.")

if __name__ == "__main__":
    main()
