import serial
import struct
import time
import socket
import json
import numpy as np
from scipy.signal import butter, filtfilt

SERIAL_PORT = "/dev/tty.usbmodem203E345C57461"  # Update to match device
BAUD_RATE = 115200
TIMEOUT = 1  # Seconds

MSP_RAW_IMU = 102  # Cmd ID for MSP_RAW_IMU

ACCEL_SENSITIVITY = 2048.0  # counts per g
GYRO_SENSITIVITY = 16.4     # LSB per degree/second (for +-2000 range)

UDP_IP = "127.0.0.1"
UDP_PORT = 51912  # Port for Unity to listen on

#METERS_TO_UNITY = 0.13350
METERS_TO_UNITY = 13.350

GRAVITY = 9.81  # m/s^2

def calculate_checksum(data):
    """Calculate MSP checksum."""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def send_msp_command(ser, command_id, payload=None):
    """Send MSP command to the drone."""
    if payload is None:
        payload = []
    size = len(payload)
    header = b'$M<'  # MSP header
    message = bytearray([size, command_id] + payload)
    checksum = calculate_checksum(message)
    packet = header + message + bytes([checksum])
    ser.write(packet)

def read_msp_response(ser):
    """Read MSP response from the drone."""
    while True:
        byte = ser.read(1)
        if byte == b'$':
            if ser.read(1) == b'M':
                direction = ser.read(1)
                if direction == b'>':  # Response from drone flight controller
                    size = ser.read(1)[0]
                    command = ser.read(1)[0]
                    data = ser.read(size)
                    checksum = ser.read(1)[0]

                    calculated_checksum = calculate_checksum(bytearray([size, command]) + data)
                    if checksum == calculated_checksum:
                        return command, data
                    else:
                        print("Checksum mismatch!")
                        return None, None

def parse_msp_raw_imu(data):
    """Parse MSP_RAW_IMU data."""
    if len(data) != 18:
        print("Invalid data length for MSP_RAW_IMU")
        return None

    # Unpack 9 integers, 2 bytes each, from the raw data
    imu_values = struct.unpack("<hhhhhhhhh", data)

    # Acceleration, gyroscope, magnetometer(no magnetometer scaling)
    ax, ay, az, gx, gy, gz, mx, my, mz = imu_values

    imu_data = {
        "acceleration": {
            "x": ax / ACCEL_SENSITIVITY,
            "y": ay / ACCEL_SENSITIVITY,
            "z": (az / ACCEL_SENSITIVITY) - 1,  # Subtract 1g to account for gravity
        },
        "gyroscope": {
            "x": gx / GYRO_SENSITIVITY,
            "y": gy / GYRO_SENSITIVITY,
            "z": gz / GYRO_SENSITIVITY,
        },
        "magnetometer": {
            "x": mx,
            "y": my,
            "z": mz,
        },
        "timestamp": time.time()
    }
    return imu_data

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

def low_pass_filter(data, cutoff_frequency, fs):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_frequency / nyquist
    b, a = butter(N=1, Wn=normal_cutoff, btype='low', analog=False)
    filtered_data = filtfilt(b, a, data, axis=0)
    return filtered_data

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        takeoff_lat = float(input("Enter the takeoff latitude: "))
        takeoff_lon = float(input("Enter the takeoff longitude: "))

        x_unity_takeoff, z_unity_takeoff = gps_to_unity(takeoff_lat, takeoff_lon)

        acc_buffer = []
        timestamp_buffer = []
        buffer_size = 10

        velocity = np.zeros(3)

        print("Starting data transmission to Unity...")

        while True:
            send_msp_command(ser, MSP_RAW_IMU)

            command, data = read_msp_response(ser)
            if command == MSP_RAW_IMU and data is not None:
                imu_data = parse_msp_raw_imu(data)
                if imu_data:
                    acc = imu_data['acceleration']
                    timestamp = imu_data['timestamp']

                    acc_sample = np.array([acc['x'], acc['y'], acc['z']])
                    acc_buffer.append(acc_sample)
                    timestamp_buffer.append(timestamp)

                    if len(acc_buffer) > buffer_size:
                        acc_buffer.pop(0)
                        timestamp_buffer.pop(0)

                    if len(acc_buffer) >= 2:
                        # Calculate sampling frequency
                        dt_list = np.diff(timestamp_buffer)
                        fs_list = 1 / dt_list
                        fs = np.median(fs_list)
                        if fs <= 0:
                            fs = 100  # Default to 100 Hz if invalid

                        acc_array = np.array(acc_buffer)

                        acc_filtered = low_pass_filter(acc_array, cutoff_freq=5, fs=fs)

                        acc_latest = acc_filtered[-1]

                        acc_corrected = acc_latest 

                        # Calculate dt
                        if len(timestamp_buffer) >= 2:
                            dt = timestamp_buffer[-1] - timestamp_buffer[-2]
                        else:
                            dt = 0.01  # Default to 10 ms

                        velocity += acc_corrected * dt

                        velocity_unity = velocity * METERS_TO_UNITY

                        packet = {
                            'timestamp': timestamp_buffer[-1],
                            'dt': dt,
                            'velocity': {
                                'x': float(velocity_unity[0]),
                                'y': float(velocity_unity[1]),
                                'z': float(velocity_unity[2])
                            },
                            'position_offset': {
                                'x': x_unity_takeoff,
                                'z': z_unity_takeoff
                            }
                        }

                        message = json.dumps(packet)

                        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

                        print(f"Sent Velocity: {velocity_unity} to Unity.")

            else:
                print("No valid IMU data received. Retrying...")
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    main()
