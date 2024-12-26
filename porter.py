import serial
import struct
import time

SERIAL_PORT = "/dev/tty.usbmodem203E345C57461"  # Update to match device
BAUD_RATE = 115200

MSP_RAW_IMU = 102  # Cmd ID for MSP_RAW_IMU

# Scaling factors
ACCEL_SENSITIVITY = 2048.0  # LSB per g (+-16 range)
GYRO_SENSITIVITY = 16.4     # LSB per degree/second (+-2000 range)

def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def send_msp_command(ser, command_id, payload=None):
    if payload is None:
        payload = []

    size = len(payload)
    header = b'$M<'  # MSP header
    message = bytearray([size, command_id] + payload)
    checksum = calculate_checksum(message)
    packet = header + message + bytes([checksum])

    ser.write(packet)

def read_msp_response(ser):
    while True:
        if ser.read(1) == b'$':  # Start of frame
            if ser.read(1) == b'M':
                direction = ser.read(1)
                if direction == b'>':  # Response from drone flight controller
                    size = ser.read(1)[0]
                    command = ser.read(1)[0]
                    data = ser.read(size)
                    checksum = ser.read(1)[0]

                    calculated_checksum = calculate_checksum(
                        bytearray([size, command]) + data
                    )
                    if checksum == calculated_checksum:
                        return command, data
                    else:
                        print("Checksum mismatch!")
                        return None, None

def parse_msp_raw_imu(data):
    if len(data) != 18:
        print("Invalid data length for MSP_RAW_IMU")
        return None

    # Unpack 9 integers, 2 bytes each, from the raw data
    imu_values = struct.unpack("<hhhhhhhhh", data)

    # Acceleration, gyroscope, magnetometer(magnetometer not scaled)
    ax, ay, az, gx, gy, gz, mx, my, mz = imu_values

    imu_data = {
        "acceleration": {
            "x": ax / ACCEL_SENSITIVITY,
            "y": ay / ACCEL_SENSITIVITY,
            "z": az / ACCEL_SENSITIVITY,
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
    }
    return imu_data

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

        while True:
            send_msp_command(ser, MSP_RAW_IMU)
            #time.sleep(0.001)

            command, data = read_msp_response(ser)
            if command == MSP_RAW_IMU and data is not None:
                imu_data = parse_msp_raw_imu(data)
                if imu_data:
                    print("Acceleration (g's):", imu_data["acceleration"])
                    print("Gyroscope (°/s):", imu_data["gyroscope"])
                    #print("Magnetometer (raw):", imu_data["magnetometer"])
                    print()
            else:
                print("No valid data received. Retrying...")

    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if ser:
            ser.close()


if __name__ == "__main__":
    main()
