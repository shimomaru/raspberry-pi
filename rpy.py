import math
import time
from mpu6050 import mpu6050
from qmc5883l import QMC5883L
from smbus2 import SMBus

# Initialize sensors
mpu = mpu6050(0x68)
mag = QMC5883L(SMBus(1))

def get_roll_pitch(accel):
    ax = accel['x']
    ay = accel['y']
    az = accel['z']

    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

    # Convert to degrees
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    return roll_deg, pitch_deg

def get_yaw(mag_data, roll, pitch):
    # Convert degrees to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)

    mx, my, mz = mag_data

    # Tilt compensation
    mx2 = mx * math.cos(pitch_rad) + mz * math.sin(pitch_rad)
    my2 = mx * math.sin(roll_rad) * math.sin(pitch_rad) + my * math.cos(roll_rad) - mz * math.sin(roll_rad) * math.cos(pitch_rad)

    # Yaw
    yaw_rad = math.atan2(-my2, mx2)
    yaw_deg = math.degrees(yaw_rad)
    if yaw_deg < 0:
        yaw_deg += 360
    return yaw_deg

try:
    while True:
        accel_data = mpu.get_accel_data()
        roll, pitch = get_roll_pitch(accel_data)

        mag_data = mag.get_magnet()
        yaw = get_yaw(mag_data, roll, pitch)

        print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped by user.")

