import time
import numpy as np
from mpu6050 import mpu6050
from qmc5883l import QMC5883L
from smbus2 import SMBus
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler

# Initialize sensors
mpu = mpu6050(0x68)
mag = QMC5883L(1)

# Initialize Madgwick filter
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion

# Sampling rate
dt = 1.0 / 50  # 50Hz

def read_normalized_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    mag_data = mag.get_magnet()

    acc = np.array([accel['x'], accel['y'], accel['z']])
    gyr = np.radians(np.array([gyro['x'], gyro['y'], gyro['z']]))  # deg/s → rad/s
    mag = np.array(mag_data)

    return acc, gyr, mag

try:
    while True:
        acc, gyr, mag = read_normalized_data()
        q = madgwick.updateIMU(q=q, acc=acc, gyr=gyr, mag=mag)
        euler = np.degrees(q2euler(q))  # roll, pitch, yaw in degrees

        roll, pitch, yaw = euler
        if yaw < 0:
            yaw += 360

        print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
        time.sleep(dt)

except KeyboardInterrupt:
    print("Stopped by user.")

