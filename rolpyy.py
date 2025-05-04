import time
import numpy as np
from mpu6050 import mpu6050
from qmc5883l_driver import QMC5883L
from ahrs.filters import Madgwick
from ahrs.common.orientation import q2euler

# Initialize sensors
mpu = mpu6050(0x68)
mag = QMC5883L()

# Initialize Madgwick filter
madgwick = Madgwick()
q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial orientation quaternion

# Sampling rate
dt = 1.0 / 50  # 50 Hz

def read_normalized_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    mag_data = mag.read_raw()

    acc = np.array([accel['x'], accel['y'], accel['z']])
    gyr = np.radians(np.array([gyro['x'], gyro['y'], gyro['z']]))  # Convert to rad/s
    magv = np.array(mag_data)

    return acc, gyr, magv

try:
    while True:
        acc, gyr, magv = read_normalized_data()
        q = madgwick.updateMARG(q=q, acc=acc, gyr=gyr, mag=magv)
        if q is not None:
            euler = np.degrees(q2euler(q))  # roll, pitch, yaw
            roll, pitch, yaw = euler
            if yaw < 0:
                yaw += 360
            print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
        else:
            print("Failed to update orientation.")
        time.sleep(dt)

except KeyboardInterrupt:
    print("Stopped by user.")

