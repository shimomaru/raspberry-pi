import os
import psutil
import threading
import time
import ctypes
import ctypes.util
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import smbus2
import math

# --- ESC PWM SETUP ---
ESC_PINS = [17, 27, 22, 23]  # TR, BR, BL, TL (in clockwise order)
GPIO.setmode(GPIO.BCM)
for pin in ESC_PINS:
    GPIO.setup(pin, GPIO.OUT)
ESC_PWM = [GPIO.PWM(pin, 50) for pin in ESC_PINS]  # 50Hz

# --- GLOBALS ---
running = True
current_pulse_widths = [1000, 1000, 1000, 1000]  # microseconds for each ESC
base_orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

# --- UTILS ---
def pulse_width_to_duty_cycle(pulse_us):
    return (pulse_us / 20000.0) * 100

def set_thread_affinity(core_id):
    p = psutil.Process(os.getpid())
    p.cpu_affinity([core_id])
    print(f"[INFO] PWM thread pinned to CPU core {core_id}")

def set_realtime_priority():
    libc = ctypes.CDLL(ctypes.util.find_library("c"))
    pid = 0  # current
    class SchedParam(ctypes.Structure):
        _fields_ = [("sched_priority", ctypes.c_int)]
    param = SchedParam(10)
    result = libc.sched_setscheduler(pid, 1, ctypes.byref(param))  # SCHED_FIFO
    if result != 0:
        print("[WARNING] Failed to set real-time priority")

# --- PWM THREAD ---
def pwm_loop():
    set_thread_affinity(3)
    set_realtime_priority()
    for pwm in ESC_PWM:
        pwm.start(0)
    try:
        while running:
            for i in range(4):
                duty = pulse_width_to_duty_cycle(current_pulse_widths[i])
                ESC_PWM[i].ChangeDutyCycle(duty)
            time.sleep(0.01)
    finally:
        for pwm in ESC_PWM:
            pwm.ChangeDutyCycle(0)
            pwm.stop()

# --- ESC ARMING ---
def send_initial_pulse(pulse_width, duration):
    duty = pulse_width_to_duty_cycle(pulse_width)
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(duty)
    time.sleep(duration)

# --- MPU + QMC SETUP ---
mpu = mpu6050(0x68)
bus = smbus2.SMBus(1)
QMC5883L_ADDR = 0x0D

# QMC INIT
bus.write_byte_data(QMC5883L_ADDR, 0x0B, 0x01)
bus.write_byte_data(QMC5883L_ADDR, 0x09, 0x1D)  # Continuous, 200Hz, 2G, 512 ODR

def read_magnetometer():
    data = bus.read_i2c_block_data(QMC5883L_ADDR, 0x00, 6)
    x = int.from_bytes(data[0:2], byteorder='little', signed=True)
    y = int.from_bytes(data[2:4], byteorder='little', signed=True)
    z = int.from_bytes(data[4:6], byteorder='little', signed=True)
    return x, y, z

def get_orientation():
    accel = mpu.get_accel_data()
    roll = math.degrees(math.atan2(accel['y'], accel['z']))
    pitch = math.degrees(math.atan(-accel['x'] / math.sqrt(accel['y']**2 + accel['z']**2)))

    mx, my, mz = read_magnetometer()
    yaw = math.degrees(math.atan2(my, mx))
    return roll, pitch, yaw

def calibrate_orientation():
    global base_orientation
    roll, pitch, yaw = get_orientation()
    base_orientation = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
    print(f"[INFO] Base orientation calibrated: {base_orientation}")

# --- MAIN LOOP ---
def main_loop():
    calibrate_orientation()
    while running:
        roll, pitch, yaw = get_orientation()
        delta_roll = roll - base_orientation['roll']
        delta_pitch = pitch - base_orientation['pitch']
        delta_yaw = yaw - base_orientation['yaw']
        print(f"ΔRoll: {delta_roll:.2f}°, ΔPitch: {delta_pitch:.2f}°, ΔYaw: {delta_yaw:.2f}°")
        time.sleep(0.1)

# --- MAIN ENTRY ---
if __name__ == "__main__":
    try:
        print("Arming ESCs...")
        for pwm in ESC_PWM:
            pwm.start(0)
        send_initial_pulse(2000, 5)
        send_initial_pulse(1000, 5)
        print("ESCs armed.")

        pwm_thread = threading.Thread(target=pwm_loop, daemon=True)
        pwm_thread.start()

        main_loop()

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        running = False
        time.sleep(0.1)
        GPIO.cleanup()
        print("Motors stopped. GPIO cleaned up.")


