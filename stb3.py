import RPi.GPIO as GPIO
import time
import threading
import os
import psutil
from mpu6050 import mpu6050
import smbus2
import math

# ======================= GPIO + ESC SETUP =======================
ESC_PINS = [17, 27, 22, 23]  # Top Right, Bottom Right, Bottom Left, Top Left
GPIO.setmode(GPIO.BCM)
for pin in ESC_PINS:
    GPIO.setup(pin, GPIO.OUT)
ESC_PWM = [GPIO.PWM(pin, 50) for pin in ESC_PINS]  # 50 Hz PWM

# ======================= GLOBAL STATE =======================
current_pulse_width = 1000  # microseconds
running = True
lock = threading.Lock()

# ======================= SENSOR SETUP =======================
mpu = mpu6050(0x68)
bus = smbus2.SMBus(1)
QMC5883L_ADDRESS = 0x0D
base_orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}

# ======================= CORE ISOLATION =======================
def isolate_core():
    p = psutil.Process(os.getpid())
    p.cpu_affinity([3])  # Use core 3 only

# ======================= PWM CONTROL =======================
def pulse_width_to_duty_cycle(pulse_us):
    return (pulse_us / 20000.0) * 100

def pwm_loop():
    global current_pulse_width, running
    isolate_core()
    while running:
        with lock:
            duty = pulse_width_to_duty_cycle(current_pulse_width)
            for pwm in ESC_PWM:
                pwm.ChangeDutyCycle(duty)
        time.sleep(0.02)  # 50 Hz

def send_initial_pulse(pulse_width, duration):
    duty = pulse_width_to_duty_cycle(pulse_width)
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(duty)
    time.sleep(duration)

# ======================= USER INPUT LOOP =======================
def input_loop():
    global current_pulse_width, running
    print("Enter PWM value (1000–2000 µs) or 'exit':")
    while running:
        user_input = input("PWM > ")
        if user_input.lower() == 'exit':
            running = False
            break
        try:
            pulse = int(user_input)
            if 1000 <= pulse <= 2000:
                with lock:
                    current_pulse_width = pulse
            else:
                print("PWM must be between 1000–2000 µs.")
        except ValueError:
            print("Invalid input.")

# ======================= ORIENTATION =======================
def read_qmc5883l():
    data = bus.read_i2c_block_data(QMC5883L_ADDRESS, 0x00, 6)
    x = data[0] | (data[1] << 8)
    y = data[2] | (data[3] << 8)
    z = data[4] | (data[5] << 8)
    if x >= 32768:
        x -= 65536
    if y >= 32768:
        y -= 65536
    if z >= 32768:
        z -= 65536
    heading_rad = math.atan2(y, x)
    if heading_rad < 0:
        heading_rad += 2 * math.pi
    heading_deg = math.degrees(heading_rad)
    return heading_deg

def get_orientation():
    accel = mpu.get_accel_data()
    roll = math.degrees(math.atan2(accel['y'], accel['z']))
    pitch = math.degrees(math.atan(-accel['x'] / math.sqrt(accel['y'] ** 2 + accel['z'] ** 2)))
    yaw = read_qmc5883l()
    return roll, pitch, yaw

def orientation_loop():
    global base_orientation
    roll0, pitch0, yaw0 = get_orientation()
    base_orientation = {'roll': roll0, 'pitch': pitch0, 'yaw': yaw0}
    print("Base orientation set.")
    while running:
        roll, pitch, yaw = get_orientation()
        print("Roll: {:.2f}°, Pitch: {:.2f}°, Yaw: {:.2f}°".format(
            roll - base_orientation['roll'],
            pitch - base_orientation['pitch'],
            yaw - base_orientation['yaw']
        ))
        time.sleep(0.1)

# ======================= MAIN =======================
try:
    for pwm in ESC_PWM:
        pwm.start(0)

    print("Arming ESCs...")
    send_initial_pulse(2000, 3)
    send_initial_pulse(1000, 3)
    print("ESCs armed.")

    # Start threads
    pwm_thread = threading.Thread(target=pwm_loop, daemon=True)
    input_thread = threading.Thread(target=input_loop)
    orientation_thread = threading.Thread(target=orientation_loop, daemon=True)

    pwm_thread.start()
    input_thread.start()
    orientation_thread.start()

    input_thread.join()

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    running = False
    time.sleep(0.1)
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(0)
        pwm.stop()
    GPIO.cleanup()
    print("Motors stopped. GPIO cleaned up.")

