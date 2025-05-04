import RPi.GPIO as GPIO
import time
import threading
from mpu6050 import mpu6050
from qmc5883l import QMC5883L
import math

# === ESC Setup ===
ESC_PINS = [17, 27, 22, 23]  # Front-Right, Back-Right, Back-Left, Front-Left
GPIO.setmode(GPIO.BCM)
for pin in ESC_PINS:
    GPIO.setup(pin, GPIO.OUT)
ESC_PWM = [GPIO.PWM(pin, 50) for pin in ESC_PINS]  # 50Hz = 20ms period

# === Global Motor Pulse Array ===
current_pulses = [1000] * 4  # Min throttle
running = True

def pulse_width_to_duty_cycle(pulse_us):
    return (pulse_us / 20000.0) * 100

def esc_output_loop():
    """Continuously update ESCs with current pulse widths."""
    while running:
        for i in range(4):
            duty = pulse_width_to_duty_cycle(current_pulses[i])
            ESC_PWM[i].ChangeDutyCycle(duty)
        time.sleep(0.02)

def arm_escs():
    for pwm in ESC_PWM:
        pwm.start(0)
    print("Arming ESCs...")
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(pulse_width_to_duty_cycle(2000))  # Max
    time.sleep(5)
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(pulse_width_to_duty_cycle(1000))  # Min
    time.sleep(5)
    print("ESCs armed.")

# === PID Controller Class ===
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint, measurement):
        error = setpoint - measurement
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp*error + self.Ki*self.integral + self.Kd*derivative

# === Sensor Setup ===
mpu = mpu6050(0x68)
mag = QMC5883L()

def get_orientation():
    accel = mpu.get_accel_data()
    mag_data = mag.get_magnet()

    ax, ay, az = accel['x'], accel['y'], accel['z']
    mx, my = mag_data[0], mag_data[1]

    roll = math.atan2(ay, az) * 180 / math.pi
    pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2)) * 180 / math.pi
    yaw = math.atan2(my, mx) * 180 / math.pi
    return roll, pitch, yaw

# === PID Controllers for each axis ===
pid_roll = PID(1.2, 0.01, 0.1)
pid_pitch = PID(1.2, 0.01, 0.1)
pid_yaw = PID(1.0, 0.01, 0.05)

# === Control Loop ===
def control_loop():
    base_throttle = 1200  # Adjust this for hover
    while running:
        roll, pitch, yaw = get_orientation()

        # Print orientation to terminal
        print(f"Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")

        roll_corr = pid_roll.compute(0, roll)
        pitch_corr = pid_pitch.compute(0, pitch)
        yaw_corr = pid_yaw.compute(0, yaw)

        # Motor Mixing Logic (Quad X)
        current_pulses[0] = base_throttle - roll_corr + pitch_corr + yaw_corr  # Front-Right
        current_pulses[1] = base_throttle - roll_corr - pitch_corr - yaw_corr  # Back-Right
        current_pulses[2] = base_throttle + roll_corr - pitch_corr + yaw_corr  # Back-Left
        current_pulses[3] = base_throttle + roll_corr + pitch_corr - yaw_corr  # Front-Left

        # Clamp pulse widths to safe range
        for i in range(4):
            current_pulses[i] = max(1000, min(2000, current_pulses[i]))

        time.sleep(0.1)

# === Main Execution ===
try:
    arm_escs()
    esc_thread = threading.Thread(target=esc_output_loop, daemon=True)
    esc_thread.start()

    control_loop()

except KeyboardInterrupt:
    print("Shutting down...")

finally:
    running = False
    time.sleep(0.1)
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(0)
        pwm.stop()
    GPIO.cleanup()
    print("Motors stopped. GPIO cleaned up.")

