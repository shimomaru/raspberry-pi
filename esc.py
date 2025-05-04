import RPi.GPIO as GPIO
import time
import threading

# Define ESC pins using BCM GPIO numbers
ESC_PINS = [17, 27, 22, 23]

# Set up GPIO
GPIO.setmode(GPIO.BCM)
for pin in ESC_PINS:
    GPIO.setup(pin, GPIO.OUT)

# Create PWM instances for each ESC
ESC_PWM = [GPIO.PWM(pin, 50) for pin in ESC_PINS]  # 50Hz for ESCs (20ms period)

# Shared state
current_pulse_width = 1000  # in microseconds
running = True

def pulse_width_to_duty_cycle(pulse_us):
    """Convert pulse width in microseconds to duty cycle percentage."""
    return (pulse_us / 20000.0) * 100

def pwm_loop():
    """Continuously update PWM signals to match the current pulse width."""
    global current_pulse_width, running
    while running:
        duty = pulse_width_to_duty_cycle(current_pulse_width)
        for pwm in ESC_PWM:
            pwm.ChangeDutyCycle(duty)
        time.sleep(0.05)  # 50ms update rate

def input_loop():
    """Update the pulse width based on user input."""
    global current_pulse_width, running
    print("Enter pulse width (1000–2000µs), or 'exit' to quit:")
    while running:
        user_input = input("> ")
        if user_input.lower() == "exit":
            running = False
            break
        try:
            pulse = int(user_input)
            if 1000 <= pulse <= 2000:
                current_pulse_width = pulse
                print(f"Updated pulse width to {pulse}µs.")
            else:
                print("Please enter a value between 1000 and 2000.")
        except ValueError:
            print("Invalid input. Please enter a number between 1000 and 2000, or 'exit'.")

def send_initial_pulse(pulse_width, duration):
    """Send initial calibration pulse to all ESCs for a specified time."""
    duty = pulse_width_to_duty_cycle(pulse_width)
    print(f"Sending {pulse_width}µs ({duty:.2f}% duty) for {duration} seconds...")
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(duty)
    time.sleep(duration)

try:
    # Start PWM output at 0% duty cycle
    for pwm in ESC_PWM:
        pwm.start(0)

    # ESC Calibration steps
    send_initial_pulse(2000, 5)  # Max throttle
    send_initial_pulse(1000, 5)  # Min throttle

    # Start PWM thread and input thread
    pwm_thread = threading.Thread(target=pwm_loop, daemon=True)
    input_thread = threading.Thread(target=input_loop)

    pwm_thread.start()
    input_thread.start()

    input_thread.join()  # Wait for input thread to finish

except KeyboardInterrupt:
    print("Interrupted by user. Stopping...")

finally:
    running = False
    time.sleep(0.1)
    for pwm in ESC_PWM:
        pwm.ChangeDutyCycle(0)
        pwm.stop()
    GPIO.cleanup()
    print("All ESCs stopped and GPIO cleaned up.")

