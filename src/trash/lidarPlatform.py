# Setup
import time
import Jetson.GPIO as GPIO

# Variables
throttlePin = 31 # Channel 3
yawPin = 29 # Channel 1
escLeftPin = 33
escRightPin = 32
throttleStart = None # So if program starts on a HIGH pulse nothing breaks
yawStart = None
throttleWidth = 1500 # Microseconds
yawWidth = 1500
offset = 0 # Offset because motors are imbalanced fsr.

# GPIO setup
GPIO.setmode(GPIO.BOARD)
GPIO.setup(throttlePin, GPIO.IN)
GPIO.setup(yawPin, GPIO.IN)
GPIO.setup(escLeftPin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(escRightPin, GPIO.OUT, initial=GPIO.LOW)

# PWM setup
leftPWM = GPIO.PWM(escLeftPin, 50)
rightPWM = GPIO.PWM(escRightPin, 50)
leftPWM.start(7.5)
rightPWM.start(7.5)

# Callback for throttle interrupt
def throttle(channel): # 'channel' parameter is needed because when Python interrupt calls this function, it always passes in the channel from where interrupt originated from. No need to use it.
  global throttleStart, throttleWidth
  if GPIO.input(throttlePin):
    throttleStart = time.perf_counter_ns()
  elif throttleStart is not None:
    throttleWidth = (time.perf_counter_ns() - throttleStart) // 1000
    throttleStart = None

# Callback for yaw interrupt
def yaw(channel):
  global yawStart, yawWidth
  if GPIO.input(yawPin):
    yawStart = time.perf_counter_ns()
  elif yawStart is not None:
    yawWidth = (time.perf_counter_ns() - yawStart) // 1000
    yawStart = None

# Writing to the ESCs
def escWrite():
    throttle = throttleWidth
    yaw = yawWidth
    # Deadband
    if abs(throttle - 1500) < 25:
        throttle = 1500
    if abs(yaw - 1500) < 25:
        yaw = 1500
    # Convert to offsets from center
    throttle -= 1500
    yaw -= 1500
    # Tank mix
    left = throttle + yaw
    right = throttle - yaw
    # Clamp (before offset because I don't want the offsets to be cut off at the very edges, we'll see what happens.)
    left = max(-500, min(500, left))
    right = max(-500, min(500, right))
    # Some adjustments due to motor issues
    left += offset # Offsets because motors are imbalanced fsr
    right -= offset
    left *= -1 # One motor's direction needs to be switched
    # Back to servo pulse widths
    left += 1500
    right += 1500
    leftPWM.ChangeDutyCycle(left / 200)
    rightPWM.ChangeDutyCycle(right / 200)

# Interrupts
GPIO.add_event_detect(throttlePin, GPIO.BOTH, callback=throttle)
GPIO.add_event_detect(yawPin, GPIO.BOTH, callback=yaw)

# Main code
try:
    while True:
        escWrite()
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    leftPWM.stop()
    rightPWM.stop()
    GPIO.cleanup()
