// Servo objects for ESC control
#include <Servo.h>
Servo leftESC;
Servo rightESC;

// Pin definitions
const int throttlePin = 2; // Hardware Interrupt 0, Channel 3 on receiver
const int yawPin      = 3; // Hardware Interrupt 1, Channel 1 on receiver
const int escLeftPin  = 5; 
const int escRightPin = 6; 

// Variables
volatile unsigned long throttleStart = 0;
volatile unsigned long yawStart      = 0;
volatile int throttleWidth           = 0; // Start at 0 so motors don't auto-arm
volatile int yawWidth                = 0;

// Expo curve
int applyExpo(int value) // Put in -500 to 500 signal here.
{
    float x = value / 500.0f;
    x = x * x;
    return (int)round(x * 500.0f);
}

// Callback for throttle interrupt
void throttleISR() {
  if (digitalRead(throttlePin) == HIGH) {
    throttleStart = micros();
  } else {
    if (throttleStart != 0) {
      throttleWidth = micros() - throttleStart;
      throttleStart = 0; // Reset so callback doesn't work if no start time.
    }
  }
}

// Callback for yaw interrupt
void yawISR() {
  if (digitalRead(yawPin) == HIGH) {
    yawStart = micros();
  } else {
    if (yawStart != 0) {
      yawWidth = micros() - yawStart;
      yawStart = 0; // Reset
    }
  }
}

void writeESC() {
  // Read volatile variables safely by temporarily disabling interrupts
  noInterrupts();
  int throttle = throttleWidth;
  int yaw = yawWidth;
  interrupts();
  // Deadband
  if (abs(throttle - 1500) < 25) {
    throttle = 1500;
  }
  if (abs(yaw - 1500) < 25) {
    yaw = 1500;
  }
  // Convert to offsets from center
  throttle -= 1500;
  yaw -= 1500;
  // Tank mix
  int left = throttle + yaw;
  int right = throttle - yaw;
  // Back to servo pulse widths (1000us to 2000us)
  left += 1500;
  right += 1500;
  // Clamp values between 1000 and 2000
  left = constrain(left, 1000, 2000);
  right = constrain(right, 1000, 2000);
  // Write PWM width directly to the ESCs
  leftESC.writeMicroseconds(left);
  rightESC.writeMicroseconds(right);
}

void setup() {
  // Initialize input pins
  pinMode(throttlePin, INPUT);
  pinMode(yawPin, INPUT);
  // Attach ESCs using the Servo library
  leftESC.attach(escLeftPin);
  rightESC.attach(escRightPin);
  // Attach hardware interrupts (triggers on both RISING and FALLING edges)
  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(yawPin), yawISR, CHANGE);
  // Don't arm when radio is not connected
  leftESC.writeMicroseconds(0);
  rightESC.writeMicroseconds(0);
  delay(1000); // Weird stuff happens to signals when starting up, so delay to not pick anything up.
}

void loop() {
  writeESC();
  delay(10);
}
