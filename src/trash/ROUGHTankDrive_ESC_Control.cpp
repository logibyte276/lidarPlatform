#include <Servo.h>

Servo escLeft;
Servo escRight;

bool leftFlip = false;
bool rightFlip = true;

// Pins
const int throttlePin = 2;
const int yawPin = 3;
const int escPinLeft = 7;
const int escPinRight = 6;

// Vars
volatile unsigned long timeStartThrottle = 0;
volatile unsigned long timeStartYaw = 0;
volatile unsigned long lastSignal = 0;
volatile unsigned int pulseWidthThrottle = 1500;
volatile unsigned int pulseWidthYaw = 1500;

// Get throttle value from receiver
void throttleISR() {
  lastSignal = millis();
  if (digitalRead(throttlePin) == HIGH)
    timeStartThrottle = micros();
  else
    pulseWidthThrottle = micros() - timeStartThrottle;
}

// Get yaw value from receiver
void yawISR() {
  lastSignal = millis();
  if (digitalRead(yawPin) == HIGH)
    timeStartYaw = micros();
  else
    pulseWidthYaw = micros() - timeStartYaw;
}

// Write to the ESC
void movement(int throttle, int yaw) {
  int t = throttle - 1500;
  int y = yaw - 1500;

  if (abs(t) < 20) t = 0;
  if (abs(y) < 20) y = 0;

  int left  = constrain(t + y, -500, 500);
  int right = constrain(t - y, -500, 500);

  // Flip directions BEFORE writing to the ESCs
  if (leftFlip)
    left = -left;

  if (rightFlip)
    right = -right;

  escLeft.writeMicroseconds(left + 1500);
  escRight.writeMicroseconds(right + 1500);

  Serial.print(left + 1500);
  Serial.print(" ");
  Serial.println(right + 1500);
}

void setup() {
  pinMode(throttlePin, INPUT);
  pinMode(yawPin, INPUT);

  escLeft.attach(escPinLeft);
  escRight.attach(escPinRight);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(throttlePin), throttleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(yawPin), yawISR, CHANGE);

  delay(3000);
}

void loop() {
  noInterrupts();
  int throttle = constrain(pulseWidthThrottle, 1000, 2000);
  int yaw = constrain(pulseWidthYaw, 1000, 2000);
  interrupts();

  movement(throttle, yaw);

  delay(20);
}
