/*
 * Tank-drive RC mixer for Arduino Nano
 * -------------------------------------
 * Reads three RC channels from an FS-iA6B receiver, mixes throttle + yaw into
 * independent left/right commands, and drives two Hobbywing QuicRun 1080 G2
 * brushed ESCs.
 *
 * Channels:
 *   D2 (INT0)     throttle   - receiver ch3
 *   D3 (INT1)     yaw        - receiver ch1
 *   D4 (PCINT20)  arm switch - receiver ch5
 *
 * D4 has no hardware interrupt on a Nano, so it uses a pin-change interrupt.
 *
 * NOTE ON STYLE: channel state is held in parallel arrays of primitive types
 * rather than in structs. This is deliberate. The Arduino IDE auto-generates
 * function prototypes and injects them ABOVE your own type definitions, so any
 * function whose signature mentions a custom struct fails to compile with
 * "does not name a type". Using only built-in types in signatures avoids it.
 *
 * SAFETY MODEL
 *   The car only moves when ALL of these hold:
 *     - all three channels have delivered a pulse within SIGNAL_TIMEOUT_US
 *     - all three pulse widths are inside a plausible RC range
 *     - the arm switch is in the armed position
 *     - at the moment of arming, both sticks were at neutral
 *   Any failure sends 1500us (neutral) to both ESCs and requires the switch to
 *   be cycled off/on to re-arm.
 */

#include <Servo.h>

// ---------------------------------------------------------------- pins ----

const uint8_t PIN_THROTTLE  = 2;
const uint8_t PIN_YAW       = 3;
const uint8_t PIN_ARM       = 4;
const uint8_t PIN_ESC_LEFT  = 5;
const uint8_t PIN_ESC_RIGHT = 6;

// ------------------------------------------------------------- tuning ----

const int PULSE_MIN_US = 900;    // narrower than this: treat as invalid
const int PULSE_MAX_US = 2100;   // wider than this: treat as invalid
const int PULSE_MID_US = 1500;

const int DEADBAND_US   = 25;    // stick slop around centre
const int OUTPUT_MIN_US = 1000;
const int OUTPUT_MAX_US = 2000;

// If no pulse arrives on a channel for this long, assume signal loss.
// 100 ms is ~5 missed frames at the receiver's ~50 Hz update rate: long
// enough to ride out one glitched frame, short enough that a runaway is
// caught in a tenth of a second.
const unsigned long SIGNAL_TIMEOUT_US = 100000UL;

// Which half of the switch travel counts as "armed".
// Set this once you know which way your switch reads - see FIRST-RUN below.
const int  ARM_THRESHOLD_US = 1500;
const bool ARM_WHEN_ABOVE   = false;   // false => armed when pulse < threshold

// Sticks must be within this of centre before arming is allowed.
// If your transmitter has trim offsets, released sticks may not sit at 1500 -
// check the serial output before assuming this value works.
const int ARM_NEUTRAL_TOLERANCE_US = 60;

// Per-motor trim, in microseconds, for two motors that don't quite match.
// Keep small - large values mean a mechanical problem, not a tuning one.
const int LEFT_TRIM_US  = 0;
const int RIGHT_TRIM_US = 0;

// Flip if a motor runs backwards relative to the other.
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;

const bool USE_EXPO     = false;  // softer response near centre
const bool DEBUG_SERIAL = true;   // print channel values at 10 Hz

// -------------------------------------------------------------- state ----

const uint8_t CH_THROTTLE  = 0;
const uint8_t CH_YAW       = 1;
const uint8_t CH_ARM       = 2;
const uint8_t NUM_CHANNELS = 3;

// Written by the ISRs, read by the main loop.
volatile unsigned long chRiseUs[NUM_CHANNELS]  = {0, 0, 0};
volatile unsigned int  chWidthUs[NUM_CHANNELS] = {0, 0, 0};
volatile unsigned long chLastUs[NUM_CHANNELS]  = {0, 0, 0};
volatile bool          chSeen[NUM_CHANNELS]    = {false, false, false};

// Snapshot taken once per loop, safe to read without disabling interrupts.
int  chWidth[NUM_CHANNELS] = {0, 0, 0};
bool chFresh[NUM_CHANNELS] = {false, false, false};

Servo leftESC;
Servo rightESC;

bool armed = false;
bool armLatched = false;   // switch must be released before re-arming

// ----------------------------------------------------------------- ISRs ---

// Shared edge handler. Kept tiny: one micros() call and a few stores.
void handleEdge(uint8_t i, bool level) {
  unsigned long now = micros();
  if (level) {
    chRiseUs[i] = now;
  } else if (chRiseUs[i] != 0) {
    unsigned long w = now - chRiseUs[i];
    chRiseUs[i] = 0;
    // Reject obvious garbage here, before storing. If a glitched pulse were
    // recorded, it would refresh the timestamp too - so the channel would
    // look FRESH while carrying nonsense, and the timeout would never fire.
    if (w >= (unsigned long)PULSE_MIN_US && w <= (unsigned long)PULSE_MAX_US) {
      chWidthUs[i] = (unsigned int)w;
      chLastUs[i]  = now;
      chSeen[i]    = true;
    }
  }
}

void throttleISR() { handleEdge(CH_THROTTLE, (PIND & _BV(PD2)) != 0); }
void yawISR()      { handleEdge(CH_YAW,      (PIND & _BV(PD3)) != 0); }

// Pin-change interrupt for port D. Only PD4 is unmasked, so any entry here
// is an edge on the arm channel.
ISR(PCINT2_vect) { handleEdge(CH_ARM, (PIND & _BV(PD4)) != 0); }

// ------------------------------------------------------------ helpers ----

// Copy all channels out from under the ISRs in one atomic block. The 16- and
// 32-bit values are read one byte at a time on an 8-bit AVR, so an interrupt
// landing mid-read would return a value that is half old and half new.
void sampleChannels() {
  unsigned int  w[NUM_CHANNELS];
  unsigned long t[NUM_CHANNELS];
  bool          s[NUM_CHANNELS];
  uint8_t i;

  noInterrupts();
  for (i = 0; i < NUM_CHANNELS; i++) {
    w[i] = chWidthUs[i];
    t[i] = chLastUs[i];
    s[i] = chSeen[i];
  }
  interrupts();

  unsigned long now = micros();
  for (i = 0; i < NUM_CHANNELS; i++) {
    chWidth[i] = (int)w[i];
    // Unsigned subtraction stays correct across the ~71 minute micros() wrap.
    chFresh[i] = s[i] && (now - t[i]) < SIGNAL_TIMEOUT_US;
  }
}

// Squares the input while preserving its sign, so full reverse stays full
// reverse. Plain x*x flips the sign of every negative input.
int applyExpo(int value) {
  float x = value / 500.0f;
  x = x * fabs(x);
  return (int)lround(x * 500.0f);
}

int applyDeadband(int offset) {
  return (abs(offset) < DEADBAND_US) ? 0 : offset;
}

void writeNeutral() {
  leftESC.writeMicroseconds(PULSE_MID_US);
  rightESC.writeMicroseconds(PULSE_MID_US);
}

// --------------------------------------------------------------- setup ----

void setup() {
  if (DEBUG_SERIAL) Serial.begin(115200);

  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_YAW, INPUT);
  pinMode(PIN_ARM, INPUT);

  leftESC.attach(PIN_ESC_LEFT);
  rightESC.attach(PIN_ESC_RIGHT);

  // The 1080 G2 arms on a steady neutral at power-up. Send it immediately and
  // hold it. writeMicroseconds(0) does NOT mean "no signal" - the Servo
  // library clamps it up to 544us, which is a real (and invalid) pulse.
  writeNeutral();

  attachInterrupt(digitalPinToInterrupt(PIN_THROTTLE), throttleISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_YAW), yawISR, CHANGE);

  // Pin-change interrupt on PD4. attachInterrupt() cannot do this - only
  // D2 and D3 have dedicated external-interrupt hardware on an ATmega328P.
  PCICR  |= _BV(PCIE2);      // enable pin-change interrupts for port D
  PCMSK2 |= _BV(PCINT20);    // unmask PD4 only

  delay(1000);  // let the receiver settle before believing anything
}

// ---------------------------------------------------------------- loop ----

void loop() {
  static unsigned long nextUpdate = 0;
  static unsigned long lastDebug = 0;

  unsigned long now = millis();
  if ((long)(now - nextUpdate) < 0) return;
  nextUpdate = now + 20;   // 50 Hz, matched to the receiver's frame rate

  sampleChannels();

  bool signalGood = chFresh[CH_THROTTLE] && chFresh[CH_YAW] && chFresh[CH_ARM];

  // --- arming state machine ---------------------------------------------
  if (!signalGood) {
    armed = false;
    armLatched = false;
  } else {
    bool switchArmed = ARM_WHEN_ABOVE
                         ? (chWidth[CH_ARM] > ARM_THRESHOLD_US)
                         : (chWidth[CH_ARM] < ARM_THRESHOLD_US);
    if (!switchArmed) {
      armed = false;
      armLatched = false;     // switch released: allow arming again
    } else if (!armed && !armLatched) {
      // Only arm from a genuinely neutral stick position. Without this,
      // flipping the switch while holding throttle launches the car.
      bool sticksCentred =
          abs(chWidth[CH_THROTTLE] - PULSE_MID_US) < ARM_NEUTRAL_TOLERANCE_US &&
          abs(chWidth[CH_YAW]      - PULSE_MID_US) < ARM_NEUTRAL_TOLERANCE_US;
      if (sticksCentred) {
        armed = true;
      } else {
        // Switch is on but sticks were not centred. Latch it off so the pilot
        // must cycle the switch, rather than having the car leap the instant
        // the sticks happen to pass through centre.
        armLatched = true;
      }
    }
  }

  // --- mix ---------------------------------------------------------------
  if (!armed) {
    writeNeutral();
  } else {
    int throttle = applyDeadband(chWidth[CH_THROTTLE] - PULSE_MID_US);
    int yawCmd   = applyDeadband(chWidth[CH_YAW]      - PULSE_MID_US);

    if (USE_EXPO) {
      throttle = applyExpo(throttle);
      yawCmd   = applyExpo(yawCmd);
    }

    int left  = throttle + yawCmd;
    int right = throttle - yawCmd;

    if (INVERT_LEFT)  left  = -left;
    if (INVERT_RIGHT) right = -right;

    left  = constrain(left  + PULSE_MID_US + LEFT_TRIM_US,
                      OUTPUT_MIN_US, OUTPUT_MAX_US);
    right = constrain(right + PULSE_MID_US + RIGHT_TRIM_US,
                      OUTPUT_MIN_US, OUTPUT_MAX_US);

    leftESC.writeMicroseconds(left);
    rightESC.writeMicroseconds(right);
  }

  // --- debug -------------------------------------------------------------
  if (DEBUG_SERIAL && (now - lastDebug) >= 100) {
    lastDebug = now;
    Serial.print(F("thr ")); Serial.print(chWidth[CH_THROTTLE]);
    Serial.print(chFresh[CH_THROTTLE] ? F(" ok") : F(" STALE"));
    Serial.print(F(" | yaw ")); Serial.print(chWidth[CH_YAW]);
    Serial.print(chFresh[CH_YAW] ? F(" ok") : F(" STALE"));
    Serial.print(F(" | arm ")); Serial.print(chWidth[CH_ARM]);
    Serial.print(chFresh[CH_ARM] ? F(" ok") : F(" STALE"));
    Serial.print(F(" | armed ")); Serial.println(armed ? F("YES") : F("no"));
  }
}

/*
 * FIRST-RUN PROCEDURE
 *
 * 1. WHEELS OFF THE GROUND, or motors unplugged from the ESCs.
 * 2. Upload, open Serial Monitor at 115200.
 * 3. With the transmitter OFF you should see STALE on every channel and
 *    "armed no". If any channel reads ok with the TX off, the wiring is
 *    picking up something it should not.
 * 4. Turn the TX on. All three should read ok with plausible values.
 * 5. Release the sticks and check thr and yaw read close to 1500. If they sit
 *    far off (transmitter trim), either zero the trim or widen
 *    ARM_NEUTRAL_TOLERANCE_US, or the car will never arm.
 * 6. Flip the arm switch down and note the arm pulse width. If down gives a
 *    value ABOVE 1500, set ARM_WHEN_ABOVE = true and re-upload.
 * 7. Confirm "armed YES" appears only with the switch down and sticks centred.
 * 8. Confirm turning the TX off mid-run immediately shows STALE and
 *    "armed no", and that both motors stop.
 *
 * Only after step 8 passes should the car go on the ground.
 */
