/*
 *
 * STAGE 8 - RC PLANE: motor (ESC) + 6 servos, from an ELRS receiver over CRSF.
 * 
 *
 * CONTROL MAP:  (letter = your label, CH = transmitter channel, GPIO = ESP32 pin)
 *
 *   A  Aileron L   CH1   GPIO18
 *   B  Aileron R   CH1   GPIO19   (reversed - moves opposite to A)
 *   C  Elevator    CH2   GPIO4    (GPIO4 required for elevator)
 *   -  Motor/ESC   CH3   GPIO13
 *   D  Rudder      CH4   GPIO21
 *   E  Flap L      CH5   GPIO14
 *   F  Flap R      CH5   GPIO15
 *
 *
 *
 * NUMBERS: stick range vs servo travel
 *
 *   stickMin/stickMax.
 *   The SERVO OUTPUT range (outMin/outPark/outMax) is how far the servo horn
 *   can physically move before it binds. Widen them per servo after checking travel (see HOW TO CALIBRATE).
 *
 *
 *
 * WIRING: (4S pack, Hobbywing Skywalker 50A, 5V UBEC)
 *
 *   Pack + / -    -> ESC power in
 *   ESC UBEC 5V   -> every servo V+ (red)   AND receiver 5V
 *   ESC UBEC GND  -> every servo GND (brown) AND receiver GND AND ESP32 GND
 *   ESP32 GPIO13  -> ESC signal
 *   ESP32 GPIO4/14/15/18/19/21 -> servo signal wires (see map above)
 *   ESP32 GPIO16  <- receiver TX   (serial CROSSES: their TX -> our RX)
 *   ESP32 GPIO17  -> receiver RX
 *   ESP32 VIN     -> NOT CONNECTED. ESP32 is powered by USB on the bench.
 *   470-1000uF capacitor across the 5V rail, near the servos.
 *
 * GPIO15 IS A "STRAPPING" PIN: if it is pulled LOW at power-up the ESP32 may
 * refuse to boot. A servo signal line sits low until this sketch runs. If the
 * board won't start with Flap R (F) plugged in, unplug just its SIGNAL wire
 * while powering up, or move F to a clean pin (22, 23, 25, 26, 27, 32, 33).
 *
 * ---------------------------------------------------------------------------
 * SAFETY - READ BEFORE CONNECTING THE PACK
 * ---------------------------------------------------------------------------
 *   - PROP OFF. Motor clamped. Hands clear.
 *   - Servo horns OFF for the first run - confirm direction and travel first.
 *   - Throttle stick fully DOWN before connecting the pack.
 *   - Throttle is capped at 40% (ESC_MAX_TEST) until you raise it on purpose.
 *   - Nothing is attached to any output pin until the radio link is up AND the
 *     throttle is confirmed at minimum. If either check fails, the sketch stops.
 *   - If the link drops: motor is cut instantly, servos glide to neutral,
 *     then go limp.
 * ============================================================================
 */

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>


/* ========================================================================
 * SECTION 1: PIN AND CHANNEL CONSTANTS
 * ====================================================================== */

// Receiver serial link. Serial always crosses: the receiver's TX (output)
// must land on the ESP32's RX (input), which is GPIO16.
#define PIN_RX 16   // GPIO16 <- receiver TX
#define PIN_TX 17   // GPIO17 -> receiver RX

const int ESC_PIN = 13;   // motor speed controller signal

// CRSF channel numbers (AlfredoCRSF counts channels starting at 1).
const int CH_AILERON  = 1;
const int CH_ELEVATOR = 2;
const int CH_THROTTLE = 3;
const int CH_RUDDER   = 4;
const int CH_FLAPS    = 5;


/* ========================================================================
 * SECTION 2: MOTOR (ESC) SETTINGS
 * ====================================================================== */

const int ESC_MIN      = 1000;  // pulse width for "motor stopped / armed"
const int ESC_FULL     = 2012;  // pulse width for full throttle
const int ESC_MAX_TEST = 1400;  // TEST CAP ~40%. Set to ESC_FULL when ready.
const int TH_DEADBAND  = 1030;  // throttle at/below this = motor forced off

// The motor is ramped UP gently but cut instantly (see loop()).
const int ESC_SLEW_US  = 20;    // max throttle increase per 20 ms cycle


/* ========================================================================
 * SECTION 3: THE SERVO TABLE  <-- this is where you tune everything
 * ------------------------------------------------------------------------
 * One row per servo. Fields:
 *   name        - label shown in the serial monitor
 *   pin         - ESP32 GPIO the signal wire connects to
 *   channel     - which transmitter channel this servo follows
 *   reversed    - false = normal, true = moves the opposite way
 *   stickMin/Center/Max - the INPUT range from the TX (the ~988-2012 numbers)
 *   outMin/Park/Max     - the servo's OUTPUT travel (start narrow, calibrate)
 * ====================================================================== */

const int NUM_SERVOS = 6;

struct ServoConfig {
  const char* name;
  int  pin;
  int  channel;
  bool reversed;
  int  stickMin, stickCenter, stickMax;
  int  outMin,   outPark,     outMax;
};

ServoConfig servoConfig[NUM_SERVOS] = {
  // name          pin  channel        reversed  stickMin  Center  stickMax   outMin  Park  outMax
  {  "AileronL",    18, CH_AILERON,    false,      988,    1500,    2012,      1300,  1500,  1700 },
  {  "AileronR",    19, CH_AILERON,    true,       988,    1500,    2012,      1300,  1500,  1700 },
  {  "Elevator",     4, CH_ELEVATOR,   false,      993,    1500,    2012,      1300,  1500,  1700 },
  {  "Rudder",      21, CH_RUDDER,     false,      998,    1500,    2012,      1300,  1500,  1700 },
  {  "FlapL",       14, CH_FLAPS,      false,      988,    1500,    2012,      1300,  1500,  1700 },
  {  "FlapR",       15, CH_FLAPS,      false,      988,    1500,    2012,      1300,  1500,  1700 },
};

// One neutral zone for all sticks: inside this window around center, the
// servo holds at its park position instead of twitching on tiny stick noise.
const int STICK_DEADZONE = 8;


/* ========================================================================
 * SECTION 4: TIMING AND FAILSAFE CONSTANTS
 * ====================================================================== */

const unsigned long LINK_TIMEOUT_MS = 400;    // no fresh data for this long = link lost
const unsigned long ARM_WAIT_MS     = 15000;  // how long to wait for link/throttle at startup
const unsigned long ESC_ARM_HOLD_MS = 3000;   // hold min throttle so the ESC arms
const unsigned long LOOP_PERIOD_MS  = 20;     // main loop runs ~50x per second
const int RAMP_STEP_US = 20;                   // servo step size when gliding to park
const int RAMP_STEP_MS = 15;                   // delay between those steps


/* ========================================================================
 * SECTION 5: GLOBAL OBJECTS AND STATE
 * ====================================================================== */

HardwareSerial crsfSerial(2);   // ESP32 hardware serial port #2, used for CRSF
AlfredoCRSF crsf;               // decodes the CRSF data from the receiver

Servo esc;                     // the motor controller, driven like a servo
Servo servos[NUM_SERVOS];      // one driver object per servo, same order as the table
int   servoPos[NUM_SERVOS];    // each servo's current commanded pulse (microseconds)

int  posEsc = ESC_MIN;         // motor's current commanded pulse
bool outputsLive = false;      // true once outputs are attached and safe to drive
unsigned long lastGoodFrame = 0;  // millis() timestamp of the last healthy link check


/* ========================================================================
 * SECTION 6: HELPER FUNCTIONS
 * ====================================================================== */

// Convert a stick reading into a servo pulse width for one servo.
// Handles the center deadzone and the "reversed" flag.
int computeServoTarget(const ServoConfig& cfg, int stick) {
  // Never trust the input past the known stick range.
  stick = constrain(stick, cfg.stickMin, cfg.stickMax);

  // Close enough to center -> park (no jitter).
  if (abs(stick - cfg.stickCenter) <= STICK_DEADZONE) return cfg.outPark;

  // Decide which physical end each stick direction drives toward.
  // Reversing simply swaps those two ends.
  int highEnd = cfg.reversed ? cfg.outMin : cfg.outMax;  // where "stick pushed up" goes
  int lowEnd  = cfg.reversed ? cfg.outMax : cfg.outMin;  // where "stick pushed down" goes

  int out;
  if (stick > cfg.stickCenter) {
    // Upper half of stick travel -> park..highEnd
    out = map(stick, cfg.stickCenter + STICK_DEADZONE, cfg.stickMax, cfg.outPark, highEnd);
  } else {
    // Lower half of stick travel -> lowEnd..park
    out = map(stick, cfg.stickMin, cfg.stickCenter - STICK_DEADZONE, lowEnd, cfg.outPark);
  }

  // Final clamp so we can never command past the servo's travel limits,
  // whichever of outMin/outMax is smaller/larger.
  int lo = min(cfg.outMin, cfg.outMax);
  int hi = max(cfg.outMin, cfg.outMax);
  return constrain(out, lo, hi);
}

// Convert the throttle stick into an ESC pulse, respecting the deadband and cap.
int computeThrottle(int stick) {
  if (stick <= TH_DEADBAND) return ESC_MIN;             // stick down = off
  int out = map(stick, TH_DEADBAND, ESC_FULL, ESC_MIN, ESC_MAX_TEST);
  return constrain(out, ESC_MIN, ESC_MAX_TEST);
}

// Move a value one limited step toward a target (used to ramp the motor up gently).
int slewToward(int current, int target, int maxStep) {
  int delta = target - current;
  if (abs(delta) <= maxStep) return target;
  return current + (delta > 0 ? maxStep : -maxStep);
}

// Glide EVERY servo to its park position at the same time, then return.
// Used on failsafe so control surfaces settle to neutral instead of snapping.
void parkAllServosGently() {
  bool stillMoving = true;
  while (stillMoving) {
    stillMoving = false;
    for (int i = 0; i < NUM_SERVOS; i++) {
      int target = servoConfig[i].outPark;
      if (servoPos[i] != target) {
        if (abs(target - servoPos[i]) <= RAMP_STEP_US) {
          servoPos[i] = target;
        } else {
          servoPos[i] += (target > servoPos[i]) ? RAMP_STEP_US : -RAMP_STEP_US;
        }
        servos[i].writeMicroseconds(servoPos[i]);
        stillMoving = true;
      }
    }
    crsf.update();          // keep the link serviced while we ramp
    delay(RAMP_STEP_MS);
  }
}

// Attach the ESC and all servos, each at a safe starting pulse.
// ORDER MATTERS: the ESC gets minimum throttle as its very first pulse.
void attachOutputs() {
  // ESP32Servo shares 4 hardware timers among all PWM channels. Claim them.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Motor first, at minimum, before any servo can move.
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, ESC_MIN, ESC_FULL);
  esc.writeMicroseconds(ESC_MIN);
  posEsc = ESC_MIN;

  // Then each servo, parked as its first pulse (prevents a startup slam).
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoConfig[i].pin, servoConfig[i].outMin, servoConfig[i].outMax);
    servos[i].writeMicroseconds(servoConfig[i].outPark);
    servoPos[i] = servoConfig[i].outPark;
  }

  outputsLive = true;
}

// Link lost: cut the motor now, glide servos to neutral, then let them go limp.
void failsafeShutdown() {
  if (!outputsLive) return;
  Serial.println("FAILSAFE: link lost. Motor off, servos parking.");

  esc.writeMicroseconds(ESC_MIN);   // instant, no ramp
  posEsc = ESC_MIN;

  parkAllServosGently();            // surfaces glide to neutral

  for (int i = 0; i < NUM_SERVOS; i++) servos[i].detach();  // go limp
  // The ESC stays attached at minimum so it never sees a floating signal line.

  outputsLive = false;
  Serial.println("Servos detached. ESC held at minimum. Waiting for link.");
}

// Link came back after a failsafe: re-attach only once throttle is at minimum.
void reArm() {
  Serial.println("Link back. Re-checking throttle before re-arming...");
  unsigned long start = millis();
  while (millis() - start < 5000) {
    crsf.update();
    if (crsf.isLinkUp() && crsf.getChannel(CH_THROTTLE) <= TH_DEADBAND) {
      for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(servoConfig[i].pin, servoConfig[i].outMin, servoConfig[i].outMax);
        servos[i].writeMicroseconds(servoConfig[i].outPark);
        servoPos[i] = servoConfig[i].outPark;
      }
      outputsLive = true;
      Serial.println("Re-armed at neutral.");
      return;
    }
    delay(20);
  }
  Serial.println("Throttle not at minimum - lower the stick to re-arm.");
}


/* ========================================================================
 * SECTION 7: SETUP  (runs once at power-up)
 * ====================================================================== */

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("STAGE 8: plane control - ESC + 6 servos");
  Serial.printf("Throttle test cap: %d us (%d%% of full)\n",
                ESC_MAX_TEST, (ESC_MAX_TEST - ESC_MIN) * 100 / (ESC_FULL - ESC_MIN));

  // Open the serial link to the receiver.
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);

  // --- GATE 1: wait for the radio link to come up ---
  Serial.println("Waiting for CRSF link...");
  unsigned long t0 = millis();
  bool linkOk = false;
  while (millis() - t0 < ARM_WAIT_MS) {
    crsf.update();
    if (crsf.isLinkUp()) { linkOk = true; break; }
    delay(10);
  }
  if (!linkOk) {
    Serial.println("ABORT: no link. Nothing attached.");
    Serial.println("Check receiver power, bind, and that receiver TX -> GPIO16.");
    while (1) delay(1000);   // stop here; safest to do nothing
  }

  // --- GATE 2: wait for the throttle stick to be at minimum ---
  Serial.println("Link up. Waiting for throttle at minimum...");
  t0 = millis();
  bool thrOk = false;
  while (millis() - t0 < ARM_WAIT_MS) {
    crsf.update();
    if (crsf.getChannel(CH_THROTTLE) <= TH_DEADBAND) { thrOk = true; break; }
    delay(10);
  }
  if (!thrOk) {
    Serial.printf("ABORT: throttle at %d us, needs <= %d. Nothing attached. Reset to retry.\n",
                  crsf.getChannel(CH_THROTTLE), TH_DEADBAND);
    while (1) delay(1000);
  }

  // Both safety gates passed - now it's safe to drive the outputs.
  Serial.println("Both gates passed. Attaching outputs.");
  attachOutputs();

  // Hold minimum throttle so the ESC completes its arming beeps.
  Serial.println("Arming ESC - holding minimum, listen for beeps...");
  unsigned long armStart = millis();
  while (millis() - armStart < ESC_ARM_HOLD_MS) {
    crsf.update();
    esc.writeMicroseconds(ESC_MIN);
    delay(20);
  }

  lastGoodFrame = millis();
  Serial.println("Ready. Motor CH3 (capped). Surfaces live.");
}


/* ========================================================================
 * SECTION 8: LOOP  (runs ~50x per second, forever)
 * ====================================================================== */

void loop() {
  crsf.update();   // pull the latest data from the receiver every cycle

  // Is the link healthy? Up, AND we've seen fresh data recently.
  bool linkUp = crsf.isLinkUp();
  if (linkUp) lastGoodFrame = millis();
  bool healthy = linkUp && (millis() - lastGoodFrame) <= LINK_TIMEOUT_MS;

  // Default everything to a safe target.
  int rawThrottle = 0;
  int targetEsc = ESC_MIN;
  int targetServo[NUM_SERVOS];
  int rawStick[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) targetServo[i] = servoConfig[i].outPark;

  if (healthy) {
    // Read the throttle and compute the capped ESC pulse.
    rawThrottle = crsf.getChannel(CH_THROTTLE);
    targetEsc = computeThrottle(rawThrottle);

    // Read each servo's channel and compute its target pulse.
    for (int i = 0; i < NUM_SERVOS; i++) {
      rawStick[i] = crsf.getChannel(servoConfig[i].channel);
      targetServo[i] = computeServoTarget(servoConfig[i], rawStick[i]);
    }
  } else {
    failsafeShutdown();   // link bad -> cut motor, park + detach servos
  }

  if (outputsLive) {
    // Motor: ramp UP gently, but drop INSTANTLY (asymmetric on purpose).
    posEsc = (targetEsc < posEsc) ? targetEsc : slewToward(posEsc, targetEsc, ESC_SLEW_US);
    posEsc = constrain(posEsc, ESC_MIN, ESC_MAX_TEST);
    esc.writeMicroseconds(posEsc);

    // Servos: write directly for immediate, full-speed response.
    for (int i = 0; i < NUM_SERVOS; i++) {
      servoPos[i] = targetServo[i];
      servos[i].writeMicroseconds(servoPos[i]);
    }
  } else if (healthy) {
    reArm();   // link recovered while safed -> try to re-arm
  }

  // Print a status line ~5x per second so you can watch what's happening.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.printf("%s Thr:%4d->%4d | ", healthy ? "UP  " : "DOWN", rawThrottle, posEsc);
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.printf("%s:%4d ", servoConfig[i].name, servoPos[i]);
    }
    Serial.println(outputsLive ? "" : " [safed]");
  }

  delay(LOOP_PERIOD_MS);
}


/* ============================================================================
 * HOW TO CALIBRATE A SERVO (do this with the horn OFF)
 * ----------------------------------------------------------------------------
 * The outMin/outPark/outMax fields start narrow (1300/1500/1700) so nothing
 * slams. To find a servo's real limits:
 *   1. Upload, open the serial monitor, move that servo's stick fully both ways.
 *   2. Watch its "Name:####" value follow the stick between outMin and outMax.
 *   3. Widen outMin down / outMax up a little at a time (e.g. 1200, then 1100...).
 *   4. When the servo BUZZES without moving, it has hit its stop - back off ~50.
 *   5. If a horn/linkage is on, find the tighter mechanical limit the same way.
 * Most servos end up near 1000 (min) and 2000 (max) with 1500 as center.
 *
 * HOW TO REVERSE A SERVO
 *   Set its "reversed" field to true (or back to false). Nothing else changes.
 *
 * HOW TO LIFT THE THROTTLE CAP
 *   Change ESC_MAX_TEST to ESC_FULL (2012) once you trust the setup. Prop off
 *   until you do, and raise it in steps if unsure.
 * ==========================================================================*/
