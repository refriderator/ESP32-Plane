/*
 * Stage 8 - plane: ESC + 6 servos off the ELRS rx (CRSF)
 *
 *   A  Aileron L   CH1   GPIO18
 *   B  Aileron R   CH1   GPIO19   reversed
 *   C  Elevator    CH2   GPIO4
 *   -  ESC         CH3   GPIO13
 *   D  Rudder      CH4   GPIO21
 *   E  Flap L      CH5   GPIO14
 *   F  Flap R      CH5   GPIO15
 *
 * Wiring (4S, Skywalker 50A, 5V UBEC):
 *   UBEC 5V/GND -> all servos + rx, GND also to ESP32
 *   GPIO13 -> ESC signal
 *   GPIO16 <- rx TX, GPIO17 -> rx RX
 *   ESP32 on USB, VIN not connected
 *   470-1000uF cap on the 5V rail near the servos
 *
 * GPIO15 is a strapping pin - if it won't boot with flap R plugged in,
 * unplug its signal wire at power up or move it (22/23/25/26/27/32/33).
 *
 * Bench testing: prop off, horns off, throttle down before plugging in the pack.
 * Throttle capped at ~40% (ESC_MAX_TEST) for now.
 * Outputs only attach once link is up + throttle is at min.
 * Link lost -> motor off, servos ease back to center then detach.
 */

#include <AlfredoCRSF.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

#define PIN_RX 16   // rx TX goes here
#define PIN_TX 17

const int ESC_PIN = 13;

// channels (AlfredoCRSF starts at 1)
const int CH_AILERON  = 1;
const int CH_ELEVATOR = 2;
const int CH_THROTTLE = 3;
const int CH_RUDDER   = 4;
const int CH_FLAPS    = 5;

// motor
const int ESC_MIN      = 1000;
const int ESC_FULL     = 2012;
const int ESC_MAX_TEST = 1400;  // ~40%, change to ESC_FULL later
const int TH_DEADBAND  = 1030;  // below this = motor off
const int ESC_SLEW_US  = 20;    // max ramp up per loop

// servos
// stick = what the tx sends, out = servo travel (start narrow, widen once checked)
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

const int STICK_DEADZONE = 8;  // around center, stops jitter

// --- timing ---
const unsigned long LINK_TIMEOUT_MS = 400;
const unsigned long ARM_WAIT_MS     = 15000;
const unsigned long ESC_ARM_HOLD_MS = 3000;
const unsigned long LOOP_PERIOD_MS  = 20;     // ~50Hz
const int RAMP_STEP_US = 20;                   // for easing servos back to center
const int RAMP_STEP_MS = 15;


HardwareSerial crsfSerial(2);
AlfredoCRSF crsf;

Servo esc;
Servo servos[NUM_SERVOS];
int   servoPos[NUM_SERVOS];

int  posEsc = ESC_MIN;
bool outputsLive = false;
unsigned long lastGoodFrame = 0;


// stick -> servo pulse, with deadzone + reverse
int computeServoTarget(const ServoConfig& cfg, int stick) {
  stick = constrain(stick, cfg.stickMin, cfg.stickMax);

  if (abs(stick - cfg.stickCenter) <= STICK_DEADZONE) return cfg.outPark;

  int highEnd = cfg.reversed ? cfg.outMin : cfg.outMax;
  int lowEnd  = cfg.reversed ? cfg.outMax : cfg.outMin;

  int out;
  if (stick > cfg.stickCenter) {
    out = map(stick, cfg.stickCenter + STICK_DEADZONE, cfg.stickMax, cfg.outPark, highEnd);
  } else {
    out = map(stick, cfg.stickMin, cfg.stickCenter - STICK_DEADZONE, lowEnd, cfg.outPark);
  }

  int lo = min(cfg.outMin, cfg.outMax);
  int hi = max(cfg.outMin, cfg.outMax);
  return constrain(out, lo, hi);
}

int computeThrottle(int stick) {
  if (stick <= TH_DEADBAND) return ESC_MIN;
  int out = map(stick, TH_DEADBAND, ESC_FULL, ESC_MIN, ESC_MAX_TEST);
  return constrain(out, ESC_MIN, ESC_MAX_TEST);
}

int slewToward(int current, int target, int maxStep) {
  int delta = target - current;
  if (abs(delta) <= maxStep) return target;
  return current + (delta > 0 ? maxStep : -maxStep);
}

// ease all servos back to center together (failsafe)
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
    crsf.update();
    delay(RAMP_STEP_MS);
  }
}

void attachOutputs() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ESC first so its first pulse is min throttle
  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, ESC_MIN, ESC_FULL);
  esc.writeMicroseconds(ESC_MIN);
  posEsc = ESC_MIN;

  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoConfig[i].pin, servoConfig[i].outMin, servoConfig[i].outMax);
    servos[i].writeMicroseconds(servoConfig[i].outPark);
    servoPos[i] = servoConfig[i].outPark;
  }

  outputsLive = true;
}

void failsafeShutdown() {
  if (!outputsLive) return;
  Serial.println("FAILSAFE: link lost. Motor off, servos parking.");

  esc.writeMicroseconds(ESC_MIN);
  posEsc = ESC_MIN;

  parkAllServosGently();

  for (int i = 0; i < NUM_SERVOS; i++) servos[i].detach();
  // leave ESC attached at min so the signal line isn't floating

  outputsLive = false;
  Serial.println("Servos detached. ESC held at minimum. Waiting for link.");
}

// link back after failsafe - only re-attach if throttle is down
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


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("STAGE 8: plane control - ESC + 6 servos");
  Serial.printf("Throttle test cap: %d us (%d%% of full)\n",
                ESC_MAX_TEST, (ESC_MAX_TEST - ESC_MIN) * 100 / (ESC_FULL - ESC_MIN));

  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);

  // wait for link
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
    while (1) delay(1000);
  }

  // wait for throttle down
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

  Serial.println("Both gates passed. Attaching outputs.");
  attachOutputs();

  // hold min so the ESC arms
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


void loop() {
  crsf.update();

  bool linkUp = crsf.isLinkUp();
  if (linkUp) lastGoodFrame = millis();
  bool healthy = linkUp && (millis() - lastGoodFrame) <= LINK_TIMEOUT_MS;

  int rawThrottle = 0;
  int targetEsc = ESC_MIN;
  int targetServo[NUM_SERVOS];
  int rawStick[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) targetServo[i] = servoConfig[i].outPark;

  if (healthy) {
    rawThrottle = crsf.getChannel(CH_THROTTLE);
    targetEsc = computeThrottle(rawThrottle);

    for (int i = 0; i < NUM_SERVOS; i++) {
      rawStick[i] = crsf.getChannel(servoConfig[i].channel);
      targetServo[i] = computeServoTarget(servoConfig[i], rawStick[i]);
    }
  } else {
    failsafeShutdown();
  }

  if (outputsLive) {
    // ramp up slowly, cut instantly
    posEsc = (targetEsc < posEsc) ? targetEsc : slewToward(posEsc, targetEsc, ESC_SLEW_US);
    posEsc = constrain(posEsc, ESC_MIN, ESC_MAX_TEST);
    esc.writeMicroseconds(posEsc);

    for (int i = 0; i < NUM_SERVOS; i++) {
      servoPos[i] = targetServo[i];
      servos[i].writeMicroseconds(servoPos[i]);
    }
  } else if (healthy) {
    reArm();
  }

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

/*
 * calibrating (horns off): start at 1300/1500/1700, move the stick to each end
 * and watch the serial monitor. Widen outMin/outMax ~100 at a time until the
 * servo buzzes without moving, then back off ~50. Usually ends up ~1000-2000.
 */
