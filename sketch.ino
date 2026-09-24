/*
  =====================================================================
   AUV FAULT DETECTION & RECOVERY SYSTEM — DEMO
   Project: BRACU Duburi — AUV Fault Detection and Recovery Subsystem
   Platform: Wokwi / TinkerCAD Circuits (Arduino Uno)
  =====================================================================

  CONCEPT
  -------
  A real AUV loses radio contact once submerged, so it must detect its
  own faults and react autonomously. This demo simulates 4 common AUV
  fault modes using potentiometers/buttons (standing in for real
  sensors) and shows the vehicle's onboard logic detecting each fault
  and executing an automatic recovery action — exactly like the real
  vehicle would.

  SIMULATED FAULTS               REAL SENSOR IT STANDS IN FOR
  -----------------------------  --------------------------------------
  A0 Depth potentiometer      -> Pressure/depth transducer
  A1 Battery potentiometer    -> Battery voltage monitor
  A2 Motor current pot        -> Thruster current sensor (e.g. ACS712)
  A3 Tilt potentiometer       -> IMU roll/pitch (attitude)
  D13 Leak pushbutton         -> Hull leak/moisture sensor

  RECOVERY ACTION
  ----------------
  On any FAULT: thrusters are commanded OFF (logged), the buzzer and
  red LED alarm, and a servo fires an "emergency ballast / marker buoy
  release" — the same abort action a real AUV takes to become
  positively buoyant and surface for recovery.

  STATE MACHINE
  --------------
  NOMINAL -> WARNING -> FAULT -> RECOVERY -> SURFACED

  =====================================================================
*/

#include <LiquidCrystal.h>
#include <Servo.h>

// ---------- Pin map ----------
const int PIN_DEPTH   = A0;
const int PIN_BATTERY = A1;
const int PIN_CURRENT = A2;
const int PIN_TILT    = A3;

const int PIN_LEAK_BTN = 13;   // INPUT_PULLUP, LOW = leak detected

const int PIN_LED_GREEN  = 6;
const int PIN_LED_YELLOW = 7;
const int PIN_LED_RED    = 8;

const int PIN_SERVO  = 9;
const int PIN_BUZZER = 10;

// LCD: RS, E, D4, D5, D6, D7
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
Servo ballastServo;

// ---------- Thresholds ----------
const float DEPTH_MAX_SAFE_M   = 80.0;   // fault beyond this depth
const float DEPTH_WARN_M       = 60.0;

const float BATT_WARN_V   = 12.5;
const float BATT_FAULT_V  = 11.0;

const float CURRENT_WARN_A  = 3.0;
const float CURRENT_FAULT_A = 4.5;

const float TILT_WARN_DEG  = 25.0;
const float TILT_FAULT_DEG = 45.0;

// ---------- State machine ----------
enum SystemState { NOMINAL, WARNING, FAULT, RECOVERY, SURFACED };
SystemState state = NOMINAL;
SystemState prevState = NOMINAL;

const int SERVO_STOWED   = 0;   // ballast/buoy retained
const int SERVO_RELEASED = 90;  // ballast dropped / buoy released

unsigned long lastBeep = 0;
bool buzzerOn = false;
unsigned long lastLog = 0;

String faultReason = "";

void setup() {
  Serial.begin(9600);
  pinMode(PIN_LEAK_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  ballastServo.attach(PIN_SERVO);
  ballastServo.write(SERVO_STOWED);

  lcd.begin(16, 2);
  lcd.print("AUV F.D.R. Demo");
  lcd.setCursor(0, 1);
  lcd.print("BRACU Duburi");
  delay(2000);
  lcd.clear();

  Serial.println(F("=== AUV Fault Detection & Recovery System — BOOT OK ==="));
}

void loop() {
  // ---------- Read & convert simulated sensors ----------
  float depth   = map(analogRead(PIN_DEPTH), 0, 1023, 0, 1200) / 10.0;      // 0-120.0 m
  float battery = 9.0 + (analogRead(PIN_BATTERY) / 1023.0) * (16.8 - 9.0);  // 9.0-16.8 V
  float current = (analogRead(PIN_CURRENT) / 1023.0) * 6.0;                 // 0-6.0 A
  float tilt    = map(analogRead(PIN_TILT), 0, 1023, -60, 60);              // -60..60 deg
  bool  leak    = (digitalRead(PIN_LEAK_BTN) == LOW);

  // ---------- Fault detection logic ----------
  bool faultLevel = false, warnLevel = false;
  faultReason = "";

  if (leak) { faultLevel = true; faultReason += "LEAK "; }

  if (depth > DEPTH_MAX_SAFE_M) { faultLevel = true; faultReason += "DEPTH "; }
  else if (depth > DEPTH_WARN_M) { warnLevel = true; }

  if (battery < BATT_FAULT_V) { faultLevel = true; faultReason += "BATTERY "; }
  else if (battery < BATT_WARN_V) { warnLevel = true; }

  if (current > CURRENT_FAULT_A) { faultLevel = true; faultReason += "THRUSTER_STALL "; }
  else if (current > CURRENT_WARN_A) { warnLevel = true; }

  if (abs(tilt) > TILT_FAULT_DEG) { faultLevel = true; faultReason += "ATTITUDE "; }
  else if (abs(tilt) > TILT_WARN_DEG) { warnLevel = true; }

  // ---------- State transition ----------
  prevState = state;
  if (faultLevel) {
    state = (state == RECOVERY || state == SURFACED) ? SURFACED : FAULT;
  } else if (warnLevel) {
    state = (state == SURFACED) ? SURFACED : WARNING;
  } else {
    state = (state == SURFACED) ? SURFACED : NOMINAL; // stays SURFACED until reset
  }

  // Edge: just entered FAULT -> run recovery sequence once
  if (state == FAULT && prevState != FAULT && prevState != RECOVERY && prevState != SURFACED) {
    triggerRecovery();
    state = RECOVERY;
  }

  // ---------- Outputs ----------
  updateIndicators();
  updateLCD(depth, battery, current, tilt, leak);
  handleBuzzer();

  if (millis() - lastLog > 700) {
    lastLog = millis();
    logToSerial(depth, battery, current, tilt, leak);
  }

  delay(50);
}

// Executes the automated abort/recovery sequence — the core deliverable
void triggerRecovery() {
  Serial.println(F(">>> FAULT DETECTED — EXECUTING AUTONOMOUS RECOVERY <<<"));
  Serial.print(F("    Fault(s): "));
  Serial.println(faultReason);
  Serial.println(F("    Action 1: Thrusters commanded OFF"));
  Serial.println(F("    Action 2: Releasing emergency ballast / marker buoy"));

  lcd.clear();
  lcd.print("FAULT: " + faultReason.substring(0, 9));
  lcd.setCursor(0, 1);
  lcd.print("RECOVERY ACTIVE");

  ballastServo.write(SERVO_RELEASED);
  delay(600); // allow servo to complete the release motion

  Serial.println(F("    Action 3: Vehicle set to ascend / surface"));
  Serial.println(F(">>> Recovery sequence complete — vehicle SURFACED <<<"));
}

void updateIndicators() {
  digitalWrite(PIN_LED_GREEN,  state == NOMINAL);
  digitalWrite(PIN_LED_YELLOW, state == WARNING);
  digitalWrite(PIN_LED_RED,    (state == FAULT || state == RECOVERY || state == SURFACED));
}

void handleBuzzer() {
  bool shouldAlarm = (state == FAULT || state == RECOVERY || state == SURFACED);
  if (!shouldAlarm) { digitalWrite(PIN_BUZZER, LOW); return; }

  // Slow chirp once surfaced (locator beacon), fast alarm during active fault
  unsigned long interval = (state == SURFACED) ? 1000 : 250;
  if (millis() - lastBeep > interval) {
    lastBeep = millis();
    buzzerOn = !buzzerOn;
    digitalWrite(PIN_BUZZER, buzzerOn ? HIGH : LOW);
  }
}

void updateLCD(float depth, float battery, float current, float tilt, bool leak) {
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD < 400) return; // limit refresh rate
  lastLCD = millis();

  if (state == RECOVERY) return; // recovery message already shown, hold it briefly

  lcd.clear();
  switch (state) {
    case NOMINAL:  lcd.print("STATUS: NOMINAL"); break;
    case WARNING:  lcd.print("STATUS: WARNING"); break;
    case FAULT:    lcd.print("STATUS: FAULT!"); break;
    case SURFACED: lcd.print("SURFACED - locate"); break;
  }
  lcd.setCursor(0, 1);
  lcd.print("D:" + String(depth, 0) + "m V:" + String(battery, 1));
}

void logToSerial(float depth, float battery, float current, float tilt, bool leak) {
  Serial.print(F("t=")); Serial.print(millis() / 1000.0, 1);
  Serial.print(F("s | STATE=")); Serial.print(stateName(state));
  Serial.print(F(" | depth=")); Serial.print(depth, 1); Serial.print(F("m"));
  Serial.print(F(" | batt=")); Serial.print(battery, 2); Serial.print(F("V"));
  Serial.print(F(" | current=")); Serial.print(current, 2); Serial.print(F("A"));
  Serial.print(F(" | tilt=")); Serial.print(tilt, 0); Serial.print(F("deg"));
  Serial.print(F(" | leak=")); Serial.print(leak ? "YES" : "no");
  if (faultReason.length() > 0) { Serial.print(F(" | FAULTS: ")); Serial.print(faultReason); }
  Serial.println();
}

String stateName(SystemState s) {
  switch (s) {
    case NOMINAL:  return "NOMINAL";
    case WARNING:  return "WARNING";
    case FAULT:    return "FAULT";
    case RECOVERY: return "RECOVERY";
    case SURFACED: return "SURFACED";
  }
  return "?";
}