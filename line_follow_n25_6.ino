// ESP32 Analog PID Line Follower  (N25 motors + pushbutton modes)  -- v6
// Hardware: RLS08 x8 | TB6612FNG | N25 motors | STBY tied to 3.3V
// Button: 3V3 --- [button] --- GPIO22   (internal pulldown used)
//
//   1 click  -> CALIBRATION (sweep bot over white line + black surface for CAL_TIME_MS)
//   2 clicks -> LINE FOLLOWING (needs a valid calibration first)
//   any press while running -> stop, back to IDLE
//
// v3 changes (low sensor range, ~41..150 counts, white line on black):
//   - MIN_RANGE lowered 300 -> 100
//   - ADC reads averaged (ADC_SAMPLES) to cut noise
//   - noise floor on normalized readings (NOISE_FLOOR)
//   - calibration prints range per sensor + live min/max while sweeping
//   - optional lower ADC attenuation for more counts on small signals
//   - Ki set to 0 and BASE_SPEED lowered for first tests
//
// v4 changes (dead-end U-turn fix):
//   - Line-lost spin is now a proper U-turn: brake, kick-start, then spin at SPIN_SPEED
//   - Spin only ends when a CENTRE sensor (2..5) sees the line (before: ANY sensor, so the
//     PID grabbed the line at the array edge, slammed to +/-255 and ping-ponged)
//   - Dead end has no side info (error ~0) -> uses UTURN_DEFAULT_DIR instead of a random sign
//   - Sharp corners still use the side the line was last seen on
//   - SPIN_SPEED raised 60 -> 90 (60 PWM can stall N25 motors when pivoting)
//   - U-turn timeout: stops to IDLE if no line found after UTURN_TIMEOUT_MS
//
// v5 changes (U-turn looping fix):
//   - v4 handed the line straight to the PID at full +/-255 after the spin; it overshot, lost
//     the line, spun again... and every re-acquire reset the timeout, so it looped forever.
//   - New ALIGN step: after the spin finds the line, brake and gently pivot (low speed,
//     no reversing the PID at full power) until the line is centred, THEN start the PID.
//   - Timeout now covers the whole lost->spin->align->lost cycle, and only clears after the bot
//     has tracked the line steadily for STABLE_MS. A loop can no longer run forever.
//
// v6 changes (junctions + breaking out of loops):
//   - Junction detection (T, +, corners): both outer sensors on one side + a centre sensor on white.
//   - On a junction the bot creeps forward (axle over the crossing), reads which exits exist
//     (L / S / R), picks one by the left-hand rule (LEFT_HAND = false for right-hand), pivots
//     onto it, aligns, and carries on. Dead ends still U-turn.
//   - Loop breaker: every turn is tallied (+1 left, -1 right). Four net turns the same way =
//     it has gone round a loop. The next multi-exit junction then takes the NEXT exit instead of
//     the default one; each further lap skips one more junction first, so it tries different
//     branches instead of the same one forever. A dead end resets the tally.
//   - MAZE_MODE = false gives the old plain line follower.

// ---------------- Pins ----------------
const int PWMA = 4,  AIN1 = 16, AIN2 = 17;
const int PWMB = 14, BIN1 = 5,  BIN2 = 23;

const int BUTTON_PIN = 22;
const int LED_PIN    = 2;     // onboard LED on most ESP32 dev boards

// Replace with your actual ADC pins if different
const int sensorPins[8] = {34, 35, 32, 33, 25, 26, 27, 13};
const int weights[8]    = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

// White line on black: sensor should read HIGHER over the white line -> false.
// If your sensors read LOWER over the white line, set this to true.
const bool SENSOR_LOW_ON_LINE = false;

// ---------------- ADC ----------------
const bool USE_LOW_ATTEN = true;   // ~0-1.1 V full scale = more counts for small signals.
                                   // Set false if a sensor can output more than ~1.1 V.
const int  ADC_SAMPLES   = 4;      // reads averaged per sensor
const int  NOISE_FLOOR   = 150;    // normalized values (0-1000) below this count as 0
                                   // lower it if edges are lost, raise it if black jitters

// ---------------- Motor config (tune for N25) ----------------
const bool INVERT_LEFT  = false;  // set true if left motor spins backwards
const bool INVERT_RIGHT = true;   // set true if right motor spins backwards

const int BASE_SPEED = 50;   // low for first tests, raise once stable
const int MIN_SPEED  = 20;
// ---------------- Line lost / U-turn ----------------
const int  SPIN_SPEED       = 90;    // pivot speed while searching (60 was too weak to turn)
const int  SPIN_KICK_SPEED  = 160;   // short boost to overcome static friction at start of spin
const unsigned long SPIN_KICK_MS      = 40;
const unsigned long LOST_BRAKE_MS     = 30;    // brief brake before spinning
const unsigned long UTURN_TIMEOUT_MS  = 4000;  // give up (go IDLE) if line not found
const int  UTURN_DEFAULT_DIR = 1;    // dead end with no side info: +1 = spin right, -1 = spin left
const float SIDE_MEMORY_MIN  = 800;  // |lastError| above this = trust the side the line was lost on
const int  RECOVER_FIRST = 2;        // line counts as re-found when a sensor in this range ...
const int  RECOVER_LAST  = 5;        // ... (centre sensors)
const int  RECOVER_MIN   = 300;      // ... reads at least this (normalized 0-1000)

// Gentle alignment after the spin finds the line
const float ALIGN_TOL       = 600;   // |position| below this = centred enough, hand over to PID
const float ALIGN_KP        = 0.05;  // pivot speed per unit of position error
const int   ALIGN_MIN_SPEED = 70;    // must still be enough to turn the N25s
const int   ALIGN_MAX_SPEED = 110;
// Recovery counts as finished after the bot tracks the line steadily for this long
const float STABLE_ERR = 1500;       // |position| must stay below this ...
const unsigned long STABLE_MS = 400; // ... for this long

// ---------------- Junctions / maze ----------------
const bool MAZE_MODE  = true;    // false = plain line follower (no junction handling)
const bool LEFT_HAND  = true;    // true = prefer L > S > R, false = prefer R > S > L
const int  JUNC_ON_MIN = 500;    // normalized reading (0-1000) that counts as "on white"
const unsigned long JUNC_INCH_MS = 150;  // creep forward so the wheel axle sits over the junction
                                         // (tune: ~ distance from sensor bar to axle / bot speed)
const int  JUNC_INCH_SPEED = 50;
const int  TURN_SPEED      = 90;         // pivot speed at junctions
const unsigned long JUNC_TURN_TIMEOUT_MS = 3000;
const unsigned long JUNC_COOLDOWN_MS     = 250;  // ignore junctions right after one is handled
const int  LOOP_TURNS    = 4;    // net quarter-turns in one direction = went round a loop
const int  LOOP_MAX_SKIP = 3;    // how many junctions successive loop-breaks may skip first

// ---------------- PID ----------------
float Kp = 0.09;   // error range is +/-3500
float Ki = 0.001;    // start at 0; add 0.001-0.01 only if steady-state drift remains
float Kd = 0.6;    // re-tune: loop is slower now because of ADC averaging

const float SPEED_SCALE    = 0.03;
const float INTEGRAL_LIMIT = 3000;
const int   LINE_THRESHOLD = 200;   // min sum of normalized readings to say "line seen"

// ---------------- Serial Plotter ----------------
const bool          PLOT_ENABLED     = true;   // set false for race runs
const unsigned long PLOT_INTERVAL_MS = 20;     // ~50 Hz
const float         PLOT_ERR_SCALE   = 0.1;    // error is +/-3500, scaled so it shares an axis with PWM

// ---------------- Calibration ----------------
const unsigned long CAL_TIME_MS = 5000;
const int           MIN_RANGE   = 100;     // min (max-min) per sensor to accept calibration
const bool          AUTO_SPIN_CAL = false; // true = bot spins in place while calibrating

// ---------------- Button timing ----------------
const unsigned long DEBOUNCE_MS  = 30;
const unsigned long CLICK_GAP_MS = 350;  // max gap between clicks of a multi-click

// ---------------- State ----------------
enum Mode { IDLE, CALIBRATING, RUNNING };
Mode mode = IDLE;

int  sensorMin[8], sensorMax[8];
bool calibrated = false;

float integral  = 0;
float lastError = 0;
bool  lineLost  = false;
int   sensorVal[8];              // latest normalized readings (0-1000)
int   spinDir   = 1;             // +1 = spin right (clockwise), -1 = spin left
unsigned long lostStart = 0;
bool  aligning      = false;     // line found again, pivoting to centre it
bool  episodeActive = false;     // a lost/U-turn recovery is in progress
unsigned long episodeStart  = 0; // when this recovery began (survives re-acquire)
unsigned long lineSeenSince = 0; // last time the line was NOT steady

enum JState { J_NONE, J_INCH, J_TURN_LEAVE, J_TURN_FIND };
JState jstate = J_NONE;
bool   jL = false, jS = false, jR = false;   // exits found at the current junction
unsigned long jStart = 0, jTurnStart = 0, junctionCooldownUntil = 0;
int    turnDir   = 1;            // +1 pivot right, -1 pivot left
int    turnSum   = 0;            // +1 per left 90, -1 per right 90
bool   loopBreak = false;        // loop detected: next multi-exit junction takes an alternate
int    skipLeft  = 0;            // multi-exit junctions still to skip before the alternate
int    loopBreaks = 0;

// button internals
bool lastReading = false, stableState = false, pressEdge = false;
unsigned long lastChange = 0, lastRelease = 0;
int  clickCount = 0;

// ======================================================
void setup() {
  Serial.begin(115200);

  int outPins[] = {PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, LED_PIN};
  for (int p : outPins) pinMode(p, OUTPUT);

  pinMode(BUTTON_PIN, INPUT_PULLDOWN);  // pressed = HIGH (button to 3V3)

  if (USE_LOW_ATTEN) analogSetAttenuation(ADC_2_5db);

  stopMotors();
  Serial.println("Ready. 1 click = calibrate, 2 clicks = run.");
}

void loop() {
  int clicks = pollButton();   // returns 0 until a click sequence has finished

  switch (mode) {
    case IDLE:
      digitalWrite(LED_PIN, LOW);
      if (clicks == 1) {
        mode = CALIBRATING;
      } else if (clicks == 2) {
        if (calibrated) {
          startRun();
        } else {
          Serial.println("Not calibrated yet - single click to calibrate first.");
          blinkLed(3, 80);
        }
      }
      break;

    case CALIBRATING:
      calibrate();
      flushButton();
      mode = IDLE;
      break;

    case RUNNING:
      if (pressEdge) {           // immediate stop on any press
        pressEdge = false;
        stopMotors();
        flushButton();
        mode = IDLE;
        Serial.println("Stopped.");
        break;
      }
      followLine();
      break;
  }
}

// ---------------- Modes ----------------
void startRun() {
  integral  = 0;
  lastError = 0;
  lineLost  = false;
  aligning  = false;
  episodeActive = false;
  jstate = J_NONE;
  junctionCooldownUntil = 0;
  turnSum = 0; loopBreak = false; skipLeft = 0; loopBreaks = 0;
  pressEdge = false;
  Serial.println("Running.");
  digitalWrite(LED_PIN, HIGH);
  delay(500);                    // time to take your hand away
  flushButton();
  mode = RUNNING;
}

void followLine() {
  float pos = 0;
  bool  seen = readPosition(pos);
  unsigned long now = millis();

  if (MAZE_MODE && jstate != J_NONE) {     // junction manoeuvre in progress
    junctionStep(now);
    return;
  }

  // Whole recovery (lost -> spin -> align -> lost ...) must finish within the timeout
  if (episodeActive && now - episodeStart > UTURN_TIMEOUT_MS) {
    abortUturn();
    return;
  }

  if (!seen) {                       // nothing under the array
    aligning = false;
    handleLineLost();
    return;
  }

  if (lineLost) {                    // spinning to find the line: wait for a CENTRE sensor
    if (!centerOnLine()) {
      handleLineLost();
      return;
    }
    lineLost = false;
    aligning = true;
    stopMotors();
    Serial.println("Line found - aligning.");
  }

  if (aligning) {
    if (fabs(pos) > ALIGN_TOL) {     // gentle pivot toward the line (+pos = line on right)
      lastError = pos;
      int s   = constrain((int)(ALIGN_KP * fabs(pos)), ALIGN_MIN_SPEED, ALIGN_MAX_SPEED);
      int dir = (pos > 0) ? 1 : -1;
      spin(dir * s);
      return;
    }
    aligning      = false;           // centred: hand over to the PID with a clean state
    integral      = 0;
    lastError     = pos;
    lineSeenSince = now;
    Serial.println("Aligned - following.");
  }

  // Recovery is over once the line has been tracked steadily for a while
  if (fabs(pos) > STABLE_ERR) lineSeenSince = now;
  else if (episodeActive && now - lineSeenSince > STABLE_MS) episodeActive = false;

  if (MAZE_MODE && now >= junctionCooldownUntil && junctionDetected()) {
    startJunction(now);
    return;
  }

  float error      = pos;
  integral         = constrain(integral + error, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float derivative = error - lastError;
  float pTerm      = Kp * error;
  float iTerm      = Ki * integral;
  float dTerm      = Kd * derivative;
  float output     = pTerm + iTerm + dTerm;
  lastError        = error;

  int speed      = constrain(BASE_SPEED - (int)(SPEED_SCALE * abs(output)), MIN_SPEED, BASE_SPEED);
  int leftSpeed  = constrain(speed + (int)output, -255, 255);
  int rightSpeed = constrain(speed - (int)output, -255, 255);

  moveMotors(leftSpeed, rightSpeed);
  plotPID(error);
}

// Serial Plotter output: Arduino IDE > Tools > Serial Plotter, 115200 baud.
// Each "Label:value" pair becomes its own trace.
void plotPID(float error) {
  if (!PLOT_ENABLED) return;

  static unsigned long lastPlot = 0;
  unsigned long now = millis();
  if (now - lastPlot < PLOT_INTERVAL_MS) return;
  lastPlot = now;

  Serial.printf("Error:%.1f Kp:%.3f Ki:%.3f Kd:%.3f\n",
                error * PLOT_ERR_SCALE, Kp, Ki, Kd);
}

// ---------------- Sensors ----------------
// Averaged ADC read to reduce noise.
int readSensor(int i) {
  long sum = 0;
  for (int k = 0; k < ADC_SAMPLES; k++) sum += analogRead(sensorPins[i]);
  return (int)(sum / ADC_SAMPLES);
}

bool readPosition(float &pos) {
  int  readings[8];
  long total = 0;

  for (int i = 0; i < 8; i++) {
    readings[i]  = normalizeReading(readSensor(i), sensorMin[i], sensorMax[i]);
    sensorVal[i] = readings[i];
    total += readings[i];
  }
  if (total < LINE_THRESHOLD) return false;

  long weightedSum = 0;
  for (int i = 0; i < 8; i++) weightedSum += (long)readings[i] * weights[i];

  pos = (float)weightedSum / total;
  return true;
}

int normalizeReading(int raw, int mn, int mx) {
  if (mx <= mn) return 0;
  int val = constrain(map(raw, mn, mx, 0, 1000), 0, 1000);
  if (SENSOR_LOW_ON_LINE) val = 1000 - val;
  return (val < NOISE_FLOOR) ? 0 : val;   // ignore black-surface noise
}

void calibrate() {
  Serial.println("Calibrating: sweep sensors over the white line AND the black surface...");

  for (int i = 0; i < 8; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }

  if (AUTO_SPIN_CAL) {
    setMotor(PWMA, AIN1, AIN2, INVERT_LEFT,   100);
    setMotor(PWMB, BIN1, BIN2, INVERT_RIGHT, -100);
  }

  unsigned long start = millis();
  unsigned long lastPrint = 0;
  while (millis() - start < CAL_TIME_MS) {
    for (int i = 0; i < 8; i++) {
      int v = readSensor(i);
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }
    digitalWrite(LED_PIN, (millis() / 100) % 2);  // fast blink = calibrating

    // live range monitor every 250 ms
    if (millis() - lastPrint >= 250) {
      lastPrint = millis();
      for (int i = 0; i < 8; i++) {
        Serial.printf("S%d:%d-%d ", i, sensorMin[i], sensorMax[i]);
      }
      Serial.println();
    }
  }
  stopMotors();
  digitalWrite(LED_PIN, LOW);

  // validate: every sensor must have seen a usable range
  calibrated = true;
  for (int i = 0; i < 8; i++) {
    int  range = sensorMax[i] - sensorMin[i];
    bool ok    = range >= MIN_RANGE;
    Serial.printf("S%d  min=%4d  max=%4d  range=%4d  %s\n",
                  i, sensorMin[i], sensorMax[i], range, ok ? "OK" : "LOW");
    if (!ok) calibrated = false;
  }

  if (calibrated) {
    Serial.println("Calibration OK. Double click to run.");
    blinkLed(2, 200);
  } else {
    Serial.println("Calibration FAILED (a sensor saw too little range). Click once to retry.");
    blinkLed(6, 70);
  }
}

// ---------------- Line lost / U-turn ----------------
bool centerOnLine() {
  for (int i = RECOVER_FIRST; i <= RECOVER_LAST; i++)
    if (sensorVal[i] >= RECOVER_MIN) return true;
  return false;
}

void handleLineLost() {
  if (!lineLost) {                         // first frame of this loss
    lineLost = true;
    if (!episodeActive) {                  // start the recovery clock only once per episode
      episodeActive = true;
      episodeStart  = millis();
    }
    // Sharp corner: line left on the side it was last seen -> go that way.
    // Dead end: error ~0, no side info -> use the default direction.
    if (fabs(lastError) > SIDE_MEMORY_MIN) {
      spinDir = (lastError > 0) ? 1 : -1;
      if (MAZE_MODE) noteTurn(spinDir > 0 ? -1 : +1);   // a corner: counts as a turn
    } else {
      spinDir = UTURN_DEFAULT_DIR;
      turnSum = 0;                                      // dead end: path reverses, restart tally
    }
    stopMotors();
    delay(LOST_BRAKE_MS);
    lostStart = millis();
    Serial.printf("Line lost -> spinning %s\n", spinDir > 0 ? "RIGHT" : "LEFT");
  }

  unsigned long t = millis() - lostStart;
  spin(spinDir * ((t < SPIN_KICK_MS) ? SPIN_KICK_SPEED : SPIN_SPEED));
}

void abortUturn() {                        // never found / never settled on the line
  stopMotors();
  lineLost      = false;
  aligning      = false;
  episodeActive = false;
  mode          = IDLE;
  Serial.println("U-turn timeout - stopped.");
}

// ---------------- Junctions ----------------
bool onWhite(int i)   { return sensorVal[i] >= JUNC_ON_MIN; }
bool leftEdge()       { return onWhite(0) && onWhite(1); }
bool rightEdge()      { return onWhite(6) && onWhite(7); }
bool centerWhite()    { return onWhite(3) || onWhite(4); }
bool centerOff()      { return sensorVal[3] < JUNC_ON_MIN / 2 && sensorVal[4] < JUNC_ON_MIN / 2; }

// Side arm + the line we are on, seen at the same moment
bool junctionDetected() {
  return (leftEdge() || rightEdge()) && centerWhite();
}

void startJunction(unsigned long now) {
  jL = leftEdge();
  jR = rightEdge();
  jS = false;
  jStart = now;
  jstate = J_INCH;
  moveMotors(JUNC_INCH_SPEED, JUNC_INCH_SPEED);
  Serial.println("Junction detected - creeping to centre.");
}

// +1 = left 90 deg, -1 = right 90 deg. Four the same way = went round a loop.
void noteTurn(int q) {
  turnSum += q;
  if (abs(turnSum) >= LOOP_TURNS) {
    turnSum    = 0;
    loopBreak  = true;
    skipLeft   = loopBreaks % (LOOP_MAX_SKIP + 1);
    loopBreaks++;
    Serial.printf("LOOP detected -> alternate exit at next junction (skipping %d first)\n", skipLeft);
  }
}

void decideJunction() {
  char ex[3];                               // available exits, best first
  int  n = 0;
  if (LEFT_HAND) { if (jL) ex[n++] = 'L'; if (jS) ex[n++] = 'S'; if (jR) ex[n++] = 'R'; }
  else           { if (jR) ex[n++] = 'R'; if (jS) ex[n++] = 'S'; if (jL) ex[n++] = 'L'; }

  if (n == 0) {                             // false alarm: just carry on
    jstate = J_NONE;
    return;
  }

  int pick = 0;
  if (n >= 2 && loopBreak) {
    if (skipLeft > 0) {
      skipLeft--;                           // not this one - try a later junction
    } else {
      pick = 1;                             // take the next exit instead of the default
      loopBreak = false;
      Serial.println("Loop break: taking alternate exit.");
    }
  }

  char c = ex[pick];
  Serial.printf("Junction exits:%s%s%s -> %c\n", jL ? " L" : "", jS ? " S" : "", jR ? " R" : "", c);

  if (c == 'S') {                           // straight on: back to the PID
    jstate    = J_NONE;
    integral  = 0;
    lastError = 0;
    junctionCooldownUntil = millis() + JUNC_COOLDOWN_MS;
    return;
  }

  turnDir    = (c == 'L') ? -1 : +1;
  jTurnStart = millis();
  noteTurn(c == 'L' ? +1 : -1);
  jstate = centerOff() ? J_TURN_FIND : J_TURN_LEAVE;
}

void abortJunction() {
  stopMotors();
  jstate = J_NONE;
  mode   = IDLE;
  Serial.println("Junction turn timeout - stopped.");
}

void junctionStep(unsigned long now) {
  switch (jstate) {
    case J_INCH:
      if (leftEdge())  jL = true;           // latch exits seen while creeping over the crossing
      if (rightEdge()) jR = true;
      if (now - jStart < JUNC_INCH_MS) {
        moveMotors(JUNC_INCH_SPEED, JUNC_INCH_SPEED);
        return;
      }
      stopMotors();
      jS = centerWhite();                   // line still ahead of us after the crossing?
      decideJunction();
      break;

    case J_TURN_LEAVE:                      // pivot off the line we arrived on...
    case J_TURN_FIND: {                     // ...then onto the chosen exit
      if (now - jTurnStart > JUNC_TURN_TIMEOUT_MS) { abortJunction(); return; }
      unsigned long t = now - jTurnStart;
      spin(turnDir * ((t < SPIN_KICK_MS) ? SPIN_KICK_SPEED : TURN_SPEED));

      if (jstate == J_TURN_LEAVE) {
        if (centerOff()) jstate = J_TURN_FIND;
      } else if (sensorVal[3] >= RECOVER_MIN || sensorVal[4] >= RECOVER_MIN) {
        stopMotors();
        jstate   = J_NONE;
        aligning = true;                    // reuse the gentle centring step
        lineLost = false;
        junctionCooldownUntil = now + JUNC_COOLDOWN_MS;
        Serial.println("Turn done - aligning.");
      }
      break;
    }

    default:
      jstate = J_NONE;
      break;
  }
}

// s > 0 = pivot right (clockwise), s < 0 = pivot left
void spin(int s) {
  setMotor(PWMA, AIN1, AIN2, INVERT_LEFT,   s);
  setMotor(PWMB, BIN1, BIN2, INVERT_RIGHT, -s);
}

// ---------------- Motors ----------------
void moveMotors(int left, int right) {
  setMotor(PWMA, AIN1, AIN2, INVERT_LEFT,  left);
  setMotor(PWMB, BIN1, BIN2, INVERT_RIGHT, right);
}

void setMotor(int pwm, int in1, int in2, bool invert, int speed) {
  if (invert) speed = -speed;
  digitalWrite(in1, speed >= 0 ? HIGH : LOW);
  digitalWrite(in2, speed >= 0 ? LOW  : HIGH);
  analogWrite(pwm, abs(speed));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

// ---------------- Button ----------------
// Debounced, non-blocking click counter. Returns the number of clicks once the
// sequence has ended (no new click within CLICK_GAP_MS), otherwise 0.
// Also sets pressEdge = true the instant a debounced press happens.
int pollButton() {
  bool reading = (digitalRead(BUTTON_PIN) == HIGH);
  unsigned long now = millis();

  if (reading != lastReading) {
    lastChange  = now;
    lastReading = reading;
  }

  if ((now - lastChange) > DEBOUNCE_MS && reading != stableState) {
    stableState = reading;
    if (stableState) {
      pressEdge = true;          // pressed
    } else {
      clickCount++;              // count on release
      lastRelease = now;
    }
  }

  if (clickCount > 0 && !stableState && (now - lastRelease) > CLICK_GAP_MS) {
    int c = clickCount;
    clickCount = 0;
    return c;
  }
  return 0;
}

void flushButton() {
  clickCount = 0;
  pressEdge  = false;
}

void blinkLed(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  delay(ms);
  }
}
