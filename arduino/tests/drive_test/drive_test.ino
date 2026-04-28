/*
 * drive_test.ino -- standalone motor + in-place-spin test for FireBot696.
 *
 * Focused sketch for checking the four FIT0186 motors / two DFR0601 drivers
 * without flashing the full firmware. No state machine, no protocol, no
 * sensors. Just open the Arduino IDE Serial Monitor at 115200 baud with
 * line ending "Newline" and type one of the single-letter commands below.
 *
 *   ? | h        print this menu
 *   f [0..255]   forward (default PWM 100)
 *   b [0..255]   backward
 *   l [0..255]   spin in place LEFT (in-place rotation)
 *   r [0..255]   spin in place RIGHT
 *   s            stop
 *   spin         5-second in-place spin: left 2.5s, then right 2.5s, then stop
 *   demo         forward 0.5s -> stop -> back 0.5s -> stop ->
 *                left 1s -> stop -> right 1s -> stop
 *   speed N      set the default PWM used when a command omits its argument
 *
 * Pin map is taken verbatim from Fire_Bot.net (the wired schematic), same
 * as the unified firmware. The PCB ties each drive signal to two Mega
 * pins, so we write identical values to both pins every cycle.
 *
 * Indoor-safe defaults: forward / back run for 500 ms in `demo`; the
 * default PWM is 100 (out of 255). Type `f 200` explicitly if the robot
 * is on blocks and you want to push harder.
 */

#include <Arduino.h>

// ---------- Pin map (FROM Fire_Bot.net) ----------
// Left pair = DFR0601 #1 ch1 + DFR0601 #2 ch1.
const uint8_t PIN_PWM1_A = 12, PIN_PWM1_B = 13;
const uint8_t PIN_INA1_A = 51, PIN_INA1_B = 52;
const uint8_t PIN_INB1_A = 49, PIN_INB1_B = 50;

// Right pair = DFR0601 #1 ch2 + DFR0601 #2 ch2.
const uint8_t PIN_PWM2_A = 10, PIN_PWM2_B = 11;
const uint8_t PIN_INA2_A = 25, PIN_INA2_B = 24;
const uint8_t PIN_INB2_A = 27, PIN_INB2_B = 26;

// ---------- Runtime state ----------
int g_default_pwm = 100;     // indoor-safe default
int g_left_cmd  = 0;
int g_right_cmd = 0;

// Non-blocking timed-action so `spin` and `demo` don't freeze the loop.
enum Action : uint8_t { ACT_NONE, ACT_STOP, ACT_FWD, ACT_BACK, ACT_LEFT, ACT_RIGHT };
struct Step { uint8_t action; uint16_t ms; uint8_t pwm; };

const uint8_t  MAX_STEPS = 8;
Step     g_seq[MAX_STEPS];
uint8_t  g_seq_len = 0;
uint8_t  g_seq_idx = 0;
uint32_t g_seq_step_start = 0;
bool     g_seq_running = false;

// Serial parser
char    g_rx[48];
uint8_t g_rx_len = 0;

// ---------- Low-level drive ----------
static inline void setPair(uint8_t a, uint8_t b, uint8_t pwm) {
  analogWrite(a, pwm);
  analogWrite(b, pwm);
}

static inline void setDir(uint8_t inaA, uint8_t inaB,
                          uint8_t inbA, uint8_t inbB, bool forward) {
  digitalWrite(inaA, forward ? HIGH : LOW);
  digitalWrite(inaB, forward ? HIGH : LOW);
  digitalWrite(inbA, forward ? LOW  : HIGH);
  digitalWrite(inbB, forward ? LOW  : HIGH);
}

static void applyDrive(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);
  g_left_cmd  = left;
  g_right_cmd = right;
  setDir(PIN_INA1_A, PIN_INA1_B, PIN_INB1_A, PIN_INB1_B, left  >= 0);
  setDir(PIN_INA2_A, PIN_INA2_B, PIN_INB2_A, PIN_INB2_B, right >= 0);
  setPair(PIN_PWM1_A, PIN_PWM1_B, (uint8_t)abs(left));
  setPair(PIN_PWM2_A, PIN_PWM2_B, (uint8_t)abs(right));
}

static void stopMotors() { applyDrive(0, 0); }

// Convenience wrappers -- skid-steer, vx+wz mixed to left/right.
static void driveForward(int pwm)  { applyDrive( pwm,  pwm); }
static void driveBackward(int pwm) { applyDrive(-pwm, -pwm); }
static void spinLeft(int pwm)      { applyDrive(-pwm,  pwm); }
static void spinRight(int pwm)     { applyDrive( pwm, -pwm); }

static void performAction(uint8_t action, uint8_t pwm) {
  switch (action) {
    case ACT_STOP:  stopMotors();          break;
    case ACT_FWD:   driveForward(pwm);     break;
    case ACT_BACK:  driveBackward(pwm);    break;
    case ACT_LEFT:  spinLeft(pwm);         break;
    case ACT_RIGHT: spinRight(pwm);        break;
  }
}

// ---------- Sequences ----------
static void startSequence(const Step* src, uint8_t n) {
  g_seq_len = n;
  for (uint8_t i = 0; i < n && i < MAX_STEPS; i++) g_seq[i] = src[i];
  g_seq_idx = 0;
  g_seq_step_start = millis();
  g_seq_running = true;
  performAction(g_seq[0].action, g_seq[0].pwm);
  Serial.print(F("seq start, step 0 for "));
  Serial.print(g_seq[0].ms); Serial.println(F(" ms"));
}

static void cancelSequence() {
  if (!g_seq_running) return;
  g_seq_running = false;
  stopMotors();
  Serial.println(F("seq cancelled"));
}

static void serviceSequence() {
  if (!g_seq_running) return;
  if ((uint32_t)(millis() - g_seq_step_start) < g_seq[g_seq_idx].ms) return;
  g_seq_idx++;
  if (g_seq_idx >= g_seq_len) {
    g_seq_running = false;
    stopMotors();
    Serial.println(F("seq done"));
    return;
  }
  g_seq_step_start = millis();
  performAction(g_seq[g_seq_idx].action, g_seq[g_seq_idx].pwm);
  Serial.print(F("seq step ")); Serial.print(g_seq_idx);
  Serial.print(F(" for ")); Serial.print(g_seq[g_seq_idx].ms); Serial.println(F(" ms"));
}

static void startSpin(int pwm) {
  Step seq[] = {
    { ACT_LEFT,  2500, (uint8_t)pwm },
    { ACT_STOP,  500,  0 },
    { ACT_RIGHT, 2500, (uint8_t)pwm },
    { ACT_STOP,  0,    0 },
  };
  startSequence(seq, 4);
}

static void startDemo(int pwm) {
  // Short forward/back bursts (indoor-safe) + longer in-place spins.
  Step seq[] = {
    { ACT_FWD,   500,  (uint8_t)pwm },
    { ACT_STOP,  500,  0 },
    { ACT_BACK,  500,  (uint8_t)pwm },
    { ACT_STOP,  500,  0 },
    { ACT_LEFT,  1000, (uint8_t)pwm },
    { ACT_STOP,  500,  0 },
    { ACT_RIGHT, 1000, (uint8_t)pwm },
    { ACT_STOP,  0,    0 },
  };
  startSequence(seq, 8);
}

// ---------- Command parser ----------
static void printMenu() {
  Serial.println();
  Serial.println(F("drive_test.ino -- motor + in-place spin test"));
  Serial.println(F("Open Serial Monitor at 115200 baud, line ending = Newline."));
  Serial.println(F("Commands (case-insensitive, space-separated):"));
  Serial.println(F("  ? | h            this menu"));
  Serial.println(F("  f [pwm]          forward"));
  Serial.println(F("  b [pwm]          backward"));
  Serial.println(F("  l [pwm]          spin LEFT in place"));
  Serial.println(F("  r [pwm]          spin RIGHT in place"));
  Serial.println(F("  s | stop         stop"));
  Serial.println(F("  spin [pwm]       in-place: 2.5s left, pause, 2.5s right"));
  Serial.println(F("  demo [pwm]       fwd 0.5s / back 0.5s / spin 1s each way"));
  Serial.println(F("  speed N          set default pwm (current = 100)"));
  Serial.print  (F("  current default pwm: ")); Serial.println(g_default_pwm);
  Serial.println();
}

static char* skipSpaces(char* p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

static bool parseArg(char* rest, int* out) {
  rest = skipSpaces(rest);
  if (!*rest) return false;
  int sign = 1;
  if (*rest == '-') { sign = -1; rest++; }
  else if (*rest == '+') rest++;
  if (*rest < '0' || *rest > '9') return false;
  int v = 0;
  while (*rest >= '0' && *rest <= '9') { v = v * 10 + (*rest - '0'); rest++; }
  *out = v * sign;
  return true;
}

static void handleLine(char* line) {
  // Lowercase the first token; leave the rest alone (we only parse numbers).
  char* p = skipSpaces(line);
  if (!*p) return;
  char* tok = p;
  while (*p && *p != ' ' && *p != '\t') {
    if (*p >= 'A' && *p <= 'Z') *p += 32;
    p++;
  }
  if (*p) { *p = 0; p++; }

  int n = g_default_pwm;

  if (strcmp(tok, "?") == 0 || strcmp(tok, "h") == 0 || strcmp(tok, "help") == 0) {
    printMenu(); return;
  }
  if (strcmp(tok, "s") == 0 || strcmp(tok, "stop") == 0) {
    cancelSequence();
    stopMotors();
    Serial.println(F("stop"));
    return;
  }
  if (strcmp(tok, "f") == 0 || strcmp(tok, "fwd") == 0 || strcmp(tok, "forward") == 0) {
    cancelSequence();
    parseArg(p, &n);
    driveForward(n);
    Serial.print(F("forward ")); Serial.println(n);
    return;
  }
  if (strcmp(tok, "b") == 0 || strcmp(tok, "back") == 0 || strcmp(tok, "backward") == 0) {
    cancelSequence();
    parseArg(p, &n);
    driveBackward(n);
    Serial.print(F("back ")); Serial.println(n);
    return;
  }
  if (strcmp(tok, "l") == 0 || strcmp(tok, "left") == 0) {
    cancelSequence();
    parseArg(p, &n);
    spinLeft(n);
    Serial.print(F("left ")); Serial.println(n);
    return;
  }
  if (strcmp(tok, "r") == 0 || strcmp(tok, "right") == 0) {
    cancelSequence();
    parseArg(p, &n);
    spinRight(n);
    Serial.print(F("right ")); Serial.println(n);
    return;
  }
  if (strcmp(tok, "spin") == 0) {
    parseArg(p, &n);
    startSpin(n);
    return;
  }
  if (strcmp(tok, "demo") == 0) {
    parseArg(p, &n);
    startDemo(n);
    return;
  }
  if (strcmp(tok, "speed") == 0) {
    if (parseArg(p, &n)) {
      g_default_pwm = constrain(n, 0, 255);
      Serial.print(F("default pwm = ")); Serial.println(g_default_pwm);
    } else {
      Serial.println(F("usage: speed <0..255>"));
    }
    return;
  }
  Serial.print(F("unknown command: ")); Serial.println(tok);
  Serial.println(F("type '?' for the menu"));
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_rx[g_rx_len] = 0;
      if (g_rx_len > 0) handleLine(g_rx);
      g_rx_len = 0;
    } else if (g_rx_len < sizeof(g_rx) - 1) {
      g_rx[g_rx_len++] = c;
    } else {
      g_rx_len = 0;
    }
  }
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  const uint8_t outs[] = {
    PIN_PWM1_A, PIN_PWM1_B, PIN_PWM2_A, PIN_PWM2_B,
    PIN_INA1_A, PIN_INA1_B, PIN_INB1_A, PIN_INB1_B,
    PIN_INA2_A, PIN_INA2_B, PIN_INB2_A, PIN_INB2_B
  };
  for (uint8_t i = 0; i < sizeof(outs); i++) {
    pinMode(outs[i], OUTPUT);
    digitalWrite(outs[i], LOW);
  }
  stopMotors();
  printMenu();
  Serial.println(F("ready"));
}

void loop() {
  pollSerial();
  serviceSequence();
}
