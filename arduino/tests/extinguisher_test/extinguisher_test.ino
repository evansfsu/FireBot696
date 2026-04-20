/*
 * extinguisher_test.ino -- standalone test for the FireBot696 extinguishing
 * mechanism: solenoid pin-puller (D7 via NMOS Q1) + A4988 stepper driving
 * the lead-screw clamp (STEP=D3, DIR=D4). No drive motors, no sensors --
 * just the two actuators that put the fire out.
 *
 * Open the Arduino IDE Serial Monitor at 115200 baud, line ending
 * "Newline". Type one of:
 *
 *   ? | h              print this menu
 *   pin on             energize solenoid (pin pulled)
 *   pin off            de-energize solenoid (reset)
 *   pulse [ms]         pin on -> wait <ms> -> pin off (default 1500 ms)
 *   advance            run lead-screw forward (auto-stops after 6 s)
 *   retract            run lead-screw reverse (auto-stops after 6 s)
 *   stop | s           halt the stepper immediately
 *   seq                full discharge sequence:
 *                         pin on (1.5 s) -> pin off ->
 *                         advance (6 s) -> pause ->
 *                         retract (6 s) -> idle
 *   abort              cancel a running sequence
 *
 * The stepper is stepped non-blockingly via micros(), so commands always
 * respond even while the lead-screw is running. EXT_*_MS constants match
 * the unified firmware so behavior is identical to what the brain node
 * drives in production.
 */

#include <Arduino.h>

// ---------- Pin map (FROM Fire_Bot.net) ----------
const uint8_t PIN_STEP = 3;
const uint8_t PIN_DIR  = 4;
const uint8_t PIN_SOL  = 7;

// ---------- Timing constants (match firebot_mega.ino) ----------
const uint32_t EXT_PIN_PULL_MS = 1500;  // default pulse length
const uint32_t EXT_ADVANCE_MS  = 6000;
const uint32_t EXT_RETRACT_MS  = 6000;
const uint16_t STEP_HALF_US    = 500;   // ~1 kHz full-step rate

// ---------- Stepper (non-blocking) ----------
int      g_step_dir = 0;                // -1 retract, 0 idle, +1 advance
uint32_t g_last_step_toggle_us = 0;
uint8_t  g_step_level = LOW;
uint32_t g_step_start_ms = 0;
uint32_t g_step_auto_stop_ms = 0;       // 0 = no auto-stop

// ---------- Solenoid (non-blocking pulse) ----------
bool     g_sol_pulse_active = false;
uint32_t g_sol_pulse_end_ms = 0;

// ---------- Discharge sequence FSM ----------
enum SeqPhase : uint8_t {
  SEQ_IDLE = 0,
  SEQ_PIN_ON,
  SEQ_PIN_HOLD,
  SEQ_ADVANCE,
  SEQ_PAUSE_AFTER_ADVANCE,
  SEQ_RETRACT,
  SEQ_DONE
};
uint8_t  g_seq = SEQ_IDLE;
uint32_t g_seq_phase_start = 0;
const uint32_t SEQ_PAUSE_MS = 500;

// ---------- Serial parser ----------
char    g_rx[48];
uint8_t g_rx_len = 0;

// ---------- Actuator helpers ----------
static void solOn()  { digitalWrite(PIN_SOL, HIGH); }
static void solOff() { digitalWrite(PIN_SOL, LOW); }

static void startPulse(uint32_t ms) {
  solOn();
  g_sol_pulse_active = true;
  g_sol_pulse_end_ms = millis() + ms;
  Serial.print(F("pulse on for ")); Serial.print(ms); Serial.println(F(" ms"));
}

static void servicePulse() {
  if (!g_sol_pulse_active) return;
  if ((int32_t)(millis() - g_sol_pulse_end_ms) >= 0) {
    g_sol_pulse_active = false;
    solOff();
    Serial.println(F("pulse off"));
  }
}

static void stepperAdvance(uint32_t auto_stop_ms) {
  digitalWrite(PIN_DIR, HIGH);
  g_step_dir = +1;
  g_step_start_ms = millis();
  g_step_auto_stop_ms = auto_stop_ms;
  Serial.print(F("advance for ")); Serial.print(auto_stop_ms); Serial.println(F(" ms"));
}

static void stepperRetract(uint32_t auto_stop_ms) {
  digitalWrite(PIN_DIR, LOW);
  g_step_dir = -1;
  g_step_start_ms = millis();
  g_step_auto_stop_ms = auto_stop_ms;
  Serial.print(F("retract for ")); Serial.print(auto_stop_ms); Serial.println(F(" ms"));
}

static void stepperStop() {
  if (g_step_dir == 0 && g_step_auto_stop_ms == 0) return;
  g_step_dir = 0;
  g_step_auto_stop_ms = 0;
  digitalWrite(PIN_STEP, LOW);
  Serial.println(F("stepper stop"));
}

static void serviceStepper() {
  if (g_step_dir == 0) return;
  uint32_t now_us = micros();
  if ((uint32_t)(now_us - g_last_step_toggle_us) >= STEP_HALF_US) {
    g_step_level = !g_step_level;
    digitalWrite(PIN_STEP, g_step_level);
    g_last_step_toggle_us = now_us;
  }
  if (g_step_auto_stop_ms &&
      (millis() - g_step_start_ms) >= g_step_auto_stop_ms) {
    stepperStop();
  }
}

// ---------- Discharge sequence ----------
static void seqEnter(uint8_t phase) {
  g_seq = phase;
  g_seq_phase_start = millis();
  switch (phase) {
    case SEQ_PIN_ON:
      solOn();
      Serial.println(F("seq: pin on"));
      break;
    case SEQ_PIN_HOLD:
      solOff();
      Serial.println(F("seq: pin released"));
      break;
    case SEQ_ADVANCE:
      stepperAdvance(EXT_ADVANCE_MS);
      Serial.println(F("seq: advance"));
      break;
    case SEQ_PAUSE_AFTER_ADVANCE:
      stepperStop();
      Serial.println(F("seq: pause"));
      break;
    case SEQ_RETRACT:
      stepperRetract(EXT_RETRACT_MS);
      Serial.println(F("seq: retract"));
      break;
    case SEQ_DONE:
      stepperStop();
      solOff();
      Serial.println(F("seq: DONE"));
      break;
    case SEQ_IDLE:
    default:
      break;
  }
}

static void seqStart() {
  Serial.println(F("seq: start"));
  seqEnter(SEQ_PIN_ON);
}

static void seqAbort() {
  if (g_seq == SEQ_IDLE) return;
  g_seq = SEQ_IDLE;
  stepperStop();
  solOff();
  Serial.println(F("seq: aborted"));
}

static void serviceSequence() {
  if (g_seq == SEQ_IDLE || g_seq == SEQ_DONE) {
    if (g_seq == SEQ_DONE) g_seq = SEQ_IDLE;
    return;
  }
  uint32_t elapsed = millis() - g_seq_phase_start;
  switch (g_seq) {
    case SEQ_PIN_ON:
      if (elapsed >= EXT_PIN_PULL_MS) seqEnter(SEQ_PIN_HOLD);
      break;
    case SEQ_PIN_HOLD:
      if (elapsed >= SEQ_PAUSE_MS) seqEnter(SEQ_ADVANCE);
      break;
    case SEQ_ADVANCE:
      if (elapsed >= EXT_ADVANCE_MS) seqEnter(SEQ_PAUSE_AFTER_ADVANCE);
      break;
    case SEQ_PAUSE_AFTER_ADVANCE:
      if (elapsed >= SEQ_PAUSE_MS) seqEnter(SEQ_RETRACT);
      break;
    case SEQ_RETRACT:
      if (elapsed >= EXT_RETRACT_MS) seqEnter(SEQ_DONE);
      break;
    default: break;
  }
}

// ---------- Command parser ----------
static void printMenu() {
  Serial.println();
  Serial.println(F("extinguisher_test.ino -- solenoid + A4988 standalone test"));
  Serial.println(F("Open Serial Monitor at 115200 baud, line ending = Newline."));
  Serial.println(F("Commands:"));
  Serial.println(F("  ? | h          this menu"));
  Serial.println(F("  pin on         solenoid energize"));
  Serial.println(F("  pin off        solenoid release"));
  Serial.println(F("  pulse [ms]     pin on -> off (default 1500 ms)"));
  Serial.println(F("  advance        lead-screw forward (auto-stops after 6 s)"));
  Serial.println(F("  retract        lead-screw reverse (auto-stops after 6 s)"));
  Serial.println(F("  stop | s       halt the stepper now"));
  Serial.println(F("  seq            full discharge: pin -> advance -> retract"));
  Serial.println(F("  abort          cancel a running sequence"));
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
  if (*rest < '0' || *rest > '9') return false;
  int v = 0;
  while (*rest >= '0' && *rest <= '9') { v = v * 10 + (*rest - '0'); rest++; }
  *out = v * sign;
  return true;
}

static char* nextToken(char* p, char** tok) {
  p = skipSpaces(p);
  *tok = p;
  while (*p && *p != ' ' && *p != '\t') {
    if (*p >= 'A' && *p <= 'Z') *p += 32;
    p++;
  }
  if (*p) { *p = 0; p++; }
  return p;
}

static void handleLine(char* line) {
  char* cmd;
  char* rest = nextToken(line, &cmd);
  if (!*cmd) return;

  if (strcmp(cmd, "?") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0) {
    printMenu(); return;
  }
  if (strcmp(cmd, "pin") == 0) {
    char* arg;
    nextToken(rest, &arg);
    if (strcmp(arg, "on") == 0 || strcmp(arg, "1") == 0) {
      seqAbort();
      solOn();
      g_sol_pulse_active = false;
      Serial.println(F("pin on"));
    } else {
      seqAbort();
      solOff();
      g_sol_pulse_active = false;
      Serial.println(F("pin off"));
    }
    return;
  }
  if (strcmp(cmd, "pulse") == 0) {
    int ms = (int)EXT_PIN_PULL_MS;
    parseArg(rest, &ms);
    if (ms < 50) ms = 50;
    seqAbort();
    startPulse((uint32_t)ms);
    return;
  }
  if (strcmp(cmd, "advance") == 0) {
    seqAbort();
    stepperAdvance(EXT_ADVANCE_MS);
    return;
  }
  if (strcmp(cmd, "retract") == 0) {
    seqAbort();
    stepperRetract(EXT_RETRACT_MS);
    return;
  }
  if (strcmp(cmd, "s") == 0 || strcmp(cmd, "stop") == 0) {
    seqAbort();
    stepperStop();
    return;
  }
  if (strcmp(cmd, "seq") == 0) {
    seqStart();
    return;
  }
  if (strcmp(cmd, "abort") == 0 || strcmp(cmd, "cancel") == 0) {
    seqAbort();
    return;
  }
  Serial.print(F("unknown: ")); Serial.println(cmd);
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
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR,  OUTPUT);
  pinMode(PIN_SOL,  OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR,  LOW);
  digitalWrite(PIN_SOL,  LOW);
  printMenu();
  Serial.println(F("ready"));
}

void loop() {
  pollSerial();
  serviceStepper();
  servicePulse();
  serviceSequence();
}
