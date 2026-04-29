/*
 * FireBot696 unified Mega firmware.
 *
 * Motor + extinguisher pins match bench-tested wiring (breadboard / harness):
 *   arduino/tested_arduino/front_reverse_spin/front_reverse_spin.ino
 *   arduino/tested_arduino/solenoid_stepper_combined/solenoid_stepper_combined.ino
 *
 * Encoder, ultrasonic, and IR/mic nets remain as in Fire_Bot.net (see docs/WIRING.md).
 * Skid-steer commands (left pair / right pair) use the same per-corner direction
 * patterns as the tested forward / reverse / spin-in-place routines.
 *
 * ============================================================================
 * TWO WAYS TO DRIVE THIS FIRMWARE
 * ============================================================================
 *
 * 1) ROS bridge protocol (used by the Pi, machine-friendly):
 *      M,<vx>,<vy>,<wz>       drive, each in -255..255 (vy ignored).
 *      E,<phase>              extinguisher: 0=off 1=pin_pull 2=screw_advance
 *                             3=screw_retract 4=stop
 *      W,<mode>               warning: 0=off 1=steady 2=countdown_beep
 *      S                      status request -> 'D,encA,encB,us,ir,mic,state'
 *      R                      e-stop (all outputs LOW, stepper disabled)
 *      C,<US|IR|MIC>,<0|1>    enable/disable optional sensor at runtime
 *
 * 2) Human command mode (typed into Arduino IDE Serial Monitor, 115200 baud,
 *    line ending "Newline" or "Both NL & CR"). Send `help` or `?` for the
 *    full menu. Short summary:
 *
 *      help                 print every command
 *      status               pretty-printed status + sensor values
 *      forward <0..255>     drive forward at given speed
 *      back <0..255>        drive backward
 *      left <0..255>        rotate left in place
 *      right <0..255>       rotate right in place
 *      drive <vx> <wz>      raw skid-steer command, each -255..255
 *      stop                 motors off
 *      pin on | pin off     solenoid directly
 *      advance | retract    run lead-screw (auto-stops after ~5.3 s, ramped)
 *      go                    full bench sequence (timers like solenoid_stepper_combined)
 *      extstop              stop the stepper immediately
 *      sensor us on|off     enable/disable HC-SR04
 *      sensor ir on|off     enable/disable KY-032
 *      sensor mic on|off    enable/disable MAX9814
 *      us | ir | mic        one-shot sensor read
 *      verbose on | off     periodic human status prints (every 500 ms)
 *      estop | reset        hard stop (same as protocol 'R')
 *      state idle           stop everything
 *      state searching      rotate in place until the next 'state' / 'stop'
 *      state awaiting       stop and wait (operator checkpoint)
 *      state approaching    drive forward until the next 'state' / 'stop'
 *      state warning        5 s audible countdown, then auto-extinguish
 *      state extinguishing  pin-pull -> advance -> retract sequence
 *      state complete       stop everything, auto-return to idle after 3 s
 *      state next           advance one step through the full sequence
 *
 *    Disabled sensors print 'off' in the human status and '-1' in the
 *    protocol status line, so teammates can tell "off" from "zero".
 *
 *    Every physical output is safe if the corresponding hardware isn't
 *    connected: missing sensors are ignored, and missing motor drivers /
 *    solenoid / stepper just mean the toggles go nowhere.
 *
 * ============================================================================
 * STANDALONE vs USB-TETHERED OPERATION
 * ============================================================================
 *
 * Two supported runtime configurations:
 *
 *   A) Arduino alone (no Pi connected)
 *      - Power the Mega from its own USB-B cable or barrel jack.
 *      - Open the Arduino IDE Serial Monitor at 115200 baud, line ending
 *        "Newline". Type `help` and drive everything from the keyboard.
 *      - The firmware sits in FW_IDLE and does nothing until a command
 *        arrives, so the robot is safe to power on even if it's sitting
 *        on the bench with the wheels in the air.
 *
 *   B) Pi 5 tethered over USB (also powers the Mega)
 *      - USB cable goes from the Pi to the Mega's USB-B port. The Pi's
 *        5 V USB rail powers the Mega's logic side. Motor / solenoid /
 *        stepper power come from the main robot battery as wired on the
 *        schematic -- the Pi does NOT power any actuators.
 *      - Opening the serial port from the Pi toggles DTR, which hard
 *        resets the Mega. setup() prints 'L,firebot_mega_ready' which the
 *        bridge node waits for before sending the initial C,US/IR/MIC
 *        configuration lines.
 *      - While the Pi is driving, a 1 s watchdog (PROTOCOL_DRIVE_WATCHDOG_MS)
 *        zeros the motors if no 'M' command arrives. That way a crashed
 *        bridge node or unplugged USB cable can't leave the robot running.
 *        Human motion commands (forward/back/left/right/drive) are NOT
 *        watchdogged -- teammates typing at the Serial Monitor expect
 *        their commands to latch until they say 'stop'.
 */

#include <Arduino.h>

// ---------- Pin map: motors + extinguisher (tested_arduino) ----------
// Front right: FP1 + FA1/FB1 | Front left: FP2 + FA2/FB2
// Rear left: RP1 + RA1/RB1    | Rear right: RP2 + RA2/RB2
const uint8_t FP1 = 12, FA1 = 51, FB1 = 49;
const uint8_t FP2 = 13, FA2 = 52, FB2 = 50;
const uint8_t RP1 = 10, RA1 = 25, RB1 = 27;
const uint8_t RP2 = 11, RA2 = 24, RB2 = 26;

// Encoders (ganged: all 4 Sensor A outputs on one net, all 4 Sensor B on another).
// We poll ONE representative pin per channel; the other three are tied to the
// same net and set to INPUT (high-Z) to avoid contention.
const uint8_t ENC_A_PINS[4] = {30, 32, 34, 36};
const uint8_t ENC_B_PINS[4] = {31, 33, 35, 37};

// A4988 stepper (lead-screw) — same nets as solenoid_stepper_combined.ino
const uint8_t PIN_STEP = 8;
const uint8_t PIN_DIR  = 9;

// Solenoid (pin-pull)
const uint8_t PIN_SOL = 4;

// HC-SR04 ultrasonic
const uint8_t PIN_US_TRIG = 48;
const uint8_t PIN_US_ECHO = 47;

// MAX9814 + KY-032 share one net. A0 reads the analog envelope,
// D46 reads the same wire digitally (KY-032 pulls it LOW when triggered).
const uint8_t PIN_OUT_ANALOG  = A0;
const uint8_t PIN_OUT_DIGITAL = 46;

// ---------- State ----------
enum FsmState : uint8_t {
  FW_IDLE = 0,
  FW_DRIVING = 1,
  FW_EXTINGUISHING = 2,
  FW_ESTOP = 3
};
uint8_t g_state = FW_IDLE;

// Drive commands (most recent)
int g_left_cmd = 0;   // -255..255 (sign = direction)
int g_right_cmd = 0;

// Pi-side drive watchdog: only armed by protocol 'M' commands with non-zero
// motion. If the Pi stops sending M faster than PROTOCOL_DRIVE_WATCHDOG_MS
// we zero the motors. This has no effect during standalone human testing
// because human motion commands clear g_protocol_drive.
bool     g_protocol_drive = false;
uint32_t g_last_protocol_drive_ms = 0;
const uint32_t PROTOCOL_DRIVE_WATCHDOG_MS = 1000;

// Optional sensor enables (set via 'C' command)
bool g_en_us  = false;
bool g_en_ir  = false;
bool g_en_mic = false;

// Aggregate encoder counters (polled, edge-detected)
volatile long g_encA_count = 0;
volatile long g_encB_count = 0;
uint8_t g_last_encA = 0;
uint8_t g_last_encB = 0;

// Extinguisher non-blocking timing
uint8_t  g_ext_phase = 0;      // 0=off 1=pin_pull 2=advance 3=retract 4=stop
uint32_t g_ext_phase_start_ms = 0;

// Step / solenoid timing — same numbers as solenoid_stepper_combined.ino
const uint16_t STEP_RAMP_START_HALF_US = 400;
const uint16_t STEP_RAMP_END_HALF_US   = 170;
const int16_t  STEP_RAMP_DELTA_HALF_US = -10;
const uint8_t  STEP_RAMP_STEPS_PER_BAND = 40;
const uint32_t EXT_STEPPER_RUN_MS = 5300;
const uint32_t EXT_PIN_PULL_MS    = 1000;
const uint32_t GO_PRE_SOL_MS   = 3000;
const uint32_t GO_POST_SOL_MS  = 3000;
const uint32_t GO_PRE_REV_MS   = 5000;

// Stepper ramp + STEP waveform — same shape as bench stepMotor(): HIGH for d µs,
// then LOW for d µs (non-blocking), not a generic square toggle.
uint32_t g_step_last_half_us = 0;
uint8_t  g_step_line_high = 0;  // 1 = STEP pin driven HIGH (mid pulse)
int      g_step_dir   = 0;     // -1, 0, +1
uint16_t g_step_half_us = STEP_RAMP_START_HALF_US;
uint8_t  g_ramp_pulses_at_speed = 0;
bool     g_stepper_ramp_complete = false;

// Full bench "GO" sequence (timed like tested sketch)
enum GoSeqState : uint8_t {
  GOSEQ_OFF = 0,
  GOSEQ_PRE_SOL,
  GOSEQ_SOL_ON,
  GOSEQ_POST_SOL,
  GOSEQ_STEP_FWD,
  GOSEQ_PRE_REV,
  GOSEQ_STEP_REV,
  GOSEQ_DONE
};
uint8_t  g_goseq = GOSEQ_OFF;
uint32_t g_goseq_phase_ms = 0;

// Ultrasonic read throttle
uint32_t g_last_us_ms = 0;
const uint32_t US_PERIOD_MS = 60;   // HC-SR04 datasheet recommends >= 50 ms
long g_last_us_cm = -1;

// Serial parser buffer
char g_rx_buf[64];
uint8_t g_rx_len = 0;

// ---------- Human test mode ----------
// An on-Mega mirror of the Pi brain states so teammates can exercise each
// phase from the Arduino IDE Serial Monitor with no Pi connected. Independent
// of the ROS protocol above; the two mechanisms coexist.
enum TestState : uint8_t {
  TS_OFF = 0,           // human test FSM inactive
  TS_IDLE,              // stopped, waiting for next 'state' command
  TS_SEARCHING,         // rotating in place
  TS_AWAITING,          // stopped, "waiting for operator confirm"
  TS_APPROACHING,       // driving forward
  TS_WARNING,           // countdown before extinguishing
  TS_EXT_PIN,           // solenoid pin-pull phase
  TS_EXT_ADVANCE,       // lead-screw advance phase
  TS_EXT_RETRACT,       // lead-screw retract phase
  TS_COMPLETE           // stopped, auto-returns to TS_IDLE
};
uint8_t  g_test_state = TS_OFF;
uint32_t g_test_state_start_ms = 0;
// Indoor-safe defaults: slow forward, slightly faster in-place rotation.
// Rotating in place doesn't translate the chassis so it can afford more PWM.
// Indoor-safe defaults = tested_arduino/front_reverse_spin motorSpeed (75)
int      g_test_rotate_speed = 75;
int      g_test_forward_speed = 75;
// Approaching in the state-mirror demo pulses forward in short chunks so the
// robot never runs more than ~half a second before re-evaluating. Teammates
// can see every step of the approach; the chassis won't cross a small room.
const uint16_t TS_APPROACH_PULSE_MS = 400;
const uint16_t TS_APPROACH_REST_MS  = 600;
bool     g_ts_approach_moving = false;
uint32_t g_ts_approach_last_edge_ms = 0;
uint8_t  g_test_warning_secs = 5;
int8_t   g_test_last_countdown = -1;

bool     g_verbose = false;
uint32_t g_last_verbose_ms = 0;
const uint32_t VERBOSE_PERIOD_MS = 500;

// Lightweight periodic sensor monitor (one line per tick) -- lets teammates
// watch the ultrasonic / IR / mic values scroll by without the full
// printHumanStatus() block. Selected via `watch <us|ir|mic|all> [ms]`.
enum WatchMode : uint8_t {
  WATCH_OFF = 0,
  WATCH_US,
  WATCH_IR,
  WATCH_MIC,
  WATCH_ALL
};
uint8_t  g_watch_mode = WATCH_OFF;
uint32_t g_watch_period_ms = 1000;
uint32_t g_last_watch_ms = 0;

// ---------- Canned test sequences ----------
// Short scripted routines so teammates can check a subsystem with one word.
// Each step is (action, duration_ms). A step with duration_ms == 0 ends the
// sequence. `test all` chains motors -> stepper -> solenoid.
struct TestStep {
  uint8_t  action;       // see actionName() below
  uint16_t duration_ms;
};

enum TestSeq : uint8_t {
  TSEQ_NONE = 0,
  TSEQ_MOTORS,
  TSEQ_SOLENOID,
  TSEQ_STEPPER,
  TSEQ_ALL_MOTORS,
  TSEQ_ALL_STEPPER,
  TSEQ_ALL_SOLENOID
};

uint8_t  g_tseq = TSEQ_NONE;
uint8_t  g_tseq_step = 0;
uint32_t g_tseq_step_start_ms = 0;
// Indoor-safe default PWM for canned tests. Teammates can still override
// with e.g. `test motors 150` when the robot is on blocks.
int      g_tseq_speed = 75;

// Action codes (shared across all sequences)
//  0 = stop motors + solenoid + stepper
//  1 = forward          2 = back
//  3 = spin left        4 = spin right
//  5 = solenoid ON      6 = solenoid OFF
//  7 = stepper advance  8 = stepper retract
// Forward / back bursts are short (~500 ms) so `test motors` won't cross a
// small room; in-place rotations can run longer because they don't translate.
static const TestStep MOTOR_SEQ[] PROGMEM = {
  {1, 500}, {0, 500}, {2, 500}, {0, 500},
  {3, 1000}, {0, 500}, {4, 1000}, {0, 500},
  {0, 0}
};
static const TestStep SOLENOID_SEQ[] PROGMEM = {
  {5, 1000}, {6, 800}, {5, 1000}, {6, 0}
};
static const TestStep STEPPER_SEQ[] PROGMEM = {
  {7, 5300}, {0, 500}, {8, 5300}, {0, 0}
};

// ---------- Forward declarations ----------
// Arduino normally auto-prototypes, but spelling these out keeps the file
// buildable in plain avr-gcc projects too.
static void handleMotorCmd(int vx, int vy, int wz);
static void startExtinguisherPhase(uint8_t phase);
static void resetStepperRamp();
static void cancelGoSequence(bool silent = true);
static void serviceGoSequence();
static void startGoSequence();
static void cancelTestSeq();
static void cancelTestState();
static void serviceDriveWatchdog();

// ---------- Helpers (motor directions from tested_arduino/front_reverse_spin) ----------
static void applyDrive(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  const uint8_t lp = (uint8_t)abs(left);
  const uint8_t rp = (uint8_t)abs(right);

  if (left >= 0) {
    digitalWrite(FA2, HIGH);
    digitalWrite(FB2, LOW);
    digitalWrite(RA1, HIGH);
    digitalWrite(RB1, LOW);
  } else {
    digitalWrite(FA2, LOW);
    digitalWrite(FB2, HIGH);
    digitalWrite(RA1, LOW);
    digitalWrite(RB1, HIGH);
  }

  if (right >= 0) {
    digitalWrite(FA1, LOW);
    digitalWrite(FB1, HIGH);
    digitalWrite(RA2, LOW);
    digitalWrite(RB2, HIGH);
  } else {
    digitalWrite(FA1, HIGH);
    digitalWrite(FB1, LOW);
    digitalWrite(RA2, HIGH);
    digitalWrite(RB2, LOW);
  }

  analogWrite(FP2, lp);
  analogWrite(RP1, lp);
  analogWrite(FP1, rp);
  analogWrite(RP2, rp);
}

static void stopMotors() {
  analogWrite(FP1, 0);
  analogWrite(FP2, 0);
  analogWrite(RP1, 0);
  analogWrite(RP2, 0);
  digitalWrite(FA1, LOW);
  digitalWrite(FB1, LOW);
  digitalWrite(FA2, LOW);
  digitalWrite(FB2, LOW);
  digitalWrite(RA1, LOW);
  digitalWrite(RB1, LOW);
  digitalWrite(RA2, LOW);
  digitalWrite(RB2, LOW);
  g_left_cmd = 0;
  g_right_cmd = 0;
}

static void resetStepperRamp() {
  g_step_half_us = STEP_RAMP_START_HALF_US;
  g_ramp_pulses_at_speed = 0;
  g_stepper_ramp_complete = false;
  digitalWrite(PIN_STEP, LOW);
  g_step_line_high = 0;
  g_step_last_half_us = micros();
}

/** Stop bench "GO" sequence. @param silent if true, no L,go_cancelled line */
static void cancelGoSequence(bool silent) {
  bool was_active = (g_goseq != GOSEQ_OFF && g_goseq != GOSEQ_DONE);
  g_goseq = GOSEQ_OFF;
  digitalWrite(PIN_SOL, LOW);
  g_step_dir = 0;
  digitalWrite(PIN_STEP, LOW);
  g_step_line_high = 0;
  if (was_active && !silent) {
    Serial.println(F("L,go_cancelled"));
  }
}

static void allOutputsOff() {
  stopMotors();
  cancelGoSequence(true);
  digitalWrite(PIN_SOL, LOW);
  g_step_dir = 0;
  digitalWrite(PIN_STEP, LOW);
  g_step_line_high = 0;
  g_ext_phase = 0;
  g_protocol_drive = false;
  g_ts_approach_moving = false;
}

// Skid-steer mix (vy ignored on purpose)
static void handleMotorCmd(int vx, int /*vy*/, int wz) {
  int left  = vx + wz;
  int right = vx - wz;
  g_left_cmd  = constrain(left,  -255, 255);
  g_right_cmd = constrain(right, -255, 255);
  applyDrive(g_left_cmd, g_right_cmd);
  g_state = (g_left_cmd || g_right_cmd) ? FW_DRIVING : FW_IDLE;
}

static void startExtinguisherPhase(uint8_t phase) {
  // Bench "go" owns solenoid + stepper timing. ROS bridge publishes E,1/E,2/E,3 on
  // every tick — the old code called cancelGoSequence() first, which zeroed
  // g_step_dir and killed the stepper mid-"go".
  if (g_goseq != GOSEQ_OFF && g_goseq != GOSEQ_DONE && phase >= 1 && phase <= 3)
    return;

  // Same E phase as last apply (Pi publishes every tick): never cancel go / reset outputs.
  if (phase == g_ext_phase)
    return;

  cancelGoSequence(true);

  g_ext_phase = phase;
  g_ext_phase_start_ms = millis();
  switch (phase) {
    case 0:  // off
      digitalWrite(PIN_SOL, LOW);
      g_step_dir = 0;
      digitalWrite(PIN_STEP, LOW);
      g_step_line_high = 0;
      g_state = (g_left_cmd || g_right_cmd) ? FW_DRIVING : FW_IDLE;
      break;
    case 1:  // pin pull: energize solenoid
      digitalWrite(PIN_SOL, HIGH);
      g_step_dir = 0;
      g_state = FW_EXTINGUISHING;
      break;
    case 2:  // advance lead-screw (same DIR as runStepperForDuration: LOW)
      digitalWrite(PIN_SOL, LOW);
      digitalWrite(PIN_DIR, LOW);
      delayMicroseconds(5);
      g_step_dir = +1;
      resetStepperRamp();
      g_state = FW_EXTINGUISHING;
      break;
    case 3:  // retract (same DIR as reverseStepperForDuration: HIGH)
      digitalWrite(PIN_SOL, LOW);
      digitalWrite(PIN_DIR, HIGH);
      delayMicroseconds(5);
      g_step_dir = -1;
      resetStepperRamp();
      g_state = FW_EXTINGUISHING;
      break;
    case 4:  // stop
    default:
      digitalWrite(PIN_SOL, LOW);
      g_step_dir = 0;
      digitalWrite(PIN_STEP, LOW);
      g_step_line_high = 0;
      g_state = (g_left_cmd || g_right_cmd) ? FW_DRIVING : FW_IDLE;
      break;
  }
}

static void serviceExtinguisher() {
  // Bench "go" owns solenoid + stepper timing; do not auto-expire Pi E,2/E,3 here.
  if (g_goseq != GOSEQ_OFF && g_goseq != GOSEQ_DONE)
    return;
  if (g_ext_phase == 0 || g_ext_phase == 4) return;
  uint32_t elapsed = millis() - g_ext_phase_start_ms;
  if (g_ext_phase == 1 && elapsed >= EXT_PIN_PULL_MS) {
    // Solenoid stays latched by default; Pi decides when to de-energize via E,0.
    // Keep it simple: after the pin-pull interval, just hold the phase until
    // the Pi issues the next E command.
    return;
  }
  if (g_ext_phase == 2 && elapsed >= EXT_STEPPER_RUN_MS) {
    startExtinguisherPhase(4);
  } else if (g_ext_phase == 3 && elapsed >= EXT_STEPPER_RUN_MS) {
    startExtinguisherPhase(4);
  }
}

static void startGoSequence() {
  if (g_goseq != GOSEQ_OFF && g_goseq != GOSEQ_DONE) {
    Serial.println(F("L,go_busy"));
    return;
  }
  cancelTestState();
  cancelGoSequence(true);
  // Drop any Pi E,* phase so we do not immediately hit EXT_STEPPER_RUN_MS and
  // startExtinguisherPhase(4) -> cancelGoSequence() (killed stepper during 'go').
  g_ext_phase = 0;
  g_ext_phase_start_ms = millis();
  digitalWrite(PIN_SOL, LOW);
  g_step_dir = 0;
  digitalWrite(PIN_STEP, LOW);
  g_step_line_high = 0;
  g_goseq = GOSEQ_PRE_SOL;
  g_goseq_phase_ms = millis();
  g_state = FW_EXTINGUISHING;
  Serial.println(F("L,go_start"));
}

/** Timers match solenoid_stepper_combined.ino; stepper uses serviceStepper() ramp. */
static void serviceGoSequence() {
  if (g_goseq == GOSEQ_OFF || g_goseq == GOSEQ_DONE) return;

  uint32_t now = millis();
  uint32_t t = now - g_goseq_phase_ms;

  switch (g_goseq) {
    case GOSEQ_PRE_SOL:
      if (t >= GO_PRE_SOL_MS) {
        g_goseq = GOSEQ_SOL_ON;
        g_goseq_phase_ms = now;
        digitalWrite(PIN_SOL, HIGH);
      }
      break;
    case GOSEQ_SOL_ON:
      if (t >= EXT_PIN_PULL_MS) {
        digitalWrite(PIN_SOL, LOW);
        g_goseq = GOSEQ_POST_SOL;
        g_goseq_phase_ms = now;
      }
      break;
    case GOSEQ_POST_SOL:
      if (t >= GO_POST_SOL_MS) {
        g_goseq = GOSEQ_STEP_FWD;
        g_goseq_phase_ms = now;
        digitalWrite(PIN_DIR, LOW);
        delayMicroseconds(5);
        g_step_dir = +1;
        resetStepperRamp();
      }
      break;
    case GOSEQ_STEP_FWD:
      if (t >= EXT_STEPPER_RUN_MS) {
        g_step_dir = 0;
        digitalWrite(PIN_STEP, LOW);
        g_step_line_high = 0;
        g_goseq = GOSEQ_PRE_REV;
        g_goseq_phase_ms = now;
      }
      break;
    case GOSEQ_PRE_REV:
      if (t >= GO_PRE_REV_MS) {
        g_goseq = GOSEQ_STEP_REV;
        g_goseq_phase_ms = now;
        digitalWrite(PIN_DIR, HIGH);
        delayMicroseconds(5);
        g_step_dir = -1;
        resetStepperRamp();
      }
      break;
    case GOSEQ_STEP_REV:
      if (t >= EXT_STEPPER_RUN_MS) {
        g_step_dir = 0;
        digitalWrite(PIN_STEP, LOW);
        g_step_line_high = 0;
        digitalWrite(PIN_SOL, LOW);
        g_goseq = GOSEQ_DONE;
        g_state = FW_IDLE;
        Serial.println(F("L,go_complete"));
      }
      break;
    default:
      break;
  }
}

static void serviceStepper() {
  if (g_step_dir == 0) return;
  // Same timing as bench stepMotor(delayUs): HIGH for delayUs, LOW for delayUs,
  // then next step — not a free-running 50% square wave (some A4988 layouts
  // misbehave if the first edge after DIR change is ambiguous).
  const uint16_t kMaxHalfEdges = 800;
  for (uint16_t n = 0; n < kMaxHalfEdges; n++) {
    uint16_t half = g_stepper_ramp_complete ? STEP_RAMP_END_HALF_US : g_step_half_us;
    uint32_t now = micros();
    if ((uint32_t)(now - g_step_last_half_us) < half) break;

    if (g_step_line_high == 0) {
      digitalWrite(PIN_STEP, HIGH);
      g_step_line_high = 1;
    } else {
      digitalWrite(PIN_STEP, LOW);
      g_step_line_high = 0;
      if (!g_stepper_ramp_complete) {
        g_ramp_pulses_at_speed++;
        if (g_ramp_pulses_at_speed >= STEP_RAMP_STEPS_PER_BAND) {
          g_ramp_pulses_at_speed = 0;
          int next = (int)g_step_half_us + STEP_RAMP_DELTA_HALF_US;
          if (next >= (int)STEP_RAMP_END_HALF_US) {
            g_step_half_us = (uint16_t)next;
          } else {
            g_step_half_us = STEP_RAMP_END_HALF_US;
            g_stepper_ramp_complete = true;
          }
        }
      }
    }
    g_step_last_half_us += half;
  }
}

static void serviceEncoders() {
  // Poll one representative pin for each aggregate channel. All 4 ganged
  // pins sit on the same net so any one will track the shared signal.
  uint8_t a = digitalRead(ENC_A_PINS[0]);
  uint8_t b = digitalRead(ENC_B_PINS[0]);
  if (a != g_last_encA) g_encA_count++;
  if (b != g_last_encB) g_encB_count++;
  g_last_encA = a;
  g_last_encB = b;
}

static long readUltrasonicCm() {
  if (!g_en_us) return -1;
  uint32_t now = millis();
  if ((uint32_t)(now - g_last_us_ms) < US_PERIOD_MS) {
    return g_last_us_cm;
  }
  g_last_us_ms = now;
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  // 25 ms timeout covers up to ~4 m
  unsigned long dur = pulseIn(PIN_US_ECHO, HIGH, 25000UL);
  if (dur == 0) {
    g_last_us_cm = -1;
  } else {
    g_last_us_cm = (long)(dur / 58UL);
  }
  return g_last_us_cm;
}

static int readIr() {
  if (!g_en_ir) return -1;
  // KY-032 pulls LOW on object detection (shared with MAX9814 on the /OUT net).
  return digitalRead(PIN_OUT_DIGITAL) == LOW ? 1 : 0;
}

static int readMic() {
  if (!g_en_mic) return -1;
  // If KY-032 is active it pulls the shared wire to ~0 V, so mic reads ~0.
  // Brain is expected to cross-check /sensors/ir before trusting mic values.
  return analogRead(PIN_OUT_ANALOG);
}

static void sendStatus() {
  long us  = readUltrasonicCm();
  int  ir  = readIr();
  int  mic = readMic();
  noInterrupts();
  long a = g_encA_count;
  long b = g_encB_count;
  interrupts();
  Serial.print(F("D,"));
  Serial.print(a);    Serial.print(',');
  Serial.print(b);    Serial.print(',');
  Serial.print(us);   Serial.print(',');
  Serial.print(ir);   Serial.print(',');
  Serial.print(mic);  Serial.print(',');
  Serial.println(g_state);
}

static void handleConfigure(const char* body) {
  char which[8];
  int val = 0;
  if (sscanf(body, "%7[^,],%d", which, &val) != 2) return;
  bool on = (val != 0);
  if (strcmp(which, "US") == 0)       g_en_us  = on;
  else if (strcmp(which, "IR") == 0)  g_en_ir  = on;
  else if (strcmp(which, "MIC") == 0) g_en_mic = on;
}

// ---------- Human-readable status ----------
static const char* testStateName(uint8_t s) {
  switch (s) {
    case TS_OFF:          return "off";
    case TS_IDLE:         return "idle";
    case TS_SEARCHING:    return "searching";
    case TS_AWAITING:     return "awaiting_confirm";
    case TS_APPROACHING:  return "approaching";
    case TS_WARNING:      return "warning";
    case TS_EXT_PIN:      return "extinguishing (pin_pull)";
    case TS_EXT_ADVANCE:  return "extinguishing (advance)";
    case TS_EXT_RETRACT:  return "extinguishing (retract)";
    case TS_COMPLETE:     return "complete";
    default:              return "?";
  }
}

static const char* fwStateName(uint8_t s) {
  switch (s) {
    case FW_IDLE:         return "IDLE";
    case FW_DRIVING:      return "DRIVING";
    case FW_EXTINGUISHING:return "EXTINGUISHING";
    case FW_ESTOP:        return "ESTOP";
    default:              return "?";
  }
}

static void printHumanStatus() {
  long us  = readUltrasonicCm();
  int  ir  = readIr();
  int  mic = readMic();
  noInterrupts();
  long a = g_encA_count;
  long b = g_encB_count;
  interrupts();

  Serial.println(F("--- firebot_mega status ---"));
  Serial.print(F("fw_state     : ")); Serial.println(fwStateName(g_state));
  Serial.print(F("test_state   : ")); Serial.println(testStateName(g_test_state));
  Serial.print(F("drive        : left=")); Serial.print(g_left_cmd);
  Serial.print(F(" right="));              Serial.println(g_right_cmd);
  Serial.print(F("ext_phase    : "));      Serial.println(g_ext_phase);
  Serial.print(F("solenoid     : "));      Serial.println(digitalRead(PIN_SOL) ? F("ON") : F("off"));
  Serial.print(F("stepper_dir  : "));      Serial.println(g_step_dir);
  Serial.print(F("encA ticks   : "));      Serial.println(a);
  Serial.print(F("encB ticks   : "));      Serial.println(b);
  Serial.print(F("ultrasonic   : "));
  if (!g_en_us) Serial.println(F("off"));
  else if (us < 0) Serial.println(F("no_echo"));
  else { Serial.print(us); Serial.println(F(" cm")); }
  Serial.print(F("ir           : "));
  if (!g_en_ir) Serial.println(F("off"));
  else Serial.println(ir ? F("TRIGGERED") : F("clear"));
  Serial.print(F("mic envelope : "));
  if (!g_en_mic) Serial.println(F("off"));
  else { Serial.print(mic); Serial.println(F(" (0..1023)")); }
  Serial.println(F("---------------------------"));
}

static void printWatchLine() {
  // One-line snapshot of whichever sensor(s) are being watched. Disabled
  // sensors show 'off' so it's obvious when nothing's being read.
  long us  = readUltrasonicCm();
  int  ir  = readIr();
  int  mic = readMic();
  Serial.print(F("W,"));
  Serial.print(millis());
  if (g_watch_mode == WATCH_US || g_watch_mode == WATCH_ALL) {
    Serial.print(F(" us="));
    if (!g_en_us)       Serial.print(F("off"));
    else if (us < 0)    Serial.print(F("no_echo"));
    else              { Serial.print(us); Serial.print(F("cm")); }
  }
  if (g_watch_mode == WATCH_IR || g_watch_mode == WATCH_ALL) {
    Serial.print(F(" ir="));
    if (!g_en_ir) Serial.print(F("off"));
    else          Serial.print(ir ? F("trig") : F("clear"));
  }
  if (g_watch_mode == WATCH_MIC || g_watch_mode == WATCH_ALL) {
    Serial.print(F(" mic="));
    if (!g_en_mic) Serial.print(F("off"));
    else           Serial.print(mic);
  }
  Serial.println();
}

static void printHelp() {
  Serial.println(F("FireBot696 Mega -- Serial Monitor help"));
  Serial.println(F("Set line ending to 'Newline' in the Serial Monitor."));
  Serial.println();
  Serial.println(F("Motion:"));
  Serial.println(F("  forward <0..255>        drive forward"));
  Serial.println(F("  back <0..255>           drive backward"));
  Serial.println(F("  left <0..255>           rotate left in place"));
  Serial.println(F("  right <0..255>          rotate right in place"));
  Serial.println(F("  drive <vx> <wz>         raw: each -255..255"));
  Serial.println(F("  stop                    motors off"));
  Serial.println();
  Serial.println(F("Extinguisher:"));
  Serial.println(F("  pin on | pin off        solenoid (pin-puller)"));
  Serial.println(F("  go                      full bench sequence (same timing as tested sketch)"));
  Serial.println(F("  advance                 lead-screw forward (~5.3 s auto-stop)"));
  Serial.println(F("  retract                 lead-screw reverse (~5.3 s auto-stop)"));
  Serial.println(F("  extstop                 halt the stepper now"));
  Serial.println();
  Serial.println(F("Sensors (disabled = '-1' / 'off'):"));
  Serial.println(F("  sensor us on|off        HC-SR04"));
  Serial.println(F("  sensor ir on|off        KY-032"));
  Serial.println(F("  sensor mic on|off       MAX9814"));
  Serial.println(F("  us | ir | mic           one-shot read"));
  Serial.println(F("  watch us [ms]           periodic ultrasonic, default 1000 ms"));
  Serial.println(F("  watch ir [ms]           periodic IR"));
  Serial.println(F("  watch mic [ms]          periodic mic envelope"));
  Serial.println(F("  watch all [ms]          periodic line for every enabled sensor"));
  Serial.println(F("  watch off               stop periodic sensor prints"));
  Serial.println();
  Serial.println(F("State mirror (brain-state demos on the Mega):"));
  Serial.println(F("  state idle              stop"));
  Serial.println(F("  state searching         spin in place"));
  Serial.println(F("  state awaiting          stop and wait"));
  Serial.println(F("  state approaching       drive forward"));
  Serial.println(F("  state warning           5s countdown, auto-extinguish"));
  Serial.println(F("  state extinguishing     pin -> advance -> retract"));
  Serial.println(F("  state complete          stop, auto-return to idle"));
  Serial.println(F("  state next              advance to the next phase"));
  Serial.println();
  Serial.println(F("Canned tests (one-word subsystem sanity checks):"));
  Serial.println(F("  test motors [speed]     fwd/back/left/right cycle"));
  Serial.println(F("  test solenoid           on/off/on/off"));
  Serial.println(F("  test stepper            advance ~5.3s then retract ~5.3s"));
  Serial.println(F("  test all [speed]        motors -> stepper -> solenoid"));
  Serial.println(F("  test stop               cancel a running sequence"));
  Serial.println();
  Serial.println(F("Misc:"));
  Serial.println(F("  help | ?                this menu"));
  Serial.println(F("  status                  pretty status dump"));
  Serial.println(F("  verbose on|off          auto-print status every 500 ms"));
  Serial.println(F("  estop | reset           hard stop all outputs"));
  Serial.println();
  Serial.println(F("The ROS protocol (M/E/W/S/R/C) still works -- the Pi uses it."));
  Serial.println(F("When the Pi is connected, a 1 s watchdog auto-stops motors if"));
  Serial.println(F("no M command arrives. Human commands (forward/left/etc) are not"));
  Serial.println(F("watchdogged; they stay latched until you type 'stop'."));
}

// ---------- Test-state machine (human mode mirror of Pi brain) ----------
static void enterTestState(uint8_t ts) {
  // A state-mirror demo cannot coexist with a canned test sequence.
  if (g_tseq != TSEQ_NONE) cancelTestSeq();
  cancelGoSequence(true);
  g_test_state = ts;
  g_test_state_start_ms = millis();
  g_test_last_countdown = -1;

  switch (ts) {
    case TS_OFF:
    case TS_IDLE:
    case TS_AWAITING:
    case TS_COMPLETE:
      stopMotors();
      if (ts == TS_COMPLETE) startExtinguisherPhase(0);
      break;
    case TS_SEARCHING:
      // Rotate in place: negative right, positive left -> left motors forward,
      // right motors backward. That's a skid-steer spin (wz only, vx=0).
      handleMotorCmd(0, 0, g_test_rotate_speed);
      break;
    case TS_APPROACHING:
      // Pulse forward; serviceTestState() flips g_ts_approach_moving every
      // TS_APPROACH_PULSE_MS / TS_APPROACH_REST_MS so the robot crawls in
      // visible short chunks rather than charging across the room.
      g_ts_approach_moving = true;
      g_ts_approach_last_edge_ms = millis();
      handleMotorCmd(g_test_forward_speed, 0, 0);
      break;
    case TS_WARNING:
      stopMotors();
      Serial.print(F("L,warn_countdown_start ")); Serial.println(g_test_warning_secs);
      break;
    case TS_EXT_PIN:
      stopMotors();
      startExtinguisherPhase(1);
      break;
    case TS_EXT_ADVANCE:
      startExtinguisherPhase(2);
      break;
    case TS_EXT_RETRACT:
      startExtinguisherPhase(3);
      break;
  }

  Serial.print(F("L,test_state=")); Serial.println(testStateName(g_test_state));
}

static void serviceTestState() {
  if (g_test_state == TS_OFF) return;
  uint32_t elapsed = millis() - g_test_state_start_ms;

  switch (g_test_state) {
    case TS_WARNING: {
      int8_t remaining = (int8_t)(g_test_warning_secs - (elapsed / 1000UL));
      if (remaining < 0) remaining = 0;
      if (remaining != g_test_last_countdown) {
        g_test_last_countdown = remaining;
        Serial.print(F("L,countdown=")); Serial.println(remaining);
      }
      if (elapsed >= (uint32_t)g_test_warning_secs * 1000UL) {
        enterTestState(TS_EXT_PIN);
      }
      break;
    }
    case TS_EXT_PIN:
      if (elapsed >= EXT_PIN_PULL_MS) enterTestState(TS_EXT_ADVANCE);
      break;
    case TS_EXT_ADVANCE:
      // serviceExtinguisher() stops the stepper after EXT_STEPPER_RUN_MS;
      // we time the test FSM to match that lead-screw run window.
      if (elapsed >= EXT_STEPPER_RUN_MS) enterTestState(TS_EXT_RETRACT);
      break;
    case TS_EXT_RETRACT:
      if (elapsed >= EXT_STEPPER_RUN_MS) enterTestState(TS_COMPLETE);
      break;
    case TS_COMPLETE:
      if (elapsed >= 3000UL) enterTestState(TS_IDLE);
      break;
    case TS_SEARCHING:
      // Keep re-asserting the drive in case something else cleared it.
      if (g_left_cmd == 0 && g_right_cmd == 0) {
        handleMotorCmd(0, 0, g_test_rotate_speed);
      }
      break;
    case TS_APPROACHING: {
      uint32_t now = millis();
      uint32_t since_edge = now - g_ts_approach_last_edge_ms;
      if (g_ts_approach_moving && since_edge >= TS_APPROACH_PULSE_MS) {
        g_ts_approach_moving = false;
        g_ts_approach_last_edge_ms = now;
        stopMotors();
        Serial.println(F("L,approach_pulse=rest"));
      } else if (!g_ts_approach_moving && since_edge >= TS_APPROACH_REST_MS) {
        g_ts_approach_moving = true;
        g_ts_approach_last_edge_ms = now;
        handleMotorCmd(g_test_forward_speed, 0, 0);
        Serial.println(F("L,approach_pulse=drive"));
      }
      break;
    }
    default:
      break;
  }
}

static uint8_t nextTestState(uint8_t cur) {
  switch (cur) {
    case TS_OFF:
    case TS_IDLE:         return TS_SEARCHING;
    case TS_SEARCHING:    return TS_AWAITING;
    case TS_AWAITING:     return TS_APPROACHING;
    case TS_APPROACHING:  return TS_WARNING;
    case TS_WARNING:
    case TS_EXT_PIN:
    case TS_EXT_ADVANCE:
    case TS_EXT_RETRACT:  return TS_COMPLETE;
    case TS_COMPLETE:     return TS_IDLE;
    default:              return TS_IDLE;
  }
}

// ---------- Canned test sequence runner ----------
static const char* actionName(uint8_t a) {
  switch (a) {
    case 0: return "stop";
    case 1: return "forward";
    case 2: return "back";
    case 3: return "spin_left";
    case 4: return "spin_right";
    case 5: return "pin_on";
    case 6: return "pin_off";
    case 7: return "advance";
    case 8: return "retract";
    default: return "?";
  }
}

static void performAction(uint8_t a) {
  switch (a) {
    case 0:
      cancelGoSequence(true);
      stopMotors();
      digitalWrite(PIN_SOL, LOW);
      g_step_dir = 0;
      digitalWrite(PIN_STEP, LOW);
      g_step_line_high = 0;
      break;
    case 1: handleMotorCmd( g_tseq_speed, 0, 0); break;
    case 2: handleMotorCmd(-g_tseq_speed, 0, 0); break;
    case 3: handleMotorCmd(0, 0,  g_tseq_speed); break;
    case 4: handleMotorCmd(0, 0, -g_tseq_speed); break;
    case 5: startExtinguisherPhase(1); break;
    case 6: startExtinguisherPhase(0); break;
    case 7: startExtinguisherPhase(2); break;
    case 8: startExtinguisherPhase(3); break;
  }
}

static const TestStep* stepsForSeq(uint8_t seq) {
  switch (seq) {
    case TSEQ_MOTORS:
    case TSEQ_ALL_MOTORS:    return MOTOR_SEQ;
    case TSEQ_SOLENOID:
    case TSEQ_ALL_SOLENOID:  return SOLENOID_SEQ;
    case TSEQ_STEPPER:
    case TSEQ_ALL_STEPPER:   return STEPPER_SEQ;
    default:                 return nullptr;
  }
}

static const char* seqName(uint8_t seq) {
  switch (seq) {
    case TSEQ_MOTORS:
    case TSEQ_ALL_MOTORS:    return "motors";
    case TSEQ_SOLENOID:
    case TSEQ_ALL_SOLENOID:  return "solenoid";
    case TSEQ_STEPPER:
    case TSEQ_ALL_STEPPER:   return "stepper";
    default:                 return "none";
  }
}

static TestStep loadStep(const TestStep* base, uint8_t idx) {
  TestStep out;
  out.action      = pgm_read_byte(&base[idx].action);
  out.duration_ms = pgm_read_word(&base[idx].duration_ms);
  return out;
}

static void runStep();  // forward decl

static void startTestSeq(uint8_t seq, int speed) {
  // A canned test owns the motors, solenoid, and stepper end-to-end, so
  // silently drop out of any running state-mirror demo first.
  if (g_test_state != TS_OFF) {
    g_test_state = TS_OFF;
    Serial.println(F("L,test_state=off"));
  }
  cancelGoSequence(true);
  g_tseq = seq;
  g_tseq_step = 0;
  g_tseq_speed = constrain(speed, 0, 255);
  Serial.print(F("L,test_begin=")); Serial.print(seqName(seq));
  Serial.print(F(" speed=")); Serial.println(g_tseq_speed);
  runStep();
}

static void cancelTestSeq() {
  if (g_tseq == TSEQ_NONE) return;
  g_tseq = TSEQ_NONE;
  g_tseq_step = 0;
  performAction(0);
  Serial.println(F("L,test_cancelled"));
}

static void runStep() {
  const TestStep* base = stepsForSeq(g_tseq);
  if (!base) { g_tseq = TSEQ_NONE; return; }
  TestStep step = loadStep(base, g_tseq_step);
  if (step.duration_ms == 0) {
    performAction(step.action);
    Serial.print(F("L,test_done=")); Serial.println(seqName(g_tseq));
    // Chain to next sub-sequence if this was part of 'test all'.
    uint8_t nxt = TSEQ_NONE;
    if (g_tseq == TSEQ_ALL_MOTORS)   nxt = TSEQ_ALL_STEPPER;
    else if (g_tseq == TSEQ_ALL_STEPPER) nxt = TSEQ_ALL_SOLENOID;
    g_tseq = TSEQ_NONE;
    g_tseq_step = 0;
    if (nxt != TSEQ_NONE) startTestSeq(nxt, g_tseq_speed);
    return;
  }
  performAction(step.action);
  g_tseq_step_start_ms = millis();
  Serial.print(F("L,test_step ")); Serial.print(seqName(g_tseq));
  Serial.print('[');  Serial.print(g_tseq_step);
  Serial.print(F("] ")); Serial.print(actionName(step.action));
  Serial.print(F(" ")); Serial.print(step.duration_ms); Serial.println(F("ms"));
}

static void serviceTestSeq() {
  if (g_tseq == TSEQ_NONE) return;
  const TestStep* base = stepsForSeq(g_tseq);
  if (!base) { g_tseq = TSEQ_NONE; return; }
  TestStep step = loadStep(base, g_tseq_step);
  if (step.duration_ms == 0) return;  // safety
  if ((uint32_t)(millis() - g_tseq_step_start_ms) >= step.duration_ms) {
    g_tseq_step++;
    runStep();
  }
}

// Pi-drive watchdog: called every loop. If we were driving on behalf of
// the Pi but no 'M' command has arrived in PROTOCOL_DRIVE_WATCHDOG_MS,
// stop the motors and disarm. This catches crashed brain nodes, closed
// serial ports, and unplugged USB cables.
static void serviceDriveWatchdog() {
  if (!g_protocol_drive) return;
  if ((uint32_t)(millis() - g_last_protocol_drive_ms) < PROTOCOL_DRIVE_WATCHDOG_MS) return;
  if (g_left_cmd != 0 || g_right_cmd != 0) {
    stopMotors();
    Serial.println(F("L,watchdog: Pi stopped sending M; motors stopped"));
  }
  g_protocol_drive = false;
}

// ---------- Human command parser ----------
static char* skipSpaces(char* p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

static char* nextToken(char* p, char** out_tok) {
  p = skipSpaces(p);
  *out_tok = p;
  while (*p && *p != ' ' && *p != '\t') p++;
  if (*p) {
    *p = 0;
    p++;
  }
  return p;
}

static void toLowerStr(char* s) {
  for (; *s; s++) if (*s >= 'A' && *s <= 'Z') *s += 32;
}

static bool parseInt(const char* tok, int* out) {
  if (!tok || !*tok) return false;
  int sign = 1;
  if (*tok == '-') { sign = -1; tok++; }
  else if (*tok == '+') tok++;
  if (!*tok) return false;
  int v = 0;
  while (*tok) {
    if (*tok < '0' || *tok > '9') return false;
    v = v * 10 + (*tok - '0');
    tok++;
  }
  *out = v * sign;
  return true;
}

static void cancelTestState() {
  if (g_test_state != TS_OFF) {
    g_test_state = TS_OFF;
    Serial.println(F("L,test_state=off"));
  }
  // Any state cancel also cancels a running canned test sequence so motion
  // commands never fight the scripted sequence timing.
  if (g_tseq != TSEQ_NONE) cancelTestSeq();
  // Human operator just took over -- the Pi watchdog should stop nannying
  // this drive command.
  g_protocol_drive = false;
  g_ts_approach_moving = false;
}

static void humanSensor(char* rest) {
  char* which; char* on_off;
  rest = nextToken(rest, &which);
  rest = nextToken(rest, &on_off);
  toLowerStr(which);
  toLowerStr(on_off);
  bool on = (strcmp(on_off, "on") == 0 || strcmp(on_off, "1") == 0 ||
             strcmp(on_off, "true") == 0);
  if      (strcmp(which, "us")  == 0) g_en_us  = on;
  else if (strcmp(which, "ir")  == 0) g_en_ir  = on;
  else if (strcmp(which, "mic") == 0) g_en_mic = on;
  else { Serial.println(F("L,err unknown sensor")); return; }
  Serial.print(F("L,sensor_")); Serial.print(which);
  Serial.print('='); Serial.println(on ? F("on") : F("off"));
}

static void humanState(char* rest) {
  char* name;
  nextToken(rest, &name);
  toLowerStr(name);
  if      (strcmp(name, "idle")          == 0) enterTestState(TS_IDLE);
  else if (strcmp(name, "searching")     == 0) enterTestState(TS_SEARCHING);
  else if (strcmp(name, "awaiting")      == 0 ||
           strcmp(name, "awaiting_confirm") == 0) enterTestState(TS_AWAITING);
  else if (strcmp(name, "approaching")   == 0) enterTestState(TS_APPROACHING);
  else if (strcmp(name, "warning")       == 0) enterTestState(TS_WARNING);
  else if (strcmp(name, "extinguishing") == 0) enterTestState(TS_EXT_PIN);
  else if (strcmp(name, "complete")      == 0) enterTestState(TS_COMPLETE);
  else if (strcmp(name, "next")          == 0) enterTestState(nextTestState(g_test_state));
  else if (strcmp(name, "off")           == 0) cancelTestState();
  else Serial.println(F("L,err unknown state"));
}

static bool handleHumanLine(char* line) {
  char* cmd;
  char* rest = nextToken(line, &cmd);
  toLowerStr(cmd);
  if (*cmd == 0) return true;  // blank line, handled

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
    printHelp(); return true;
  }
  if (strcmp(cmd, "status") == 0) {
    printHumanStatus(); return true;
  }
  if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "halt") == 0) {
    cancelTestState();
    handleMotorCmd(0, 0, 0);
    Serial.println(F("L,stopped"));
    return true;
  }
  if (strcmp(cmd, "estop") == 0 || strcmp(cmd, "reset") == 0) {
    cancelTestState();
    allOutputsOff();
    g_state = FW_ESTOP;
    Serial.println(F("L,estop"));
    return true;
  }
  if (strcmp(cmd, "verbose") == 0) {
    char* on_off; nextToken(rest, &on_off); toLowerStr(on_off);
    g_verbose = (strcmp(on_off, "on") == 0 || strcmp(on_off, "1") == 0);
    Serial.print(F("L,verbose=")); Serial.println(g_verbose ? F("on") : F("off"));
    return true;
  }

  // Motion shorthands (one speed arg)
  int spd = 0;
  if (strcmp(cmd, "forward") == 0) {
    char* a; nextToken(rest, &a);
    if (!parseInt(a, &spd)) spd = 75;
    cancelTestState();
    handleMotorCmd(spd, 0, 0);
    return true;
  }
  if (strcmp(cmd, "back") == 0 || strcmp(cmd, "backward") == 0 ||
      strcmp(cmd, "reverse") == 0) {
    char* a; nextToken(rest, &a);
    if (!parseInt(a, &spd)) spd = 75;
    cancelTestState();
    handleMotorCmd(-spd, 0, 0);
    return true;
  }
  if (strcmp(cmd, "left") == 0) {
    char* a; nextToken(rest, &a);
    if (!parseInt(a, &spd)) spd = 75;
    cancelTestState();
    handleMotorCmd(0, 0, spd);
    return true;
  }
  if (strcmp(cmd, "right") == 0) {
    char* a; nextToken(rest, &a);
    if (!parseInt(a, &spd)) spd = 75;
    cancelTestState();
    handleMotorCmd(0, 0, -spd);
    return true;
  }
  if (strcmp(cmd, "drive") == 0) {
    char* a; char* b;
    rest = nextToken(rest, &a);
    nextToken(rest, &b);
    int vx = 0, wz = 0;
    if (parseInt(a, &vx) && parseInt(b, &wz)) {
      cancelTestState();
      handleMotorCmd(vx, 0, wz);
    } else {
      Serial.println(F("L,err usage: drive <vx> <wz>"));
    }
    return true;
  }

  // Extinguisher shorthands
  if (strcmp(cmd, "pin") == 0) {
    char* on_off; nextToken(rest, &on_off); toLowerStr(on_off);
    if (strcmp(on_off, "on") == 0 || strcmp(on_off, "1") == 0) {
      startExtinguisherPhase(1);
    } else {
      startExtinguisherPhase(0);
    }
    return true;
  }
  if (strcmp(cmd, "go") == 0) { startGoSequence(); return true; }
  if (strcmp(cmd, "advance") == 0) { startExtinguisherPhase(2); return true; }
  if (strcmp(cmd, "retract") == 0) { startExtinguisherPhase(3); return true; }
  if (strcmp(cmd, "extstop") == 0) { startExtinguisherPhase(4); return true; }

  // Sensor aliases
  if (strcmp(cmd, "sensor") == 0) { humanSensor(rest); return true; }
  if (strcmp(cmd, "us") == 0) {
    if (!g_en_us) { Serial.println(F("L,us=off -- enable with 'sensor us on'")); return true; }
    long v = readUltrasonicCm();
    Serial.print(F("L,us=")); Serial.println(v);
    return true;
  }
  if (strcmp(cmd, "ir") == 0) {
    if (!g_en_ir) { Serial.println(F("L,ir=off -- enable with 'sensor ir on'")); return true; }
    Serial.print(F("L,ir=")); Serial.println(readIr() ? F("TRIGGERED") : F("clear"));
    return true;
  }
  if (strcmp(cmd, "mic") == 0) {
    if (!g_en_mic) { Serial.println(F("L,mic=off -- enable with 'sensor mic on'")); return true; }
    Serial.print(F("L,mic=")); Serial.println(readMic());
    return true;
  }

  // Periodic sensor monitor: `watch us [ms]`, `watch ir [ms]`, `watch mic [ms]`,
  // `watch all [ms]`, `watch off`. Doesn't interfere with anything else --
  // independent timer that just prints a short line in the log.
  if (strcmp(cmd, "watch") == 0) {
    char* which; char* arg;
    rest = nextToken(rest, &which);
    nextToken(rest, &arg);
    toLowerStr(which);
    int period = (int)g_watch_period_ms;
    if (*arg) parseInt(arg, &period);
    if (period < 50) period = 50;  // avoid flooding the Serial Monitor

    uint8_t mode = WATCH_OFF;
    if      (strcmp(which, "us")  == 0) mode = WATCH_US;
    else if (strcmp(which, "ir")  == 0) mode = WATCH_IR;
    else if (strcmp(which, "mic") == 0) mode = WATCH_MIC;
    else if (strcmp(which, "all") == 0) mode = WATCH_ALL;
    else if (strcmp(which, "off") == 0 || strcmp(which, "stop") == 0) {
      g_watch_mode = WATCH_OFF;
      Serial.println(F("L,watch=off"));
      return true;
    } else {
      Serial.println(F("L,err usage: watch us|ir|mic|all [ms] | watch off"));
      return true;
    }
    g_watch_mode = mode;
    g_watch_period_ms = (uint32_t)period;
    g_last_watch_ms = millis();
    Serial.print(F("L,watch=")); Serial.print(which);
    Serial.print(F(" period=")); Serial.print(g_watch_period_ms); Serial.println(F("ms"));
    // Warn (but don't block) if the user forgot to enable the sensor.
    if ((mode == WATCH_US  && !g_en_us)  ||
        (mode == WATCH_IR  && !g_en_ir)  ||
        (mode == WATCH_MIC && !g_en_mic)) {
      Serial.println(F("L,hint: sensor is off -- enable with 'sensor <us|ir|mic> on'"));
    }
    return true;
  }

  // Brain-state mirror
  if (strcmp(cmd, "state") == 0) { humanState(rest); return true; }

  // Canned subsystem tests: `test motors [speed]`, `test solenoid`,
  // `test stepper`, `test all`, `test stop`.
  if (strcmp(cmd, "test") == 0) {
    char* which; char* arg;
    rest = nextToken(rest, &which);
    nextToken(rest, &arg);
    toLowerStr(which);
    int speed = g_tseq_speed;
    if (*arg) parseInt(arg, &speed);
    if      (strcmp(which, "motors")   == 0) startTestSeq(TSEQ_MOTORS, speed);
    else if (strcmp(which, "solenoid") == 0 ||
             strcmp(which, "pin")      == 0) startTestSeq(TSEQ_SOLENOID, speed);
    else if (strcmp(which, "stepper")  == 0 ||
             strcmp(which, "screw")    == 0) startTestSeq(TSEQ_STEPPER, speed);
    else if (strcmp(which, "all")      == 0) startTestSeq(TSEQ_ALL_MOTORS, speed);
    else if (strcmp(which, "stop")     == 0 ||
             strcmp(which, "cancel")   == 0) cancelTestSeq();
    else Serial.println(F("L,err usage: test motors|solenoid|stepper|all|stop [speed]"));
    return true;
  }

  return false;  // not a known human command
}

static bool isProtocolLine(const char* line) {
  // Machine protocol lines are a single uppercase letter followed by
  // either end-of-string or a comma. Everything else is handled by the
  // human parser.
  char c = line[0];
  if (c < 'A' || c > 'Z') return false;
  return line[1] == 0 || line[1] == ',';
}

static void handleLine(char* line) {
  if (line[0] == 0) return;

  if (!isProtocolLine(line)) {
    if (!handleHumanLine(line)) {
      Serial.print(F("L,err unknown command: "));
      Serial.println(line);
      Serial.println(F("L,type 'help' for the menu"));
    }
    return;
  }

  switch (line[0]) {
    case 'M': {
      int vx = 0, vy = 0, wz = 0;
      if (sscanf(line + 1, ",%d,%d,%d", &vx, &vy, &wz) == 3) {
        cancelTestState();
        g_protocol_drive = (vx != 0 || vy != 0 || wz != 0);
        g_last_protocol_drive_ms = millis();
        handleMotorCmd(vx, vy, wz);
      }
      break;
    }
    case 'E': {
      int phase = 0;
      if (sscanf(line + 1, ",%d", &phase) == 1) {
        startExtinguisherPhase((uint8_t)phase);
      }
      break;
    }
    case 'W': {
      // No dedicated warning output is wired per the netlist. We acknowledge
      // the command and log it so the Pi-side countdown still works for the UI.
      int mode = 0;
      if (sscanf(line + 1, ",%d", &mode) == 1) {
        Serial.print(F("L,warn="));
        Serial.println(mode);
      }
      break;
    }
    case 'S':
      sendStatus();
      break;
    case 'R':
      cancelTestState();
      allOutputsOff();
      g_state = FW_ESTOP;
      Serial.println(F("L,estop"));
      break;
    case 'C':
      if (line[1] == ',') handleConfigure(line + 2);
      break;
    default:
      break;
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_rx_buf[g_rx_len] = 0;
      handleLine(g_rx_buf);
      g_rx_len = 0;
    } else if (g_rx_len < sizeof(g_rx_buf) - 1) {
      g_rx_buf[g_rx_len++] = c;
    } else {
      g_rx_len = 0;  // overflow, discard
    }
  }
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);

  const uint8_t outputs[] = {
    FP1, FP2, RP1, RP2,
    FA1, FB1, FA2, FB2,
    RA1, RB1, RA2, RB2,
    PIN_STEP, PIN_DIR, PIN_SOL, PIN_US_TRIG
  };
  for (uint8_t i = 0; i < sizeof(outputs); i++) {
    pinMode(outputs[i], OUTPUT);
    digitalWrite(outputs[i], LOW);
  }

  // One pin per ganged encoder net is our sampler; leave the other three
  // floating (INPUT, high-Z) so they cannot fight the shared signal.
  pinMode(ENC_A_PINS[0], INPUT_PULLUP);
  pinMode(ENC_B_PINS[0], INPUT_PULLUP);
  for (uint8_t i = 1; i < 4; i++) {
    pinMode(ENC_A_PINS[i], INPUT);
    pinMode(ENC_B_PINS[i], INPUT);
  }

  pinMode(PIN_US_ECHO, INPUT);
  pinMode(PIN_OUT_DIGITAL, INPUT);  // shared analog/digital sense line

  g_last_encA = digitalRead(ENC_A_PINS[0]);
  g_last_encB = digitalRead(ENC_B_PINS[0]);

  allOutputsOff();
  Serial.println(F("L,firebot_mega_ready"));
  Serial.println(F("L,type 'help' for the Serial Monitor command menu"));
}

void loop() {
  pollSerial();
  serviceGoSequence();
  serviceStepper();
  serviceExtinguisher();
  serviceEncoders();
  serviceTestState();
  serviceTestSeq();
  serviceDriveWatchdog();

  if (g_verbose) {
    uint32_t now = millis();
    if ((uint32_t)(now - g_last_verbose_ms) >= VERBOSE_PERIOD_MS) {
      g_last_verbose_ms = now;
      printHumanStatus();
    }
  }

  if (g_watch_mode != WATCH_OFF) {
    uint32_t now = millis();
    if ((uint32_t)(now - g_last_watch_ms) >= g_watch_period_ms) {
      g_last_watch_ms = now;
      printWatchLine();
    }
  }
}
