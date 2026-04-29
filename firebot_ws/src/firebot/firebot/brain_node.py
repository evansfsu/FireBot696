"""Brain node: 7-state FireBot696 supervisor.

States: IDLE, SEARCHING, AWAITING_CONFIRM, APPROACHING, WARNING,
        EXTINGUISHING, COMPLETE.

**Simple mission flow** (`simple_mission_flow: true`): IDLE stays put until
``/alarm/trigger`` or ``idle_exit_min_fire_sec`` of continuous fire (typ. 3 s
to reject brief noise). Centering uses ``simple_mission_center_band_frac``
(|x_offset| in [-band,+band], same idea as rotation_center_test ``center_band_frac``),
not the tight ``center_offset_thresh`` used in full-flow approach.
Accumulate ``simple_fire_confirm_sec`` (e.g. 12) of continuous fire to **arm**;
brief loss of fire uses ``lost_fire_grace_sec`` then **SEEK_SPIN** (same before
and after arm; fire-confirm timer resets while lost). Seek/track yaw is either
``simple_seek_mode=continuous`` or ``pulse`` (short bursts + min interval, like
``rotation_center_test_node``). After armed,
``center_hold_before_warning_sec`` of continuous centering (fire still visible
with the wide band above) → WARNING → ``warning_seconds`` → EXTINGUISHING →
COMPLETE → IDLE (skips AWAITING_CONFIRM and APPROACHING). The first time
IDLE → SEARCHING, ``corner_exit_forward_sec`` / ``corner_exit_speed`` still apply
(straight crawl before spin/center logic).

Full-flow SEARCHING begins with a short straight forward segment (corner exit).
Approach ends when the fire is centered and bbox area lies in
[approach_bbox_min_frac, approach_bbox_max_frac], and APPROACHING has lasted at
least approach_gate_min_sec — geared to slow indoor motion and ~1–3 FPS vision.

Drive output is `geometry_msgs/Twist` on `/cmd/drive` (skid-steer: linear.x and
angular.z; bridge forwards vy=0).

Extra approach conditions via `approach_strategy`:
  * yolo_only              (default)
  * yolo_ultrasonic        + HC-SR04 within approach_distance_cm
  * yolo_ultrasonic_ir     + KY-032 object detected
"""

from typing import Any, Dict, Iterable, Optional
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist

from firebot_interfaces.msg import FireDetection


class State:
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    AWAITING_CONFIRM = 'AWAITING_CONFIRM'
    APPROACHING = 'APPROACHING'
    WARNING = 'WARNING'
    EXTINGUISHING = 'EXTINGUISHING'
    COMPLETE = 'COMPLETE'


VALID_STRATEGIES = ('yolo_only', 'yolo_ultrasonic', 'yolo_ultrasonic_ir')


class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')

        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('stable_frames', 5)
        self.declare_parameter('center_offset_thresh', 0.08)
        self.declare_parameter('approach_bbox_min_frac', 0.10)
        self.declare_parameter('approach_bbox_max_frac', 0.30)
        self.declare_parameter('approach_gate_min_sec', 15.0)
        self.declare_parameter('corner_exit_forward_sec', 2.0)
        self.declare_parameter('corner_exit_speed', 30.0)
        self.declare_parameter('v_approach', 32.0)
        self.declare_parameter('kp_yaw', 50.0)
        self.declare_parameter('rotate_speed', 32.0)
        self.declare_parameter('search_timeout_sec', 30.0)
        self.declare_parameter('confirm_timeout_sec', 30.0)
        self.declare_parameter('warning_seconds', 10)
        self.declare_parameter('discharge_seconds', 6.0)
        # Extinguish E,2 advance + dwell + E,3 retract (match firebot_mega EXT_* ms).
        self.declare_parameter('ext_stepper_advance_sec', 5.3)
        self.declare_parameter('ext_stepper_dwell_sec', 10.0)
        self.declare_parameter('ext_stepper_retract_sec', 5.3)
        self.declare_parameter('complete_hold_sec', 3.0)

        self.declare_parameter('approach_strategy', 'yolo_only')
        self.declare_parameter('approach_distance_cm', 50)
        self.declare_parameter('safety_stop_cm', 20)
        self.declare_parameter('alarm_from_audio', False)
        self.declare_parameter('mic_alarm_threshold', 600)
        self.declare_parameter('mic_hold_ms', 500)
        # When true and /alarm/trigger latched: SEARCHING does not time out (publish false to clear latch).
        self.declare_parameter('alarm_inhibits_search_timeout', True)

        self.declare_parameter('approach_pulse_ms', 400)
        self.declare_parameter('approach_rest_ms', 600)
        self.declare_parameter('approach_max_sec', 60.0)

        # Skip confirm + approach: SEARCHING → WARNING once fire stays centered this long.
        self.declare_parameter('simple_mission_flow', False)
        self.declare_parameter('center_hold_before_warning_sec', 15.0)
        # Seconds of continuous fire in IDLE before entering SEARCHING (0 = any frame; reduces false exits).
        self.declare_parameter('idle_exit_min_fire_sec', 0.0)
        # Simple flow: continuous fire time before "armed" (lost fire before then → IDLE if no alarm).
        self.declare_parameter('simple_fire_confirm_sec', 12.0)
        # Simple flow: after armed, hold this long with no fire before starting seek spin.
        self.declare_parameter('lost_fire_grace_sec', 5.0)
        # Max time in SEARCHING for simple mission (0 = auto: max(search_timeout, confirm+center+120)).
        self.declare_parameter('simple_search_timeout_sec', 0.0)
        self.declare_parameter('simple_progress_log_period_sec', 1.0)
        # Simple mission: |x_offset| <= this counts as "centered" (same idea as rotation_center_test center_band_frac).
        self.declare_parameter('simple_mission_center_band_frac', 0.70)
        # simple_seek_mode: "continuous" (default) or "pulse" (rotation_center_test-style bursts).
        self.declare_parameter('simple_seek_mode', 'continuous')
        self.declare_parameter('simple_pulse_rotate_sec', 0.25)
        self.declare_parameter('simple_pulse_rest_sec', 0.35)
        self.declare_parameter('simple_pulse_min_interval_sec', 3.0)
        self.declare_parameter('simple_pulse_seek_alternate', True)
        self.declare_parameter('simple_pulse_seek_sign', 1.0)

        self._load_parameters_from_declarations()

        self.drive_pub = self.create_publisher(Twist, '/cmd/drive', 10)
        self.ext_pub = self.create_publisher(Int32, '/cmd/extinguisher', 10)
        self.warn_pub = self.create_publisher(Int32, '/cmd/warning', 10)
        self.state_pub = self.create_publisher(String, '/firebot/state', 10)
        self.countdown_pub = self.create_publisher(Int32, '/firebot/countdown', 10)

        self.create_subscription(FireDetection, '/fire/detection', self._on_detection, 10)
        self.create_subscription(Bool, '/alarm/trigger', self._on_alarm, 10)
        self.create_subscription(Bool, '/ui/confirm', self._on_confirm, 10)
        self.create_subscription(String, '/ui/state_override', self._on_state_override, 10)
        self.create_subscription(Int32, '/sensors/ultrasonic', self._on_ultrasonic, 10)
        self.create_subscription(Bool, '/sensors/ir', self._on_ir, 10)
        self.create_subscription(Int32, '/sensors/audio', self._on_audio, 10)

        self.state = State.IDLE
        self.state_enter = self.get_clock().now()
        self.latest_det = FireDetection()
        self.centered_streak = 0
        self.alarm_latched = False
        self.confirm_latched = None  # True = confirm, False = deny, None = no input
        self.us_cm = -1
        self.ir_triggered = False
        self.mic_level = -1
        self.mic_over_since_ns = None
        self.last_countdown = -1
        self.approach_pulse_edge_ns = 0
        self.approach_pulse_driving = False
        self.corner_exit_done = True
        self._centered_hold_sec = 0.0
        self._idle_fire_accum = 0.0
        self._simple_mission_armed = False
        self._fire_confirm_accum = 0.0
        self._lost_since = None
        self._last_brain_tick = self.get_clock().now()
        self._last_simple_log = self.get_clock().now()
        self._simple_p_phase = 'IDLE'
        self._simple_p_enter = self.get_clock().now()
        self._simple_p_last_start = None
        self._simple_p_cmd = 0.0
        self._simple_p_seek_sign = self.simple_pulse_seek_sign

        self.add_on_set_parameters_callback(self._on_parameters_set)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'brain_node up (strategy={self.approach_strategy}, '
            f'alarm_from_audio={self.alarm_from_audio}, '
            f'simple_mission_flow={self.simple_mission_flow}, '
            f'center_hold_before_warning={self.center_hold_before_warning:.1f}s, '
            f'idle_exit_min_fire={self.idle_exit_min_fire:.2f}s, '
            f'simple_fire_confirm={self.simple_fire_confirm_sec:.1f}s, '
            f'simple_center_band=|x_off|<={self.simple_mission_center_band:.2f}, '
            f'simple_seek_mode={self.simple_seek_mode}, '
            f'search_timeout={self.search_timeout:.0f}s '
            f'simple_search_cap={self._effective_search_timeout_sec():.0f}s '
            f'alarm_inhibits_search_timeout={self.alarm_inhibits_search_timeout})'
        )

    def _load_parameters_from_declarations(
        self, incoming_params: Optional[Iterable] = None
    ) -> None:
        """Refresh cached fields from declared parameters.

        When ``incoming_params`` is set (from ``add_on_set_parameters_callback``), those
        values are used in place of the not-yet-updated parameter server snapshot.
        """
        incoming: Optional[Dict[str, Any]] = None
        if incoming_params is not None:
            incoming = {p.name: p.value for p in incoming_params}

        def pv(name: str):
            if incoming is not None and name in incoming:
                return incoming[name]
            return self.get_parameter(name).value

        self.conf_thresh = float(pv('confidence_threshold'))
        self.stable_frames = int(pv('stable_frames'))
        self.center_tol = float(pv('center_offset_thresh'))
        self.bbox_area_min = float(pv('approach_bbox_min_frac'))
        self.bbox_area_max = float(pv('approach_bbox_max_frac'))
        self.approach_gate_min_sec = float(pv('approach_gate_min_sec'))
        self.corner_exit_forward_sec = float(pv('corner_exit_forward_sec'))
        self.corner_exit_speed = float(pv('corner_exit_speed'))
        self.v_approach = float(pv('v_approach'))
        self.kp_yaw = float(pv('kp_yaw'))
        self.rot_speed = float(pv('rotate_speed'))
        self.search_timeout = float(pv('search_timeout_sec'))
        self.confirm_timeout = float(pv('confirm_timeout_sec'))
        self.warning_secs = float(pv('warning_seconds'))
        self.warning_secs = max(0.0, self.warning_secs)
        self.discharge_secs = float(pv('discharge_seconds'))
        self.ext_stepper_advance = float(pv('ext_stepper_advance_sec'))
        self.ext_stepper_advance = max(0.5, min(120.0, self.ext_stepper_advance))
        self.ext_stepper_dwell = float(pv('ext_stepper_dwell_sec'))
        self.ext_stepper_dwell = max(0.0, min(120.0, self.ext_stepper_dwell))
        self.ext_stepper_retract = float(pv('ext_stepper_retract_sec'))
        self.ext_stepper_retract = max(0.5, min(120.0, self.ext_stepper_retract))
        self.complete_hold = float(pv('complete_hold_sec'))

        strat = str(pv('approach_strategy'))
        if strat not in VALID_STRATEGIES:
            self.get_logger().warn(
                f'unknown approach_strategy {strat!r}; falling back to yolo_only'
            )
            strat = 'yolo_only'
        self.approach_strategy = strat
        self.approach_dist_cm = int(pv('approach_distance_cm'))
        self.safety_stop_cm = int(pv('safety_stop_cm'))
        self.alarm_from_audio = bool(pv('alarm_from_audio'))
        self.mic_alarm_thresh = int(pv('mic_alarm_threshold'))
        self.mic_hold_ms = int(pv('mic_hold_ms'))
        self.alarm_inhibits_search_timeout = bool(pv('alarm_inhibits_search_timeout'))

        self.approach_pulse_ms = int(pv('approach_pulse_ms'))
        self.approach_rest_ms = int(pv('approach_rest_ms'))
        self.approach_max_sec = float(pv('approach_max_sec'))

        self.simple_mission_flow = bool(pv('simple_mission_flow'))
        self.center_hold_before_warning = float(pv('center_hold_before_warning_sec'))
        self.center_hold_before_warning = max(0.0, self.center_hold_before_warning)
        self.idle_exit_min_fire = float(pv('idle_exit_min_fire_sec'))
        self.idle_exit_min_fire = max(0.0, self.idle_exit_min_fire)
        self.simple_fire_confirm_sec = float(pv('simple_fire_confirm_sec'))
        self.simple_fire_confirm_sec = max(0.0, self.simple_fire_confirm_sec)
        self.lost_fire_grace_sec = float(pv('lost_fire_grace_sec'))
        self.lost_fire_grace_sec = max(0.0, self.lost_fire_grace_sec)
        self.simple_search_timeout_override = float(pv('simple_search_timeout_sec'))
        self._simple_log_period = float(pv('simple_progress_log_period_sec'))
        self._simple_log_period = max(0.2, min(10.0, self._simple_log_period))
        self.simple_mission_center_band = float(pv('simple_mission_center_band_frac'))
        self.simple_mission_center_band = max(0.02, min(1.0, self.simple_mission_center_band))
        mode = str(pv('simple_seek_mode')).strip().lower()
        if mode not in ('continuous', 'pulse'):
            mode = 'continuous'
        self.simple_seek_mode = mode
        self.simple_pulse_rotate = float(pv('simple_pulse_rotate_sec'))
        self.simple_pulse_rotate = max(0.05, min(2.0, self.simple_pulse_rotate))
        self.simple_pulse_rest = float(pv('simple_pulse_rest_sec'))
        self.simple_pulse_rest = max(0.0, min(2.0, self.simple_pulse_rest))
        self.simple_pulse_min_interval = float(pv('simple_pulse_min_interval_sec'))
        self.simple_pulse_min_interval = max(0.0, min(60.0, self.simple_pulse_min_interval))
        self.simple_pulse_seek_alternate = bool(pv('simple_pulse_seek_alternate'))
        self.simple_pulse_seek_sign = float(pv('simple_pulse_seek_sign'))
        self.simple_pulse_seek_sign = 1.0 if self.simple_pulse_seek_sign >= 0 else -1.0

    def _on_parameters_set(self, params):
        bad = []
        for p in params:
            if p.name == 'approach_strategy':
                s = str(p.value).strip()
                if s not in VALID_STRATEGIES:
                    bad.append(f'approach_strategy must be one of {VALID_STRATEGIES}')
            elif p.name == 'simple_seek_mode':
                m = str(p.value).strip().lower()
                if m not in ('continuous', 'pulse'):
                    bad.append('simple_seek_mode must be continuous or pulse')
        if bad:
            return SetParametersResult(successful=False, reason='; '.join(bad))

        old_mode = self.simple_seek_mode
        old_seek_sign = self.simple_pulse_seek_sign
        self._load_parameters_from_declarations(incoming_params=params)
        if old_mode != self.simple_seek_mode:
            self._reset_simple_pulse_fsm(reset_seek_sign=False)
            self.get_logger().info(f'runtime params: simple_seek_mode -> {self.simple_seek_mode!r}')
        elif old_seek_sign != self.simple_pulse_seek_sign:
            self._reset_simple_pulse_fsm(reset_seek_sign=True)
        else:
            self.get_logger().info(
                'runtime params updated: '
                + ', '.join(f'{p.name}={p.value}' for p in params)
            )
        return SetParametersResult(successful=True)

    def _effective_search_timeout_sec(self) -> float:
        """SEARCHING time limit: avoid simple-mission abort while confirming + centering + reacquire."""
        if self.alarm_inhibits_search_timeout and self.alarm_latched:
            return float('inf')
        if not self.simple_mission_flow:
            return self.search_timeout
        if self.simple_search_timeout_override > 0.0:
            return self.simple_search_timeout_override
        auto_floor = (
            self.simple_fire_confirm_sec
            + self.center_hold_before_warning
            + 3.0 * self.lost_fire_grace_sec
            + 120.0
        )
        return max(self.search_timeout, auto_floor)

    def _log_simple_progress(self, line: str) -> None:
        now = self.get_clock().now()
        if (now - self._last_simple_log).nanoseconds / 1e9 < self._simple_log_period:
            return
        self._last_simple_log = now
        self.get_logger().info(line)

    def _on_detection(self, msg: FireDetection):
        self.latest_det = msg

    def _on_alarm(self, msg: Bool):
        if msg.data:
            self.alarm_latched = True
            self.get_logger().warn('alarm latched')
        else:
            if self.alarm_latched:
                self.get_logger().info('alarm latch cleared')
            self.alarm_latched = False

    def _on_confirm(self, msg: Bool):
        self.confirm_latched = bool(msg.data)

    def _on_state_override(self, msg: String):
        target = msg.data.strip().upper()
        if target in vars(State).values():
            self.get_logger().warn(f'state override -> {target}')
            self._set_state(target)

    def _on_ultrasonic(self, msg: Int32):
        self.us_cm = int(msg.data)

    def _on_ir(self, msg: Bool):
        self.ir_triggered = bool(msg.data)

    def _on_audio(self, msg: Int32):
        self.mic_level = int(msg.data)
        if not self.alarm_from_audio:
            self.mic_over_since_ns = None
            return
        now = self.get_clock().now().nanoseconds
        if self.mic_level >= self.mic_alarm_thresh:
            if self.mic_over_since_ns is None:
                self.mic_over_since_ns = now
            elif (now - self.mic_over_since_ns) / 1e6 >= self.mic_hold_ms:
                if not self.alarm_latched:
                    self.alarm_latched = True
                    self.get_logger().warn('alarm auto-latched from audio envelope')
        else:
            self.mic_over_since_ns = None

    def _set_state(self, new_state: str):
        if new_state == self.state:
            return
        old_state = self.state
        self.get_logger().info(f'state: {self.state} -> {new_state}')
        self.state = new_state
        self.state_enter = self.get_clock().now()
        self.centered_streak = 0
        self.last_countdown = -1
        self.approach_pulse_edge_ns = 0
        self.approach_pulse_driving = False
        if new_state == State.SEARCHING and old_state == State.IDLE:
            self.corner_exit_done = False
            self._simple_mission_armed = False
            self._fire_confirm_accum = 0.0
            self._lost_since = None
            self._centered_hold_sec = 0.0
            self._reset_simple_pulse_fsm(reset_seek_sign=True)
        if new_state == State.IDLE:
            self._reset_transient()
            self._centered_hold_sec = 0.0
            self._idle_fire_accum = 0.0
            self._simple_mission_armed = False
            self._fire_confirm_accum = 0.0
            self._lost_since = None
        if new_state == State.SEARCHING and old_state != State.IDLE:
            self._centered_hold_sec = 0.0

    def _reset_simple_pulse_fsm(self, reset_seek_sign: bool = False) -> None:
        self._simple_p_phase = 'IDLE'
        self._simple_p_enter = self.get_clock().now()
        self._simple_p_last_start = None
        self._simple_p_cmd = 0.0
        if reset_seek_sign:
            self._simple_p_seek_sign = self.simple_pulse_seek_sign

    def _simple_output_rotation(self, wz_cmd: float, alternate_seek_burst: bool) -> None:
        """Seek/track yaw: continuous command or pulsed bursts (rotation_center_test style)."""
        if self.simple_seek_mode != 'pulse':
            if abs(wz_cmd) < 1e-6:
                self._stop()
            else:
                self._drive(angular_z=wz_cmd)
            return

        now = self.get_clock().now()
        if abs(wz_cmd) < 1e-6:
            self._stop()
            self._simple_p_phase = 'IDLE'
            return

        if self._simple_p_phase != 'IDLE' and (self._simple_p_cmd > 0) != (wz_cmd > 0):
            self._simple_p_phase = 'IDLE'
            self._stop()

        if self._simple_p_phase == 'IDLE':
            if self._simple_p_last_start is not None:
                gap = (now - self._simple_p_last_start).nanoseconds / 1e9
                if gap < self.simple_pulse_min_interval:
                    self._stop()
                    return
            self._simple_p_phase = 'DRIVE'
            self._simple_p_enter = now
            self._simple_p_cmd = wz_cmd
            self._simple_p_last_start = now
            self._drive(angular_z=wz_cmd)
            if alternate_seek_burst and self.simple_pulse_seek_alternate:
                self._simple_p_seek_sign *= -1.0
            return

        elapsed = (now - self._simple_p_enter).nanoseconds / 1e9
        if self._simple_p_phase == 'DRIVE':
            if elapsed >= self.simple_pulse_rotate:
                self._stop()
                self._simple_p_phase = 'REST'
                self._simple_p_enter = now
            else:
                self._drive(angular_z=self._simple_p_cmd)
            return

        if self._simple_p_phase == 'REST':
            if elapsed >= self.simple_pulse_rest:
                self._simple_p_phase = 'IDLE'
                self._simple_p_enter = now
            self._stop()

    def _reset_transient(self):
        self.alarm_latched = False
        self.confirm_latched = None
        self.centered_streak = 0

    def _time_in_state(self) -> float:
        return (self.get_clock().now() - self.state_enter).nanoseconds / 1e9

    def _drive(self, linear_x: float = 0.0, angular_z: float = 0.0):
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self.drive_pub.publish(t)

    def _stop(self):
        self._drive(0.0, 0.0)

    def _send_ext(self, phase: int):
        msg = Int32()
        msg.data = int(phase)
        self.ext_pub.publish(msg)

    def _send_warn(self, mode: int):
        msg = Int32()
        msg.data = int(mode)
        self.warn_pub.publish(msg)

    def _detection_is_fire(self) -> bool:
        det = self.latest_det
        return bool(det.detected) and float(det.confidence) >= self.conf_thresh

    def _centered_enough(self) -> bool:
        return abs(float(self.latest_det.x_offset)) <= self.center_tol

    def _simple_mission_centered(self) -> bool:
        """Wide FOV band for simple mission (matches rotation_center_test semantics)."""
        return abs(float(self.latest_det.x_offset)) <= self.simple_mission_center_band

    def _approach_gate_satisfied(self) -> bool:
        det = self.latest_det
        if not self._detection_is_fire():
            return False
        if not self._centered_enough():
            return False
        area = float(det.bbox_area)
        if area < self.bbox_area_min or area > self.bbox_area_max:
            return False
        if self._time_in_state() < self.approach_gate_min_sec:
            return False
        if self.approach_strategy == 'yolo_only':
            return True
        if self.us_cm < 0:
            return False  # sensor required but reported disabled
        if self.us_cm > self.approach_dist_cm:
            return False
        if self.approach_strategy == 'yolo_ultrasonic':
            return True
        return bool(self.ir_triggered)

    def _tick(self):
        now = self.get_clock().now()
        dt_raw = (now - self._last_brain_tick).nanoseconds / 1e9
        self._last_brain_tick = now
        self._brain_dt = max(0.0, min(0.5, dt_raw))
        if self._brain_dt < 1e-6:
            self._brain_dt = 0.1

        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        handler = {
            State.IDLE: self._tick_idle,
            State.SEARCHING: self._tick_searching,
            State.AWAITING_CONFIRM: self._tick_await_confirm,
            State.APPROACHING: self._tick_approaching,
            State.WARNING: self._tick_warning,
            State.EXTINGUISHING: self._tick_extinguishing,
            State.COMPLETE: self._tick_complete,
        }.get(self.state)
        if handler:
            handler()

    def _tick_idle(self):
        self._stop()
        if self.alarm_latched:
            self._set_state(State.SEARCHING)
            return
        if self._detection_is_fire():
            self._idle_fire_accum += self._brain_dt
        else:
            self._idle_fire_accum = 0.0
        if self.idle_exit_min_fire <= 0.0:
            if self._detection_is_fire():
                self._set_state(State.SEARCHING)
        elif self._idle_fire_accum >= self.idle_exit_min_fire:
            self._set_state(State.SEARCHING)

    def _tick_searching(self):
        limit = self._effective_search_timeout_sec()
        if self._time_in_state() > limit:
            self.get_logger().info(f'search timeout ({limit:.0f}s); back to IDLE')
            self._stop()
            self._reset_transient()
            self._set_state(State.IDLE)
            return

        if not self.corner_exit_done:
            if self._time_in_state() < self.corner_exit_forward_sec:
                self.centered_streak = 0
                self._centered_hold_sec = 0.0
                self._drive(linear_x=self.corner_exit_speed, angular_z=0.0)
                if self.simple_mission_flow:
                    self._log_simple_progress(
                        f'simple: CORNER_EXIT t={self._time_in_state():.1f}/'
                        f'{self.corner_exit_forward_sec:.1f}s vx={self.corner_exit_speed:.0f}'
                    )
                return
            self.corner_exit_done = True

        if self.simple_mission_flow:
            self._tick_searching_simple()
            return

        det = self.latest_det
        if not self._detection_is_fire():
            self.centered_streak = 0
            self._drive(angular_z=self.rot_speed)
            return

        offset = float(det.x_offset)
        if abs(offset) <= self.center_tol:
            self.centered_streak += 1
            self._stop()
            if self.centered_streak >= self.stable_frames:
                self._set_state(State.AWAITING_CONFIRM)
            return

        self.centered_streak = 0
        direction = -1.0 if offset > 0 else 1.0
        self._drive(angular_z=direction * self.rot_speed)

    def _tick_searching_simple(self):
        """SEARCHING: fire confirm → arm; centered dwell; lost-fire grace then seek."""
        now = self.get_clock().now()

        if not self._detection_is_fire():
            self._centered_hold_sec = 0.0
            if not self._simple_mission_armed:
                self._fire_confirm_accum = 0.0
            if self._lost_since is None:
                self._lost_since = now
            lost_elapsed = (now - self._lost_since).nanoseconds / 1e9
            if lost_elapsed < self.lost_fire_grace_sec:
                self._reset_simple_pulse_fsm(reset_seek_sign=False)
                self._stop()
                self._log_simple_progress(
                    f'simple: LOST_FIRE_GRACE {lost_elapsed:.1f}/{self.lost_fire_grace_sec:.1f}s '
                    f'armed={self._simple_mission_armed} seek={self.simple_seek_mode} '
                    f'(confirm resets until reacquire)'
                )
                return
            if self.simple_seek_mode == 'pulse':
                wz = self._simple_p_seek_sign * self.rot_speed
                self._simple_output_rotation(wz, True)
            else:
                self._simple_output_rotation(self.rot_speed, False)
            self._log_simple_progress(
                f'simple: SEEK_SPIN seek={self.simple_seek_mode} '
                f'armed={self._simple_mission_armed} '
                f'fire_confirm={self._fire_confirm_accum:.1f}/{self.simple_fire_confirm_sec:.1f}s'
            )
            return

        had_lost = self._lost_since is not None
        self._lost_since = None
        if had_lost:
            self._reset_simple_pulse_fsm(reset_seek_sign=False)

        if not self._simple_mission_armed:
            self._fire_confirm_accum += self._brain_dt
            if self._fire_confirm_accum >= self.simple_fire_confirm_sec:
                self._simple_mission_armed = True
                self.get_logger().info(
                    f'simple mission ARMED (fire confirm {self.simple_fire_confirm_sec:.1f}s)'
                )

        if not self._simple_mission_centered():
            self._centered_hold_sec = 0.0
            offset = float(self.latest_det.x_offset)
            direction = -1.0 if offset > 0 else 1.0
            side = 'left' if direction > 0 else 'right'
            wz = direction * self.rot_speed
            self._simple_output_rotation(wz, False)
            self._log_simple_progress(
                f'simple: TRACK_FIRE seek={self.simple_seek_mode} x_off={offset:+.3f} '
                f'rotate_{side} wz_cmd={wz:.0f} '
                f'armed={self._simple_mission_armed} |x|≤{self.simple_mission_center_band:.2f} '
                f'confirm={self._fire_confirm_accum:.1f}/{self.simple_fire_confirm_sec:.1f}s'
            )
            return

        self._simple_output_rotation(0.0, False)
        if not self._simple_mission_armed:
            self._log_simple_progress(
                f'simple: CENTERED_HOLD seek={self.simple_seek_mode} '
                f'confirm={self._fire_confirm_accum:.1f}/'
                f'{self.simple_fire_confirm_sec:.1f}s (|x|≤{self.simple_mission_center_band:.2f})'
            )
            return
        self._centered_hold_sec += self._brain_dt
        self._log_simple_progress(
            f'simple: DWELL_BEFORE_WARN seek={self.simple_seek_mode} '
            f'center_hold={self._centered_hold_sec:.1f}/'
            f'{self.center_hold_before_warning:.1f}s armed=yes'
        )
        if self._centered_hold_sec >= self.center_hold_before_warning:
            self.get_logger().info(
                f'center hold {self._centered_hold_sec:.1f}s '
                f'>= {self.center_hold_before_warning:.1f}s -> WARNING'
            )
            self._set_state(State.WARNING)

    def _tick_await_confirm(self):
        self._stop()
        if self.confirm_latched is True:
            self.confirm_latched = None
            self._set_state(State.APPROACHING)
            return
        if self.confirm_latched is False:
            self.get_logger().warn('user denied; back to IDLE')
            self._set_state(State.IDLE)
            return
        if self._time_in_state() > self.confirm_timeout:
            self.get_logger().info('confirm timeout; back to IDLE')
            self._set_state(State.IDLE)

    def _tick_approaching(self):
        # Bail out on lost detection.
        if not self._detection_is_fire():
            self.get_logger().info('detection lost in APPROACHING; back to SEARCHING')
            self._stop()
            self._set_state(State.SEARCHING)
            return

        # Hard timeout so we never drive indefinitely. This protects the robot
        # in indoor spaces if the bbox-area gate never quite triggers (e.g.
        # dim target, odd angle, partial occlusion).
        if self._time_in_state() > self.approach_max_sec:
            self.get_logger().warn(
                f'approach_max_sec ({self.approach_max_sec:.0f}s) exceeded; '
                'stopping and proceeding to WARNING'
            )
            self._stop()
            self._set_state(State.WARNING)
            return

        # Ultrasonic-based wall guard (only when ultrasonic is in the strategy).
        if self.approach_strategy in ('yolo_ultrasonic', 'yolo_ultrasonic_ir'):
            if 0 <= self.us_cm <= self.safety_stop_cm:
                self.get_logger().warn(
                    f'safety stop ({self.us_cm} cm <= {self.safety_stop_cm}); '
                    'holding position'
                )
                self._stop()
                if self._approach_gate_satisfied():
                    self._set_state(State.WARNING)
                return

        # Gate reached -> transition to WARNING.
        if self._approach_gate_satisfied():
            self._stop()
            self._set_state(State.WARNING)
            return

        # Pulsed short-burst drive for indoor safety. Drive forward for
        # approach_pulse_ms, coast for approach_rest_ms, repeat. Yaw correction
        # piggybacks on the drive pulse so the robot still re-centers while
        # making progress.
        now_ns = self.get_clock().now().nanoseconds
        if self.approach_pulse_edge_ns == 0:
            self.approach_pulse_edge_ns = now_ns
            self.approach_pulse_driving = True
            self.get_logger().info('approach pulse: drive')
        since_edge_ms = (now_ns - self.approach_pulse_edge_ns) / 1e6

        if self.approach_pulse_driving and since_edge_ms >= self.approach_pulse_ms:
            self.approach_pulse_driving = False
            self.approach_pulse_edge_ns = now_ns
            self._stop()
            self.get_logger().debug('approach pulse: rest')
            return
        if (not self.approach_pulse_driving) and since_edge_ms >= self.approach_rest_ms:
            self.approach_pulse_driving = True
            self.approach_pulse_edge_ns = now_ns
            self.get_logger().debug('approach pulse: drive')

        if self.approach_pulse_driving:
            offset = float(self.latest_det.x_offset)
            angular_z = -self.kp_yaw * offset
            self._drive(linear_x=self.v_approach, angular_z=angular_z)
        else:
            self._stop()

    def _tick_warning(self):
        self._stop()
        self._send_warn(1)
        elapsed = self._time_in_state()
        rem_int = max(0, int(math.ceil(self.warning_secs - elapsed - 1e-9)))
        if rem_int != self.last_countdown:
            self.last_countdown = rem_int
            msg = Int32()
            msg.data = rem_int
            self.countdown_pub.publish(msg)
            self.get_logger().warn(f'warning countdown: {rem_int}s')
        if elapsed >= self.warning_secs:
            self._send_warn(0)
            self._set_state(State.EXTINGUISHING)

    def _tick_extinguishing(self):
        elapsed = self._time_in_state()
        # Match firmware: E,1 ~EXT_PIN_PULL_MS, E,2 advance+dwell then auto E,3 retract.
        pin_pull_dur = 1.0
        t_after_pin = pin_pull_dur + self.ext_stepper_advance + self.ext_stepper_dwell
        t_after_retract = t_after_pin + self.ext_stepper_retract
        if elapsed < pin_pull_dur:
            self._send_ext(1)
        elif elapsed < t_after_pin:
            self._send_ext(2)
        elif elapsed < t_after_retract:
            self._send_ext(3)
        else:
            self._send_ext(4)
            self._set_state(State.COMPLETE)

    def _tick_complete(self):
        self._stop()
        self._send_ext(0)
        self._send_warn(0)
        if self._time_in_state() >= self.complete_hold:
            self._set_state(State.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
