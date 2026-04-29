"""Test node: wait for stable fire detection, then center it with short in-place rotation bursts.

Subscribes: /fire/detection (FireDetection)
Publishes: /cmd/drive (geometry_msgs/Twist) — **do not run with brain_node** on the same topic.

Behavior
--------
1. WAIT_FIRE — Stop. Accumulate ``fire_continuous_sec`` while ``detected`` and confidence are OK;
   reset to 0 on any missed frame. Log ~1 Hz: ``waiting_for_align: fire_continuous= X / Y s`` (default Y=12 s).
2. ALIGN — After ``fire_confirm_sec`` (default 12 s), pulse-rotate: ``pulse_rotate_sec`` drive,
   ``pulse_rest_sec`` coast, at most one new burst every ``pulse_min_interval_sec`` (default 5 s).
   If the flame drops out, the robot holds still for ``lost_fire_grace_sec`` (default 5 s) before
   SEEK rotation; if detection returns earlier, SEEK never starts for that dropout.
   Only ``angular.z``; sign matches brain SEARCHING. SEEK uses ``seek_rotation_sign`` / ``seek_alternate``.

**Center band:** ``center_band_frac`` (default **0.75**) = treat the middle fraction of the frame width
as "centered": ``|x_offset| <= center_band_frac`` (for 0.75, only the outer ~12.5% each side nudges).

Tune times in ``firebot_params.yaml`` under ``rotation_center_test: ros__parameters``.

x_offset is -1..+1 (negative = frame left). This matches ``vision_node`` and ``brain_node``.

Runs without ROS on host if detection + bridge come from Docker (UDP/picam).

Examples::

  # Minimal stack: bridge + this node (no brain). Provide /fire/detection elsewhere.
  ros2 launch firebot rotation_center_test.launch.py

  ros2 run firebot rotation_center_test_node --ros-args \\
    -p center_band_frac:=0.75 -p pulse_rotate_sec:=0.25 -p pulse_min_interval_sec:=5.0
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from firebot_interfaces.msg import FireDetection


class Mode:
    WAIT_FIRE = 'WAIT_FIRE'
    ALIGN = 'ALIGN'


class PulsePhase:
    IDLE = 'IDLE'  # plan next pulse
    DRIVE = 'DRIVE'
    REST = 'REST'


class RotationCenterTestNode(Node):
    def __init__(self):
        super().__init__('rotation_center_test')

        self.declare_parameter('fire_confirm_sec', 12.0)
        self.declare_parameter('pulse_rotate_sec', 0.25)
        self.declare_parameter('pulse_rest_sec', 0.35)
        # Minimum time between *starts* of rotation bursts (limits jerk; e.g. one burst per 5 s).
        self.declare_parameter('pulse_min_interval_sec', 5.0)
        self.declare_parameter('rotate_speed', 32.0)
        # Middle fraction of frame width treated as "centered" |x_offset| <= this (0.75 ≈ middle 75%).
        self.declare_parameter('center_band_frac', 0.75)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('seek_rotation_sign', 1.0)
        self.declare_parameter('seek_alternate', True)
        # In ALIGN: seconds with no fire before SEEK rotation is allowed (hold still while waiting for re-detect).
        self.declare_parameter('lost_fire_grace_sec', 5.0)
        self.declare_parameter('log_flip_left_right', False)
        self.declare_parameter('wait_log_period_sec', 1.0)
        self.declare_parameter('centered_log_period_sec', 2.0)

        self._fire_confirm = float(self.get_parameter('fire_confirm_sec').value)
        self._pulse_rotate = float(self.get_parameter('pulse_rotate_sec').value)
        self._pulse_rest = float(self.get_parameter('pulse_rest_sec').value)
        self._pulse_min_interval = float(self.get_parameter('pulse_min_interval_sec').value)
        self._rot_speed = float(self.get_parameter('rotate_speed').value)
        self._center_band = float(self.get_parameter('center_band_frac').value)
        self._center_band = max(0.02, min(1.0, self._center_band))
        self._conf_thresh = float(self.get_parameter('confidence_threshold').value)
        self._seek_sign = float(self.get_parameter('seek_rotation_sign').value)
        self._seek_sign = 1.0 if self._seek_sign >= 0 else -1.0
        self._seek_alternate = bool(self.get_parameter('seek_alternate').value)
        self._lost_fire_grace = float(self.get_parameter('lost_fire_grace_sec').value)
        self._lost_fire_grace = max(0.0, self._lost_fire_grace)
        self._log_flip = bool(self.get_parameter('log_flip_left_right').value)
        self._wait_log_period = float(self.get_parameter('wait_log_period_sec').value)
        self._centered_log_period = float(self.get_parameter('centered_log_period_sec').value)

        self._pub = self.create_publisher(Twist, '/cmd/drive', 10)
        self.create_subscription(FireDetection, '/fire/detection', self._on_det, 10)

        self._latest = FireDetection()
        self._mode = Mode.WAIT_FIRE
        self._pulse = PulsePhase.IDLE
        self._phase_enter = self.get_clock().now()
        self._pulse_angular_z = 0.0
        self._fire_continuous_sec = 0.0
        self._last_tick = self.get_clock().now()
        self._last_wait_log = self.get_clock().now()
        self._last_centered_log = self.get_clock().now()
        self._current_seek_sign = self._seek_sign
        self._last_drive_start = None  # rclpy.time.Time; None = no burst yet in ALIGN
        self._lost_since = None  # rclpy.time.Time when fire was last lost in ALIGN; None = currently have fire timing ok
        self._last_lost_grace_log = self.get_clock().now()

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            'rotation_center_test_node up — /cmd/drive; '
            f'center_band_frac={self._center_band:.2f} pulse_rotate={self._pulse_rotate:.2f}s '
            f'pulse_rest={self._pulse_rest:.2f}s min_interval={self._pulse_min_interval:.2f}s '
            f'lost_fire_grace={self._lost_fire_grace:.2f}s '
            '(do not run brain_node concurrently)'
        )

    def _on_det(self, msg: FireDetection) -> None:
        self._latest = msg

    def _is_fire(self) -> bool:
        d = self._latest
        return bool(d.detected) and float(d.confidence) >= self._conf_thresh

    def _centered(self) -> bool:
        return abs(float(self._latest.x_offset)) <= self._center_band

    def _drive(self, angular_z: float) -> None:
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = float(angular_z)
        self._pub.publish(t)

    def _stop(self) -> None:
        self._drive(0.0)

    def _dir_label(self, angular_z: float) -> str:
        z = -angular_z if self._log_flip else angular_z
        if z > 1e-6:
            return 'right'
        if z < -1e-6:
            return 'left'
        return 'none'

    def _center_angular_sign(self) -> float:
        """Same convention as brain_node._tick_searching."""
        offset = float(self._latest.x_offset)
        direction = -1.0 if offset > 0 else 1.0
        return direction

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds / 1e9
        self._last_tick = now

        if self._is_fire():
            self._fire_continuous_sec += dt
        else:
            self._fire_continuous_sec = 0.0

        if self._mode == Mode.WAIT_FIRE:
            self._stop()
            self._pulse = PulsePhase.IDLE
            if (now - self._last_wait_log).nanoseconds / 1e9 >= self._wait_log_period:
                self._last_wait_log = now
                self.get_logger().info(
                    f'action=WAIT_FIRE waiting_for_align fire_continuous='
                    f'{self._fire_continuous_sec:.2f}/{self._fire_confirm:.1f}s '
                    f'detected={self._latest.detected} conf={self._latest.confidence:.2f}'
                )
            if self._fire_continuous_sec >= self._fire_confirm:
                self.get_logger().info(
                    f'action=ARM_ALIGN fire_continuous={self._fire_continuous_sec:.2f}s '
                    f'>= {self._fire_confirm:.1f}s — starting pulse centering'
                )
                self._mode = Mode.ALIGN
                self._pulse = PulsePhase.IDLE
                self._phase_enter = now
                self._last_drive_start = None
                self._lost_since = None
            return

        # ALIGN
        assert self._mode == Mode.ALIGN

        if self._is_fire() and self._centered():
            self._lost_since = None
            self._stop()
            self._pulse = PulsePhase.IDLE
            if (now - self._last_centered_log).nanoseconds / 1e9 >= self._centered_log_period:
                self._last_centered_log = now
                self.get_logger().info(
                    f'action=CENTERED in_frame=yes x_offset={self._latest.x_offset:.3f} '
                    f'(band |off|<={self._center_band:.2f}) '
                    f'fire_continuous={self._fire_continuous_sec:.2f}s'
                )
            return

        if self._is_fire():
            self._lost_since = None
        else:
            if self._lost_since is None:
                self._lost_since = now

        if not self._is_fire() and self._lost_since is not None:
            lost_elapsed = (now - self._lost_since).nanoseconds / 1e9
            if lost_elapsed < self._lost_fire_grace:
                self._stop()
                self._pulse = PulsePhase.IDLE
                if (now - self._last_lost_grace_log).nanoseconds / 1e9 >= self._wait_log_period:
                    self._last_lost_grace_log = now
                    self.get_logger().info(
                        f'action=LOST_FIRE_GRACE hold_seconds={lost_elapsed:.2f}/'
                        f'{self._lost_fire_grace:.1f}s (no SEEK until grace elapses)'
                    )
                return

        elapsed = (now - self._phase_enter).nanoseconds / 1e9

        if self._pulse == PulsePhase.DRIVE:
            if elapsed >= self._pulse_rotate:
                self._stop()
                self._pulse = PulsePhase.REST
                self._phase_enter = now
            else:
                self._drive(self._pulse_angular_z)
            return

        if self._pulse == PulsePhase.REST:
            if elapsed >= self._pulse_rest:
                self._pulse = PulsePhase.IDLE
                self._phase_enter = now
            else:
                self._stop()
            return

        # IDLE — plan next pulse (rate-limited)
        if self._last_drive_start is not None:
            gap = (now - self._last_drive_start).nanoseconds / 1e9
            if gap < self._pulse_min_interval:
                self._stop()
                return

        if not self._is_fire():
            sign = self._current_seek_sign
            self._pulse_angular_z = sign * self._rot_speed
            self.get_logger().info(
                f'action=SEEK_IN_FRAME dir={self._dir_label(self._pulse_angular_z)} '
                f'pulse_rotate={self._pulse_rotate:.2f}s fire_continuous='
                f'{self._fire_continuous_sec:.2f}s (lost frame)'
            )
            if self._seek_alternate:
                self._current_seek_sign *= -1.0
        else:
            sign = self._center_angular_sign()
            self._pulse_angular_z = sign * self._rot_speed
            off = float(self._latest.x_offset)
            self.get_logger().info(
                f'action=ROTATE_TO_CENTER dir={self._dir_label(self._pulse_angular_z)} '
                f'pulse_rotate={self._pulse_rotate:.2f}s x_offset={off:.3f} '
                f'fire_continuous={self._fire_continuous_sec:.2f}s'
            )

        self._pulse = PulsePhase.DRIVE
        self._phase_enter = now
        self._last_drive_start = now
        self._drive(self._pulse_angular_z)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RotationCenterTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
