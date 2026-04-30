"""Arduino bridge: single pyserial connection to firebot_mega.

Subscribes:
  * /cmd/drive         (geometry_msgs/Twist) -> M,vx,vy,wz
  * /cmd/extinguisher  (std_msgs/Int32)      -> E,<phase>
  * /cmd/warning       (std_msgs/Int32)      -> W,<mode>
  * /cmd/estop         (std_msgs/Bool)       -> R

On startup, sends C,US,<0|1> / C,IR,<0|1> / C,MIC,<0|1> lines based on
`enable_ultrasonic / enable_ir / enable_mic` parameters.

Publishes:
  * /sensors/encoders   Int32MultiArray [encA_total, encB_total]
  * /sensors/ultrasonic Int32  (-1 when disabled)
  * /sensors/ir         Bool   (only published when enabled)
  * /sensors/audio      Int32  (only published when enabled)
  * /firmware/state     Int32  (mirror of firmware state enum)
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Int32MultiArray
from geometry_msgs.msg import Twist

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def _clip(val: float, lo: int = -255, hi: int = 255) -> int:
    return max(lo, min(hi, int(val)))


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('sensor_poll_hz', 10.0)
        self.declare_parameter('angular_scale', 80.0)
        self.declare_parameter('enable_ultrasonic', False)
        self.declare_parameter('enable_ir', False)
        self.declare_parameter('enable_mic', False)

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baud_rate').value)
        self.poll_hz = float(self.get_parameter('sensor_poll_hz').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.en_us = bool(self.get_parameter('enable_ultrasonic').value)
        self.en_ir = bool(self.get_parameter('enable_ir').value)
        self.en_mic = bool(self.get_parameter('enable_mic').value)

        self.enc_pub = self.create_publisher(Int32MultiArray, '/sensors/encoders', 10)
        self.us_pub = self.create_publisher(Int32, '/sensors/ultrasonic', 10)
        self.ir_pub = self.create_publisher(Bool, '/sensors/ir', 10)
        self.audio_pub = self.create_publisher(Int32, '/sensors/audio', 10)
        self.fw_state_pub = self.create_publisher(Int32, '/firmware/state', 10)

        self.create_subscription(Twist, '/cmd/drive', self._on_drive, 10)
        self.create_subscription(Int32, '/cmd/extinguisher', self._on_ext, 10)
        self.create_subscription(Int32, '/cmd/warning', self._on_warn, 10)
        self.create_subscription(Bool, '/cmd/estop', self._on_estop, 10)

        self.ser = None
        # Mega echoes L,warn=N every poll; log only when N changes.
        self._mega_last_warn_payload = None
        self._serial_retry_s = 2.0
        self._next_serial_try = 0.0
        self._serial_fail_count = 0
        if SERIAL_AVAILABLE:
            self._try_serial_connect()
        else:
            self.get_logger().warn('pyserial not installed; bridge will run headless')

        period = 1.0 / max(self.poll_hz, 0.1)
        self.create_timer(period, self._poll)
        self.get_logger().info('arduino_bridge_node up')

    def _try_serial_connect(self) -> bool:
        """Open serial to Mega. Retries from _poll if device appears late (Docker bind / USB)."""
        if not SERIAL_AVAILABLE:
            return False
        if self.ser is not None and self.ser.is_open:
            return True
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(2.0)  # Mega USB reset settle
            self.get_logger().info(f'serial opened: {self.port} @ {self.baud}')
            self._serial_fail_count = 0
            self._configure_sensors()
            return True
        except serial.SerialException as exc:
            self.ser = None
            self._serial_fail_count += 1
            if self._serial_fail_count == 1 or self._serial_fail_count % 15 == 0:
                self.get_logger().warn(
                    f'serial open failed (attempt {self._serial_fail_count}): {exc}'
                )
            return False

    def _configure_sensors(self):
        self._send(f'C,US,{int(self.en_us)}')
        self._send(f'C,IR,{int(self.en_ir)}')
        self._send(f'C,MIC,{int(self.en_mic)}')

    def _send(self, cmd: str):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.ser.write((cmd + '\n').encode('ascii'))
        except Exception as exc:
            self.get_logger().error(f'serial write failed: {exc}')

    def _on_drive(self, msg: Twist):
        vx = _clip(msg.linear.x)
        vy = 0  # PCB cannot strafe; keep for protocol parity
        wz = _clip(msg.angular.z * self.angular_scale)
        self._send(f'M,{vx},{vy},{wz}')

    def _on_ext(self, msg: Int32):
        self._send(f'E,{int(msg.data)}')

    def _on_warn(self, msg: Int32):
        self._send(f'W,{int(msg.data)}')

    def _on_estop(self, msg: Bool):
        if msg.data:
            self._send('R')

    def _poll(self):
        if SERIAL_AVAILABLE and (self.ser is None or not self.ser.is_open):
            now = time.monotonic()
            if now >= self._next_serial_try:
                self._next_serial_try = now + self._serial_retry_s
                self._try_serial_connect()
        self._send('S')
        if self.ser is None or not self.ser.is_open:
            return
        try:
            raw = self.ser.readline().decode('ascii', errors='ignore').strip()
        except Exception:
            return
        if not raw:
            return

        if raw.startswith('L,'):
            payload = raw[2:].strip()
            if payload.startswith('warn='):
                if payload == self._mega_last_warn_payload:
                    return
                self._mega_last_warn_payload = payload
                self.get_logger().info(f'mega: {payload}')
                return
            self.get_logger().info(f'mega: {payload}')
            return
        if not raw.startswith('D,'):
            return

        parts = raw.split(',')
        if len(parts) < 7:
            return
        try:
            encA = int(parts[1])
            encB = int(parts[2])
            us = int(parts[3])
            ir = int(parts[4])
            mic = int(parts[5])
            fw_state = int(parts[6])
        except ValueError:
            return

        enc_msg = Int32MultiArray()
        enc_msg.data = [encA, encB]
        self.enc_pub.publish(enc_msg)

        us_msg = Int32()
        us_msg.data = us
        self.us_pub.publish(us_msg)

        if self.en_ir and ir >= 0:
            ir_msg = Bool()
            ir_msg.data = (ir == 1)
            self.ir_pub.publish(ir_msg)
        if self.en_mic and mic >= 0:
            mic_msg = Int32()
            mic_msg.data = mic
            self.audio_pub.publish(mic_msg)

        fw_msg = Int32()
        fw_msg.data = fw_state
        self.fw_state_pub.publish(fw_msg)

    def destroy_node(self):
        try:
            self._send('R')
        except Exception:
            pass
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
