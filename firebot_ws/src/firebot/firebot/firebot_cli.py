"""`firebot` console entry point.

Minimal CLI that replaces scattered `ros2 topic pub` commands. Uses one-shot
rclpy publishers so no long-running node is needed.

Examples:
  firebot alarm
  firebot alarm --off
  firebot confirm
  firebot confirm --deny
  firebot state SEARCHING
  firebot drive 80 0 0
  firebot extinguish 1
  firebot warn 1
  firebot estop
  firebot status
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import Twist


def _publish_once(topic: str, msg_type, msg, node_name: str = 'firebot_cli'):
    rclpy.init()
    try:
        node = rclpy.create_node(node_name)
        pub = node.create_publisher(msg_type, topic, 10)
        # Give the discovery a beat so latched subscribers actually hear us.
        for _ in range(10):
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def _subscribe_once(topic: str, msg_type, timeout_sec: float = 2.0,
                    node_name: str = 'firebot_cli_sub') -> Optional[object]:
    rclpy.init()
    result: list = []
    try:
        node = rclpy.create_node(node_name)

        def cb(msg):
            result.append(msg)

        node.create_subscription(msg_type, topic, cb, 10)
        start = time.monotonic()
        while not result and (time.monotonic() - start) < timeout_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
    finally:
        rclpy.shutdown()
    return result[0] if result else None


def _cmd_alarm(args):
    msg = Bool()
    msg.data = not args.off
    _publish_once('/alarm/trigger', Bool, msg)
    print(f'alarm -> {msg.data}')


def _cmd_confirm(args):
    msg = Bool()
    msg.data = not args.deny
    _publish_once('/ui/confirm', Bool, msg)
    print(f'confirm -> {msg.data}')


def _cmd_state(args):
    msg = String()
    msg.data = args.name.strip().upper()
    _publish_once('/ui/state_override', String, msg)
    print(f'state override -> {msg.data}')


def _cmd_drive(args):
    msg = Twist()
    msg.linear.x = float(args.vx)
    msg.linear.y = float(args.vy)
    msg.angular.z = float(args.wz)
    _publish_once('/cmd/drive', Twist, msg)
    print(f'drive -> vx={args.vx} vy={args.vy} wz={args.wz}')


def _cmd_extinguish(args):
    msg = Int32()
    msg.data = int(args.phase)
    _publish_once('/cmd/extinguisher', Int32, msg)
    print(f'extinguisher -> {args.phase}')


def _cmd_warn(args):
    msg = Int32()
    msg.data = int(args.mode)
    _publish_once('/cmd/warning', Int32, msg)
    print(f'warning -> {args.mode}')


def _cmd_estop(_args):
    msg = Bool()
    msg.data = True
    _publish_once('/cmd/estop', Bool, msg)
    print('estop -> sent')


def _cmd_status(args):
    out = _subscribe_once('/firebot/state', String, timeout_sec=args.timeout)
    if out is None:
        print('no /firebot/state message within timeout')
        sys.exit(1)
    print(f'state: {out.data}')


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='firebot', description='FireBot696 operator CLI')
    sub = p.add_subparsers(dest='cmd', required=True)

    a = sub.add_parser('alarm', help='publish /alarm/trigger')
    a.add_argument('--off', action='store_true', help='publish false instead of true')
    a.set_defaults(func=_cmd_alarm)

    c = sub.add_parser('confirm', help='publish /ui/confirm')
    c.add_argument('--deny', action='store_true', help='publish false instead of true')
    c.set_defaults(func=_cmd_confirm)

    s = sub.add_parser('state', help='force a state transition via /ui/state_override')
    s.add_argument('name', help='target state name (IDLE, SEARCHING, ...)')
    s.set_defaults(func=_cmd_state)

    d = sub.add_parser('drive', help='publish a one-shot /cmd/drive Twist')
    d.add_argument('vx', type=float)
    d.add_argument('vy', type=float)
    d.add_argument('wz', type=float)
    d.set_defaults(func=_cmd_drive)

    e = sub.add_parser('extinguish', help='publish /cmd/extinguisher phase (0-4)')
    e.add_argument('phase', type=int, choices=[0, 1, 2, 3, 4])
    e.set_defaults(func=_cmd_extinguish)

    w = sub.add_parser('warn', help='publish /cmd/warning mode (0-2)')
    w.add_argument('mode', type=int, choices=[0, 1, 2])
    w.set_defaults(func=_cmd_warn)

    es = sub.add_parser('estop', help='publish /cmd/estop=true')
    es.set_defaults(func=_cmd_estop)

    st = sub.add_parser('status', help='print the latest /firebot/state value')
    st.add_argument('--timeout', type=float, default=2.0)
    st.set_defaults(func=_cmd_status)

    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == '__main__':
    main()
