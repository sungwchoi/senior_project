#!/usr/bin/env python3
# /robot/range_error → near/adjust/far 판단해서
# /robot/cmd_forward, /robot/cmd_backward, /robot/range_state 출력
# (옵션) USB 시리얼로 'F'/'B'/'S' 전송

import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

try:
    import serial  # optional
except Exception:
    serial = None


class FollowSignalPublisher(Node):
    def __init__(self):
        super().__init__('follow_signal_publisher')
        p = self.declare_parameter

        self.in_topic  = p('in_topic', '/robot/range_error').get_parameter_value().string_value
        self.forward_topic  = p('forward_topic',  '/robot/cmd_forward').get_parameter_value().string_value
        self.backward_topic = p('backward_topic', '/robot/cmd_backward').get_parameter_value().string_value
        self.state_topic    = p('state_topic',    '/robot/range_state').get_parameter_value().string_value

        self.adjust_deadband = float(p('adjust_deadband', 0.30).get_parameter_value().double_value)
        self.hysteresis      = float(p('hysteresis', 0.05).get_parameter_value().double_value)

        self.enable_serial   = p('enable_serial', False).get_parameter_value().bool_value
        self.serial_port     = p('serial_port', '/dev/ttyACM0').get_parameter_value().string_value
        self.serial_baud     = int(p('serial_baud', 115200).get_parameter_value().integer_value)
        self.serial_on_change_only = p('serial_on_change_only', True).get_parameter_value().bool_value
        self.serial_min_period     = float(p('serial_min_period', 0.4).get_parameter_value().double_value)

        self._last_state: Optional[str] = None
        self._last_serial_sent: Optional[str] = None
        self._last_serial_time = 0.0
        self._ser = None
        self._lock = threading.Lock()

        self.pub_fwd   = self.create_publisher(Bool,   self.forward_topic,  10)
        self.pub_bwd   = self.create_publisher(Bool,   self.backward_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic,    10)
        self.sub_err   = self.create_subscription(Float32, self.in_topic, self.on_error, 10)

        if self.enable_serial and serial is not None:
            self.create_timer(2.0, self._ensure_serial)

        self.get_logger().info(
            f"[follow_signal_publisher] in:{self.in_topic} → "
            f"out:{self.forward_topic},{self.backward_topic},{self.state_topic} | "
            f"deadband=±{self.adjust_deadband:.2f}m, hysteresis={self.hysteresis:.2f}m"
        )

    def classify(self, err: float) -> str:
        # err>0: FAR(앞으로), err<0: NEAR(뒤로), deadband: ADJUST(정지)
        ad = self.adjust_deadband 
        h = self.hysteresis
        s = self._last_state        
        
        if s == 'NEAR': 
            return 'NEAR' if err <= -ad else 'ADJUST'
        
        if s == 'FAR': 
            return 'FAR' if err >= +ad else 'ADJUST'

        if err <= -(ad + h): 
            return 'NEAR'
        elif err >= +(ad + h):
            return 'FAR'
        else:
            return 'ADJUST'

    def on_error(self, msg: Float32):
        err = float(msg.data)
        with self._lock:
            state = self.classify(err)
            if state != self._last_state:
                self.get_logger().info(f"range_error={err:.3f} → {state}")
                self._last_state = state

            self.pub_fwd.publish(Bool(data=(state == 'FAR')))
            self.pub_bwd.publish(Bool(data=(state == 'NEAR')))
            self.pub_state.publish(String(data=state))

            if self.enable_serial and serial is not None:
                ch = 'S'
                if state == 'FAR': ch = 'F'
                elif state == 'NEAR': ch = 'B'
                now, send_ok = time.time(), True
                if self.serial_on_change_only and ch == self._last_serial_sent:
                    if now - self._last_serial_time < self.serial_min_period:
                        send_ok = False
                if send_ok:
                    self._serial_write(ch)
                    self._last_serial_sent = ch
                    self._last_serial_time = now

    def _ensure_serial(self):
        if not self.enable_serial or serial is None: return
        if self._ser is None:
            try:
                self._ser = serial.Serial(self.serial_port, self.serial_baud, timeout=0.1)
                self.get_logger().info(f"Serial connected: {self.serial_port}")
                time.sleep(0.2)
            except Exception as e:
                self._ser = None
                self.get_logger().warn(f"Serial open failed: {e}")

    def _serial_write(self, ch: str):
        if self._ser is None: self._ensure_serial()
        if self._ser is not None:
            try:
                self._ser.write(ch.encode('ascii'))
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
                try: self._ser.close()
                except Exception: pass
                self._ser = None


def main(args=None):
    rclpy.init(args=args)
    node = FollowSignalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
