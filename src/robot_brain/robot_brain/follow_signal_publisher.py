#!/usr/bin/env python3
# FollowSignalPublisher:
# /robot/range_error → NEAR/ADJUST/FAR 분류
# /robot/cmd_forward, /robot/cmd_backward, /robot/range_state 퍼블리시
# (옵션) USB 시리얼로 'B'/'F'/'S' 및 'NEAR'/'FAR'/'OK' 전송 (payload 모드 선택)

import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String

try:
    import serial  # pyserial
except Exception:
    serial = None

STATE_TO_CHAR = {"FAR": "F", "NEAR": "B", "ADJUST": "S"}
STATE_TO_WORD = {"FAR": "FAR", "NEAR": "NEAR", "ADJUST": "OK"}  # ADJUST=OK 텍스트로 매핑


class FollowSignalPublisher(Node):
    def __init__(self):
        super().__init__('follow_signal_publisher')
        p = self.declare_parameter

        # 토픽/파라미터
        self.in_topic  = p('in_topic', '/robot/range_error').get_parameter_value().string_value
        self.forward_topic  = p('forward_topic',  '/robot/cmd_forward').get_parameter_value().string_value
        self.backward_topic = p('backward_topic', '/robot/cmd_backward').get_parameter_value().string_value
        self.state_topic    = p('state_topic',    '/robot/range_state').get_parameter_value().string_value

        self.adjust_deadband = float(p('adjust_deadband', 0.30).get_parameter_value().double_value)
        self.hysteresis      = float(p('hysteresis', 0.05).get_parameter_value().double_value)

        self.enable_serial   = p('enable_serial', True).get_parameter_value().bool_value
        # 기본값을 ACM0로 (상황에 맞게 런치에서 덮어쓰기)
        self.serial_port     = p('serial_port', '/dev/ttyACM0').get_parameter_value().string_value
        self.serial_baud     = int(p('serial_baud', 115200).get_parameter_value().integer_value)
        self.serial_on_change_only = p('serial_on_change_only', False).get_parameter_value().bool_value
        self.serial_min_period     = float(p('serial_min_period', 0.4).get_parameter_value().double_value)

        # NEW: 전송 형태 ('both' | 'char' | 'word')
        self.serial_payload_mode = p('serial_payload_mode', 'both').get_parameter_value().string_value

        self._last_state: Optional[str] = None
        self._last_serial_sent: Optional[str] = None
        self._last_serial_time = 0.0
        self._ser = None
        self._lock = threading.Lock()

        # Pub/Sub
        self.pub_fwd   = self.create_publisher(Bool,   self.forward_topic,  10)
        self.pub_bwd   = self.create_publisher(Bool,   self.backward_topic, 10)
        self.pub_state = self.create_publisher(String, self.state_topic,    10)
        self.sub_err   = self.create_subscription(Float32, self.in_topic, self.on_error, 10)

        # 시리얼 유지 타이머
        if self.enable_serial and serial is not None:
            self.create_timer(1.0, self._ensure_serial)

        self.get_logger().info(
            f"[follow_signal_publisher] in:{self.in_topic} → "
            f"out:{self.forward_topic},{self.backward_topic},{self.state_topic} | "
            f"deadband=±{self.adjust_deadband:.2f}m, hysteresis={self.hysteresis:.2f}m | "
            f"serial={self.enable_serial} {self.serial_port} @{self.serial_baud} mode={self.serial_payload_mode}"
        )

    # --- 상태 분류 로직(히스테리시스) ---
    def classify(self, err: float) -> str:
        ad = self.adjust_deadband
        h  = self.hysteresis
        s  = self._last_state

        if s == 'NEAR':
            return 'NEAR' if err <= -ad else 'ADJUST'
        if s == 'FAR':
            return 'FAR'  if err >= +ad else 'ADJUST'

        if err <= -(ad + h):
            return 'NEAR'
        elif err >= +(ad + h):
            return 'FAR'
        else:
            return 'ADJUST'

    # --- 에러 입력 콜백 ---
    def on_error(self, msg: Float32):
        err = float(msg.data)
        with self._lock:
            state = self.classify(err)
            if state != self._last_state:
                self.get_logger().info(f"range_error={err:.3f} → {state}")
                self._last_state = state

            # 토픽 퍼블리시
            self.pub_fwd.publish(Bool(data=(state == 'FAR')))
            self.pub_bwd.publish(Bool(data=(state == 'NEAR')))
            self.pub_state.publish(String(data=state))

            # 시리얼 전송
            if self.enable_serial and serial is not None:
                ch = STATE_TO_CHAR[state]  # 'F','B','S'
                now = time.time()
                send_ok = True
                if self.serial_on_change_only and ch == self._last_serial_sent:
                    if now - self._last_serial_time < self.serial_min_period:
                        send_ok = False
                if send_ok:
                    self._serial_send(state, ch)
                    self._last_serial_sent = ch
                    self._last_serial_time = now

    # --- 시리얼 연결 보장 ---
    def _ensure_serial(self):
        if not self.enable_serial or serial is None:
            return
        if self._ser is None:
            try:
                self._ser = serial.Serial(
                    self.serial_port, self.serial_baud,
                    timeout=0.05, write_timeout=0.2,
                    rtscts=False, dsrdtr=False, xonxoff=False
                )
                try:
                    # UNO 계열은 DTR 토글 시 리셋됨 → 약간의 안정화 대기
                    self._ser.reset_input_buffer()
                    self._ser.reset_output_buffer()
                    self._ser.setDTR(True)
                    self._ser.setRTS(True)
                except Exception:
                    pass
                self.get_logger().info(f"Serial connected: {self.serial_port} @{self.serial_baud}")
                time.sleep(0.3)
            except Exception as e:
                self._ser = None
                self.get_logger().warn(f"Serial open failed: {e}")

    # --- 실제 송신 ---
    def _serial_send(self, state: str, ch: str):
        if self._ser is None:
            self._ensure_serial()
        if self._ser is None:
            return
        mode = (self.serial_payload_mode or 'both').lower()
        try:
            payloads = []
            if mode in ('char', 'both'):
                payloads.append(ch)  # 'B'/'F'/'S'
            if mode in ('word', 'both'):
                payloads.append(STATE_TO_WORD[state])  # 'NEAR'/'FAR'/'OK'

            for p in payloads:
                line = (p + "\n").encode('ascii', errors='ignore')
                self._ser.write(line)
                self.get_logger().info(f"Serial TX → {self.serial_port}: {repr(p)}")
        except Exception as e:
            self.get_logger().warn(f"Serial write failed: {e}")
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


def main(args=None):
    rclpy.init(args=args)
    node = FollowSignalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
