# #!/usr/bin/env python3
# import time, threading
# from typing import Optional, Iterable, Set

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# from std_msgs.msg import String

# try:
#     from vision_msgs.msg import Detection2DArray
# except Exception:
#     Detection2DArray = None  # vision_msgs 없으면 옵션 기능 비활성

# try:
#     import serial
# except Exception:
#     serial = None


# class JoyToSerial(Node):
#     """
#     한 노드에서 다음을 모두 같은 시리얼 포트로 전송:
#       - 조이스틱: FORWARD/LEFT/RIGHT/BACK/STOP (혹은 F/L/R/B/S)
#       - 라이다 상태: OKAY/NEAR/FAR (ADJUST -> OKAY로 정규화)
#       - YOLO 감지: PERSON/WEAPON

#     YOLO 입력은 두 경로를 지원:
#       (A) std_msgs/String: /robot/yolo_alarm  (데이터: "PERSON" or "WEAPON")
#       (B) vision_msgs/Detection2DArray: /yolo/detections  (라벨 매핑으로 판단)
#     """

#     def __init__(self):
#         super().__init__('joy_to_serial')
#         p = self.declare_parameter

#         # -------- 조이스틱 입력 맵핑 --------
#         self.joy_topic   = p('joy_topic', '/joy').get_parameter_value().string_value
#         self.axis_x      = int(p('axis_x', 0).get_parameter_value().integer_value)
#         self.axis_y      = int(p('axis_y', 1).get_parameter_value().integer_value)
#         self.hat_x       = int(p('hat_x', 6).get_parameter_value().integer_value)
#         self.hat_y       = int(p('hat_y', 7).get_parameter_value().integer_value)
#         self.deadzone    = float(p('deadzone', 0.4).get_parameter_value().double_value)
#         self.prefer_hat  = p('prefer_hat', True).get_parameter_value().bool_value
#         self.invert_x    = p('invert_x', False).get_parameter_value().bool_value
#         self.invert_y    = p('invert_y', False).get_parameter_value().bool_value

#         # -------- 시리얼 설정 --------
#         self.port        = p('port', '/dev/ttyACM0').get_parameter_value().string_value
#         self.baud        = int(p('baud', 115200).get_parameter_value().integer_value)
#         # 기본: 단어 기반 프로토콜 (아두이노가 단어 받도록 구성)
#         self.payload_mode = p('payload_mode', 'word').get_parameter_value().string_value  # 'char'|'word'|'both'
#         self.on_change_only = p('on_change_only', True).get_parameter_value().bool_value
#         self.min_period  = float(p('min_period', 0.15).get_parameter_value().double_value)

#         # -------- 라이다 상태 구독 --------
#         self.range_state_topic = p('range_state_topic', '/robot/range_state').get_parameter_value().string_value

#         # -------- YOLO 입력(둘 다 옵션) --------
#         self.yolo_string_topic = p('yolo_string_topic', '/robot/yolo_alarm').get_parameter_value().string_value
#         self.yolo_det_topic    = p('yolo_detection_topic', '/yolo/detections').get_parameter_value().string_value
#         # 라벨 매핑(쉼표로 구분된 문자열; 소문자 비교)
#         self.yolo_person_labels_csv = p('yolo_person_labels', 'person').get_parameter_value().string_value
#         self.yolo_weapon_labels_csv = p('yolo_weapon_labels', 'weapon,gun,knife,pistol,revolver,rifle').get_parameter_value().string_value
#         # 중복 전송 방지용 디바운싱
#         self.yolo_min_period = float(p('yolo_min_period', 0.6).get_parameter_value().double_value)

#         # 내부 상태
#         self._ser = None
#         self._lock = threading.Lock()
#         self._last_cmd: Optional[str] = None
#         self._last_cmd_time = 0.0

#         self._last_yolo_word: Optional[str] = None
#         self._last_yolo_time = 0.0

#         # 구독 설정
#         self.sub_joy    = self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
#         self.sub_state  = self.create_subscription(String, self.range_state_topic, self.on_range_state, 10)

#         # YOLO 문자열 토픽
#         self.sub_yolo_s = self.create_subscription(String, self.yolo_string_topic, self.on_yolo_string, 10)

#         # YOLO Detection2DArray 토픽 (vision_msgs 있을 때만)
#         if Detection2DArray is not None and self.yolo_det_topic:
#             self.sub_yolo_d = self.create_subscription(Detection2DArray, self.yolo_det_topic, self.on_yolo_detection, 10)
#         else:
#             self.sub_yolo_d = None

#         # 라벨 세트
#         self.yolo_person_labels: Set[str] = self._csv_to_set(self.yolo_person_labels_csv)
#         self.yolo_weapon_labels: Set[str] = self._csv_to_set(self.yolo_weapon_labels_csv)

#         if serial is not None:
#             self.create_timer(1.0, self._ensure_serial)

#         self.get_logger().info(
#             f"[joy_to_serial] Port={self.port}@{self.baud} mode={self.payload_mode} | "
#             f"joy={self.joy_topic} prefer_hat={self.prefer_hat} dz={self.deadzone} inv=({self.invert_x},{self.invert_y}) | "
#             f"range_state={self.range_state_topic} | "
#             f"yolo_string={self.yolo_string_topic} yolo_dets={self.yolo_det_topic} "
#             f"person={sorted(self.yolo_person_labels)} weapon={sorted(self.yolo_weapon_labels)}"
#         )

#     # ---------------- Serial ----------------
#     def _ensure_serial(self):
#         if self._ser is None and serial is not None:
#             try:
#                 self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
#                 time.sleep(0.2)
#                 self.get_logger().info(f"Serial connected: {self.port}@{self.baud}")
#             except Exception as e:
#                 self._ser = None
#                 self.get_logger().warn(f"Serial open failed: {e}")

#     def _send_line(self, text: str):
#         """줄단위 전송 (아두이노는 라인 단위 토큰 파서)"""
#         if not text:
#             return
#         if self._ser is None:
#             self._ensure_serial()
#         if self._ser:
#             try:
#                 payload = (text + "\n").encode('ascii', errors='ignore')
#                 self._ser.write(payload)
#                 self.get_logger().info(f"Serial TX → {self.port}: {repr(text)}")
#             except Exception as e:
#                 self.get_logger().warn(f"Serial write failed: {e}")
#                 try: self._ser.close()
#                 except Exception: pass
#                 self._ser = None

#     def _csv_to_set(self, csv: str) -> Set[str]:
#         return {s.strip().lower() for s in csv.split(',') if s.strip()}

#     # ---------------- Joy → Serial ----------------
#     def on_joy(self, msg: Joy):
#         def val(arr, idx):
#             try:
#                 return arr[idx]
#             except Exception:
#                 return 0.0

#         # D-pad 우선/스틱
#         if self.prefer_hat:
#             x = float(val(msg.axes, self.hat_x))
#             y = float(val(msg.axes, self.hat_y))
#         else:
#             x = float(val(msg.axes, self.axis_x))
#             y = float(val(msg.axes, self.axis_y))

#         if self.invert_x: x = -x
#         if self.invert_y: y = -y

#         cmd = self._decide_cmd(x, y, self.deadzone)  # 'F','B','L','R','S'
#         if cmd is None:
#             return

#         now = time.time()
#         if self.on_change_only and cmd == self._last_cmd and (now - self._last_cmd_time) < self.min_period:
#             return

#         words = {'F': 'FORWARD', 'B': 'BACK', 'L': 'LEFT', 'R': 'RIGHT', 'S': 'STOP'}
#         m = (self.payload_mode or 'word').lower()
#         if m == 'word':
#             self._send_line(words[cmd])
#         elif m == 'both':
#             self._send_line(cmd)
#             self._send_line(words[cmd])
#         else:
#             self._send_line(cmd)

#         self._last_cmd = cmd
#         self._last_cmd_time = now

#     def _decide_cmd(self, x: float, y: float, dz: float) -> Optional[str]:
#         # 우선순위: 전/후 → 좌/우
#         if y >  dz:  return 'F'
#         if y < -dz:  return 'B'
#         if x >  dz:  return 'R'
#         if x < -dz:  return 'L'
#         return 'S'

#     # ---------------- Range state → Serial ----------------
#     def on_range_state(self, msg: String):
#         word = (msg.data or '').strip().upper()
#         if not word:
#             return
#         if word == 'ADJUST':
#             word = 'OKAY'
#         # 라이다 단어 그대로 전송
#         self._send_line(word)

#     # ---------------- YOLO (String) → Serial ----------------
#     def on_yolo_string(self, msg: String):
#         # "PERSON" 또는 "WEAPON" 을 직접 받는 경우
#         word = (msg.data or '').strip().upper()
#         if word not in ('PERSON', 'WEAPON'):
#             return
#         self._maybe_send_yolo(word)

#     # ---------------- YOLO (Detection2DArray) → Serial ----------------
#     def on_yolo_detection(self, msg):
#         """
#         detection 결과의 class_name 혹은 class_id를 label로 보고,
#         person/weapon 레이블이 하나라도 있으면 PERSON/WEAPON 신호를 전송.
#         """
#         try:
#             # 최신 vision_msgs 포맷을 가정: detection.results[*].hypothesis.class_id (str) 또는 .score
#             found_person = False
#             found_weapon = False
#             for det in msg.detections:
#                 for res in det.results:
#                     # class_id가 문자열 라벨인 경우가 많음
#                     lab = (getattr(res.hypothesis, 'class_id', '') or '').strip().lower()
#                     if not lab:
#                         continue
#                     if lab in self.yolo_person_labels:
#                         found_person = True
#                     if lab in self.yolo_weapon_labels:
#                         found_weapon = True

#             if found_weapon:
#                 self._maybe_send_yolo('WEAPON')
#             elif found_person:
#                 self._maybe_send_yolo('PERSON')
#         except Exception as e:
#             self.get_logger().warn(f"YOLO detection parse failed: {e}")

#     # ---------------- YOLO 공통 전송(디바운스) ----------------
#     def _maybe_send_yolo(self, word: str):
#         now = time.time()
#         if self._last_yolo_word == word and (now - self._last_yolo_time) < self.yolo_min_period:
#             return
#         self._send_line(word)  # PERSON → 아두이노 5번 LED 1회, WEAPON → 8회 (스케치가 처리)
#         self._last_yolo_word = word
#         self._last_yolo_time = now


# def main(args=None):
#     rclpy.init(args=args)
#     node = JoyToSerial()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()





#!/usr/bin/env python3
import time, threading
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

try:
    import serial
except Exception:
    serial = None


class JoyToSerial(Node):
    def __init__(self):
        super().__init__('joy_to_serial')
        p = self.declare_parameter

        # 입력 토픽 & 맵핑
        self.joy_topic   = p('joy_topic', '/joy').get_parameter_value().string_value
        self.axis_x      = int(p('axis_x', 0).get_parameter_value().integer_value)
        self.axis_y      = int(p('axis_y', 1).get_parameter_value().integer_value)
        self.hat_x       = int(p('hat_x', 6).get_parameter_value().integer_value)
        self.hat_y       = int(p('hat_y', 7).get_parameter_value().integer_value)
        self.deadzone    = float(p('deadzone', 0.4).get_parameter_value().double_value)
        self.prefer_hat  = p('prefer_hat', True).get_parameter_value().bool_value

        # 방향 반전 옵션
        self.invert_x    = p('invert_x', False).get_parameter_value().bool_value
        self.invert_y    = p('invert_y', False).get_parameter_value().bool_value

        # 직렬 설정(조이스틱/라이다/YOLO 모두 같은 포트로 보냄)
        self.port        = p('port', '/dev/ttyACM0').get_parameter_value().string_value
        self.baud        = int(p('baud', 115200).get_parameter_value().integer_value)
        self.payload_mode = p('payload_mode', 'word').get_parameter_value().string_value  # 'char'|'word'|'both'
        self.on_change_only = p('on_change_only', True).get_parameter_value().bool_value
        self.min_period  = float(p('min_period', 0.15).get_parameter_value().double_value)

        # NEW: 토픽 이름
        self.range_state_topic = p('range_state_topic', '/robot/range_state').get_parameter_value().string_value
        self.yolo_event_topic  = p('yolo_event_topic', '/robot/yolo_event').get_parameter_value().string_value

        self._ser = None
        self._last_sent: Optional[str] = None
        self._last_time = 0.0
        self._lock = threading.Lock()

        # 구독
        self.sub_joy    = self.create_subscription(Joy, self.joy_topic, self.on_joy, 10)
        self.sub_state  = self.create_subscription(String, self.range_state_topic, self.on_range_state, 10)
        self.sub_yolo   = self.create_subscription(String, self.yolo_event_topic, self.on_yolo_event, 10)

        if serial is not None:
            self.create_timer(1.0, self._ensure_serial)

        self.get_logger().info(
            f"joy_to_serial: serial {self.port}@{self.baud} mode={self.payload_mode} "
            f"| joy_topic={self.joy_topic} prefer_hat={self.prefer_hat} deadzone={self.deadzone} "
            f"| invert_x={self.invert_x} invert_y={self.invert_y} "
            f"| range_state={self.range_state_topic} yolo_event={self.yolo_event_topic}"
        )

    # ---------- Serial ----------
    def _ensure_serial(self):
        if self._ser is None and serial is not None:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
                time.sleep(0.2)
                self.get_logger().info(f"Serial connected: {self.port}@{self.baud}")
            except Exception as e:
                self._ser = None
                self.get_logger().warn(f"Serial open failed: {e}")

    def _write_line(self, text: str, tag: str):
        if not text:
            return
        if self._ser is None:
            self._ensure_serial()
        if self._ser:
            try:
                self._ser.write((text + "\n").encode('ascii', errors='ignore'))
                self.get_logger().info(f"Serial TX ({tag}) → {self.port}: '{text}'")
            except Exception as e:
                self.get_logger().warn(f"Serial write failed ({tag}): {e}")
                try: self._ser.close()
                except Exception: pass
                self._ser = None

    # ---------- Joy ----------
    def on_joy(self, msg: Joy):
        def val(arr, idx):
            try: return arr[idx]
            except Exception: return 0.0

        # D-pad 우선/스틱 선택
        if self.prefer_hat:
            x = val(msg.axes, self.hat_x)
            y = val(msg.axes, self.hat_y)
        else:
            x = val(msg.axes, self.axis_x)
            y = val(msg.axes, self.axis_y)

        if self.invert_x: x = -x
        if self.invert_y: y = -y

        cmd = self.decide_cmd(x, y, self.deadzone)  # 'F','B','L','R','S'
        if cmd is None:
            return

        now = time.time()
        if self.on_change_only and cmd == self._last_sent and (now - self._last_time) < self.min_period:
            return

        self._serial_send_joy(cmd)
        self._last_sent = cmd
        self._last_time = now

    def decide_cmd(self, x: float, y: float, dz: float) -> Optional[str]:
        if y >  dz:  return 'F'
        if y < -dz:  return 'B'
        if x >  dz:  return 'R'
        if x < -dz:  return 'L'
        return 'S'

    def _serial_send_joy(self, cmd: str):
        words = {'F': 'FORWARD', 'B': 'BACK', 'L': 'LEFT', 'R': 'RIGHT', 'S': 'STOP'}
        m = (self.payload_mode or 'word').lower()
        if m == 'word':
            payloads = [words[cmd]]
        elif m == 'both':
            payloads = [cmd, words[cmd]]
        else:
            payloads = [cmd]
        for p in payloads:
            self._write_line(p, "joy")

    # ---------- Range state (NEAR/FAR/ADJUST) ----------
    def on_range_state(self, msg: String):
        word = (msg.data or '').strip().upper()
        if not word:
            return
        if word == 'ADJUST':
            word = 'OKAY'  # 아두이노 스케치가 OKAY 기대
        self._write_line(word, "range_state")

    # ---------- YOLO event (PERSON/WEAPON) ----------
    def on_yolo_event(self, msg: String):
        word = (msg.data or '').strip().upper()  # PERSON / WEAPON
        if not word:
            return
        self._write_line(word, "yolo_event")


def main(args=None):
    rclpy.init(args=args)
    node = JoyToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
