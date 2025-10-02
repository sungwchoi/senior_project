# # ~/senior_project/src/robot_brain/robot_brain/distance_keeper.py
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from std_msgs.msg import String, Float32
# from obstacle_detector.msg import Obstacles
# from visualization_msgs.msg import Marker, MarkerArray
# from builtin_interfaces.msg import Duration

# class DistanceKeeper(Node):
#     def __init__(self):
#         super().__init__('distance_keeper')

#         # --- Parameters (핵심) ---
#         p = self.declare_parameter
#         self.target   = p('target_distance', 1.5).get_parameter_value().double_value
#         self.tol_ok   = p('tol_ok', 0.20).get_parameter_value().double_value
#         self.tol_wide = p('tol_slow', 0.35).get_parameter_value().double_value
#         self.r_min    = p('min_detect_radius', 0.30).get_parameter_value().double_value
#         self.r_max    = p('max_detect_radius', 3.50).get_parameter_value().double_value
#         self.alpha    = p('ema_alpha', 0.35).get_parameter_value().double_value
#         self.max_age  = p('max_age_sec', 0.8).get_parameter_value().double_value
#         self.rate_hz  = p('publish_rate_hz', 15.0).get_parameter_value().double_value

#         # 섹터: 전방 -45°~+45° → center=0.0, width=pi/2(=1.570796)
#         self.sector_center = p('sector_center', 0.0).get_parameter_value().double_value
#         self.sector_width  = p('sector_width', 1.570796).get_parameter_value().double_value

#         # RViz 마커 프레임
#         self.frame_id = p('frame_id', 'velodyne').get_parameter_value().string_value

#         # --- Parameters (정확도 향상용 튜닝) ---
#         # 원(다리) 반경 범위로 후보 게이팅
#         self.leg_r_min = p('leg_radius_min', 0.05).get_parameter_value().double_value   # 5 cm
#         self.leg_r_max = p('leg_radius_max', 0.18).get_parameter_value().double_value   # 18 cm

#         # 이전 타깃 근처를 선호하기 위한 게이트/가중치
#         self.theta_gate = p('theta_gate', 0.35).get_parameter_value().double_value      # ~20 deg
#         self.r_gate     = p('range_gate', 0.50).get_parameter_value().double_value      # 0.5 m
#         self.w_dr       = p('cost_w_dr', 0.4).get_parameter_value().double_value        # 거리 변화 가중치
#         self.w_dth      = p('cost_w_dth', 0.2).get_parameter_value().double_value       # 각도 변화 가중치

#         # 미디안 윈도우
#         self.med_N      = int(p('median_window', 5).get_parameter_value().integer_value or 5)

#         # --- I/O ---
#         qos_rel = QoSProfile(depth=10,
#                              reliability=QoSReliabilityPolicy.RELIABLE,
#                              history=QoSHistoryPolicy.KEEP_LAST)
#         self.sub = self.create_subscription(Obstacles, '/obstacles', self.on_obstacles, qos_rel)
#         self.cmd_pub = self.create_publisher(String,  '/follow_cmd', 10)
#         self.err_pub = self.create_publisher(Float32, '/range_error', 10)
#         self.marker_pub = self.create_publisher(MarkerArray, '/distance_markers', 10)

#         # --- State ---
#         self._last_stamp = None
#         self._raw_d = None       # 미디안 결과
#         self._ema_d = None
#         self._last_xy = None
#         self._prev_xy = None
#         self._prev_r  = None
#         self._prev_th = None
#         self._med_buf = []

#         self._timer = self.create_timer(max(1.0/self.rate_hz, 0.02), self.tick)

#     # --- helpers ---
#     @staticmethod
#     def _angdiff(a, b):
#         return (a - b + math.pi) % (2.0*math.pi) - math.pi

#     def _in_sector(self, th):
#         return abs(self._angdiff(th, self.sector_center)) <= (0.5 * self.sector_width)

#     def _cost(self, r, th):
#         """이전 타깃에 가까울수록 비용을 낮게 → 점프 감소"""
#         if self._prev_r is None:
#             return r
#         dr  = abs(r - self._prev_r)
#         dth = abs(self._angdiff(th, self._prev_th))
#         return r + self.w_dr * dr + self.w_dth * dth

#     def _publish_text_marker(self):
#         if self._last_xy is None:
#             return
#         x, y = self._last_xy
#         arr = MarkerArray()

#         clear = Marker()
#         clear.action = Marker.DELETEALL
#         clear.header.frame_id = self.frame_id
#         clear.header.stamp = self.get_clock().now().to_msg()
#         arr.markers.append(clear)

#         txt = Marker()
#         txt.header.frame_id = self.frame_id
#         txt.header.stamp = clear.header.stamp
#         txt.ns = "dk_text"
#         txt.id = 1
#         txt.type = Marker.TEXT_VIEW_FACING
#         txt.action = Marker.ADD
#         txt.pose.position.x = float(x)
#         txt.pose.position.y = float(y)
#         txt.pose.position.z = 0.6
#         txt.scale.z = 0.32
#         txt.color.a = 1.0
#         txt.color.r = 1.0
#         txt.color.g = 1.0
#         txt.color.b = 1.0
#         d_show = self._ema_d if self._ema_d is not None else self._raw_d
#         txt.text = f"{d_show:.2f} m"
#         txt.lifetime = Duration(sec=1)
#         arr.markers.append(txt)

#         self.marker_pub.publish(arr)

#     def _clear_markers(self):
#         arr = MarkerArray()
#         m = Marker()
#         m.action = Marker.DELETEALL
#         m.header.frame_id = self.frame_id
#         m.header.stamp = self.get_clock().now().to_msg()
#         arr.markers.append(m)
#         self.marker_pub.publish(arr)

#     # --- core callbacks ---
#     def on_obstacles(self, msg: Obstacles):
#         """원(표면거리) & 세그먼트(중점) 중 비용 최소 후보 1개 선택 → 미디안→EMA"""
#         best = None  # (cost, r_eff, x, y, th)

#         # 1) circles: 원 반경 게이팅 + 표면 거리 사용
#         for c in msg.circles:
#             x, y = c.center.x, c.center.y
#             th = math.atan2(y, x)
#             if not self._in_sector(th):
#                 continue

#             r_center = math.hypot(x, y)

#             # 반경 필터(다리 유사한 사이즈만)
#             c_rad = getattr(c, 'radius', None)
#             if c_rad is not None:
#                 if not (self.leg_r_min <= c_rad <= self.leg_r_max):
#                     continue
#                 r_eff = max(r_center - c_rad, 0.0)  # 표면까지 거리
#             else:
#                 r_eff = r_center

#             if not (self.r_min <= r_eff <= self.r_max):
#                 continue

#             cost = self._cost(r_eff, th)
#             if best is None or cost < best[0]:
#                 best = (cost, r_eff, x, y, th)

#         # 2) segments: 중점 사용 (선 길이 게이팅이 필요하면 여기 추가)
#         for s in msg.segments:
#             mx = 0.5*(s.first_point.x + s.last_point.x)
#             my = 0.5*(s.first_point.y + s.last_point.y)
#             th = math.atan2(my, mx)
#             if not self._in_sector(th):
#                 continue

#             r_eff = math.hypot(mx, my)
#             if not (self.r_min <= r_eff <= self.r_max):
#                 continue

#             cost = self._cost(r_eff, th)
#             if best is None or cost < best[0]:
#                 best = (cost, r_eff, mx, my, th)

#         # 후보 없음
#         if best is None:
#             self._last_xy = None
#             self._prev_xy = None
#             self._prev_r  = None
#             self._prev_th = None
#             self._clear_markers()
#             return

#         _, r_eff, bx, by, bth = best

#         # 3) 미디안 + EMA
#         self._med_buf.append(r_eff)
#         if len(self._med_buf) > self.med_N:
#             self._med_buf.pop(0)
#         r_med = sorted(self._med_buf)[len(self._med_buf)//2]

#         self._raw_d = r_med
#         self._ema_d = r_med if self._ema_d is None else (self.alpha * r_med + (1.0 - self.alpha) * self._ema_d)

#         # 4) 상태/마커/기억 갱신
#         self._last_stamp = self.get_clock().now()
#         self._last_xy = (bx, by)
#         self._prev_xy = (bx, by)
#         self._prev_r  = r_eff
#         self._prev_th = bth
#         self._publish_text_marker()

#     def _decide(self, d):
#         up = self.target + self.tol_wide
#         lo = self.target - self.tol_wide
#         ok_hi = self.target + self.tol_ok
#         ok_lo = self.target - self.tol_ok
#         if d > up:              return "FAR"     # 멀다 → 앞으로
#         if d < lo:              return "NEAR"    # 가깝다 → 뒤로/정지
#         if ok_lo <= d <= ok_hi: return "OK"      # 유지
#         return "ADJUST"                          # 미세 조정

#     def tick(self):
#         now = self.get_clock().now()
#         cmd = String(); err = Float32()

#         # 타임아웃: 타깃 없음
#         if self._last_stamp is None or (now - self._last_stamp).nanoseconds * 1e-9 > self.max_age:
#             cmd.data = "NO_TARGET"; err.data = 0.0
#             self.cmd_pub.publish(cmd); self.err_pub.publish(err)
#             self._clear_markers()
#             return

#         d = self._ema_d if self._ema_d is not None else self._raw_d
#         cmd.data = self._decide(d)
#         err.data = float(d - self.target)
#         self.cmd_pub.publish(cmd)
#         self.err_pub.publish(err)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DistanceKeeper()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# 여기가 수정본 위에가 기존 거

# ~/senior_project/src/robot_brain/robot_brain/distance_keeper.py
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Float32
from obstacle_detector.msg import Obstacles
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

class DistanceKeeper(Node):
    """
    /obstacles에서 가장 가까운 물체를 선택해 거리 유지 상태(FAR/NEAR/OK/ADJUST/NO_TARGET)를 결정.
    RViz에는 텍스트 마커(거리 | 상태) 1개만 가볍게 표시.
    """
    def __init__(self):
        super().__init__('distance_keeper')

        # --- 핵심 파라미터 ---
        p = self.declare_parameter
        self.target   = p('target_distance', 1.3).get_parameter_value().double_value
        self.tol_ok   = p('tol_ok', 0.15).get_parameter_value().double_value
        self.tol_wide = p('tol_slow', 0.35).get_parameter_value().double_value
        self.r_min    = p('min_detect_radius', 0.05).get_parameter_value().double_value
        self.r_max    = p('max_detect_radius', 2.75).get_parameter_value().double_value
        self.alpha    = p('ema_alpha', 0.35).get_parameter_value().double_value
        self.max_age  = p('max_age_sec', 0.8).get_parameter_value().double_value
        self.rate_hz  = p('publish_rate_hz', 15.0).get_parameter_value().double_value

        # 섹터: 전방 -45°~+45° → center=0.0, width=pi/2(=1.570796)
        self.sector_center = p('sector_center', 0.0).get_parameter_value().double_value
        self.sector_width  = p('sector_width', 1.570796).get_parameter_value().double_value

        # RViz 마커 프레임 및 표기 모드(기본: 거리+상태 둘 다)
        self.frame_id    = p('frame_id', 'velodyne').get_parameter_value().string_value
        self.marker_mode = p('marker_mode', 'both').get_parameter_value().string_value  # 'both'|'status'|'distance'

        # --- 정확도 향상용 튜닝 ---
        self.leg_r_min = p('leg_radius_min', 0.05).get_parameter_value().double_value   # 5 cm
        self.leg_r_max = p('leg_radius_max', 0.18).get_parameter_value().double_value   # 18 cm
        self.theta_gate = p('theta_gate', 0.35).get_parameter_value().double_value      # ~20 deg
        self.r_gate     = p('range_gate', 0.50).get_parameter_value().double_value      # 0.5 m
        self.w_dr       = p('cost_w_dr', 0.4).get_parameter_value().double_value
        self.w_dth      = p('cost_w_dth', 0.2).get_parameter_value().double_value
        self.med_N      = int(p('median_window', 5).get_parameter_value().integer_value or 5)

        # --- I/O ---
        qos_rel = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Obstacles, '/obstacles', self.on_obstacles, qos_rel)
        self.cmd_pub = self.create_publisher(String,  '/follow_cmd', 10)
        self.err_pub = self.create_publisher(Float32, '/range_error', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/distance_markers', 10)

        # --- 상태 ---
        self._last_stamp = None
        self._raw_d = None          # 미디안 결과
        self._ema_d = None
        self._last_xy = None
        self._prev_xy = None
        self._prev_r  = None
        self._prev_th = None
        self._med_buf = []
        self._last_cmd_str = None   # RViz 텍스트 표시에 사용

        self._timer = self.create_timer(max(1.0/self.rate_hz, 0.02), self.tick)

    # ------------- helpers -------------
    @staticmethod
    def _angdiff(a, b):
        return (a - b + math.pi) % (2.0*math.pi) - math.pi

    def _in_sector(self, th):
        return abs(self._angdiff(th, self.sector_center)) <= (0.5 * self.sector_width)

    def _cost(self, r, th):
        """이전 타깃에 가까울수록 비용 낮게 → 점프 감소"""
        if self._prev_r is None:
            return r
        dr  = abs(r - self._prev_r)
        dth = abs(self._angdiff(th, self._prev_th))
        return r + self.w_dr * dr + self.w_dth * dth

    def _status_color(self, status):
        """상태별 색상: (r,g,b)"""
        if status == 'OK':
            return (0.2, 1.0, 0.2)     # 초록
        if status == 'FAR':
            return (1.0, 1.0, 0.0)     # 노랑
        if status == 'NEAR':
            return (1.0, 0.2, 0.2)     # 빨강
        if status == 'ADJUST':
            return (0.4, 0.7, 1.0)     # 하늘
        if status == 'NO_TARGET':
            return (0.8, 0.8, 0.8)     # 회색
        return (1.0, 1.0, 1.0)

    # ------------- RViz marker -------------
    def _publish_text_marker(self):
        if self._last_xy is None:
            return
        x, y = self._last_xy
        arr = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = self.frame_id
        clear.header.stamp = self.get_clock().now().to_msg()
        arr.markers.append(clear)

        txt = Marker()
        txt.header.frame_id = self.frame_id
        txt.header.stamp = clear.header.stamp
        txt.ns = "dk_text"
        txt.id = 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(x)
        txt.pose.position.y = float(y)
        txt.pose.position.z = 0.6
        txt.scale.z = 0.36
        txt.color.a = 1.0

        # --- 텍스트 구성: distance / status / both ---
        d_show = self._ema_d if self._ema_d is not None else self._raw_d
        status = self._last_cmd_str

        if self.marker_mode == 'distance':
            txt.text = f"{d_show:.2f} m" if d_show is not None else "—"
        elif self.marker_mode == 'status':
            txt.text = status if status is not None else "—"
        else:  # 'both'
            if d_show is not None and status is not None:
                txt.text = f"{d_show:.2f} m | {status}"
            elif d_show is not None:
                txt.text = f"{d_show:.2f} m"
            elif status is not None:
                txt.text = status
            else:
                txt.text = "—"

        # 상태 기반 색상
        if status is not None:
            cr, cg, cb = self._status_color(status)
        else:
            cr, cg, cb = (1.0, 1.0, 1.0)
        txt.color.r = cr; txt.color.g = cg; txt.color.b = cb

        txt.lifetime = Duration(sec=1)
        arr.markers.append(txt)

        self.marker_pub.publish(arr)

    def _clear_markers(self):
        arr = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    # ------------- core callbacks -------------
    def on_obstacles(self, msg: Obstacles):
        """원(표면거리) & 세그먼트(중점) 중 비용 최소 후보 1개 선택 → 미디안→EMA"""
        best = None  # (cost, r_eff, x, y, th)

        # 1) circles: 다리 반경 게이팅 + 표면거리(center-radius)
        for c in msg.circles:
            x, y = c.center.x, c.center.y
            th = math.atan2(y, x)
            if not self._in_sector(th):
                continue

            r_center = math.hypot(x, y)
            c_rad = getattr(c, 'radius', None)
            if c_rad is not None:
                if not (self.leg_r_min <= c_rad <= self.leg_r_max):
                    continue
                r_eff = max(r_center - c_rad, 0.0)
            else:
                r_eff = r_center

            if not (self.r_min <= r_eff <= self.r_max):
                continue

            cost = self._cost(r_eff, th)
            if best is None or cost < best[0]:
                best = (cost, r_eff, x, y, th)

        # 2) segments: 중점 사용
        for s in msg.segments:
            mx = 0.5*(s.first_point.x + s.last_point.x)
            my = 0.5*(s.first_point.y + s.last_point.y)
            th = math.atan2(my, mx)
            if not self._in_sector(th):
                continue

            r_eff = math.hypot(mx, my)
            if not (self.r_min <= r_eff <= self.r_max):
                continue

            cost = self._cost(r_eff, th)
            if best is None or cost < best[0]:
                best = (cost, r_eff, mx, my, th)

        if best is None:
            self._last_xy = None
            self._prev_xy = None
            self._prev_r  = None
            self._prev_th = None
            self._clear_markers()
            return

        _, r_eff, bx, by, bth = best

        # 3) 미디안 + EMA
        self._med_buf.append(r_eff)
        if len(self._med_buf) > self.med_N:
            self._med_buf.pop(0)
        r_med = sorted(self._med_buf)[len(self._med_buf)//2]

        self._raw_d = r_med
        self._ema_d = r_med if self._ema_d is None else (self.alpha * r_med + (1.0 - self.alpha) * self._ema_d)

        # 4) 상태/마커/기억 갱신
        self._last_stamp = self.get_clock().now()
        self._last_xy = (bx, by)
        self._prev_xy = (bx, by)
        self._prev_r  = r_eff
        self._prev_th = bth

        # 즉시 최신 텍스트 갱신
        self._publish_text_marker()

    def _decide(self, d):
        up = self.target + self.tol_wide
        lo = self.target - self.tol_wide
        ok_hi = self.target + self.tol_ok
        ok_lo = self.target - self.tol_ok
        if d > up:              return "FAR"     # 멀다 → 앞으로
        if d < lo:              return "NEAR"    # 가깝다 → 뒤로/정지
        if ok_lo <= d <= ok_hi: return "OK"      # 유지
        return "ADJUST"                          # 미세 조정

    def tick(self):
        now = self.get_clock().now()
        cmd = String(); err = Float32()

        # 타임아웃: 타깃 없음
        if self._last_stamp is None or (now - self._last_stamp).nanoseconds * 1e-9 > self.max_age:
            cmd.data = "NO_TARGET"; err.data = 0.0
            self._last_cmd_str = cmd.data
            self.cmd_pub.publish(cmd); self.err_pub.publish(err)
            self._clear_markers()
            return

        d = self._ema_d if self._ema_d is not None else self._raw_d
        cmd.data = self._decide(d)
        err.data = float(d - self.target)
        self._last_cmd_str = cmd.data

        self.cmd_pub.publish(cmd)
        self.err_pub.publish(err)

        # RViz 텍스트도 주기적으로 최신 상태로 갱신
        self._publish_text_marker()

def main(args=None):
    rclpy.init(args=args)
    node = DistanceKeeper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
