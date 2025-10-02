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
    def __init__(self):
        super().__init__('distance_keeper')

        # --- Parameters ---
        p = self.declare_parameter
        self.target   = p('target_distance', 1.5).get_parameter_value().double_value
        self.tol_ok   = p('tol_ok', 0.20).get_parameter_value().double_value
        self.tol_wide = p('tol_slow', 0.35).get_parameter_value().double_value
        self.r_min    = p('min_detect_radius', 0.30).get_parameter_value().double_value
        self.r_max    = p('max_detect_radius', 3.50).get_parameter_value().double_value
        self.alpha    = p('ema_alpha', 0.35).get_parameter_value().double_value
        self.max_age  = p('max_age_sec', 0.8).get_parameter_value().double_value
        self.rate_hz  = p('publish_rate_hz', 15.0).get_parameter_value().double_value

        # 섹터: -45°~+45° 쓰려면 center=0.0, width=pi/2(=1.570796)
        self.sector_center = p('sector_center', 0.0).get_parameter_value().double_value
        self.sector_width  = p('sector_width', 1.570796).get_parameter_value().double_value

        # RViz 마커용 frame
        self.frame_id = p('frame_id', 'velodyne').get_parameter_value().string_value

        # --- I/O ---
        qos_rel = QoSProfile(depth=10,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Obstacles, '/obstacles', self.on_obstacles, qos_rel)
        self.cmd_pub = self.create_publisher(String,  '/follow_cmd', 10)
        self.err_pub = self.create_publisher(Float32, '/range_error', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/distance_markers', 10)

        # --- State ---
        self._last_stamp = None
        self._raw_d = None
        self._ema_d = None
        self._last_xy = None  # 텍스트 표시 위치

        self._timer = self.create_timer(max(1.0/self.rate_hz, 0.02), self.tick)

    # angle helpers
    @staticmethod
    def _angdiff(a, b):
        return (a - b + math.pi) % (2.0*math.pi) - math.pi

    def _in_sector(self, th):
        return abs(self._angdiff(th, self.sector_center)) <= (0.5 * self.sector_width)

    def on_obstacles(self, msg: Obstacles):
        """가장 가까운 물체(원 중심 + 세그먼트 중점) 하나만 선택"""
        closest = None
        best_xy = None

        # circles
        for c in msg.circles:
            x, y = c.center.x, c.center.y
            r = math.hypot(x, y)
            if self.r_min <= r <= self.r_max and self._in_sector(math.atan2(y, x)):
                if closest is None or r < closest:
                    closest = r
                    best_xy = (x, y)

        # segments (midpoint)
        for s in msg.segments:
            mx = 0.5*(s.first_point.x + s.last_point.x)
            my = 0.5*(s.first_point.y + s.last_point.y)
            r = math.hypot(mx, my)
            if self.r_min <= r <= self.r_max and self._in_sector(math.atan2(my, mx)):
                if closest is None or r < closest:
                    closest = r
                    best_xy = (mx, my)

        if closest is not None:
            self._raw_d = closest
            self._ema_d = closest if self._ema_d is None else (self.alpha*closest + (1.0-self.alpha)*self._ema_d)
            self._last_stamp = self.get_clock().now()
            self._last_xy = best_xy
            self._publish_text_marker()  # 라이트 텍스트 마커 1개만
        else:
            # 타겟 없음 → 기존 마커 제거
            self._clear_markers()

    # 라이트 텍스트 마커 발행(1개)
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
        txt.scale.z = 0.32
        txt.color.a = 1.0
        txt.color.r = 1.0
        txt.color.g = 1.0
        txt.color.b = 1.0
        d_show = self._ema_d if self._ema_d is not None else self._raw_d
        txt.text = f"{d_show:.2f} m"
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

    def _decide(self, d):
        up = self.target + self.tol_wide
        lo = self.target - self.tol_wide
        ok_hi = self.target + self.tol_ok
        ok_lo = self.target - self.tol_ok
        if d > up:                return "FAR"     # 멀다 → 앞으로
        if d < lo:                return "NEAR"    # 가깝다 → 뒤로/정지
        if ok_lo <= d <= ok_hi:   return "OK"      # 유지
        return "ADJUST"                           # 미세 조정 영역

    def tick(self):
        now = self.get_clock().now()
        cmd = String(); err = Float32()

        # 타임아웃: 타겟 없음
        if self._last_stamp is None or (now - self._last_stamp).nanoseconds * 1e-9 > self.max_age:
            cmd.data = "NO_TARGET"; err.data = 0.0
            self.cmd_pub.publish(cmd); self.err_pub.publish(err)
            self._clear_markers()
            return

        d = self._ema_d if self._ema_d is not None else self._raw_d
        cmd.data = self._decide(d)
        err.data = float(d - self.target)
        self.cmd_pub.publish(cmd)
        self.err_pub.publish(err)

def main(args=None):
    rclpy.init(args=args)
    node = DistanceKeeper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
