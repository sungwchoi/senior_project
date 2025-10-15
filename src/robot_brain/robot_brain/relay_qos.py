# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# from sensor_msgs.msg import LaserScan

# class QoSRelay(Node):
#     def __init__(self):
#         super().__init__('qos_relay')

        
#         qos_sub = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )
        
#         # /scan 구독 (기본 Reliable)
#         self.sub = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.callback,
#             qos_sub
#         )

#         # /scan_filtered 발행 (Best Effort)
#         qos_pub = QoSProfile(
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )
#         self.pub = self.create_publisher(LaserScan, '/scan_filtered', qos_pub)

#         self.get_logger().info("QoS Relay started: /scan (Best Effort) → /scan_filtered (Reliable)")

#     def callback(self, msg: LaserScan):
#         self.pub.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = QoSRelay()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

class QoSRelay(Node):
    def __init__(self):
        # 노드 이름은 런치에서 안 써도 되지만 관례상 relay_qos로 맞춰둠
        super().__init__('relay_qos')

        # 1) 토픽 파라미터 선언 및 읽기 (기본값은 상대 이름으로 두면 네임스페이스가 자동 적용됨)
        in_topic_param  = self.declare_parameter('in_topic',  'scan').get_parameter_value().string_value
        out_topic_param = self.declare_parameter('out_topic', 'scan_reliable').get_parameter_value().string_value

        # 2) QoS 설정: 구독=BestEffort(센서), 퍼블=Reliable
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 3) 구독/퍼블 생성 (파라미터 토픽 사용)
        self.sub = self.create_subscription(
            LaserScan,
            in_topic_param,
            self.callback,
            sub_qos
        )
        self.pub = self.create_publisher(LaserScan, out_topic_param, pub_qos)

        # 4) 실제 적용된 토픽명을 로그로 출력(네임스페이스 포함해서 보기 좋게)
        self.get_logger().info(
            f"QoS Relay started: {self.get_namespace()}/{in_topic_param} (BestEffort) → "
            f"{self.get_namespace()}/{out_topic_param} (Reliable)"
        )

    def callback(self, msg: LaserScan):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
