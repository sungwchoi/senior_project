import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

class QoSRelay(Node):
    def __init__(self):
        super().__init__('qos_relay')

        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # /scan 구독 (기본 Reliable)
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            qos_sub
        )

        # /scan_filtered 발행 (Best Effort)
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', qos_pub)

        self.get_logger().info("QoS Relay started: /scan (Best Effort) → /scan_filtered (Reliable)")

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
