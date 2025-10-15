import os, sys, argparse
import numpy as np
import torch, cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
from rclpy.utilities import remove_ros_args

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # -------- 파라미터 선언 --------
        p = self.declare_parameter
        self.weights = p('weights', '').get_parameter_value().string_value
        self.repo_path = p('repo_path', '').get_parameter_value().string_value
        self.device = p('device', 'cpu').get_parameter_value().string_value
        self.img_size = int(p('img_size', 640).get_parameter_value().integer_value or 640)
        self.conf_thres = float(p('conf_thres', 0.25).get_parameter_value().double_value or 0.25)
        self.iou_thres = float(p('iou_thres', 0.45).get_parameter_value().double_value or 0.45)
        self.max_det = int(p('max_det', 100).get_parameter_value().integer_value or 100)
        self.stride = int(p('stride', 1).get_parameter_value().integer_value or 1)

        # 키 호환: (런치에 맞춤) subscribe_image/publish_* 와 (노드 기본) *_topic 모두 지원
        source_topic = p('source_topic', '').get_parameter_value().string_value
        subscribe_image = p('subscribe_image', '').get_parameter_value().string_value
        self.source_topic = subscribe_image or source_topic or '/camera/image_raw'

        annotated_topic = p('annotated_topic', '').get_parameter_value().string_value
        publish_overlay = p('publish_overlay', '').get_parameter_value().string_value
        self.annotated_topic = publish_overlay or annotated_topic or '/robot/yolo/annotated'

        detections_topic = p('detections_topic', '').get_parameter_value().string_value
        publish_boxes = p('publish_boxes', '').get_parameter_value().string_value
        self.dets_topic = publish_boxes or detections_topic or '/robot/yolo/detections'

        self.publish_annotated = p('publish_annotated', True).get_parameter_value().bool_value

        # -------- YOLOv5 import 경로 --------
        if self.repo_path and self.repo_path not in sys.path:
            sys.path.insert(0, self.repo_path)
        try:
            from models.common import DetectMultiBackend
            from utils.torch_utils import select_device
            from utils.augmentations import letterbox
            from utils.general import non_max_suppression, scale_boxes
        except Exception as e:
            self.get_logger().error(f"Failed to import yolov5 modules: {e}")
            raise
        self.DetectMultiBackend = DetectMultiBackend
        self.select_device = select_device
        self.letterbox = letterbox
        self.nms = non_max_suppression
        self.scale_boxes = scale_boxes

        # -------- 모델 로드 --------
        if not self.weights or not os.path.exists(self.weights):
            self.get_logger().warn(f"weights not found: '{self.weights}'. Set param 'weights' correctly.")
        self.bridge = CvBridge()
        self.dev = self.select_device(self.device)
        self.model = self.DetectMultiBackend(self.weights, device=self.dev, dnn=False, data=None, fp16=False)
        self.names = getattr(self.model, 'names', [str(i) for i in range(1000)])

        # -------- 통신 설정 --------
        qos = QoSProfile(depth=2,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, self.source_topic, self.on_image, qos)
        self.pub_dets = self.create_publisher(Detection2DArray, self.dets_topic, 10)
        self.pub_img  = self.create_publisher(Image, self.annotated_topic, 10) if self.publish_annotated else None

        self.frame_idx = 0
        self.get_logger().info(
            f"YOLO ready | weights={self.weights} device={self.device} "
            f"src={self.source_topic} dets={self.dets_topic} ann={self.annotated_topic}"
        )

    def on_image(self, msg: Image):
        self.frame_idx += 1
        if self.stride > 1 and (self.frame_idx % self.stride) != 0:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f"cv_bridge failed: {e}")
            return

        im = self.letterbox(cv_img, self.img_size, stride=32, auto=True)[0]
        im = im[:, :, ::-1].transpose(2, 0, 1)
        im = np.ascontiguousarray(im)
        im = torch.from_numpy(im).to(self.dev).float() / 255.0
        if im.ndimension() == 3:
            im = im.unsqueeze(0)

        with torch.no_grad():
            pred = self.model(im, augment=False, visualize=False)
            det = self.nms(pred, self.conf_thres, self.iou_thres, classes=None, max_det=self.max_det)[0]

        dets = Detection2DArray()
        dets.header = msg.header
        annotated = cv_img.copy()

        if len(det):
            det[:, :4] = self.scale_boxes(im.shape[2:], det[:, :4], cv_img.shape).round()
            for *xyxy, conf, cls in det.cpu().numpy():
                x1, y1, x2, y2 = map(float, xyxy)
                w, h = x2 - x1, y2 - y1
                d = Detection2D()
                d.header = msg.header
                d.bbox = BoundingBox2D()
                d.bbox.center.position.x = x1 + w/2.0
                d.bbox.center.position.y = y1 + h/2.0
                d.bbox.size_x = w; d.bbox.size_y = h
                hyp = ObjectHypothesisWithPose()
                cname = self.names[int(cls)] if int(cls) < len(self.names) else str(int(cls))
                hyp.hypothesis.class_id = cname
                hyp.hypothesis.score = float(conf)
                d.results.append(hyp)
                dets.detections.append(d)

                if self.publish_annotated:
                    cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                    cv2.putText(annotated, f"{cname} {conf:.2f}", (int(x1), max(0,int(y1)-5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        self.pub_dets.publish(dets)
        if self.publish_annotated and self.pub_img and self.pub_img.get_subscription_count() > 0:
            try:
                self.pub_img.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))
            except Exception as e:
                self.get_logger().warning(f"annotated publish failed: {e}")

def main():
    # ROS 인자 제거 → argparse가 종료하지 않도록
    clean_argv = remove_ros_args(sys.argv)

    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    # 필요하면 여기서 스탠드얼론 실행용 CLI 옵션 추가 (선택)
    # parser.add_argument('--dummy', action='store_true')

    # 알 수 없는 인자는 무시(ROS 관련 잔여 인자 안전 처리)
    _, _unknown = parser.parse_known_args(clean_argv[1:])

    rclpy.init(args=None)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
