#!/usr/bin/env python3
import os
import cv2
import yaml
import time
import math
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# YAML 저장 (camera_calibration_parsers 포맷)
def save_yaml(path, width, height, camera_name, K, D, R, P, distortion_model='plumb_bob'):
    data = {
        'image_width': int(width),
        'image_height': int(height),
        'camera_name': str(camera_name),
        'camera_matrix': {
            'rows': 3, 'cols': 3, 'data': K.reshape(-1).tolist()
        },
        'distortion_model': distortion_model,
        'distortion_coefficients': {
            'rows': 1, 'cols': len(D), 'data': D.reshape(-1).tolist()
        },
        'rectification_matrix': {
            'rows': 3, 'cols': 3, 'data': R.reshape(-1).tolist()
        },
        'projection_matrix': {
            'rows': 3, 'cols': 4, 'data': P.reshape(-1).tolist()
        },
    }
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        yaml.dump(data, f, sort_keys=False)
    return path

class CalibratorNode(Node):
    def __init__(self, args):
        super().__init__('camera_calibrator')

        self.args = args
        self.bridge = CvBridge()
        self.last_img_bgr = None
        self.last_stamp = None
        self.img_size = None  # (w, h)

        # 체커보드(내부 코너 개수)와 월드 좌표 생성
        self.pattern_size = (args.cols, args.rows)  # (cols, rows) = (nx, ny) inner corners
        self.square_size = float(args.square)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 3D-2D 대응점 누적 버퍼
        self.objpoints = []   # 3D
        self.imgpoints = []   # 2D

        # 한 샷에 사용할 3D 그리드 (Z=0 평면)
        objp = np.zeros((self.pattern_size[1] * self.pattern_size[0], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.template_objp = objp

        # 구독자
        self.image_sub = self.create_subscription(
            Image, self.args.image_topic, self.on_image, 10
        )

        self.get_logger().info(
            f"Calibrator started. topic='{self.args.image_topic}', pattern={self.pattern_size}, "
            f"square={self.square_size} m"
        )
        self.get_logger().info("Keys: [c]=capture, [r]=reset, [s]=solve+save, [q]=quit")

    def on_image(self, msg: Image):
        try:
            # 원하는 입력 인코딩(BGR8 권장)
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.args.encoding)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge convert failed: {e}")
            return

        self.last_img_bgr = img
        self.last_stamp = msg.header.stamp
        h, w = img.shape[:2]
        self.img_size = (w, h)

    def draw_status(self, img):
        """오버레이 텍스트/안내"""
        s = img.copy()
        n = len(self.imgpoints)
        cv2.putText(s, f"samples: {n}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(s, "[c] capture  [r] reset  [s] solve+save  [q] quit",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        return s

    def find_and_draw(self, img):
        """체커보드 찾고 그리기(발견 여부, 코너들, 그린 프레임)"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        disp = img.copy()

        if ret:
            # 서브픽셀 보정
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            cv2.drawChessboardCorners(disp, self.pattern_size, corners, ret)
            return True, corners, disp
        else:
            return False, None, disp

    def add_sample(self, corners):
        self.objpoints.append(self.template_objp.copy())
        self.imgpoints.append(corners.reshape(-1, 2))
        self.get_logger().info(f"Captured sample #{len(self.imgpoints)}")

    def reset(self):
        self.objpoints.clear()
        self.imgpoints.clear()
        self.get_logger().info("Samples cleared.")

    def solve_and_save(self):
        if not self.imgpoints or not self.objpoints or self.img_size is None:
            self.get_logger().warn("Not enough samples or unknown image size.")
            return False

        w, h = self.img_size
        # 초기 K 추정: 초점 대충 중간값
        K = np.array([[0.8*w, 0,     0.5*w],
                      [0,     0.8*h, 0.5*h],
                      [0,     0,     1.0]], dtype=np.float64)
        dist = np.zeros((5, 1), dtype=np.float64)

        flags = 0  # 필요 시 cv2.CALIB_FIX_ASPECT_RATIO 등 추가 가능
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=self.objpoints,
            imagePoints=self.imgpoints,
            imageSize=(w, h),
            cameraMatrix=K,
            distCoeffs=dist,
            flags=flags
        )

        # 재투영 오차
        total_err = 0.0
        total_pts = 0
        for i, (objp, imgp) in enumerate(zip(self.objpoints, self.imgpoints)):
            proj, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], K, dist)
            e = cv2.norm(imgp.reshape(-1, 1, 2), proj, cv2.NORM_L2)
            total_err += e*e
            total_pts += len(objp)
        rmse = math.sqrt(total_err / max(total_pts, 1))
        self.get_logger().info(f"Calibration done. Reprojection RMSE: {rmse:.3f} px")

        # rectification & projection (mono는 R=I, P=[K|0])
        R = np.eye(3, dtype=np.float64)
        P = np.zeros((3, 4), dtype=np.float64)
        P[:3, :3] = K

        out_path = self.args.output
        save_yaml(out_path, w, h, self.args.camera_name, K, dist.reshape(-1), R, P)
        self.get_logger().info(f"Saved YAML → {out_path}")
        return True

def main():
    parser = argparse.ArgumentParser(description="ROS2 chessboard camera calibrator")
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--encoding', default='bgr8', help='desired cv_bridge encoding (bgr8/rgb8)')
    parser.add_argument('--rows', type=int, required=True, help='inner corners (rows, short side)')
    parser.add_argument('--cols', type=int, required=True, help='inner corners (cols, long side)')
    parser.add_argument('--square', type=float, required=True, help='square size in meters')
    parser.add_argument('--camera-name', default='usb_cam')
    parser.add_argument('--output', default=os.path.expanduser('~/.ros/camera_info/usb_cam.yaml'))
    args = parser.parse_args()

    rclpy.init()
    node = CalibratorNode(args)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            frame = node.last_img_bgr
            if frame is None:
                time.sleep(0.01)
                continue

            found, corners, disp = node.find_and_draw(frame)
            disp = node.draw_status(disp)

            if found:
                cv2.putText(disp, "CHESSBOARD FOUND (press 'c' to capture)", (10, disp.shape[0]-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(disp, "Show chessboard clearly", (10, disp.shape[0]-15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.imshow("calibrator", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                node.reset()
            elif key == ord('c') and found and corners is not None:
                node.add_sample(corners)
            elif key == ord('s'):
                if node.solve_and_save():
                    break

    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
