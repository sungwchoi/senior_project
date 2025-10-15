#!/usr/bin/env python3
import cv2, numpy as np, yaml, argparse, time, os

def save_ros_yaml(path, cam_name, w, h, K, D, R, P, model="plumb_bob"):
    data = {
        "image_width": int(w), "image_height": int(h),
        "camera_name": cam_name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": K.reshape(-1).tolist()},
        "distortion_model": model,
        "distortion_coefficients": {"rows": 1, "cols": len(D), "data": D.reshape(-1).tolist()},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": R.reshape(-1).tolist()},
        "projection_matrix": {"rows": 3, "cols": 4, "data": P.reshape(-1).tolist()},
        "binning_x": 0, "binning_y": 0,
        "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False},
    }
    os.makedirs(os.path.dirname(os.path.expanduser(path)), exist_ok=True)
    with open(os.path.expanduser(path), "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"[OK] Saved calibration to: {path}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--size", type=str, default="640x480", help="e.g. 640x480")
    ap.add_argument("--rows", type=int, required=True, help="inner corners (rows)")
    ap.add_argument("--cols", type=int, required=True, help="inner corners (cols)")
    ap.add_argument("--square", type=float, required=True, help="square size in meters")
    ap.add_argument("--name", default="usb_cam", help="camera_name in YAML")
    ap.add_argument("--output", default="~/.ros/camera_info/usb_cam.yaml")
    ap.add_argument("--max", type=int, default=20, help="max captures")
    args = ap.parse_args()

    w_req, h_req = map(int, args.size.lower().split("x"))
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {args.device}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w_req)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h_req)

    pattern_size = (args.cols, args.rows)  # (cols, rows)
    sq = args.square
    objp = np.zeros((args.rows*args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * sq

    objpoints, imgpoints = [], []
    win = "calibration"
    print("[i] Keys: [c]=capture, [r]=reset, [s]=solve+save, [q]=quit")
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01); continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size,
                        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        vis = frame.copy()
        msg = f"found={found}  samples={len(imgpoints)} / {args.max}"
        if found:
            cv2.drawChessboardCorners(vis, pattern_size, corners, found)
        cv2.putText(vis, msg, (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0) if found else (0,0,255), 2)
        cv2.imshow(win, vis)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('r'):
            objpoints.clear(); imgpoints.clear()
            print("[i] reset")
        elif k == ord('c') and found:
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            objpoints.append(objp.copy())
            imgpoints.append(corners2)
            print(f"[i] captured #{len(imgpoints)}")
            if len(imgpoints) >= args.max:
                print("[i] reached max samples")
        elif k == ord('s'):
            if len(imgpoints) < 8:
                print("[!] need at least ~8 good views")
                continue
            ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            print(f"[i] reprojection error: {ret:.4f}")
            h, w = gray.shape[:2]
            R = np.eye(3, dtype=np.float64)
            P = np.zeros((3,4), dtype=np.float64)
            P[:3,:3] = K
            save_ros_yaml(args.output, args.name, w, h, K, D, R, P)
            print("[i] saved. press 'q' to quit or continue capturing.")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
