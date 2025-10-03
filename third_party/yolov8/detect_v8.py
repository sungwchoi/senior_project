import cv2
from ultralytics import YOLO

# 1. YOLOv8n 모델 로드 (처음 실행 시 자동으로 다운로드됩니다)
model = YOLO('yolov8n.pt')

# 2. 웹캠 열기 (0은 첫 번째 웹캠을 의미합니다)
cap = cv2.VideoCapture(0)

# 3. 웹캠에서 프레임을 계속 읽어와서 처리
while cap.isOpened():
    # 프레임 읽기
    success, frame = cap.read()

    if success:
        # 4. YOLOv8로 객체 탐지 수행
        results = model(frame)

        # 5. 결과 영상 가져오기 (바운딩 박스가 그려진 영상)
        annotated_frame = results[0].plot()

        # 6. 결과 영상 보여주기
        cv2.imshow("YOLOv8 Detection", annotated_frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # 프레임을 더 이상 읽을 수 없으면 종료
        break

# 7. 자원 해제
cap.release()
cv2.destroyAllWindows()
