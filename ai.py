import cv2
import time
import queue
import threading
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2

# -----------------------------
# Config
# -----------------------------
MODEL_PATH = "yolov8n.pt"   # jouw getrainde bierglas-model
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CONF_THRESHOLD = 0.25
FRAME_SKIP = 2  # detectie op elke 2e frame voor hogere FPS

# -----------------------------
# Camera setup
# -----------------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "BGR888"}
)
picam2.configure(config)
picam2.start()
time.sleep(1)

# -----------------------------
# Thread-safe queues
# -----------------------------
frame_queue = queue.Queue(maxsize=2)
result_queue = queue.Queue(maxsize=2)

# -----------------------------
# YOLO model (CPU)
# -----------------------------
model = YOLO(MODEL_PATH)
model.to("cpu")

# -----------------------------
# Capture thread
# -----------------------------
def capture_thread():
    while True:
        frame = picam2.capture_array()
        if not frame_queue.full():
            frame_queue.put(frame)

# -----------------------------
# Inference thread
# -----------------------------
def inference_thread():
    frame_count = 0
    while True:
        if not frame_queue.empty():
            frame = frame_queue.get()
            frame_count += 1
            # detecteer alleen elke FRAME_SKIP frames
            if frame_count % FRAME_SKIP != 0:
                continue
            results = model(frame)
            detections = []
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    if conf < CONF_THRESHOLD:
                        continue
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    detections.append((x1, y1, x2, y2, conf))
            if not result_queue.full():
                result_queue.put((frame, detections))

# -----------------------------
# Pixel filter: alleen geel & wit zichtbaar
# -----------------------------
def filter_yellow_white(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Geel
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Wit
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    
    mask = cv2.bitwise_or(mask_yellow, mask_white)
    filtered = cv2.bitwise_and(frame, frame, mask=mask)
    return filtered

# -----------------------------
# Start threads
# -----------------------------
threading.Thread(target=capture_thread, daemon=True).start()
threading.Thread(target=inference_thread, daemon=True).start()

print("CPU YOLOv8n-tiny gestart, druk 'q' om te stoppen")

# -----------------------------
# Display loop
# -----------------------------
while True:
    if not result_queue.empty():
        frame, detections = result_queue.get()
        
        # Filter geel/wit pixels
        frame = filter_yellow_white(frame)
        
        # Teken detecties
        for x1, y1, x2, y2, conf in detections:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{conf:.2f}", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        
        cv2.imshow("Bierglas Detectie", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
picam2.stop()
