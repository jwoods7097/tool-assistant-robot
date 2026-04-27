import cv2
import numpy as np
import requests
from ultralytics import YOLO
from cv2 import aruco

STREAM_URL = "http://172.20.10.9/stream"

# Load YOUR custom trained model
model = YOLO("best.pt")

# Initialize ArUco detector
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary)

# Marker size (meters)
MARKER_SIZE = 0.023
HALF = MARKER_SIZE / 2

# Define marker center positions (meters)
# 🔧 EDIT THESE to match your real setup
marker_centers = {
    0: (0.0215, 0.1985),
    1: (0.2865, 0.1995),
    2: (0.0225, 0.0195),
    3: (0.2875, 0.0195),
}

H = None  # Homography

print("Connecting to stream...")
stream = requests.get(STREAM_URL, stream=True, timeout=100)
bytes_data = b""

for chunk in stream.iter_content(chunk_size=1024):
    bytes_data += chunk
    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes_data[a:b+2]
        bytes_data = bytes_data[b+2:]
        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            # Convert to grayscale (optional but helps)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect markers
            corners, ids, rejected = detector.detectMarkers(gray)
            
            # Detect tools using YOLO
            results = model(frame, verbose=False, conf=0.5)
            frame = results[0].plot()

            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)

                world_points = []
                image_points = []

                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id not in marker_centers:
                        continue

                    cx, cy = marker_centers[marker_id]

                    # World corners (must match OpenCV order)
                    marker_world = np.array([
                        [cx - HALF, cy - HALF],  # top-left
                        [cx + HALF, cy - HALF],  # top-right
                        [cx + HALF, cy + HALF],  # bottom-right
                        [cx - HALF, cy + HALF],  # bottom-left
                    ], dtype=np.float32)

                    marker_img = corners[i][0].astype(np.float32)

                    world_points.append(marker_world)
                    image_points.append(marker_img)

                if len(world_points) == 4:  # Need all 4 markers for homography
                    world_points = np.vstack(world_points)
                    image_points = np.vstack(image_points)

                    # Compute homography
                    H, _ = cv2.findHomography(image_points, world_points)

            # Print detected tools
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                label = model.names[cls_id]
                conf = float(box.conf[0])

                if H is not None:
                    u, v, _, _ = box.xywh[0]
                    pt = np.array([[[u, v]]], dtype=np.float32)
                    world_pt = cv2.perspectiveTransform(pt, H)
                    x, y = world_pt[0][0]

                    print(f"Detected: {label} ({conf:.2f}) at ({u:.1f}, {v:.1f}) pixels → ({x:.3f}, {y:.3f}) meters")

            H = None  # Reset homography each frame (recompute if markers detected)
            
            cv2.imshow("Foam Tool Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()