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
            
            # Print detected tools
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                label = model.names[cls_id]
                conf = float(box.conf[0])
                print(f"Detected: {label} ({conf:.2f})")

            if ids is not None:
                # Draw markers
                aruco.drawDetectedMarkers(frame, corners, ids)

                for i, marker_id in enumerate(ids):
                    # Get corners for this marker
                    pts = corners[i][0]  # shape (4,2)

                    # Compute center
                    center = np.mean(pts, axis=0)
                    cx, cy = int(center[0]), int(center[1])

                    # Draw center
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                    # Print info
                    print(f"Marker {marker_id[0]} at pixel ({cx}, {cy})")

                    # Label on screen
                    cv2.putText(
                        frame,
                        f"ID {marker_id[0]}",
                        (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )
            
            cv2.imshow("Foam Tool Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()