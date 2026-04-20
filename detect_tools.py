import cv2
import numpy as np
import requests
from ultralytics import YOLO

STREAM_URL = "http://192.168.0.159/stream"

# Load YOUR custom trained model
model = YOLO("best.pt")

print("Connecting to stream...")
stream = requests.get(STREAM_URL, stream=True, timeout=10)
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
            results = model(frame, verbose=False, conf=0.5)
            annotated = results[0].plot()
            
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                label = model.names[cls_id]
                conf = float(box.conf[0])
                print(f"Detected: {label} ({conf:.2f})")
            
            cv2.imshow("Foam Tool Detection", annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()