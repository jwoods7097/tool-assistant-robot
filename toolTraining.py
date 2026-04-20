import cv2
import numpy as np
import requests
import os

STREAM_URL = "http://192.168.0.159/stream"

# Change this for each tool: "hammer", "screwdriver", "wrench", "pliers"
TOOL_NAME = "screwdriver"

save_dir = f"dataset/{TOOL_NAME}"
os.makedirs(save_dir, exist_ok=True)
count = len(os.listdir(save_dir))

print(f"Capturing images for: {TOOL_NAME}")
print("Press SPACE to save image, Q to quit")

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
            cv2.putText(frame, f"{TOOL_NAME} | saved: {count} | SPACE=save Q=quit", 
                       (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            cv2.imshow("Capture", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                path = f"{save_dir}/{TOOL_NAME}_{count:04d}.jpg"
                cv2.imwrite(path, frame)
                count += 1
                print(f"Saved {path}")
            elif key == ord('q'):
                break

cv2.destroyAllWindows()