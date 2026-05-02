import cv2
import numpy as np
import requests
import serial
import time
from ultralytics import YOLO
from cv2 import aruco

# =========================
# CONFIG
# =========================

STREAM_URL = "http://192.168.0.159/stream"
MODEL_PATH = "best.pt"

ARDUINO_PORT = "COM5"
BAUD_RATE = 115200

CONFIDENCE = 0.5

MARKER_SIZE = 0.023
HALF = MARKER_SIZE / 2

marker_centers = {
    0: (0.0215, 0.1985),
    1: (0.2865, 0.1995),
    2: (0.0225, 0.0195),
    3: (0.2875, 0.0195),
}

# calibrated from your test
ROBOT_BASE_X = 0.160
ROBOT_BASE_Y = -0.028

FLIP_X = False
FLIP_Y = False

# target point for perfect pick
TARGET_X = 0
TARGET_Y = 120

ERROR_THRESHOLD_MM = 12
MAX_CORRECTIONS = 3
MOVE_WAIT = 4.0

# safety range
MIN_X_MM = -160
MAX_X_MM = 160
MIN_Y_MM = 60
MAX_Y_MM = 170

# =========================
# SETUP
# =========================

print("Loading YOLO model...")
model = YOLO(MODEL_PATH)

print("Connecting to Arduino...")
arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
time.sleep(2)
arduino.reset_input_buffer()
arduino.write(b"HOME\n")
time.sleep(1)

print("Connecting to ESP32 stream...")
stream = requests.get(STREAM_URL, stream=True, timeout=100)

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary)

bytes_data = b""

correction_count = 0
waiting_for_robot = False
last_move_time = 0
grabbed = False

print("System ready.")

# =========================
# HELPER
# =========================

def get_best_tool(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    H = None

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        world_points = []
        image_points = []

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in marker_centers:
                continue

            cx, cy = marker_centers[marker_id]

            marker_world = np.array([
                [cx - HALF, cy - HALF],
                [cx + HALF, cy - HALF],
                [cx + HALF, cy + HALF],
                [cx - HALF, cy + HALF],
            ], dtype=np.float32)

            marker_img = corners[i][0].astype(np.float32)

            world_points.append(marker_world)
            image_points.append(marker_img)

        if len(world_points) >= 4:
            world_points = np.vstack(world_points)
            image_points = np.vstack(image_points)
            H, _ = cv2.findHomography(image_points, world_points)

    results = model(frame, verbose=False, conf=CONFIDENCE)
    annotated = results[0].plot()

    if H is None:
        return None, annotated

    best_tool = None
    best_conf = 0

    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        label = model.names[cls_id]
        conf = float(box.conf[0])

        u, v, _, _ = box.xywh[0]

        pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
        world_pt = cv2.perspectiveTransform(pt, H)

        world_x, world_y = world_pt[0][0]

        robot_x_m = world_x - ROBOT_BASE_X
        robot_y_m = world_y - ROBOT_BASE_Y

        if FLIP_X:
            robot_x_m = -robot_x_m

        if FLIP_Y:
            robot_y_m = -robot_y_m

        x_mm = int(robot_x_m * 1000)
        y_mm = int(robot_y_m * 1000)

        if conf > best_conf:
            best_conf = conf
            best_tool = {
                "label": label,
                "conf": conf,
                "x_mm": x_mm,
                "y_mm": y_mm,
                "world_x": world_x,
                "world_y": world_y,
            }

    return best_tool, annotated


# =========================
# MAIN LOOP
# =========================

for chunk in stream.iter_content(chunk_size=1024):
    bytes_data += chunk

    a = bytes_data.find(b'\xff\xd8')
    b = bytes_data.find(b'\xff\xd9')

    if a == -1 or b == -1:
        continue

    jpg = bytes_data[a:b + 2]
    bytes_data = bytes_data[b + 2:]

    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

    if frame is None:
        continue

    best_tool, annotated_frame = get_best_tool(frame)

    now = time.time()

    if best_tool is not None and not grabbed:
        x_mm = best_tool["x_mm"]
        y_mm = best_tool["y_mm"]

        err_x = x_mm - TARGET_X
        err_y = y_mm - TARGET_Y

        print(
            f"Detected {best_tool['label']} ({best_tool['conf']:.2f}) "
            f"robot=({x_mm},{y_mm}) mm error=({err_x},{err_y})"
        )

        if (
            MIN_X_MM <= x_mm <= MAX_X_MM and
            MIN_Y_MM <= y_mm <= MAX_Y_MM
        ):

            if not waiting_for_robot:
                if abs(err_x) <= ERROR_THRESHOLD_MM and abs(err_y) <= ERROR_THRESHOLD_MM:
                    cmd = f"PICK {x_mm} {y_mm}\n"
                    arduino.write(cmd.encode())
                    print("Close enough. Sending PICK:", cmd.strip())
                    grabbed = True

                elif correction_count < MAX_CORRECTIONS:
                    cmd = f"MOVE {x_mm} {y_mm}\n"
                    arduino.write(cmd.encode())

                    print("Correction move:", cmd.strip())

                    correction_count += 1
                    waiting_for_robot = True
                    last_move_time = now

                else:
                    print("Max corrections reached. Sending GRAB.")
                    arduino.write(b"GRAB\n")
                    grabbed = True

            else:
                if now - last_move_time > MOVE_WAIT:
                    waiting_for_robot = False

        else:
            print("Tool outside safe robot range.")

    else:
        if not grabbed:
            print("No valid tool / ArUco missing.")

    cv2.imshow("Visual Servo Tool Pick", annotated_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if key == ord('r'):
        print("Resetting correction loop.")
        correction_count = 0
        waiting_for_robot = False
        grabbed = False
        arduino.write(b"HOME\n")

    if key == ord('g'):
        print("Manual GRAB.")
        arduino.write(b"GRAB\n")
        grabbed = True

cv2.destroyAllWindows()
arduino.close()