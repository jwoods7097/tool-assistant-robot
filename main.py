import cv2
import numpy as np
import requests
import math
import time
import serial
import threading
import queue
from ultralytics import YOLO
from cv2 import aruco

# -----------------------------
# Configuration
# -----------------------------
STREAM_URL = "http://172.20.10.9/stream"

SERIAL_PORT = "COM3"      # Change to "/dev/ttyUSB0" on Linux/macOS
BAUD_RATE = 115200

# Load your custom trained model
model = YOLO("best.pt")

# ArUco setup
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector = aruco.ArucoDetector(dictionary)

# Marker size (meters)
MARKER_SIZE = 0.023
HALF = MARKER_SIZE / 2

# Define marker center positions (meters)
# Edit these to match your real setup
marker_centers = {
    0: (0.0215, 0.1985),
    1: (0.2865, 0.1995),
    2: (0.0225, 0.0195),
    3: (0.2875, 0.0195),
}

# -----------------------------
# IK Configuration
# -----------------------------
BASE_HEIGHT = 10.1  # Distance from table to shoulder joint

# Arm dimensions in centimeters
L1 = 5.8   # upper arm length
L2 = 6.2   # forearm length

# Gripper geometry
GRIPPER_LENGTH = 10.4
GRIPPER_ANGLE_DEG = 60.0   # angle above the xy plane

# Servo calibration offsets
BASE_OFFSET = 0.0
SHOULDER_OFFSET = 90.0
ELBOW_OFFSET = 90.0

# Invert servos if your arm moves the wrong way
INVERT_BASE = False
INVERT_SHOULDER = False
INVERT_ELBOW = False


# -----------------------------
# Helpers
# -----------------------------
def clamp(value, lo=0.0, hi=180.0):
    return max(lo, min(hi, value))


def send_angles(ser, base, shoulder, elbow):
    msg = f"{int(round(base))},{int(round(shoulder))},{int(round(elbow))},{int(round(90 - GRIPPER_ANGLE_DEG))},90\n"
    ser.write(msg.encode("utf-8"))
    print("Sent:", msg.strip())


def send_reset(ser):
    msg = "90,90,90,90,90\n"
    ser.write(msg.encode("utf-8"))
    print("Sent reset:", msg.strip())


def to_robot(x, y):
    # Convert from world coordinates (centimeters) to robot coordinates (centimeters)
    return x - 15.3, y - 7.5, -2


def compensate_for_gripper(x, y, z):
    """
    Treat (x, y, z) as the desired gripper TIP position.
    Convert it to the wrist position by subtracting the gripper vector.
    """
    r = math.sqrt(x * x + y * y)

    if r == 0:
        raise ValueError("Target is directly above the base; gripper direction in xy is undefined.")

    horiz_offset = GRIPPER_LENGTH * math.cos(math.radians(GRIPPER_ANGLE_DEG))
    vert_offset = GRIPPER_LENGTH * math.sin(math.radians(GRIPPER_ANGLE_DEG))

    wrist_r = r - horiz_offset
    wrist_z = z - vert_offset

    if wrist_r < 0:
        raise ValueError("Target is too close for the gripper length and angle.")

    scale = wrist_r / r
    wrist_x = x * scale
    wrist_y = y * scale

    return wrist_x, wrist_y, wrist_z


def transform_coordinates(x, y, z):
    # Flip z axis
    z = -z

    # Remap y so 0 is closest and 12 is farthest
    y = L1 + L2 - y

    return x, y, z


def inverse_kinematics(x, y):
    x, y, z = to_robot(x, y)

    # Back out the gripper tip offset
    # x, y, z = compensate_for_gripper(x, y, z)

    # Apply coordinate transforms
    x, y, z = transform_coordinates(x, y, z)

    # Base rotation
    base_rad = math.atan2(y, x)
    base_deg = math.degrees(base_rad)

    # Distance from base axis to target in horizontal plane
    r = math.sqrt(x * x + y * y)

    # Distance from shoulder joint to target point
    d = math.sqrt(r * r + z * z)

    # Check reachability
    if d > (L1 + L2) or d < abs(L1 - L2):
        raise ValueError("Target is out of reach")

    # Elbow
    cos_elbow = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_internal_rad = math.acos(cos_elbow)

    # Shoulder
    cos_shoulder = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d)
    cos_shoulder = max(-1.0, min(1.0, cos_shoulder))
    shoulder_offset_rad = math.acos(cos_shoulder)
    target_angle_rad = math.atan2(z, r)

    shoulder_rad = target_angle_rad + shoulder_offset_rad

    # Convert to servo-friendly angles
    base = base_deg + BASE_OFFSET
    shoulder = math.degrees(shoulder_rad) + SHOULDER_OFFSET
    elbow = 180.0 - math.degrees(elbow_internal_rad) + ELBOW_OFFSET

    if INVERT_BASE:
        base = 180.0 - base
    if INVERT_SHOULDER:
        shoulder = 180.0 - shoulder
    if INVERT_ELBOW:
        elbow = 180.0 - elbow

    return clamp(base), clamp(shoulder), clamp(elbow)


def parse_xy(line):
    parts = line.strip().split()
    if len(parts) != 2:
        raise ValueError("Enter exactly 2 values: x y")
    return float(parts[0]), float(parts[1])


def compute_homography(frame):
    """
    Detect ArUco markers and compute homography if all 4 reference markers are visible.
    Returns: (H, corners, ids)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    H = None
    if ids is not None:
        world_points = []
        image_points = []

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id not in marker_centers:
                continue

            cx, cy = marker_centers[marker_id]

            marker_world = np.array([
                [cx - HALF, cy - HALF],  # top-left
                [cx + HALF, cy - HALF],  # top-right
                [cx + HALF, cy + HALF],  # bottom-right
                [cx - HALF, cy + HALF],  # bottom-left
            ], dtype=np.float32)

            marker_img = corners[i][0].astype(np.float32)

            world_points.append(marker_world)
            image_points.append(marker_img)

        if len(world_points) == 4:
            world_points = np.vstack(world_points)
            image_points = np.vstack(image_points)
            H, _ = cv2.findHomography(image_points, world_points)

    return H, corners, ids


def detect_requested_tool_once(frame, requested_tool):
    """
    Run YOLO once on the current frame and return the best matching detection
    for the requested tool label, or None if not found.
    """
    results = model(frame, verbose=False, conf=0.5)
    detections = results[0].boxes

    requested_tool = requested_tool.strip().lower()
    best = None

    for box in detections:
        cls_id = int(box.cls[0])
        label = model.names[cls_id]
        conf = float(box.conf[0])

        if label.strip().lower() != requested_tool:
            continue

        if best is None or conf > best["conf"]:
            u, v, _, _ = box.xywh[0].tolist()
            best = {
                "label": label,
                "conf": conf,
                "u": u,
                "v": v,
            }

    annotated = results[0].plot()
    return best, annotated


class StreamReader:
    """
    Continuously reads the MJPEG stream in a background thread and keeps
    the most recent decoded frame available.
    """
    def __init__(self, url):
        self.url = url
        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        self.stream = requests.get(self.url, stream=True, timeout=100)
        self.iterator = self.stream.iter_content(chunk_size=1024)
        self.bytes_data = b""

        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            try:
                chunk = next(self.iterator)
            except StopIteration:
                time.sleep(0.05)
                continue
            except Exception as e:
                print("Stream error:", e)
                time.sleep(0.1)
                continue

            self.bytes_data += chunk
            a = self.bytes_data.find(b"\xff\xd8")
            b = self.bytes_data.find(b"\xff\xd9")

            if a != -1 and b != -1 and b > a:
                jpg = self.bytes_data[a:b + 2]
                self.bytes_data = self.bytes_data[b + 2:]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                if frame is not None:
                    with self.lock:
                        self.frame = frame

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def stop(self):
        self.running = False


def input_worker(request_queue, stop_event):
    """
    Separate blocking terminal input loop so the stream window stays live.
    Commands:
      - tool name: detect and move if found
      - reset: send all servos to 90 degrees
      - q / quit / exit: stop program
    """
    print("Ready. Type a tool name and press Enter.")
    print("Type 'reset' to send all servos to 90 degrees.")
    print("Type 'q' to quit.\n")

    while not stop_event.is_set():
        try:
            line = input("Command or tool > ").strip()
        except EOFError:
            stop_event.set()
            break

        if not line:
            continue

        if line.lower() in ("q", "quit", "exit"):
            stop_event.set()
            break

        request_queue.put(line)


# -----------------------------
# Main loop
# -----------------------------
def main():
    print("Starting stream...")
    stream_reader = StreamReader(STREAM_URL)

    print("Opening serial port...")
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        time.sleep(2)

        request_queue = queue.Queue()
        stop_event = threading.Event()

        prompt_thread = threading.Thread(
            target=input_worker,
            args=(request_queue, stop_event),
            daemon=True
        )
        prompt_thread.start()

        last_status = "Waiting for a command..."
        last_homography = None

        while not stop_event.is_set():
            frame = stream_reader.get_frame()

            if frame is not None:
                display = frame.copy()

                cv2.putText(
                    display,
                    last_status,
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA
                )

                cv2.imshow("Foam Tool Detection", display)

            # Process any pending user request without blocking the stream
            try:
                command = request_queue.get_nowait()
            except queue.Empty:
                command = None

            if command is not None:
                lower = command.strip().lower()

                if lower == "reset":
                    send_reset(ser)
                    last_status = "Reset all servos to 90 degrees"
                    continue

                current_frame = stream_reader.get_frame()
                if current_frame is None:
                    last_status = "No frame available."
                    continue

                # Try to compute a fresh homography first
                H, corners, ids = compute_homography(current_frame)
                if H is not None:
                    last_homography = H

                # Run YOLO only once for this user request
                best_match, annotated = detect_requested_tool_once(current_frame, command)

                if ids is not None:
                    aruco.drawDetectedMarkers(annotated, corners, ids)

                if best_match is None:
                    print(f"'{command}' not detected in this frame. Doing nothing.")
                    last_status = f"'{command}' not detected."
                    cv2.imshow("Foam Tool Detection", annotated)
                    continue

                # Use the current homography if available, otherwise fall back to the last good one
                active_h = H if H is not None else last_homography

                if active_h is None:
                    print("Tool detected, but no valid homography is available yet.")
                    last_status = "Detected tool, but no homography."
                    cv2.imshow("Foam Tool Detection", annotated)
                    continue

                if H is None:
                    print("Using last saved homography because markers were not visible in this frame.")
                    last_status = "Using last saved homography"

                pt = np.array([[[best_match["u"], best_match["v"]]]], dtype=np.float32)
                world_pt = cv2.perspectiveTransform(pt, active_h)
                x_m, y_m = world_pt[0][0]

                print(
                    f"Detected: {best_match['label']} "
                    f"({best_match['conf']:.2f}) at "
                    f"({best_match['u']:.1f}, {best_match['v']:.1f}) pixels -> "
                    f"({x_m:.3f}, {y_m:.3f}) meters"
                )

                # Convert meters to centimeters for IK
                x_cm = x_m * 100.0
                y_cm = y_m * 100.0

                try:
                    base, shoulder, elbow = inverse_kinematics(x_cm, y_cm)
                    print(f"IK angles -> base={base:.1f}, shoulder={shoulder:.1f}, elbow={elbow:.1f}")
                    send_angles(ser, base, shoulder, elbow)
                    last_status = f"Moved to {best_match['label']}"
                except Exception as e:
                    print("IK error:", e)
                    last_status = f"IK error: {e}"

                cv2.imshow("Foam Tool Detection", annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                stop_event.set()
                break

            time.sleep(0.01)

    stream_reader.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()