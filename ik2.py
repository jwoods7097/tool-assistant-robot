import math
import time
import serial

# -----------------------------
# Configuration
# -----------------------------
SERIAL_PORT = "COM3"      # Change to "/dev/ttyUSB0" on Linux/macOS
BAUD_RATE = 115200

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
    msg = f"{int(round(base))},{int(round(shoulder))},{int(round(elbow))},{int(round(90-GRIPPER_ANGLE_DEG))},90\n"
    ser.write(msg.encode("utf-8"))
    print("Sent:", msg.strip())

def to_robot(x, y):
    # Convert from world coordinates (meters) to robot coordinates (meters)
    # Assuming the robot's base is at (0, 0) and the workspace is in the positive quadrant
    return x - 15.3, y + 5.35, 0

def compensate_for_gripper(x, y, z):
    """
    Treat (x, y, z) as the desired gripper TIP position.
    Convert it to the wrist position by subtracting the gripper vector.

    Assumption:
    - The gripper points outward along the same horizontal direction as the target.
    - GRIPPER_ANGLE_DEG is measured above the xy plane.
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

    # Keep the same x/y direction, just shorten the radius
    scale = wrist_r / r
    wrist_x = x * scale
    wrist_y = y * scale

    return wrist_x, wrist_y, wrist_z

def transform_coordinates(x, y, z):
    # Flip z axis and apply base height offset
    z = -z - BASE_HEIGHT

    # Remap y so 0 is closest and 12 is farthest
    y = L1 + L2 - y

    return x, y, z

def inverse_kinematics(x, y):
    x, y, z = to_robot(x, y)

    # First back out the gripper tip offset
    x, y, z = compensate_for_gripper(x, y, z)

    # Then apply your coordinate transforms
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

    # Law of cosines for elbow
    cos_elbow = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_internal_rad = math.acos(cos_elbow)

    # Shoulder angle
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

# -----------------------------
# Main loop
# -----------------------------
def main():
    print("Opening serial port...")
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        time.sleep(2)
        print("Ready. Enter target coordinates as: x y")
        print("These coordinates are for the gripper tip.")
        print("Type 'q' to quit.\n")

        while True:
            line = input("Target x y > ").strip()
            if line.lower() in ("q", "quit", "exit"):
                break

            try:
                x, y = parse_xy(line)
                base, shoulder, elbow = inverse_kinematics(x, y)

                print(f"IK angles -> base={base:.1f}, shoulder={shoulder:.1f}, elbow={elbow:.1f}")
                send_angles(ser, base, shoulder, elbow)

            except Exception as e:
                print("Error:", e)

if __name__ == "__main__":
    main()