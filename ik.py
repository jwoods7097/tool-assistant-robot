import math
import time
from serial import Serial
from dataclasses import dataclass
from typing import Optional, Dict

# Robot arm segment lengths (meters)
L1 = 0.101 # table to shoulder
L2 = 0.058 # shoulder to elbow
L3 = 0.062 # elbow to wrist
L4 = 0.075 # wrist to gripper center
L5 = 0.029 # gripper center to tip

# Serial port configuration
ARM_PORT = "COM3"
BAUD = 115200

# cam = Serial(CAM_PORT, BAUD, timeout=0.2)
arm = Serial(ARM_PORT, BAUD, timeout=0.2)
time.sleep(2)

def to_robot(x, y):
    # Convert from world coordinates (meters) to robot coordinates (meters)
    # Assuming the robot's base is at (0, 0) and the workspace is in the positive quadrant
    return x - 0.153, y + 0.0535

@dataclass(frozen=True)
class ServoCal:
    zero: float
    direction: float = 1.0
    min_value: float = 0.0
    max_value: float = 180.0

    def to_servo(self, joint_angle_rad: float) -> float:
        return self.zero + self.direction * math.degrees(joint_angle_rad)

# Default calibration assumptions:
# base=0 points along +x
# shoulder=180 means upper arm is horizontal
# elbow=90 means elbow is straight
# wrist=90 means gripper axis is straight with forearm
#
# Flip direction to -1 if a servo moves the wrong way.
DEFAULT_CAL = {
    "base":     ServoCal(0.0,  +1.0),
    "shoulder": ServoCal(180.0,  +1.0),
    "elbow":    ServoCal(90.0, +1.0),
    "wrist":    ServoCal(90.0, +1.0),
}

def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def estimate_max_gripper_center_radius(
    *,
    tip_z: float = 0.0,
    tool_angle_limits_deg=(-90.0, 0.0),
    samples: int = 3000,
):
    """
    Estimates max horizontal radius for the gripper center while the tip is at tip_z.

    tool_angle:
      -90 degrees = gripper points straight down
        0 degrees = gripper points horizontally outward
    """
    lo, hi = tool_angle_limits_deg
    if lo > hi:
        lo, hi = hi, lo

    best = None

    for i in range(samples):
        t = i / (samples - 1)
        tool_angle_deg = lo + (hi - lo) * t
        phi = math.radians(tool_angle_deg)

        # Wrist height relative to shoulder
        wrist_z = tip_z - (L4 + L5) * math.sin(phi)
        dz = wrist_z - L1

        remaining = (L2 + L3) ** 2 - dz ** 2
        if remaining < 0:
            continue

        # Max gripper-center radius for this tool angle.
        r = L4 * math.cos(phi) + math.sqrt(remaining)

        if best is None or r > best[0]:
            best = (r, tool_angle_deg)

    return best


def miniarm_ik_xy(
    x: float,
    y: float,
    *,
    gripper_servo: Optional[float] = None,
    tip_z: float = 0.0,

    # None means "choose automatically".
    # -90 means gripper points straight down.
    # 0 means gripper points horizontally outward.
    tool_angle_deg: Optional[float] = None,

    # Used only when tool_angle_deg is None.
    preferred_tool_angle_deg: float = -45.0,
    tool_angle_limits_deg=(-90.0, 0.0),
    angle_step_deg: float = 0.25,

    elbow: str = "up",
    servo_cal: Optional[Dict[str, ServoCal]] = None,
    check_limits: bool = True,
    return_debug: bool = False,
) -> Dict[str, float]:
    """
    IK for x, y gripper-center target, with gripper tip at z=tip_z.

    Coordinate system:
      x, y are horizontal table coordinates in meters.
      x=0, y=0 is centered on the base servo axis.
      z=0 is the table surface.

    Target:
      The gripper CENTER is placed at horizontal coordinate x, y.
      The gripper TIP is placed at z=tip_z.

    tool_angle_deg:
      Angle of the gripper/tool in the vertical arm plane.

      -90 degrees: gripper points straight down.
       0 degrees: gripper points horizontally outward.

      If tool_angle_deg=None, the function searches for a reachable angle.
      This lets the gripper length contribute to the reachable x, y range.

    Returns:
      Servo values in degrees.
    """

    if elbow not in ("up", "down", "auto"):
        raise ValueError("elbow must be 'up', 'down', or 'auto'")

    cal = DEFAULT_CAL.copy()
    if servo_cal:
        cal.update(servo_cal)

    r_center = math.hypot(x, y)

    if r_center < 1e-12:
        base_yaw = 0.0
    else:
        base_yaw = math.atan2(y, x)

    if tool_angle_deg is None:
        lo, hi = tool_angle_limits_deg
        if lo > hi:
            lo, hi = hi, lo

        n = max(2, int(round((hi - lo) / angle_step_deg)) + 1)
        tool_angles = [
            math.radians(lo + (hi - lo) * i / (n - 1))
            for i in range(n)
        ]
        preferred_phi = math.radians(preferred_tool_angle_deg)
    else:
        tool_angles = [math.radians(tool_angle_deg)]
        preferred_phi = math.radians(tool_angle_deg)

    candidates = []

    min_wrist_reach = abs(L2 - L3)
    max_wrist_reach = L2 + L3

    for phi in tool_angles:
        # Gripper center height required so that the tip is at tip_z.
        gripper_center_z = tip_z - L5 * math.sin(phi)

        # Convert desired gripper-center pose into wrist target.
        # This is where L4 and L5 affect reach.
        wrist_r = r_center - L4 * math.cos(phi)
        wrist_z = gripper_center_z - L4 * math.sin(phi)

        dx = wrist_r
        dz = wrist_z - L1

        d = math.hypot(dx, dz)

        if d > max_wrist_reach + 1e-9:
            continue

        if d < min_wrist_reach - 1e-9:
            continue

        cos_q2 = (d * d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
        cos_q2 = max(-1.0, min(1.0, cos_q2))

        q2_abs = math.acos(cos_q2)

        # Two elbow configurations.
        for q2 in (q2_abs, -q2_abs):
            q1 = math.atan2(dz, dx) - math.atan2(
                L3 * math.sin(q2),
                L2 + L3 * math.cos(q2),
            )

            q3 = phi - q1 - q2

            elbow_z = L1 + L2 * math.sin(q1)

            servos = {
                "base": cal["base"].to_servo(base_yaw),
                "shoulder": cal["shoulder"].to_servo(q1),
                "elbow": cal["elbow"].to_servo(q2),
                "wrist": cal["wrist"].to_servo(q3),
            }

            if gripper_servo is not None:
                servos["gripper"] = float(gripper_servo)

            if check_limits:
                bad = []

                for name, value in servos.items():
                    if name == "gripper":
                        lo_limit, hi_limit = 0.0, 180.0
                    else:
                        lo_limit = cal[name].min_value
                        hi_limit = cal[name].max_value

                    if value < lo_limit - 1e-6 or value > hi_limit + 1e-6:
                        bad.append(name)

                if bad:
                    continue

            # Cost function:
            # 1. Prefer the requested/preferred tool angle.
            # 2. Prefer the requested elbow posture.
            # 3. Slightly prefer not being fully stretched.
            cost = abs(_wrap_pi(phi - preferred_phi)) * 100.0

            if elbow == "up":
                cost += -0.1 * elbow_z
            elif elbow == "down":
                cost += 0.1 * elbow_z

            cost += 0.5 * (d / max_wrist_reach)

            debug = {
                "r_center_m": r_center,
                "gripper_center_z_m": gripper_center_z,
                "wrist_target_r_m": wrist_r,
                "wrist_target_z_m": wrist_z,
                "shoulder_to_wrist_m": d,
                "base_yaw_deg": math.degrees(base_yaw),
                "tool_angle_deg": math.degrees(phi),
                "shoulder_joint_q1_deg": math.degrees(q1),
                "elbow_joint_q2_deg": math.degrees(q2),
                "wrist_joint_q3_deg": math.degrees(q3),
                "elbow_z_m": elbow_z,
            }

            candidates.append((cost, servos, debug))

    if not candidates:
        max_est = estimate_max_gripper_center_radius(
            tip_z=tip_z,
            tool_angle_limits_deg=tool_angle_limits_deg,
        )

        extra = ""
        if max_est is not None:
            max_r, best_angle = max_est
            extra = (
                f" Approx max gripper-center radius over "
                f"tool_angle_limits_deg={tool_angle_limits_deg} is "
                f"{max_r:.3f} m at tool_angle_deg={best_angle:.1f}."
            )

        raise ValueError(
            f"No IK solution found for x={x:.3f}, y={y:.3f}, "
            f"radius={r_center:.3f} m."
            + extra
        )

    candidates.sort(key=lambda item: item[0])
    _, servos, debug = candidates[0]

    if return_debug:
        servos = dict(servos)
        servos["_debug"] = debug

    return servos

def send_servo_angles(angles):
    # Convert to integers
    angles = list(angles.values())
    angles = [int(a) for a in angles]

    # Create comma-separated string
    msg = ",".join(map(str, angles)) + "\n"

    # Send
    arm.write(msg.encode('utf-8'))

    print("Sent:", msg.strip())

if __name__ == "__main__":
    # Example usage
    while True:
        target_x, target_y = input("Enter target x,y (meters): ").split(",")
        target_x, target_y = to_robot(float(target_x), float(target_y))
        angles = miniarm_ik_xy(target_x, target_y, gripper_servo=90)
        if angles:
            send_servo_angles(angles)