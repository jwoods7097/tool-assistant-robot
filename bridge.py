import time
from serial import Serial
import os
import msvcrt

os.makedirs("data", exist_ok=True)

CAM_PORT = "COM6"
ARM_PORT = "COM3"
BAUD = 115200

# cam = Serial(CAM_PORT, BAUD, timeout=0.2)
arm = Serial(ARM_PORT, BAUD, timeout=0.2)

time.sleep(2)
print("Bridge running...")
print("Type keys to send them to the serial port. Press Esc to quit.")

while True:
    # Send any pending keystrokes to the serial port
    while msvcrt.kbhit():
        key = msvcrt.getch()

        # Exit on Esc
        if key == b"\x1b":
            print("Exiting...")
            arm.close()
            raise SystemExit

        # Ignore special keys like arrows/function keys
        if key in (b"\x00", b"\xe0"):
            msvcrt.getch()
            continue

        arm.write(key)

    # Read incoming serial data
    servo_data = arm.readline().decode(errors="ignore").strip()
    if not servo_data:
        continue

    with open("data/servo_data.txt", "a") as f:
        f.write(servo_data + "\n")