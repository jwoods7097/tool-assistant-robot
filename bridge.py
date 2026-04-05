import time
import serial

CAM_PORT = "COM6"
ARM_PORT = "COM5"
BAUD = 115200

cam = serial.Serial(CAM_PORT, BAUD, timeout=0.2)
arm = serial.Serial(ARM_PORT, BAUD, timeout=0.2)

time.sleep(2)
print("Bridge running...")

while True:
    line = cam.readline().decode(errors="ignore").strip()
    if not line:
        continue

    print("cam:", line)
    arm.write((line + "\n").encode())