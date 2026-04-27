Vision Setup Guide

Requirements
Before starting, make sure you have Arduino IDE, Python 3.11+, and the Hiwonder ESP32-CAM Carrier Board V1.0. You will also need the following files: miniarm_stream.ino, best.pt, and detect_tools.py.

Step 1 — Install Python Dependencies
Open a terminal and run `pip install ultralytics opencv-python requests numpy` to install all required Python packages.

Step 2 — Flash the ESP32-CAM
Open Arduino IDE and open the miniarm_stream.ino file. At the top of the file, update the WiFi credentials by replacing YOUR_WIFI_NAME and YOUR_WIFI_PASSWORD with your actual network details. Then configure the board settings:

- Board to ESP32S3 Dev Module
- PSRAM to OPI PSRAM
- Flash Size to 8MB
- Partition Scheme to Huge APP (3MB No OTA)

Select the correct COM port for your board and click Upload. Once uploaded, open the Serial Monitor at 115200 baud and press the RST button on the board — it will print the camera's IP address which you will need for the next step.

Step 3 — Update the Stream URL
Open detect_tools.py and on line 4, replace the IP address in STREAM_URL with the IP address printed in the Serial Monitor, so it looks like STREAM_URL = "http://YOUR_ESP32_IP/stream".

Step 4 — Run Detection
Make sure best.pt and detect_tools.py are in the same folder, then run python detect_tools.py in your terminal. A window will open showing the live camera feed with tool detections drawn on screen. The model can detect hammers, pliers, screwdrivers, and wrenches. Press Q to quit.
