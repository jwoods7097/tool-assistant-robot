from cv2 import aruco
import cv2

# Choose dictionary
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Generate 4 markers (IDs 0–3)
marker_size_px = 400  # resolution (high for clean printing)

for marker_id in range(4):
    marker = aruco.generateImageMarker(dictionary, marker_id, marker_size_px)
    filename = f"data/marker_{marker_id}.png"
    cv2.imwrite(filename, marker)
