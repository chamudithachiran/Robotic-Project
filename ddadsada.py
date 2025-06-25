import cv2
import numpy as np
import requests
import tkinter as tk
from threading import Thread

# Load ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

esp32_ip = "http://192.168.4.1"  # ESP32's IP
distance_threshold = 60  # Pixels before stopping
target_id = 1  # Default target ID

# Function to send commands to ESP32
def send_command_to_esp32(command):
    try:
        response = requests.get(f"{esp32_ip}/{command}")
        print(f"Sent: {command}, Response: {response.text}")
    except Exception as e:
        print(f"Error: {e}")

# Function to get marker orientation
def get_marker_angle(corners):
    top_left, top_right = corners[0][0], corners[0][1]
    dx, dy = top_right[0] - top_left[0], top_right[1] - top_left[1]
    return np.arctan2(dy, dx) * (180 / np.pi)  # Convert radians to degrees

# Function to navigate robot
def navigate_to_target(x1, y1, x0, y0, angle):
    dx, dy = x0 - x1, y0 - y1

    if abs(angle) > 10:  # If not facing correctly, rotate first
        return "rotate_right" if angle < 0 else "rotate_left"

    if abs(dx) < distance_threshold and abs(dy) < distance_threshold:
        return "stop"

    return "move_forward" if dy > 0 else "move_backward"

# Function to update target ID from GUI
def set_target_id():
    global target_id
    try:
        target_id = int(target_entry.get())
        print(f"Target ID set to {target_id}")
    except ValueError:
        print("Invalid target ID")

# Function to run Tkinter in a separate thread
def run_gui():
    global target_entry
    root = tk.Tk()
    root.title("Set Target ID")

    tk.Label(root, text="Target ID:").pack()
    target_entry = tk.Entry(root)
    target_entry.pack()
    set_button = tk.Button(root, text="Set ID", command=set_target_id)
    set_button.pack()

    root.mainloop()

# Start the GUI in a separate thread
gui_thread = Thread(target=run_gui, daemon=True)
gui_thread.start()

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        centers = {}
        angles = {}

        for i in range(len(ids)):
            center = np.mean(corners[i][0], axis=0)
            x, y = int(center[0]), int(center[1])
            angle = get_marker_angle(corners[i])

            centers[ids[i][0]] = (x, y)
            angles[ids[i][0]] = angle

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"ID: {ids[i][0]}, Angle: {int(angle)}°", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if target_id in centers and 0 in centers:
            x1, y1 = centers[target_id]
            x0, y0 = centers[0]
            angle = angles[0]

            cv2.line(frame, (x1, y1), (x0, y0), (255, 0, 0), 2)

            command = navigate_to_target(x1, y1, x0, y0, angle)
            print(f"Navigation: {command}")
            send_command_to_esp32(command)
        else:
            send_command_to_esp32("stop")

    cv2.imshow("ArUco Navigation", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
