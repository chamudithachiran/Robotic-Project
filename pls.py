import cv2
import numpy as np
import requests
import tkinter as tk
from threading import Thread
import time

# ArUco settings
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# ESP32 Wi-Fi settings
esp32_ip = "http://192.168.4.1"  # Change this to your ESP32's IP
distance_threshold = 30
angle_threshold = 10
target_id = 1
moving = False

# Function to send HTTP request to ESP32
def send_command_to_esp32(command):
    try:
        response = requests.get(f"{esp32_ip}/send_command", params={"cmd": command})
        print(f"Sent: {command}, Response: {response.text}")
    except Exception as e:
        print(f"Error: {e}")

# Function to get marker angle
def get_marker_angle(corners):
    top_left, top_right = corners[0][0], corners[0][1]
    dx, dy = top_right[0] - top_left[0], top_right[1] - top_left[1]
    return np.arctan2(dy, dx) * (180 / np.pi)

# Function to calculate angle difference
def angle_difference(angle1, angle2):
    return (angle2 - angle1 + 180) % 360 - 180  # Keep in [-180, 180]

# Function to navigate robot
def navigate_to_target(x1, y1, x0, y0, robot_angle):
    global moving
    dx, dy = x0 - x1, y0 - y1

    if abs(dx) < distance_threshold and abs(dy) < distance_threshold:
        if moving:
            send_command_to_esp32("target_reached_sound")  # Play arrival sound
        moving = False
        return "stop"

    desired_angle = np.arctan2(dy, dx) * (180 / np.pi)
    angle_diff = angle_difference(robot_angle, desired_angle)

    if abs(angle_diff) > angle_threshold:
        return "rotate_right" if angle_diff > 0 else "rotate_left"

    if not moving:
        send_command_to_esp32("start_sound")  # Play start movement sound
        moving = True

    return "move_forward"

# Function to update target ID from GUI
def set_target_id(new_id):
    global target_id, moving
    target_id = new_id
    print(f"Target ID set to {target_id}")
    send_command_to_esp32("start_sound")  # Play sound when target is set
    moving = False  # Reset movement tracking

# Function to fetch IR remote input from ESP32
def fetch_ir_remote():
    global target_id
    while True:
        try:
            response = requests.get(f"{esp32_ip}/get_ir_code")
            if response.status_code == 200:
                ir_code = response.text.strip()
                print(f"Received IR Code: {ir_code}")
                try:
                    new_id = int(ir_code)
                    if 1 <= new_id <= 5:
                        set_target_id(new_id)
                except ValueError:
                    pass
        except Exception as e:
            print(f"Error fetching IR code: {e}")
        time.sleep(0.5)  # Add a small delay to prevent tight loop

# Function to run GUI
def run_gui():
    def update_label(text):
        label.config(text=text)

    root = tk.Tk()
    root.title("Set Target ID")

    label = tk.Label(root, text="Waiting for IR signal...", font=("Arial", 14))
    label.pack(pady=10)

    button_frame = tk.Frame(root)
    button_frame.pack()

    for i in range(1, 6):  # Buttons for ID1 to ID5
        tk.Button(button_frame, text=f"ID {i}", command=lambda i=i: set_target_id(i)).grid(row=0, column=i-1, padx=5, pady=5)
    
    root.mainloop()

# Start GUI and IR reading in separate threads
Thread(target=fetch_ir_remote, daemon=True).start()  # Start IR fetching in a separate thread
Thread(target=run_gui, daemon=True).start()  # Run the GUI in the main thread

# Start video capture
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame, trying again...")
        continue

    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    if ids is not None:
        centers, angles = {}, {}
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
            robot_angle = angles[0]

            cv2.line(frame, (x1, y1), (x0, y0), (255, 0, 0), 2)

            command = navigate_to_target(x1, y1, x0, y0, robot_angle)
            print(f"Navigation: {command}")
            send_command_to_esp32(command)
        else:
            send_command_to_esp32("stop")

    cv2.imshow("ArUco Navigation", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
