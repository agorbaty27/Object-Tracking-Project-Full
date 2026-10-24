import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import threading
from flask import Flask, jsonify, request
from flask_cors import CORS

# --- CONFIGURATION ---
REAL_MODE = False
PORT = '/dev/cu.usbmodem141201'
BAUD = 115200

# --- STATE ---
state = {
    "Kp": 0.05, "Kd": 0.01, "deadzone": 15,
    "min_x": 20, "max_x": 160,
    "min_y": 40, "max_y": 140,

    "servo_x": 90.0, "servo_y": 90.0,
    "target_x": None, "target_y": None,

    "tracking": False,
    "manual_mode": False,
    "manual_x": 90.0,
    "manual_y": 90.0,

    "s3": 90.0,
    "s4": 90.0,
    "s5": 90.0,
    "s6": 180.0,

    "s3_tilt_ratio": 0.5,
    "s4_tilt_ratio": 0.3,
}

state_lock = threading.Lock()

app = Flask(__name__)
CORS(app)

@app.route('/state')
def get_state():
    with state_lock:
        return jsonify(state)

@app.route('/update', methods=['POST'])
def update_state():
    data = request.json
    with state_lock:
        for key in ['Kp','Kd','deadzone','min_x','max_x','min_y','max_y','s3_tilt_ratio','s4_tilt_ratio']:
            if key in data:
                state[key] = float(data[key])
    return jsonify({"ok": True})

@app.route('/manual', methods=['POST'])
def manual_control():
    data = request.json
    with state_lock:
        if 'manual_mode' in data:
            state['manual_mode'] = bool(data['manual_mode'])
        if 'x' in data:
            state['manual_x'] = float(data['x'])
        if 'y' in data:
            state['manual_y'] = float(data['y'])
    return jsonify({"ok": True})

@app.route('/arm', methods=['POST'])
def arm_control():
    data = request.json
    with state_lock:
        for key in ['s3','s4','s5','s6']:
            if key in data:
                state[key] = float(data[key])
    return jsonify({"ok": True})

def send_servos(arduino, s1, s2, s3, s4, s5, s6):
    if arduino is None:
        return
    data_str = f"{int(s1)},{int(s2)},{int(s3)},{int(s4)},{int(s5)},{int(s6)}\n"
    arduino.write(data_str.encode())

def run_flask():
    app.run(port=5050, debug=False, use_reloader=False)

flask_thread = threading.Thread(target=run_flask, daemon=True)
flask_thread.start()

print("Tuning UI: open tuner.html in browser (Flask on port 5050)")

# --- INIT ---
sim_servo_x = 90.0
sim_servo_y = 90.0

last_error_x = 0
last_error_y = 0

last_send_time = 0

arduino = None

if REAL_MODE:
    try:
        arduino = serial.Serial(port=PORT, baudrate=BAUD, timeout=0.01)
        arduino.reset_input_buffer()
        arduino.reset_output_buffer()
        time.sleep(2)
        print("Connected to Arduino")
    except Exception as e:
        print(f"Serial Error: {e}")

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7
)

cap = cv2.VideoCapture(0)

while cap.isOpened():

    success, frame = cap.read()
    if not success:
        break

    frame = cv2.flip(frame, 1)

    h, w, _ = frame.shape

    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    with state_lock:

        Kp = state["Kp"]
        Kd = state["Kd"]

        deadzone = int(state["deadzone"])

        min_x = state["min_x"]
        max_x = state["max_x"]

        min_y = state["min_y"]
        max_y = state["max_y"]

        manual_mode = state["manual_mode"]

        manual_x = state["manual_x"]
        manual_y = state["manual_y"]

        s3 = state["s3"]
        s4 = state["s4"]
        s5 = state["s5"]
        s6 = state["s6"]

        s3_tilt_ratio = state["s3_tilt_ratio"]
        s4_tilt_ratio = state["s4_tilt_ratio"]

    # --- MANUAL MODE ---
    if manual_mode:

        sim_servo_x = np.clip(manual_x, min_x, max_x)
        sim_servo_y = np.clip(manual_y, min_y, max_y)

        last_error_x = 0
        last_error_y = 0

        with state_lock:
            state["servo_x"] = round(sim_servo_x,1)
            state["servo_y"] = round(sim_servo_y,1)
            state["tracking"] = False

        current_time = time.time()

        if REAL_MODE and arduino and (current_time-last_send_time>0.03):

            send_servos(arduino,sim_servo_x,sim_servo_y,s3,s4,s5,s6)
            last_send_time = current_time

        vx = int(np.interp(sim_servo_x,[0,180],[0,w]))
        vy = int(np.interp(sim_servo_y,[0,180],[h,0]))

        cv2.drawMarker(frame,(vx,vy),(0,165,255),cv2.MARKER_CROSS,20,2)

        cv2.putText(frame,"MANUAL",(10,30),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,165,255),2)

        cv2.imshow("Robot Control",frame)

        if cv2.waitKey(1)&0xFF==ord('q'):
            break

        continue

    # --- PID TRACKING MODE ---
    if results.multi_hand_landmarks:

        for hand_lms in results.multi_hand_landmarks:

            mp_draw.draw_landmarks(
                frame,
                hand_lms,
                mp_hands.HAND_CONNECTIONS
            )

            hx = int(hand_lms.landmark[9].x * w)
            hy = int(hand_lms.landmark[9].y * h)

            view_x = int(np.interp(sim_servo_x,[0,180],[0,w]))
            view_y = int(np.interp(sim_servo_y,[0,180],[h,0]))

            error_x = hx - view_x
            error_y = hy - view_y

            if abs(error_x) < deadzone:
                error_x = 0
            if abs(error_y) < deadzone:
                error_y = 0

            change_x = error_x - last_error_x
            change_y = error_y - last_error_y

            output_x = (error_x * Kp) + (change_x * Kd)
            output_y = (error_y * Kp) + (change_y * Kd)

            target_x = np.clip(sim_servo_x + output_x, min_x, max_x)
            target_y = np.clip(sim_servo_y - output_y, min_y, max_y)

            SMOOTH = 0.25

            sim_servo_x = sim_servo_x + SMOOTH*(target_x-sim_servo_x)
            sim_servo_y = sim_servo_y + SMOOTH*(target_y-sim_servo_y)

            last_error_x = error_x
            last_error_y = error_y

            # apply tilt coupling AFTER output calculated
            s3 = float(np.clip(s3 + output_y*s3_tilt_ratio,0,135))
            s4 = float(np.clip(s4 - output_y*s4_tilt_ratio,0,90))

            with state_lock:
                state["s3"] = s3
                state["s4"] = s4

            current_time = time.time()

            if REAL_MODE and arduino and (current_time-last_send_time>0.05):

                send_servos(
                    arduino,
                    sim_servo_x,
                    sim_servo_y,
                    s3,
                    s4,
                    s5,
                    s6
                )

                last_send_time = current_time

        with state_lock:

            state["servo_x"] = round(sim_servo_x,1)
            state["servo_y"] = round(sim_servo_y,1)

            state["target_x"] = hx
            state["target_y"] = hy

            state["tracking"] = True

    else:

        last_error_x = 0
        last_error_y = 0

        with state_lock:
            state["tracking"] = False
            state["target_x"] = None
            state["target_y"] = None

    vx = int(np.interp(sim_servo_x,[0,180],[0,w]))
    vy = int(np.interp(sim_servo_y,[0,180],[h,0]))

    cv2.drawMarker(frame,(vx,vy),(0,0,255),cv2.MARKER_CROSS,20,2)

    cv2.imshow("Robot Control",frame)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break


# --- SHUTDOWN ---
if arduino:

    print("\nShutting down — centering all servos...")

    for _ in range(5):

        arduino.write(b"999,999,999,999,999,999\n")
        time.sleep(0.02)

    arduino.close()

cap.release()
cv2.destroyAllWindows()