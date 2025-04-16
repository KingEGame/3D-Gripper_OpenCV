import serial
import cv2
import mediapipe as mp
import serial.tools.list_ports
import numpy as np

# config
write_video = False  # Set to True if you want to record video
debug = False  # Set to False to enable actual servo control
cam_source = 0  # 0 for default laptop webcam

# Gripper sensitivity settings
fist_threshold = 0.35  # Adjusted threshold for better sensitivity
GRIPPER_OPEN_ANGLE = 20    # Changed: 20 degrees when hand is open (was 0)
GRIPPER_CLOSE_ANGLE = 160  # Changed: 160 degrees when hand is closed (was 180)

# Serial configuration for Arduino Mega 2560
BAUD_RATE = 115200   # Make sure this matches Arduino code

# Find Arduino Mega port
arduino_port = None
for port in serial.tools.list_ports.comports():
    if "VID:PID=2341:0042" in port.hwid:  # Arduino Mega 2560 identifier
        arduino_port = port.device
        break

if arduino_port is None:
    print("Arduino Mega 2560 not found. Available ports:")
    for port in serial.tools.list_ports.comports():
        print(f"Port: {port.device}, Description: {port.description}, Hardware ID: {port.hwid}")
    print("Running in debug mode...")
    debug = True
else:
    try:
        ser = serial.Serial(arduino_port, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino Mega 2560 on {arduino_port}")
        # Reset Arduino
        ser.dtr = False
        ser.dtr = True
    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
        print("Running in debug mode...")
        debug = True

# # Arm twist control parameters (commented out)
# x_min = 0
# x_mid = 75
# x_max = 150
# palm_angle_min = -50
# palm_angle_mid = 20
# y_min = 0
# y_mid = 90
# y_max = 180
# wrist_y_min = 0.3
# wrist_y_max = 0.9
# z_min = 10
# z_mid = 90
# z_max = 180
# plam_size_min = 0.1
# plam_size_max = 0.3

# Gripper control parameters
claw_open_angle = 60
claw_close_angle = 0

# Initialize servo angles (only for gripper now)
servo_angle = [claw_open_angle]  # Only gripper angle
prev_servo_angle = servo_angle

# Initialize MediaPipe with GPU acceleration
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# Initialize camera with error handling
def initialize_camera(source=0):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print("Error: Could not open camera. Trying alternative sources...")
        # Try alternative camera indices
        for i in range(1, 5):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                print(f"Successfully opened camera {i}")
                return cap
        raise Exception("No camera found. Please check your webcam connection.")
    return cap

# Initialize camera
try:
    cap = initialize_camera(cam_source)
    print("Camera initialized successfully")
except Exception as e:
    print(f"Error initializing camera: {e}")
    exit(1)

# video writer
if write_video:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 60.0, (640, 480))

clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
# map_range = lambda x, in_min, in_max, out_min, out_max: abs((x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

# Calibration settings for grip detection
CALIBRATION_SAMPLES = 10  # Number of samples to collect for calibration
calibration_data = {
    'open': [],
    'closed': []
}
is_calibrating = True
calibration_step = 0

def calculate_hand_state(hand_landmarks):
    # Get key points
    wrist = hand_landmarks.landmark[0]
    thumb_tip = hand_landmarks.landmark[4]
    index_tip = hand_landmarks.landmark[8]
    middle_tip = hand_landmarks.landmark[12]
    ring_tip = hand_landmarks.landmark[16]
    pinky_tip = hand_landmarks.landmark[20]
    
    # Calculate distances from wrist to finger tips
    distances = []
    for tip in [thumb_tip, index_tip, middle_tip, ring_tip, pinky_tip]:
        dist = np.sqrt((tip.x - wrist.x)**2 + (tip.y - wrist.y)**2 + (tip.z - wrist.z)**2)
        distances.append(dist)
    
    # Calculate average distance
    avg_distance = np.mean(distances)
    
    # Calculate palm size (distance from wrist to middle finger base)
    middle_base = hand_landmarks.landmark[9]
    palm_size = np.sqrt((middle_base.x - wrist.x)**2 + (middle_base.y - wrist.y)**2 + (middle_base.z - wrist.z)**2)
    
    # Normalize distance by palm size
    normalized_distance = avg_distance / palm_size
    
    return normalized_distance

# Check if the hand is a fist (for gripper control)
def is_fist(hand_landmarks, palm_size):
    normalized_distance = calculate_hand_state(hand_landmarks)
    
    # Use calibrated values if available
    if calibration_data['open'] and calibration_data['closed']:
        open_avg = sum(calibration_data['open']) / len(calibration_data['open'])
        closed_avg = sum(calibration_data['closed']) / len(calibration_data['closed'])
        dynamic_threshold = (open_avg + closed_avg) / 2
        return normalized_distance < dynamic_threshold
    else:
        return normalized_distance < fist_threshold

def get_gripper_angle(hand_landmarks):
    # Only calculate gripper angle based on fist detection
    WRIST = hand_landmarks.landmark[0]
    INDEX_FINGER_MCP = hand_landmarks.landmark[5]
    # calculate the distance between the wrist and the index finger for palm size
    palm_size = ((WRIST.x - INDEX_FINGER_MCP.x)**2 + (WRIST.y - INDEX_FINGER_MCP.y)**2 + (WRIST.z - INDEX_FINGER_MCP.z)**2)**0.5

    # Return gripper angle based on fist detection - REVERSED LOGIC
    if is_fist(hand_landmarks, palm_size):
        return [GRIPPER_CLOSE_ANGLE]  # Close gripper (180 degrees) when hand is closed
    else:
        return [GRIPPER_OPEN_ANGLE]   # Open gripper (0 degrees) when hand is open

# Configure MediaPipe to use GPU
with mp_hands.Hands(
    model_complexity=1,  # Increased model complexity for better accuracy
    min_detection_confidence=0.7,  # Increased confidence threshold
    min_tracking_confidence=0.7,   # Increased tracking confidence
    max_num_hands=1,
    static_image_mode=False
) as hands:
    try:
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Ignoring empty camera frame.")
                continue

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)

            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if results.multi_hand_landmarks:
                if len(results.multi_hand_landmarks) == 1:
                    hand_landmarks = results.multi_hand_landmarks[0]
                    
                    # Calibration mode
                    if is_calibrating:
                        normalized_distance = calculate_hand_state(hand_landmarks)
                        
                        if calibration_step < CALIBRATION_SAMPLES:
                            cv2.putText(image, f"Calibration: Open hand ({calibration_step + 1}/{CALIBRATION_SAMPLES})", 
                                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                            calibration_data['open'].append(normalized_distance)
                        elif calibration_step < CALIBRATION_SAMPLES * 2:
                            cv2.putText(image, f"Calibration: Closed hand ({calibration_step - CALIBRATION_SAMPLES + 1}/{CALIBRATION_SAMPLES})", 
                                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            calibration_data['closed'].append(normalized_distance)
                        
                        calibration_step += 1
                        if calibration_step >= CALIBRATION_SAMPLES * 2:
                            is_calibrating = False
                            print("Calibration complete!")
                            print(f"Open hand average: {sum(calibration_data['open']) / len(calibration_data['open'])}")
                            print(f"Closed hand average: {sum(calibration_data['closed']) / len(calibration_data['closed'])}")
                    
                    # Normal operation mode
                    else:
                        servo_angle = get_gripper_angle(hand_landmarks)
                        if servo_angle != prev_servo_angle:
                            print("Gripper angle: ", servo_angle)
                            prev_servo_angle = servo_angle
                            if not debug:
                                ser.write(bytearray(servo_angle))
                else:
                    print("More than one hand detected")
                
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
            
            # Flip the image horizontally for a selfie-view display.
            image = cv2.flip(image, 1)
            
            # Show gripper angle and calibration status with better positioning
            if is_calibrating:
                if calibration_step < CALIBRATION_SAMPLES:
                    cv2.putText(image, f"Open hand ({calibration_step + 1}/{CALIBRATION_SAMPLES})", 
                              (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                else:
                    cv2.putText(image, f"Close hand ({calibration_step - CALIBRATION_SAMPLES + 1}/{CALIBRATION_SAMPLES})", 
                              (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                # Position text in top-right corner for better visibility
                gripper_state = "OPEN" if servo_angle[0] == GRIPPER_OPEN_ANGLE else "CLOSE"
                text = f"Gripper: {gripper_state}"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                text_x = image.shape[1] - text_size[0] - 30  # 30 pixels from right edge
                cv2.putText(image, text, (text_x, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
            cv2.imshow('Hand Gesture Control - Gripper', image)

            # Check for window close or ESC key
            key = cv2.waitKey(5) & 0xFF
            if key == 27 or cv2.getWindowProperty('Hand Gesture Control - Gripper', cv2.WND_PROP_VISIBLE) < 1:
                print("Window closed or ESC pressed. Cleaning up...")
                break

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Cleanup
        print("Cleaning up resources...")
        if write_video:
            out.release()
        cap.release()
        cv2.destroyAllWindows()
        if not debug and 'ser' in locals():
            # Send final command to open gripper before closing
            ser.write(bytearray([GRIPPER_OPEN_ANGLE]))
            ser.close()
        print("Cleanup complete. Program terminated.")