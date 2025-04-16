import serial
import cv2
import mediapipe as mp
import serial.tools.list_ports
import numpy as np
import tensorflow as tf
from concurrent.futures import ThreadPoolExecutor

# Configure TensorFlow for maximum GPU performance
tf.config.optimizer.set_jit(True)  # Enable XLA optimization
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
            # Set memory limit to avoid OOM errors
            tf.config.set_logical_device_configuration(
                gpu,
                [tf.config.LogicalDeviceConfiguration(memory_limit=4096)])
        print("GPU configuration successful")
    except RuntimeError as e:
        print(f"GPU configuration error: {e}")

# Performance optimization settings
PROCESS_WIDTH = 640   # Processing width
PROCESS_HEIGHT = 480  # Processing height
FRAME_BUFFER_SIZE = 1 # Reduced buffer size to minimize latency

# config
write_video = False
debug = False
cam_source = 0

# Gripper sensitivity settings
fist_threshold = 0.8  # Threshold for fist detection
GRIPPER_OPEN_ANGLE = 170    # Fully open position (0 degrees)
GRIPPER_CLOSE_ANGLE = 10  # Fully closed position (160 degrees)

# Serial configuration for Arduino Mega 2560
BAUD_RATE = 115200

class GripperController:
    def __init__(self):
        # Initialize with closed state (10 degrees)
        self.prev_servo_angle = [GRIPPER_CLOSE_ANGLE]
        self.debug = False
        self.setup_arduino()
        self.setup_mediapipe()
        
        # Add state tracking with smaller history
        self.is_closed = True  # Start in closed state
        self.distance_history = []
        self.history_size = 3  # Reduced history size
        self.close_threshold = 0.85  # Adjusted for better fist detection
        self.open_threshold = 1.2   # Adjusted for better open hand detection
        
    def setup_arduino(self):
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
            self.debug = True
        else:
            try:
                self.ser = serial.Serial(arduino_port, BAUD_RATE, timeout=1)
                print(f"Connected to Arduino Mega 2560 on {arduino_port}")
                # Reset Arduino and set initial position
                self.ser.dtr = False
                self.ser.dtr = True
                # Send initial closed position
                if not self.debug:
                    self.ser.write(bytearray([GRIPPER_CLOSE_ANGLE]))
            except serial.SerialException as e:
                print(f"Error connecting to Arduino: {e}")
                print("Running in debug mode...")
                self.debug = True
    
    def setup_mediapipe(self):
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        
        # MediaPipe configuration for GPU acceleration
        self.MP_HANDS_CONFIG = {
            'model_complexity': 1,
            'min_detection_confidence': 0.7,
            'min_tracking_confidence': 0.7,
            'max_num_hands': 1,
            'static_image_mode': False
        }
    
    def process_frame(self, frame, hands):
        """Process frame using GPU acceleration"""
        try:
            # Mirror the frame horizontally
            frame = cv2.flip(frame, 1)
            
            # Resize first to reduce processing load
            frame = cv2.resize(frame, (PROCESS_WIDTH, PROCESS_HEIGHT))
            
            # Make frame writable
            frame.flags.writeable = False
            
            # Convert frame to RGB for MediaPipe
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            results = hands.process(frame_rgb)
            
            # Make frame writable again
            frame.flags.writeable = True
            
            # Convert back to BGR for display
            frame_display = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            
            return results, frame_display
            
        except Exception as e:
            print(f"Error in process_frame: {e}")
            return None, frame
    
    def calculate_hand_state(self, hand_landmarks):
        """Calculate hand state with improved fist detection"""
        # Get fingertip positions
        thumb_tip = hand_landmarks.landmark[4]
        index_tip = hand_landmarks.landmark[8]
        middle_tip = hand_landmarks.landmark[12]
        ring_tip = hand_landmarks.landmark[16]
        pinky_tip = hand_landmarks.landmark[20]
        
        # Get finger base positions
        thumb_base = hand_landmarks.landmark[2]
        index_base = hand_landmarks.landmark[5]
        middle_base = hand_landmarks.landmark[9]
        ring_base = hand_landmarks.landmark[13]
        pinky_base = hand_landmarks.landmark[17]
        
        # Calculate distances from tips to bases
        tips = np.array([[tip.x, tip.y, tip.z] for tip in [thumb_tip, index_tip, middle_tip, ring_tip, pinky_tip]])
        bases = np.array([[base.x, base.y, base.z] for base in [thumb_base, index_base, middle_base, ring_base, pinky_base]])
        
        # Calculate normalized distances for each finger
        distances = np.sqrt(np.sum((tips - bases) ** 2, axis=1))
        palm_width = np.sqrt(np.sum((np.array([index_base.x, index_base.y, index_base.z]) - 
                                    np.array([pinky_base.x, pinky_base.y, pinky_base.z])) ** 2))
        
        # Normalize by palm width
        normalized_distances = distances / palm_width
        
        # Calculate average normalized distance
        avg_normalized_distance = np.mean(normalized_distances)
        
        return avg_normalized_distance
    
    def get_gripper_angle(self, hand_landmarks):
        """Get gripper angle with natural mapping and state persistence"""
        normalized_distance = self.calculate_hand_state(hand_landmarks)
        
        # Add to history and maintain size
        self.distance_history.append(normalized_distance)
        if len(self.distance_history) > self.history_size:
            self.distance_history.pop(0)
        
        # Calculate smoothed distance
        avg_distance = sum(self.distance_history) / len(self.distance_history)
        
        # Print for debugging
        print(f"Hand openness: {normalized_distance:.2f}, Average: {avg_distance:.2f}, Is Closed: {self.is_closed}")
        
        # Apply hysteresis with reversed logic:
        # When hand is closed (small distance) -> GRIPPER_CLOSE_ANGLE (10)
        # When hand is open (large distance) -> GRIPPER_OPEN_ANGLE (170)
        if not self.is_closed and avg_distance < self.close_threshold:
            self.is_closed = True
            return [GRIPPER_CLOSE_ANGLE]  # 10 degrees when hand is closed
        elif self.is_closed and avg_distance > self.open_threshold:
            self.is_closed = False
            return [GRIPPER_OPEN_ANGLE]   # 170 degrees when hand is open
        else:
            # Maintain current state
            return [GRIPPER_CLOSE_ANGLE] if self.is_closed else [GRIPPER_OPEN_ANGLE]
    
    def run(self):
        try:
            # Initialize camera with optimized settings
            cap = cv2.VideoCapture(cam_source, cv2.CAP_DSHOW)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, FRAME_BUFFER_SIZE)
            
            if not cap.isOpened():
                raise Exception("Could not open camera")
            
            print("Camera initialized successfully")
            print(f"Initial state: CLOSED (Angle: {GRIPPER_CLOSE_ANGLE})")
            
            # Send initial position to Arduino
            if not self.debug and hasattr(self, 'ser'):
                self.ser.write(bytearray([GRIPPER_CLOSE_ANGLE]))

            frame_count = 0
            skip_frames = 1  # Process every nth frame
            
            # Initialize MediaPipe Hands with GPU optimization
            with self.mp_hands.Hands(**self.MP_HANDS_CONFIG) as hands:
                while cap.isOpened():
                    # Skip frames if needed
                    frame_count += 1
                    if frame_count % skip_frames != 0:
                        continue
                        
                    # Clear buffer
                    for _ in range(FRAME_BUFFER_SIZE):
                        cap.grab()
                    
                    success, frame = cap.read()
                    if not success:
                        print("Failed to read camera frame")
                        continue

                    # Process frame
                    results, frame = self.process_frame(frame, hands)
                    if results is None:
                        continue
                    
                    if results.multi_hand_landmarks:
                        if len(results.multi_hand_landmarks) == 1:
                            hand_landmarks = results.multi_hand_landmarks[0]
                            
                            # Calculate hand state
                            normalized_distance = self.calculate_hand_state(hand_landmarks)
                            
                            # Determine gripper angle
                            new_servo_angle = self.get_gripper_angle(hand_landmarks)
                            
                            # Draw hand landmarks
                            self.mp_drawing.draw_landmarks(
                                frame,
                                hand_landmarks,
                                self.mp_hands.HAND_CONNECTIONS,
                                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                                self.mp_drawing_styles.get_default_hand_connections_style())
                            
                            # Display gripper state
                            gripper_state = "CLOSE" if new_servo_angle[0] == GRIPPER_CLOSE_ANGLE else "OPEN"
                            cv2.putText(frame, f"Gripper: {gripper_state}", 
                                      (frame.shape[1] - 200, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            
                            # Update servo angle if changed
                            if new_servo_angle != self.prev_servo_angle:
                                print("Gripper angle: ", new_servo_angle)
                                self.prev_servo_angle = new_servo_angle
                                if not self.debug:
                                    self.ser.write(bytearray(new_servo_angle))
                    
                    # Show frame
                    cv2.imshow('Hand Gesture Control - Gripper', frame)
                    
                    # Check for exit with shorter wait time
                    if cv2.waitKey(1) & 0xFF == 27 or cv2.getWindowProperty('Hand Gesture Control - Gripper', cv2.WND_PROP_VISIBLE) < 1:
                        break
                    
                    # Release some memory
                    del results
                        
        except Exception as e:
            print(f"Error: {e}")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            if not self.debug and hasattr(self, 'ser'):
                self.ser.write(bytearray([GRIPPER_CLOSE_ANGLE]))  # Return to closed position
                self.ser.close()
            print("Cleanup complete. Program terminated.")

def main():
    controller = GripperController()
    controller.run()

if __name__ == "__main__":
    main()