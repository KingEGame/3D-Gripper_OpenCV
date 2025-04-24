import tkinter as tk
from tkinter import ttk, messagebox
import cv2
from PIL import Image, ImageTk
import serial.tools.list_ports
from main import GripperController, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE
import threading
import time

class GripperControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("3D Gripper Control")
        self.root.geometry("1200x800")
        
        # Variables
        self.running = False
        self.camera_active = False
        self.gripper_controller = None
        self.camera_source = tk.StringVar(value="0")
        self.fist_threshold = tk.DoubleVar(value=0.8)
        self.selected_port = tk.StringVar()
        self.gripper_status = tk.StringVar(value="Gripper Status: Not Active")
        
        # Performance settings
        self.target_fps = 30
        self.frame_time = 1.0 / self.target_fps
        self.last_frame_time = 0
        self.frame_count = 0
        self.fps = 0
        self.last_fps_update = time.time()
        
        self.create_gui()
        self.update_port_list()
        
    def create_gui(self):
        # Main container
        main_container = ttk.Frame(self.root, padding="10")
        main_container.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Left panel - Controls
        left_panel = ttk.LabelFrame(main_container, text="Controls", padding="10")
        left_panel.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # Arduino Connection
        ttk.Label(left_panel, text="Arduino Port:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.port_combo = ttk.Combobox(left_panel, textvariable=self.selected_port)
        self.port_combo.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        ttk.Button(left_panel, text="Refresh Ports", command=self.update_port_list).grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Camera Settings
        ttk.Label(left_panel, text="Camera Source:").grid(row=3, column=0, sticky=tk.W, pady=5)
        ttk.Entry(left_panel, textvariable=self.camera_source).grid(row=4, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Sensitivity Settings
        ttk.Label(left_panel, text="Fist Threshold:").grid(row=5, column=0, sticky=tk.W, pady=5)
        ttk.Scale(left_panel, from_=0.0, to=1.0, variable=self.fist_threshold, orient=tk.HORIZONTAL).grid(row=6, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Control Buttons
        ttk.Button(left_panel, text="Start", command=self.start_system).grid(row=7, column=0, sticky=(tk.W, tk.E), pady=5)
        ttk.Button(left_panel, text="Stop", command=self.stop_system).grid(row=8, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Status, FPS and Gripper State
        self.status_label = ttk.Label(left_panel, text="Status: Stopped")
        self.status_label.grid(row=9, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Gripper Status with visual indicator
        gripper_frame = ttk.Frame(left_panel)
        gripper_frame.grid(row=10, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.gripper_status_label = ttk.Label(gripper_frame, textvariable=self.gripper_status)
        self.gripper_status_label.pack(side=tk.LEFT, padx=5)
        
        self.gripper_indicator = tk.Canvas(gripper_frame, width=20, height=20)
        self.gripper_indicator.pack(side=tk.RIGHT, padx=5)
        self.update_gripper_indicator("gray")  # Initial state
        
        self.fps_label = ttk.Label(left_panel, text="FPS: 0")
        self.fps_label.grid(row=11, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Right panel - Camera Feed
        right_panel = ttk.LabelFrame(main_container, text="Camera Feed", padding="10")
        right_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.camera_label = ttk.Label(right_panel)
        self.camera_label.grid(row=0, column=0)
        
        # Configure grid weights
        main_container.columnconfigure(1, weight=1)
        main_container.rowconfigure(0, weight=1)
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(0, weight=1)
        
    def update_port_list(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.selected_port.set(ports[0])
            
    def start_system(self):
        if self.running:
            messagebox.showwarning("Warning", "System is already running!")
            return
            
        try:
            self.gripper_controller = GripperController()
            self.gripper_controller.debug = False
            
            # Update settings
            self.gripper_controller.fist_threshold = self.fist_threshold.get()
            self.gripper_controller.cam_source = int(self.camera_source.get())
            
            # Start camera thread
            self.running = True
            self.camera_thread = threading.Thread(target=self.camera_loop)
            self.camera_thread.daemon = True
            self.camera_thread.start()
            
            self.status_label.config(text="Status: Running")
            messagebox.showinfo("Success", "System started successfully!")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start system: {str(e)}")
            self.running = False
            
    def stop_system(self):
        if not self.running:
            messagebox.showwarning("Warning", "System is not running!")
            return
            
        self.running = False
        if self.gripper_controller:
            if hasattr(self.gripper_controller, 'ser') and self.gripper_controller.ser:
                self.gripper_controller.ser.close()
        self.status_label.config(text="Status: Stopped")
        
    def update_fps(self):
        current_time = time.time()
        self.frame_count += 1
        
        if current_time - self.last_fps_update >= 1.0:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_fps_update = current_time
            self.fps_label.config(text=f"FPS: {self.fps}")
        
    def update_gripper_indicator(self, color):
        """Update the gripper status indicator color"""
        self.gripper_indicator.delete("all")
        self.gripper_indicator.create_oval(2, 2, 18, 18, fill=color, outline="black")
        
    def camera_loop(self):
        # Set up camera with performance optimizations
        cap = cv2.VideoCapture(self.gripper_controller.cam_source, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Reduced resolution
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        
        with self.gripper_controller.mp_hands.Hands(
            model_complexity=0,  # Use fastest model
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            max_num_hands=1) as hands:
            
            while self.running:
                current_time = time.time()
                elapsed = current_time - self.last_frame_time
                
                # Frame rate control
                if elapsed < self.frame_time:
                    time.sleep(self.frame_time - elapsed)
                
                ret, frame = cap.read()
                if not ret:
                    break
                
                # Process frame
                frame = cv2.resize(frame, (640, 480))  # Ensure consistent size
                results, frame_display = self.gripper_controller.process_frame(frame, hands)
                
                if results and results.multi_hand_landmarks:
                    # Draw hand landmarks
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.gripper_controller.mp_drawing.draw_landmarks(
                            frame_display,
                            hand_landmarks,
                            self.gripper_controller.mp_hands.HAND_CONNECTIONS,
                            self.gripper_controller.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.gripper_controller.mp_drawing_styles.get_default_hand_connections_style())
                        
                        # Get and send gripper angle
                        servo_angle = self.gripper_controller.get_gripper_angle(hand_landmarks)
                        
                        # Update gripper status
                        if servo_angle[0] == GRIPPER_OPEN_ANGLE:
                            self.gripper_status.set("Gripper Status: OPEN")
                            self.update_gripper_indicator("green")
                        else:
                            self.gripper_status.set("Gripper Status: CLOSED")
                            self.update_gripper_indicator("red")
                        
                        # Only try to write to serial if we have a connection and not in debug mode
                        if (not self.gripper_controller.debug and 
                            hasattr(self.gripper_controller, 'ser') and 
                            self.gripper_controller.ser and 
                            self.gripper_controller.ser.is_open):
                            self.gripper_controller.ser.write(bytearray(servo_angle))
                else:
                    # No hand detected
                    self.gripper_status.set("Gripper Status: No Hand Detected")
                    self.update_gripper_indicator("gray")
                
                # Convert frame to PhotoImage and display
                frame_rgb = cv2.cvtColor(frame_display, cv2.COLOR_BGR2RGB)
                frame_pil = Image.fromarray(frame_rgb)
                frame_tk = ImageTk.PhotoImage(image=frame_pil)
                
                self.camera_label.configure(image=frame_tk)
                self.camera_label.image = frame_tk
                
                # Update FPS counter
                self.update_fps()
                self.last_frame_time = current_time
                
        cap.release()
        self.camera_label.configure(image='')
        self.gripper_status.set("Gripper Status: Not Active")
        self.update_gripper_indicator("gray")
        
if __name__ == "__main__":
    root = tk.Tk()
    app = GripperControlGUI(root)
    root.mainloop() 