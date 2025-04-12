# 3D-Gripper_OpenCV

A project for creating and controlling a 3D gripper model using OpenCV and computer vision. This project combines hardware control with computer vision to create an interactive gripper system.

## Project Structure

```
3D-Gripper_OpenCV/
├── arduino/         # Arduino control code
├── python/          # Python vision and control code
├── images/          # Project images and documentation
└── sketch_apr11a/   # Additional sketches and models
```

## Prerequisites

- Python 3.7 or higher
- Arduino IDE
- A compatible webcam or camera device
- Arduino board (compatible with the gripper hardware)
- USB cable for Arduino communication

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/3D-Gripper_OpenCV.git
cd 3D-Gripper_OpenCV
```

2. Install Python dependencies:
```bash
cd python
pip install -r requirements.txt
```

3. Arduino Setup:
   - Open the Arduino IDE
   - Navigate to the `arduino` folder in this project
   - Open the main sketch file
   - Install required Arduino libraries through the Library Manager
   - Select your Arduino board and port
   - Upload the code to your Arduino

## Configuration

1. Camera Setup:
   - Connect your webcam or camera device
   - Update the camera source in the Python code:
   ```python
   # For USB webcam
   cam_source = 0  # or 1 for secondary camera
   
   # For IP camera or DroidCam
   cam_source = "http://your_camera_ip:port/video"
   ```

2. Arduino Communication:
   - Update the COM port in the Python code to match your Arduino:
   ```python
   ser = serial.Serial('COM4', 115200)  # Change COM4 to your port
   ```

## Usage

1. Start the Python application:
```bash
cd python
python main.py
```

2. The system will:
   - Initialize the camera
   - Connect to the Arduino
   - Begin processing video feed
   - Control the gripper based on visual input

## Development

For contributing to this project:
1. Create your own branch:
```bash
git checkout -b feature/your-feature-name
```

2. Make your changes
3. Test thoroughly
4. Create a pull request to merge into main

## Troubleshooting

- If the gripper is not responding, check:
  - Arduino connection and port
  - Camera connection and configuration
  - Python dependencies installation
  - Serial communication settings

- For camera issues:
  - Verify camera permissions
  - Check camera index/URL
  - Ensure proper lighting conditions

## License

[Add your license information here]

## Contributing

Please read our contributing guidelines before submitting pull requests. Create your own branch or folder for your changes before merging into main.

## Contact

[Add contact information here]
