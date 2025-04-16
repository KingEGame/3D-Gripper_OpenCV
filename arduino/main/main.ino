#include <Servo.h>

// Configuration
#define GRIPPER_SERVO_PIN 5  // Servo motor pin for gripper
#define SERIAL_BAUD_RATE 115200

// MG90S servo specifications
#define SERVO_MIN_PULSE 500    // Minimum pulse width (microseconds)
#define SERVO_MAX_PULSE 2400   // Maximum pulse width (microseconds)
#define SERVO_MIN_ANGLE 10      // Minimum angle (matching Python)
#define SERVO_MAX_ANGLE 170    // Maximum angle (matching Python)

// Gripper angles - Matching Python configuration
#define GRIPPER_OPEN_ANGLE 170      // Open position (0 degrees)
#define GRIPPER_CLOSE_ANGLE 10   // Closed position (160 degrees)
#define GRIPPER_DEFAULT_ANGLE 10 // Start from closed position

// Smooth movement settings
#define STEP_DELAY 15            // Faster movement
#define STEPS_PER_MOVE 10       // Fewer steps for quicker response
#define ACCELERATION_STEPS 3     // Shorter acceleration phase

// Safety settings
#define MAX_ANGLE_CHANGE_PER_STEP 5    // Larger steps allowed
#define POSITION_TOLERANCE 2            // Tighter position control
#define STALL_TIMEOUT 500              // Shorter timeout
#define DIRECTION_CHANGE_DELAY 50       // Shorter direction change delay

// Servo object
Servo gripperServo;

// Variables for tracking angle
byte current_angle;
byte target_angle;
bool is_moving = false;
unsigned long last_step_time = 0;
unsigned long movement_start_time = 0;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Attach gripper servo
    gripperServo.attach(GRIPPER_SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    
    // Initialize to closed position
    gripperServo.write(GRIPPER_DEFAULT_ANGLE);
    current_angle = GRIPPER_DEFAULT_ANGLE;
    target_angle = GRIPPER_DEFAULT_ANGLE;
    
    delay(500); // Initial delay for servo to reach position
}

void loop() {
    if (Serial.available() > 0) {
        target_angle = Serial.read();
        
        // Constrain angle to valid range
        target_angle = constrain(target_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        
        if (target_angle != current_angle) {
            is_moving = true;
            movement_start_time = millis();
            last_step_time = millis();
        }
    }
    
    // Handle movement
    if (is_moving && millis() - last_step_time >= STEP_DELAY) {
        int remaining_distance = target_angle - current_angle;
        
        // Check for stall condition
        if (millis() - movement_start_time > STALL_TIMEOUT) {
            is_moving = false;
            return;
        }
        
        // Calculate step size
        int step_size = remaining_distance > 0 ? 
            min(MAX_ANGLE_CHANGE_PER_STEP, remaining_distance) : 
            max(-MAX_ANGLE_CHANGE_PER_STEP, remaining_distance);
        
        // Move one step
        current_angle += step_size;
        
        // Check if we've reached target
        if (abs(current_angle - target_angle) <= POSITION_TOLERANCE) {
            current_angle = target_angle;
            is_moving = false;
        }
        
        // Update servo position
        gripperServo.write(current_angle);
        last_step_time = millis();
    }
} 