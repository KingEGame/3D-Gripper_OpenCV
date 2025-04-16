#include <Servo.h>

// Configuration
#define GRIPPER_SERVO_PIN 5  // Servo motor pin for gripper
#define SERIAL_BAUD_RATE 115200

// MG90S servo specifications
#define SERVO_MIN_PULSE 500    // Minimum pulse width (microseconds)
#define SERVO_MAX_PULSE 2400   // Maximum pulse width (microseconds)
#define SERVO_MIN_ANGLE 10     // Safe minimum angle for MG90S
#define SERVO_MAX_ANGLE 170    // Safe maximum angle for MG90S

// Gripper angles - Calibrated for gear mechanism and MG90S limits
#define GRIPPER_OPEN_ANGLE 30     // Safe open position
#define GRIPPER_CLOSE_ANGLE 150   // Safe close position
#define GRIPPER_DEFAULT_ANGLE 30  // Start from open position

// Smooth movement settings
#define STEP_DELAY 25            // Delay between steps
#define STEPS_PER_MOVE 20       // Number of steps for full movement
#define ACCELERATION_STEPS 5     // Steps for acceleration/deceleration

// Safety settings
#define MAX_ANGLE_CHANGE_PER_STEP 3    // Maximum angle change per step
#define POSITION_TOLERANCE 3            // Acceptable position error
#define STALL_TIMEOUT 1000             // Maximum time to reach position (ms)
#define DIRECTION_CHANGE_DELAY 100      // Delay when changing direction (ms)

// Servo object
Servo gripperServo;

// Variables for tracking angle
byte current_angle;
byte previous_angle;
byte target_angle;
bool is_moving = false;
bool changing_direction = false;
unsigned long last_step_time = 0;
unsigned long last_command_time = 0;
unsigned long movement_start_time = 0;
const unsigned long COMMAND_TIMEOUT = 5000;
int current_step = 0;

// Function to calculate dynamic step size based on acceleration curve
int calculateStepSize(int remaining_distance, int current_step) {
    // Acceleration phase
    if (current_step < ACCELERATION_STEPS) {
        return max(1, abs(remaining_distance) / (STEPS_PER_MOVE - current_step));
    }
    // Deceleration phase
    else if (current_step > STEPS_PER_MOVE - ACCELERATION_STEPS) {
        return max(1, abs(remaining_distance) / (STEPS_PER_MOVE - current_step));
    }
    // Constant speed phase
    else {
        return max(1, abs(remaining_distance) / (STEPS_PER_MOVE / 2));
    }
}

// Function to verify angle is within safe limits
bool isAngleSafe(int angle) {
    return (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE);
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Attach gripper servo with calibrated pulse width range for MG90S
    gripperServo.attach(GRIPPER_SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    
    // Verify default angle is safe
    if (!isAngleSafe(GRIPPER_DEFAULT_ANGLE)) {
        Serial.println("Warning: Default angle outside safe range");
        return;
    }
    
    // Initialize to default position
    gripperServo.write(GRIPPER_DEFAULT_ANGLE);
    current_angle = GRIPPER_DEFAULT_ANGLE;
    previous_angle = GRIPPER_DEFAULT_ANGLE;
    target_angle = GRIPPER_DEFAULT_ANGLE;
    
    delay(500); // Initial delay for servo to reach position
}

void loop() {
    if (Serial.available() > 0) {
        target_angle = Serial.read();
        
        // Safety check for received angle
        if (!isAngleSafe(target_angle)) {
            Serial.println("Error: Requested angle outside safe range");
            return;
        }
        
        // Constrain angle to valid range
        target_angle = constrain(target_angle, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);
        
        // Check if direction change is needed
        if ((target_angle > current_angle && previous_angle > current_angle) ||
            (target_angle < current_angle && previous_angle < current_angle)) {
            changing_direction = true;
            delay(DIRECTION_CHANGE_DELAY);
        }
        
        if (target_angle != current_angle) {
            is_moving = true;
            current_step = 0;
            movement_start_time = millis();
            last_step_time = millis();
            last_command_time = millis();
        }
    }
    
    // Handle smooth movement
    if (is_moving && millis() - last_step_time >= STEP_DELAY) {
        int remaining_distance = target_angle - current_angle;
        
        // Check for stall condition
        if (millis() - movement_start_time > STALL_TIMEOUT) {
            Serial.println("Warning: Movement timeout");
            is_moving = false;
            return;
        }
        
        // Calculate step size with acceleration
        int step_size = calculateStepSize(remaining_distance, current_step);
        step_size = constrain(step_size, -MAX_ANGLE_CHANGE_PER_STEP, MAX_ANGLE_CHANGE_PER_STEP);
        
        // Ensure we don't overshoot
        if (remaining_distance > 0) {
            step_size = min(step_size, remaining_distance);
        } else {
            step_size = max(-step_size, remaining_distance);
        }
        
        // Verify next position is safe
        int next_angle = current_angle + step_size;
        if (!isAngleSafe(next_angle)) {
            Serial.println("Error: Movement would exceed safe range");
            is_moving = false;
            return;
        }
        
        // Move one step
        current_angle = next_angle;
        current_step++;
        
        // Check if we've reached target
        if (abs(current_angle - target_angle) <= POSITION_TOLERANCE) {
            current_angle = target_angle;
            is_moving = false;
            changing_direction = false;
        }
        
        // Update servo position
        gripperServo.write(current_angle);
        previous_angle = current_angle;
        last_step_time = millis();
        
        // Debug output
        if (abs(current_angle - target_angle) > POSITION_TOLERANCE) {
            Serial.print("Position error: ");
            Serial.println(abs(current_angle - target_angle));
        }
    }
    
    // Safety timeout handling
    if (millis() - last_command_time > COMMAND_TIMEOUT && !is_moving) {
        if (current_angle != GRIPPER_DEFAULT_ANGLE) {
            target_angle = GRIPPER_DEFAULT_ANGLE;
            is_moving = true;
            current_step = 0;
            movement_start_time = millis();
            last_step_time = millis();
        }
    }
} 