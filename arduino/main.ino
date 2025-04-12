#include <Servo.h>

// Configuration
#define GRIPPER_SERVO_PIN 5  // Servo motor pin for gripper
#define SERIAL_BAUD_RATE 115200
#define SAFETY_TIMEOUT 1000  // Time in milliseconds before returning to default position

// Gripper angles
#define GRIPPER_OPEN_ANGLE 180   // Angle when gripper is fully open
#define GRIPPER_CLOSE_ANGLE 0    // Angle when gripper is fully closed
#define GRIPPER_DEFAULT_ANGLE 180 // Default position (open)

// Servo object
Servo gripperServo;

// Variables for tracking angle
byte current_angle;
byte previous_angle;
long last_command_time = millis();

void setup()
{
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Attach gripper servo
    gripperServo.attach(GRIPPER_SERVO_PIN);
    gripperServo.write(GRIPPER_DEFAULT_ANGLE);
    current_angle = GRIPPER_DEFAULT_ANGLE;
    previous_angle = GRIPPER_DEFAULT_ANGLE;
}

void loop()
{
    // Check for incoming serial data
    if (Serial.available() > 0)
    {
        // Read new angle for gripper
        current_angle = Serial.read();
        
        // Constrain angle to valid range
        current_angle = constrain(current_angle, GRIPPER_CLOSE_ANGLE, GRIPPER_OPEN_ANGLE);
        
        // Update servo position if angle has changed
        if (current_angle != previous_angle)
        {
            gripperServo.write(current_angle);
            previous_angle = current_angle;
        }
        
        // Update last command time
        last_command_time = millis();
    }

    // Safety feature: Return to default position if no commands received
    if (millis() - last_command_time > SAFETY_TIMEOUT)
    {
        gripperServo.write(GRIPPER_DEFAULT_ANGLE);
        current_angle = GRIPPER_DEFAULT_ANGLE;
        previous_angle = GRIPPER_DEFAULT_ANGLE;
    }
}
