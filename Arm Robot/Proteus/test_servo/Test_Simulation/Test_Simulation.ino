#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo minimum and maximum pulse lengths (in microseconds)
#define SERVOMIN  135
#define SERVOMAX  520

// Servo channels (adjust based on your connections)
#define BASE_CHANNEL      0
#define SHOULDER_CHANNEL  1
#define ELBOW_CHANNEL     2
#define GRIPPER_CHANNEL   3

// Initial positions
int basePos = 90;
int shoulderPos = 90;
int elbowPos = 90;
int gripperPos = 90;

void setup() {
  Serial.begin(9600);
  Serial.println("4-DOF Robotic Arm Initializing...");
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz
  
  // Move all servos to initial position
  setServoAngle(BASE_CHANNEL, basePos);
  setServoAngle(SHOULDER_CHANNEL, shoulderPos);
  setServoAngle(ELBOW_CHANNEL, elbowPos);
  setServoAngle(GRIPPER_CHANNEL, gripperPos);
  
  delay(2000);  // Allow servos to reach position
}

void loop() {
  // Example movement sequence
  moveServo(BASE_CHANNEL, 45, 20);    // Rotate base to 45 degrees
  moveServo(SHOULDER_CHANNEL, 120, 20); // Raise shoulder
  moveServo(ELBOW_CHANNEL, 60, 20);   // Bend elbow
  moveServo(GRIPPER_CHANNEL, 30, 10); // Close gripper
  
  delay(1000);
  
  // Return to initial positions
  moveServo(GRIPPER_CHANNEL, 90, 10); // Open gripper
  moveServo(ELBOW_CHANNEL, 90, 20);
  moveServo(SHOULDER_CHANNEL, 90, 20);
  moveServo(BASE_CHANNEL, 90, 20);
  
  delay(2000);
}

// Function to set servo angle
void setServoAngle(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// Function for smooth servo movement
void moveServo(uint8_t channel, int targetAngle, int speed) {
  int currentAngle;
  
  // Get current angle based on channel
  switch(channel) {
    case BASE_CHANNEL: currentAngle = basePos; break;
    case SHOULDER_CHANNEL: currentAngle = shoulderPos; break;
    case ELBOW_CHANNEL: currentAngle = elbowPos; break;
    case GRIPPER_CHANNEL: currentAngle = gripperPos; break;
    default: return;
  }
  
  // Move servo gradually
  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      setServoAngle(channel, pos);
      delay(speed);
    }
  } else {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      setServoAngle(channel, pos);
      delay(speed);
    }
  }
  
  // Update position
  switch(channel) {
    case BASE_CHANNEL: basePos = targetAngle; break;
    case SHOULDER_CHANNEL: shoulderPos = targetAngle; break;
    case ELBOW_CHANNEL: elbowPos = targetAngle; break;
    case GRIPPER_CHANNEL: gripperPos = targetAngle; break;
  }
}