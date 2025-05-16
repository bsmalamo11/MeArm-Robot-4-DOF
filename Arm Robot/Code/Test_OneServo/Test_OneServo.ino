#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Adjust these as needed for your specific servo
#define SERVO_MIN 135  // Pulse length for 0 degrees
#define SERVO_MAX 520  // Pulse length for 180 degrees

int currentAngle = 90;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz

  delay(10);  // Short delay after initializing

  // Move servo to initial position
  pwm.setPWM(0, 0, angleToPulse(currentAngle));
  Serial.print(" Servo set to initial angle: ");
  Serial.println(currentAngle);

  Serial.println("\n Type angle commands in Serial Monitor:");
  Serial.println(" 90   -> move to 90 degrees");
  Serial.println(" -30  -> move 30 degrees back");
  Serial.println(" +45  -> move 45 degrees forward");
}

// Converts angle to pulse length
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    int targetAngle;

    if (input.startsWith("-") || input.startsWith("+")) {
      int delta = input.toInt();
      targetAngle = currentAngle + delta;
    } else {
      targetAngle = input.toInt();
    }

    targetAngle = constrain(targetAngle, 0, 180);

    pwm.setPWM(0, 0, angleToPulse(targetAngle));
    currentAngle = targetAngle;

    Serial.print(" Moved to ");
    Serial.print(currentAngle);
    Serial.println(" degrees.");
  }
}