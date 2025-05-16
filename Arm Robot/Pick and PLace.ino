/*
    Pick and place with PCA9685 - Smooth Movement Edition
    Enhanced for smoother servo movements
*/

#include <math.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define square(x) ((x)*(x))
#define cube(x) ((x)*(x)*(x))

// PCA9685 I2C address (default is 0x40)
#define PCA9685_ADDR 0x40

// Servo pin assignments
#define BASE_PIN 0
#define SHOULDER_PIN 1
#define ELBOW_PIN 2
#define END_EFFECTOR_PIN 3

#define SERVO_MIN_POS 0
#define SERVO_MAX_POS 180

#define MIN_PWM 135
#define MAX_PWM 520
#define FREQUENCY 50

#define BASE_OFFSET 0
#define SHOULDER_OFFSET 0
#define ELBOW_OFFSET 0
#define END_EFFECTOR_OFFSET 0

#define JOINTS 3

// Movement parameters
#define DEFAULT_SPEED 0.8     // 0.0 to 1.0 (slower to faster)
#define ACCELERATION_STEPS 20 // Number of steps for acceleration/deceleration
#define MIN_UPDATE_INTERVAL 10 // Minimum time between servo updates (ms)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

typedef struct {
    uint8_t pin;
    int offset;
    int min_pos;
    int max_pos;
} RobotServo_t;

typedef struct {
    double a;
    double b;
    double c;
    double d;
} CubicPol_t;

typedef struct {
    double l0;
    double h1;
    double l1;
    double l2;
    double l3;
    double l3I;
    double l3O;
    double l4;
    double l5;
    double d5;
} RobotParams_t;

typedef struct {
    double x;
    double y;
    double z;
} RobotPosition_t;

double q_current[JOINTS] = {0, 0, 0};
double q_target[JOINTS] = {0, 0, 0};

RobotPosition_t currentPos = {0, 0, 0};
RobotPosition_t homePos = {0, 0, 0};

RobotServo_t robotServos[JOINTS] = {{BASE_PIN, BASE_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS},
                                    {SHOULDER_PIN, SHOULDER_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS},
                                    {ELBOW_PIN, ELBOW_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS}};

RobotServo_t endEffector = {END_EFFECTOR_PIN, END_EFFECTOR_OFFSET, SERVO_MIN_POS, SERVO_MAX_POS};

// Updated robot parameters with 2cm height from ground
RobotParams_t params = {0, 30, 15, 80, 80, 35, 35, 80, 65, 5};

// Home configuration with elbow inverted (180 to 0)
double q_home[JOINTS] = {90, 90, 180};
int end_effector_angle = 120;

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing PCA9685...");
    
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);
    delay(1000);

    // Moves all servos to home position smoothly
    for(int i=0; i<JOINTS; i++) {
        q_current[i] = q_home[i];
        writeServo(robotServos[i], (int)q_current[i]);
        delay(100); // Small delay between servo initialization
    }
    writeServo(endEffector, end_effector_angle);
    forwardKin(q_current, params, &currentPos);

    homePos.x = currentPos.x;
    homePos.y = currentPos.y;
    homePos.z = currentPos.z;

    Serial.println("Moving to home position");
    delay(2000);

    printPos();

    // Example positions - adjust these as needed for your application
    RobotPosition_t pos1 = {140, -5, 144};
    RobotPosition_t pos2 = {140, -5, 30};
    RobotPosition_t pos3 = {200, -5, 30};
    RobotPosition_t pos4 = {200, -5, 40};
    RobotPosition_t pos5 = {140, 140, 100};
    RobotPosition_t pos6 = {0, 300, 200};

    // Execute movements with smoother transitions
    smoothMoveL(pos1, 2.0);
    end_effector_grab();
    smoothMoveL(pos2, 2.0);
    smoothMoveL(pos3, 2.0);
    smoothMoveL(pos4, 1.5);
    smoothMoveL(pos5, 2.0);
    smoothMoveL(pos6, 2.5);
    end_effector_release();
    smoothMoveL(homePos, 2.5);

    delay(2000);
    // Detaches all servos
    for(int i=0; i<JOINTS; i++)
        detachServo(robotServos[i]);
    detachServo(endEffector);
    
    Serial.println("Detached Servos");
}

void loop() {}

// New smoother movement implementation
void smoothMoveL(const RobotPosition_t &target, float duration) {
    RobotPosition_t startPos = currentPos;
    double startAngles[JOINTS];
    double targetAngles[JOINTS];
    
    // Get current angles
    for(int i=0; i<JOINTS; i++) {
        startAngles[i] = q_current[i];
    }
    
    // Calculate target angles
    inverseKin(target, params, targetAngles);
    targetAngles[2] = 180 - targetAngles[2]; // Invert elbow angle
    
    // Calculate total distance for each joint
    double distances[JOINTS];
    double maxDistance = 0;
    for(int i=0; i<JOINTS; i++) {
        distances[i] = abs(targetAngles[i] - startAngles[i]);
        if(distances[i] > maxDistance) {
            maxDistance = distances[i];
        }
    }
    
    // Adjust duration based on the maximum movement distance
    duration = max(duration, maxDistance / (180.0 * DEFAULT_SPEED));
    
    unsigned long startTime = millis();
    unsigned long endTime = startTime + (duration * 1000);
    unsigned long now = millis();
    
    while(now < endTime) {
        float progress = (float)(now - startTime) / (duration * 1000);
        
        // Apply easing function (cubic ease-in-out)
        if (progress < 0.5) {
            progress = 2 * square(progress);
        } else {
            progress = -1 + (4 - 2 * progress) * progress;
        }
        
        // Calculate intermediate angles
        for(int i=0; i<JOINTS; i++) {
            double angle = startAngles[i] + progress * (targetAngles[i] - startAngles[i]);
            writeServo(robotServos[i], angle);
            q_current[i] = angle;
        }
        
        // Update current position for debugging
        forwardKin(q_current, params, &currentPos);
        
        // Wait for next update
        delay(MIN_UPDATE_INTERVAL);
        now = millis();
    }
    
    // Ensure final position is exactly the target
    for(int i=0; i<JOINTS; i++) {
        writeServo(robotServos[i], targetAngles[i]);
        q_current[i] = targetAngles[i];
    }
    
    forwardKin(q_current, params, &currentPos);
    Serial.println("Movement completed");
}

// Rest of your existing functions remain the same (forwardKin, inverseKin, etc.)
// [Previous implementations of forwardKin, inverseKin, writeServo, detachServo, dist, printPos, etc.]

void forwardKin(const double q[JOINTS], const RobotParams_t &params, RobotPosition_t *target)
{
    double q1 = q[0]*PI/180;
    double q2 = (180 - q[2])*PI/180; // Invert elbow angle for calculations
    double q3 = q[1]*PI/180;

    double phi = q3 + q2 - PI/2;
    double f = sqrt(square(params.l3I) + square(params.l2) - 2*params.l3I*params.l2*cos(phi));

    double mu = acos( (square(f) + square(params.l3O) - square(params.l4)) / (2*params.l3O*f) );
    double q30 = asin(params.l3I*sin(phi) / f) + mu;

    double r = -params.l2 * cos(q2) - params.l3 * cos(q2+q30);

    target->x = params.l0 + (r+params.l1+params.l5)*sin(q1) - params.d5*cos(q1);
    target->y = -(r+params.l1+params.l5)*cos(q1) - params.d5*sin(q1);
    target->z = params.h1 + params.l2*sin(q2) + params.l3*sin(q2+q30);
}

void inverseKin(const RobotPosition_t &target, const RobotParams_t &params, double *q)
{
    // Step 1
    double q1 = atan2(target.x-params.l0, -target.y);
    q1 += asin( params.d5 / sqrt(square((target.x-params.l0)) + square(target.y)) );

    double pwx = target.x - params.l5 * sin(q1) - params.l0;
    double pwy = target.y + params.l5 * cos(q1);
    double pwz = target.z;

    // Step 2
    double r = sqrt(square(pwx) + square(pwy)) - params.l1;
    double ze = pwz - params.h1;
    double alpha = atan2(ze, r);
    double s = sqrt(square(r) + square(ze));

    double q30 = PI - acos((square(params.l3) + square(params.l2) - square(s)) / (2*params.l2*params.l3) );

    double q2 = PI - alpha - acos((square(s) + square(params.l2) - square(params.l3)) / (2*s*params.l2) );

    // Step 3
    double e = sqrt(square(params.l3O) + square(params.l2) - 2*params.l2*params.l3O*cos(q30) );

    double psi = asin(params.l3O * sin(q30) / e);
    double phi = acos((square(e) + square(params.l3I) - square(params.l4)) / (2*e*params.l3I));

    double q3 = psi + phi + PI/2 - q2;

    q[0] = round(q1 * 180 / PI);
    q[1] = round(q3 * 180 / PI);
    q[2] = round(q2 * 180 / PI); // This will be inverted in moveL function
}

void moveJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const RobotPosition_t target, const float T, const RobotParams_t &params)
{
    inverseKin(target, params, q_target);
    moveAbsJ(robotServos, q0, q_target, T);

    for(int j=0; j<JOINTS; j++)
        q_current[j] = q_target[j];
}

void moveAbsJ(const RobotServo_t robotServos[JOINTS], const double q0[JOINTS], const double qT[JOINTS], double T)
{
    CubicPol_t robot_pol[JOINTS] = {{0.0, 0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0, 0.0},
                                    {0.0, 0.0, 0.0, 0.0}};
    
    double angle[JOINTS] = {0.0, 0.0, 0.0};
    
    for(int i=0; i<JOINTS; i++)
        computePol(robot_pol[i], q0[i], qT[i], T);
    
    unsigned long time_ms = 0;
    unsigned long base_time = millis();

    while(1)
    {
        time_ms = millis() - base_time;
for(int i=0; i<JOINTS; i++)
        {
            angle[i] = evaluatePol(robot_pol[i], time_ms);
            writeServo(robotServos[i], angle[i]);
        }

        if (time_ms>1000) 
            break;

        delay(20);
    }
}

void computePol(CubicPol_t &pol, int q0, int qT, double T)
{
    pol.a = (-2.0*(qT-q0)) / (cube(T));
    pol.b = (3.0*(qT-q0)) / (square(T));
    pol.c = 0.0;
    pol.d = 1.0 * q0;
}

void computePol(CubicPol_t &pol, RobotPosition_t p0, RobotPosition_t pT, double T)
{
    pol.a = -2.0*dist(p0, pT) / (cube(T));
    pol.b = 3*dist(p0, pT) / (square(T));
    pol.c = 0;
    pol.d = 0;
}

double evaluatePol(CubicPol_t &pol, unsigned long time_ms)
{
    double t = (1.0 * time_ms) / 1000;
    return pol.a * (cube(t)) + pol.b * (square(t)) + pol.c * t + pol.d;
}

void writeServo(const RobotServo_t &servo, int angle)
{
    int pulse_width;
    angle = constrain(angle, servo.min_pos, servo.max_pos);
    pulse_width = map(angle+servo.offset, 0, 180, MIN_PWM, MAX_PWM);
    pwm.setPWM(servo.pin, 0, pulse_width);
}

void detachServo(RobotServo_t &servo)
{
    pwm.setPWM(servo.pin, 0, 0);
}

double dist(RobotPosition_t p1, RobotPosition_t p2)
{
    return sqrt(square(p1.x-p2.x) + square(p1.y-p2.y) + square(p1.z-p2.z));
}

void printPos()
{
    Serial.print("Current Position (x, y, z): ");
    Serial.print(currentPos.x);
    Serial.print(", ");
    Serial.print(currentPos.y);
    Serial.print(", ");
    Serial.println(currentPos.z);
}

void end_effector_grab() {
    int startAngle = end_effector_angle;
    int targetAngle = 10;
    unsigned long duration = 1000; // 1 second for grab
    
    unsigned long startTime = millis();
    unsigned long endTime = startTime + duration;
    unsigned long now = millis();
    
    while(now < endTime) {
        float progress = (float)(now - startTime) / duration;
        end_effector_angle = startAngle + progress * (targetAngle - startAngle);
        writeServo(endEffector, end_effector_angle);
        delay(20);
        now = millis();
    }
    
    end_effector_angle = targetAngle;
    writeServo(endEffector, end_effector_angle);
}

void end_effector_release() {
    int startAngle = end_effector_angle;
    int targetAngle = 120;
    unsigned long duration = 800; // 0.8 second for release
    
    unsigned long startTime = millis();
    unsigned long endTime = startTime + duration;
    unsigned long now = millis();
    
    while(now < endTime) {
        float progress = (float)(now - startTime) / duration;
        end_effector_angle = startAngle + progress * (targetAngle - startAngle);
        writeServo(endEffector, end_effector_angle);
        delay(20);
        now = millis();
    }
    
    end_effector_angle = targetAngle;
    writeServo(endEffector, end_effector_angle);
}