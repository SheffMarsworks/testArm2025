// Constants for motor control
#define FORWARD 1
#define REVERSE 0

// Motor driver pins (example configuration)
#define MOTOR1_PWM 5  // PWM pin for motor 1
#define MOTOR1_DIR 6  // Direction pin for motor 1
#define MOTOR2_PWM 9  // PWM pin for motor 2
#define MOTOR2_DIR 10 // Direction pin for motor 2
#define MOTOR3_PWM 11 // PWM pin for motor 3
#define MOTOR3_DIR 12 // Direction pin for motor 3
#define HOME_SENSOR_PIN 3  // Home position sensor pin

// enter the pin
#define ENCODER_PWM_A 0
#define ENCODER_PWM_B 0

void setup() {
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(HOME_SENSOR_PIN, INPUT);

    pinMode(ENCODER_PWM_A, INPUT);
    pinMode(ENCODER_PWM_B, INPUT);


    Serial.begin(9600);
}

//we need to accellrate/deccelerate smooth

// Function to control motor speed and direction
void motor_control(int motor_ID, int direction, float speed, float acc) {
    int pwmValue = constrain(speed * 255 / 100, 0, 255);  // Convert speed (0-100%) to PWM

    if (motor_ID == 1) {
        digitalWrite(MOTOR1_DIR, direction == FORWARD ? HIGH : LOW); // Set direction
        analogWrite(MOTOR1_PWM, pwmValue); // Set speed
        delay(acc * 100);  // Simulate acceleration
    } else if (motor_ID == 2) {
        digitalWrite(MOTOR2_DIR, direction == FORWARD ? HIGH : LOW); // Set direction
        analogWrite(MOTOR2_PWM, pwmValue); // Set speed
        delay(acc * 100);  // Simulate acceleration
    } else if (motor_ID == 3) {
        digitalWrite(MOTOR3_DIR, direction == FORWARD ? HIGH : LOW); // Set direction
        analogWrite(MOTOR3_PWM, pwmValue); // Set speed
        delay(acc * 100);  // Simulate acceleration
    }
}

// Function to move motor to a specific angle
void motor_angle_control(int motor_ID, float angle, float speed, float acc) {
    int steps = (int)(angle / 1.8);  // Assuming 1.8° per step (modify as per motor spec)
    int delayBetweenSteps = 1000 / (speed * steps);  // Calculate step delay based on speed

    for (int i = 0; i < steps; i++) {
        if (motor_ID == 1) {
            digitalWrite(MOTOR1_DIR, FORWARD);
            analogWrite(MOTOR1_PWM, 200); // Sample speed PWM
        } else if (motor_ID == 2) {
            digitalWrite(MOTOR2_DIR, FORWARD);
            analogWrite(MOTOR2_PWM, 200); // Sample speed PWM
        } else if (motor_ID == 3) {
            digitalWrite(MOTOR3_DIR, FORWARD);
            analogWrite(MOTOR3_PWM, 200); // Sample speed PWM
        }
        delay(delayBetweenSteps); // Wait before the next step
    } //will pospone till finished with electronics
}

// Function to check if motor is in the home position
int home_position() {
    int success = digitalRead(HOME_SENSOR_PIN);  // Read from the home position sensor
    if (success == HIGH) {
        return 1;  // Home position reached
    } else {
        return 0;  // Not in home position
    } // in home position function add movement for each joint and once it reaches the limit switch reset. call home position function till the switch is triggered..
}

// Main loop
void loop() {
    //Controller inputs needs to be added
    motor_control(1, FORWARD, 50, 1.0);
    delay(2000);
    motor_control(1, REVERSE, 30, 0.5);
    delay(2000);

    motor_control(2, FORWARD, 50, 1.0);
    delay(2000);
    motor_control(2, REVERSE, 30, 0.5);
    delay(2000);

    motor_control(3, FORWARD, 50, 1.0);
    delay(2000);
    motor_control(3, REVERSE, 30, 0.5);
    delay(2000);

    motor_angle_control(1, 90, 10, 1);
    motor_angle_control(2, 90, 10, 1);
    motor_angle_control(3, 90, 10, 1);

    if (home_position() == 1) {
        Serial.println("Motor is in home position.");
    } else {
        Serial.println("Motor is NOT in home position.");
    }
    delay(1000);
}


void oneMotorTest(){

}


int getEncoderSignals(){
  Serial.println(digitalRead(ENCODER_PWM_A));
}


