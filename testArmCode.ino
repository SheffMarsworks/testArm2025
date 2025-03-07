// Constants for motor control
#define FORWARD 1
#define REVERSE 0

// Motor driver pins (example configuration)
#define MOTOR1_PWM 2  // PWM pin for motor 1
#define MOTOR1_DIR 24  // Direction pin for motor 1
#define MOTOR2_PWM 9  // PWM pin for motor 2
#define MOTOR2_DIR 26 // Direction pin for motor 2
#define MOTOR3_PWM 11 // PWM pin for motor 3
#define MOTOR3_DIR 12 // Direction pin for motor 3
#define HOME_SENSOR_PIN 3  // Home position sensor pin

// enter the pin
#define encoderPinA 31
#define encoderPinB 40

//The gearRatio of the motor gearbox
const int gearRatio = 64; 

volatile int encoderCountAnalog; // Counter variable
volatile int encoderDirection;



void setup() {
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR3_DIR, OUTPUT);
    pinMode(HOME_SENSOR_PIN, INPUT);

    pinMode(encoderPinA, INPUT_PULLUP); // Set pins as input with internal pullup
    pinMode(encoderPinB, INPUT_PULLUP);

    pinMode(A0, INPUT);


    Serial.begin(300);
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

  encoderCountAnalog = analogRead(A0);

    printEncoderSignals();
    //Controller inputs needs to be added
    motor_control(1, FORWARD, 60, 1.0);
    delay(5000);
    /*motor_control(1, REVERSE, 30, 0.5);
    delay(5000);

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
    motor_angle_control(3, 90, 10, 1);*/
    
}


void oneMotorTest(){

}

double getEncoderDegrees(int rawSignals){
  // 7 is the number of signals per rotation 
  double rotations = (double) rawSignals / 7;
  double outputRotation = rotations / (double) gearRatio;
  double degrees = outputRotation * 360.0;

  return degrees;
}

void checkHomePosition(){
  if (home_position() == 1) {
    Serial.println("Motor is in home position.");
  } else {
    Serial.println("Motor is NOT in home position.");
  }
}


int getEncoderSignals(){
  int currentAState = digitalRead(encoderPinA);
  int currentBState = digitalRead(encoderPinB);

  if (currentAState == LOW && currentBState == HIGH) { // Check for rotation direction
    encoderDirection++; 

  } else if (currentAState == HIGH && currentBState == LOW) {
    encoderDirection--;

  }

  return encoderDirection;
}

void printEncoderSignals(){
  Serial.println("RAW ENCODER SIGNALS: "+ (String)encoderCountAnalog+"\n");
  Serial.println("DEGREES: " + (String) getEncoderDegrees(encoderCountAnalog)+"\n");
}




//PID algorithm trial
void pidMotorAngleControl(int motor_ID, int setpoint, double currentPos, double kp, double ki, double kd) {
  double error = setpoint - currentPos;
  static double previousError[3] = {0, 0, 0};
  static double integral[3] = {0, 0, 0};

  int motorIndex = motor_ID - 1;

  integral[motorIndex] += error;
  integral[motorIndex] = constrain(integral[motorIndex], -1000, 1000);
  double integralTerm = ki * integral[motorIndex];

  double derivative = kd * (error - previousError[motorIndex]);

  double pidOutput = kp * error + integralTerm + derivative;

  int pwmOutput = constrain(pidOutput, -100, 100);

  int direction = pwmOutput >= 0 ? 1 : 0; // Assuming 1 is FORWARD, 0 is BACKWARD
  float speed = abs(pwmOutput);
  motor_control(motor_ID, direction, speed, 0);
  previousError[motorIndex] = error;
}





