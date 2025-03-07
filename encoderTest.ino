// ------------------------------------------------------------------------
//  Gimson Encoder with Quadrature Decoding (State Machine / Lookup Table)
// ------------------------------------------------------------------------

// Pins used for the encoder signals
#define ENCODER_PIN_A 20  // Must be an interrupt-capable pin on Arduino Uno/Nano
#define ENCODER_PIN_B 21  // Must be an interrupt-capable pin on Arduino Uno/Nano

// Constants for motor control
#define FORWARD 1
#define REVERSE 0

// Motor driver pins (example configuration)
#define MOTOR1_PWM 2  // PWM pin for motor 1
#define MOTOR1_DIR 24  // Direction pin for motor 1

// Motor gearbox ratio (if any)
const int GEAR_RATIO = 361;  

// 7 pulses per channel per revolution of the motor shaft => 28 counts/rev in full quadrature
const int MOTOR_COUNTS_PER_REV = 7 * 4;  
// If the motor has a 64:1 gearbox, total output-shaft counts per revolution:
const int OUTPUT_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV * GEAR_RATIO;  
// That would be 28 * 64 = 1792 for the example

// Volatile because it's changed inside an ISR
volatile long encoderCount = 0;

// We'll store the "old state" of the encoder in a static variable inside the ISR
// but you could also store it globally if you prefer.
  
// Quadrature decoder lookup table
// oldState (rows) vs newState (columns)
// Each cell is the increment (+1, -1, or 0) to apply to the encoder count.
static const int8_t QUAD_TABLE[4][4] = {
  // newState ->   0     1     2     3
  // oldState
  /*0*/ {  0,  +1,  -1,   0},
  /*1*/ { -1,   0,   0,  +1},
  /*2*/ { +1,   0,   0,  -1},
  /*3*/ {  0,  -1,  +1,   0}
};

// Function to control motor speed and direction
void motor_control(int motor_ID, int direction, float speed, float acc) {
  int pwmValue = constrain(speed * 255 / 100, 0, 255);  // Convert speed (0-100%) to PWM

  if (motor_ID == 1) {
      digitalWrite(MOTOR1_DIR, direction == FORWARD ? HIGH : LOW); // Set direction
      analogWrite(MOTOR1_PWM, pwmValue); // Set speed
      delay(acc * 100);  // Simulate acceleration
  }
}

void setup() {
  Serial.begin(9600);

  // Set up the encoder pins as inputs with pullups
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  // Attach interrupts on both pins, triggered on CHANGE
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), encoderISR, CHANGE);

  // ...set up any motor driver pins here...
}

// Example loop: print the encoder count & degrees every second
void loop() {

  motor_control(1, FORWARD, 60, 1.0);
  
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    long countSnapshot;
    noInterrupts();
    countSnapshot = encoderCount; // copy volatile variable while interrupts are disabled
    interrupts();
    
    // Convert raw counts to degrees of the output shaft (post-gearbox)
    double degrees = (double)countSnapshot / (double)OUTPUT_COUNTS_PER_REV * 360.0;
    
    Serial.print("Encoder Counts = ");
    Serial.print(countSnapshot);
    Serial.print("\t=> Degrees (output) = ");
    Serial.println(degrees, 2);
  }

  // ...other code, motor control, etc...
}

// ------------------------------------------------------------------------
// Interrupt Service Routine
// ------------------------------------------------------------------------
void encoderISR() {
  // We'll maintain the old state in a static variable.  
  // Each channel can be 0 or 1 => we combine them into a 2-bit number [0..3].
  static uint8_t oldState = 0;

  // Current state: bit1 = Channel A, bit0 = Channel B
  //   (shift A up by 1 bit, then OR with B)
  uint8_t newState = (digitalRead(ENCODER_PIN_A) << 1) | digitalRead(ENCODER_PIN_B);

  // Use the lookup table to find how much to add to encoderCount
  int8_t increment = QUAD_TABLE[oldState][newState];

  encoderCount += increment;
  oldState = newState;
}

// ------------------------------------------------------------------------
// Helper function (optional): Convert raw counts to degrees at motor or output
// ------------------------------------------------------------------------
double getOutputDegrees(long counts) {
  return ((double)counts / OUTPUT_COUNTS_PER_REV) * 360.0;
}