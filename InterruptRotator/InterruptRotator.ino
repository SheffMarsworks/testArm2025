// -----
// InterruptRotator.ino - Example for the RotaryEncoder library.
// This class is implemented for use with the Arduino environment.
//
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD 3-Clause License. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 04.02.2021 conditions and settings added for ESP8266
// 03.07.2022 avoid ESP8266 compiler warnings.
// 03.07.2022 encoder instance not static.
// -----

// This example checks the state of the rotary encoder using interrupts and in the loop() function.
// The current position and direction is printed on output when changed.

// Hardware setup:
// Attach a rotary encoder with output pins to
// * 2 and 3 on Arduino UNO. (supported by attachInterrupt)
// * A2 and A3 can be used when directly using the ISR interrupts, see comments below.
// * D5 and D6 on ESP8266 board (e.g. NodeMCU).
// Swap the pins when direction is detected wrong.
// The common contact should be attached to ground.
//
// Hints for using attachinterrupt see https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

#include <Arduino.h>
#include <RotaryEncoder.h>

// Example for Arduino UNO with input signals on pin 2 and 3
#define PIN_IN1 21
#define PIN_IN2 20



// Constants for motor control
#define FORWARD 1
#define REVERSE 0

// Motor driver pins (example configuration)
#define MOTOR1_PWM 2  // PWM pin for motor 1
#define MOTOR1_DIR 24  // Direction pin for motor 1



// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

// This interrupt routine will be called on any change of one of the input signals
void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}


void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("InterruptRotator example for the RotaryEncoder library.");

  // setup the rotary encoder functionality

  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);

  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);
} // setup()


// Function to control motor speed and direction
void motor_control(int motor_ID, int direction, float speed, float acc) {
    int pwmValue = constrain(speed * 255 / 100, 0, 255);  // Convert speed (0-100%) to PWM

    if (motor_ID == 1) {
        digitalWrite(MOTOR1_DIR, direction == FORWARD ? HIGH : LOW); // Set direction
        analogWrite(MOTOR1_PWM, pwmValue); // Set speed
        delay(acc * 100);  // Simulate acceleration
    }
}


// Read the current position of the encoder and print out when changed.
void loop()
{
  motor_control(1, FORWARD, 60, 1.0);

  static int pos = 0;

  encoder->tick(); // just call tick() to check the state.

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
    pos = newPos;
  } // if
  Serial.print("8:");
    Serial.print(analogRead(PIN_IN1));
    Serial.print(" 9:");
    Serial.println(analogRead(PIN_IN2));


} // loop ()


// To use other pins with Arduino UNO you can also use the ISR directly.
// Here is some code for A2 and A3 using ATMega168 ff. specific registers.

// Setup flags to activate the ISR PCINT1.
// You may have to modify the next 2 lines if using other pins than A2 and A3
//   PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
//   PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3.
// ISR(PCINT1_vect) {
//   encoder->tick(); // just call tick() to check the state.
// }

// The End
