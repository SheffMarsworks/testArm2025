// Define encoder pins
/*
const int encoderPinA = 2;

const int encoderPinB = 3;



volatile int encoderCount = 0; // Counter variable

int lastAState = LOW; // Variable to store previous state of pin A



void setup() {

  pinMode(encoderPinA, INPUT_PULLUP); // Set pins as input with internal pullup

  pinMode(encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), checkEncoder, FALLING); // Attach interrupt on falling edge of pin A

}



void loop() {

  // Read encoder values and update counter here

  // ... 

}



void checkEncoder() {

  int currentAState = digitalRead(encoderPinA);

  int currentBState = digitalRead(encoderPinB);

  

  if (currentAState == LOW && currentBState == HIGH) { // Check for rotation direction

    encoderCount++; 

  } else if (currentAState == HIGH && currentBState == LOW) {

    encoderCount--;

  }


  lastAState = currentAState; // Update last state

}
*/