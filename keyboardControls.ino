int userDegrees = 0;

void setup() {
  Serial.begin(9600); 
  Serial.println("Type a letter: ");
}

void loop() {
  if (Serial.available() > 0) { // Check if data is available from the serial port
    String inputString = Serial.readStringUntil('\n'); // Read the input until a newline character
 
    int inputInteger = inputString.toInt(); // Convert the string to an integer
    Serial.print("You entered: ");
    Serial.println(inputInteger);
    userDegrees = inputInteger;
  }
}


int getUserInput(int userDegrees){
  return userDegrees;
}