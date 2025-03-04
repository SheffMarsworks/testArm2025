const double kP = 1.0;
const double kI = 0.0;
const double kD = 0.0;

const double setpoint = 100;

/*
void setup() {
  Serial.begin(9600); 
  Serial.println("Enter a degree from 0 to 90:");
}

void loop() {
  if (Serial.available() > 0) { 
    String inputString = Serial.readStringUntil('\n'); 
    int inputInteger = inputString.toInt(); 
    Serial.print("Entered Value: ");
    Serial.println(inputInteger);

    //Set motor power to getPIDCalculation
    
  }
}*/

double getEncoderDegrees() {
	return 0.0;
}


// all the necessary calculation to set the motor
double getPIDCalculation(double kP,double kI,double kD, int setpoint){
  double currentValue = getEncoderDegrees(); //To be implemented
  double error = setpoint - currentValue;
  
  return kP * currentValue * error;
}

