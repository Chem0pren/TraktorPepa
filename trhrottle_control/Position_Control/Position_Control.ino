//pins
int ENA = 3; // enable pin 
int pwm[2] = {4, 5}; // motor driver pwm pins 
int encoder = A6; // encoder pin

//smoothing 
const int numReadings = 50;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;  



int PEDAL_IN1 = A0;
//constants
float offset = 0;
String input;

void setup() {
  pinMode(ENA, OUTPUT);
  Serial.begin(38400);
  moveTo(0);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
  readings[thisReading] = 0;
  }
}

void loop() {
  int PedalInput1 = analogRead(PEDAL_IN1);
  
  
    // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(PEDAL_IN1);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
 // Serial.println(average);
  delay(1);  // delay in between reads for stability


  int PedalAngle = map(PedalInput1, 74, 473, 0, 90);
  

  //Serial.println(PedalAngle);

  moveTo(PedalAngle);
  Serial.println(getPos());
  //inputPos();
}

float moveTo(float setpoint){ // moves the servo to an input position
  float error, out;

  float CW_error = setpoint - getPos(); //calculates the error if moving CW
  if(CW_error < 0){ //if CW_error is less than 0
    CW_error = CW_error+360;
  }

  //Serial.print(CW_error);

  float CCW_error = getPos()-setpoint; //calculates the error if moving CCW
  if(CCW_error < 0){ //if CCW_error is less than 0
    CCW_error = CCW_error+360;
  }

  // if CW_error is smaller than CCW_error (or both are equal) then error should be CW_error
  if(CW_error < CCW_error || CW_error == CCW_error){ 
    error = CW_error;
  }
  
  // if CCW_error is smaller then CW_error then make error CCW_error 
  else if(CCW_error < CW_error){ 
    error = -1*CCW_error; //makes error negative
  }
  
  out = 20*error;
  out = constrain(out, -255, 255); //constrains output to have maximum magnitude of 255 
  //Serial.print(abs(out));

  
  if (abs(out) < 25) { // If output is too small, motor won't move
      out = 0; // Keep motor off in the dead zone
  } 
  else if (out > 0) { // Ensure minimum PWM for movement
      out = max(out, 50); // Set a minimum threshold for forward
  } 
  else if (out < 0) {
      out = min(out, -50); // Set a minimum threshold for reverse
  }
  


  if(out > 0){ //if output is positive move CW
    analogWrite(ENA, abs(out)); //sets enable pin HIGH
    analogWrite(pwm[0], 0);
    analogWrite(pwm[1], 255);
  }
  else if(out < 0){ //if output is negative move CCW
    analogWrite(ENA, abs(out)); //sets enable pin HIGH
    analogWrite(pwm[0], 255);
    analogWrite(pwm[1], 0);
  }
  else if(out == 0){
    analogWrite(ENA, 0); //sets enable pin HIGH
    analogWrite(pwm[0], 0);
    analogWrite(pwm[1], 0);
  }
}

float getPos(){ // gets and returns encoder position
  // gets raw 10 bit position from the encoder and maps it to angle value
  // adds offset value
  float pos = map(analogRead(encoder), 0, 1023, 0, 359) + offset;
  //corrects position if needed
  if (pos < 0) {
    pos = 359 + pos;
  }

  else if (pos > 359) {
    pos = pos - 359;
  }

  return pos;
}

void inputPos(){ // moves servo to the position typed in the serial monitor
  if (Serial.available() > 0) {    // is a character available?
    input = Serial.readString();
  }
  moveTo(input.toInt());
  Serial.println(getPos());
}
