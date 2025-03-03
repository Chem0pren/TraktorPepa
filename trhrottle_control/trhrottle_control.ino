#include <U8glib.h>
#include <PIDController.h>
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);


//throttle potenciometers
int PEDAL_IN1 = A0;
int PEDAL_IN2 = A1;

// MT6701 Magnetic Encoder 
const int ENCODER = A6; 

//H-bridge output
#define enA 3
#define MOTOR_CCW 4
#define MOTOR_CW 5

// properties
float potVal = 0;
float pedalValue1 = 0;
float pedalValue2 = 0;
float throttlePercent = 0;
float CurrentAngle = 0;
float UserCurrentAngle = 0;
//int inputResMin = 516;
//int inputResMax = 938;
int motor_pwm_value = 0;

//PID
#define __Kp 200 // Proportional constant
#define __Ki 0.8 // Integral Constant
#define __Kd 2000 // Derivative Constant

/*
#define __Kp 260 // Proportional constant
#define __Ki 2.7 // Integral Constant
#define __Kd 2000 // Derivative Constant
*/

PIDController pidcontroller;


//remove
//String incomingByte;
float motorVal = 0;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(MOTOR_CW, OUTPUT); // MOTOR_CW as Output
  pinMode(MOTOR_CCW, OUTPUT); // MOTOR_CW as Output
  pinMode(ENCODER, INPUT); // ENCODER as Input
  pinMode(enA, OUTPUT);
  
  pidcontroller.begin(); // initialize the PID instance
  pidcontroller.tune(__Kp , __Ki , __Kd); // Tune the PID, arguments: kP, kI, kD
  pidcontroller.limit(-255, 255); // Limit the PID output between -255 to 255, this is important to get rid of integral windup!

  //display setup 
  u8g.setColorIndex(1); // display draws with pixel on
  
  //serial setup
  Serial.begin(38400);

  //potrebuju tohle? 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  // The power-on delay is 3000 milliseconds, waiting for the motor to be initialized
  delay(3000);
  }

}

void loop() {

  //get values from sensors
  CurrentAngle = GetEncoderAngle(ENCODER);
  int PedalInput1 = analogRead(PEDAL_IN1);
  int PedalInput2 = analogRead(PEDAL_IN2);
  
  int PedalAngle = map(PedalInput1, 74, 473, 0, 90);

  
  // encoder test
  Serial.print("Angle: ");
  Serial.println(CurrentAngle); // Print the angle to the Serial Monitor
  //delay(100); // Delay for readability
  Serial.print("PedalAngle: ");
  Serial.println(PedalAngle); // Print the angle to the Serial Monitor
  

  //PID control
  if(PedalAngle > 1){
    pidcontroller.setpoint(PedalAngle);
    // remove angle set by thumpad
  }
  else{
    pidcontroller.setpoint(10);
  }


  // 45 0 

  
  float AngleError = CurrentAngle - PedalAngle;
  if (AngleError > 180) AngleError -= 360;  // Wraparound correction
  if (AngleError < -180) AngleError += 360;

  //Serial.print("Angle Error: ");
  //Serial.println(AngleError);

  motor_pwm_value = pidcontroller.compute(AngleError);  // Use corrected error

  Serial.print("PID: ");
  Serial.println(motor_pwm_value); // Print the angle to the Serial Monitor

  /*
  if (Serial.available()) {
      String inputString = Serial.readStringUntil('\n');  // Read until newline
      float receivedValue = inputString.toFloat();        // Convert to float
      Serial.print("Received: ");
      Serial.println(receivedValue);

      analogWrite(enA,receivedValue);
      digitalWrite(MOTOR_CW, HIGH);
      digitalWrite(MOTOR_CCW, LOW);
  }
  */
  



  //run motor
  
  if (motor_pwm_value > 0) {
      motor_cw(abs(motor_pwm_value));
  }
  else
  {
      motor_ccw(abs(motor_pwm_value));
  }
  
  delay(10); 

// front end
u8g.firstPage();
do {

  //drawThrottle(10,20,String(angle));

//draw(0,10,TimeString);
} while (u8g.nextPage());

}



void drawServoInfo(int pos_x,int pos_y,String message)
{
u8g.setFont(u8g_font_04b_03b);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);
}

void drawThrottle(int pos_x,int pos_y,String message)
{
u8g.setFont(u8g_font_04b_03b);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);
}

float GetEncoderAngle(int encoderPin)
{
int sensorValue = analogRead(encoderPin); // Read the analog value from encoder
float angle = map(sensorValue, 0, 1023, 0, 3600); // Map the value to 0-360 degrees
return angle/10;
}


void motor_cw(int power) {
  if (power > 20) {
    analogWrite(enA,power);
    digitalWrite(MOTOR_CW, HIGH);
    digitalWrite(MOTOR_CCW, LOW);
  }
  else {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
  }
}

void motor_ccw(int power) {
  if (power > 20) {
    analogWrite(enA,abs(power));
    digitalWrite(MOTOR_CCW, HIGH);
    digitalWrite(MOTOR_CW, LOW);
    //Serial.println("motor CW off");
  }
  else {
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
   // Serial.println("motor CCW off");
  }
}


