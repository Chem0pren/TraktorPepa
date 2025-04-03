#include <U8glib.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);

//pins
const int ENA = 3; // enable pin 
const int pwm[2] = {4, 5}; // motor driver pwm pins 
const int encoder = A6; // encoder pin

int PEDAL_IN1 = A0;
//constants
float offset = 0;
String input;

void setup() {
  pinMode(ENA, OUTPUT);
  Serial.begin(38400);
  moveTo(0);
  u8g.setColorIndex(1); // display draws with pixel on

  
}

void loop() {
  int PedalInput1 = analogRead(PEDAL_IN1);
  delay(1);  // delay in between reads for stability
  int PedalAngle = map(PedalInput1, 74, 473, 0, 90);
 
  moveTo(PedalAngle);

  // front end
  u8g.firstPage();
  do {

    //drawServoInfo(10,10,String(test));
    drawThrottle(40,40,String(100));
   //drawThrottleTriangle(100);

  //draw(0,10,TimeString);
  } while (u8g.nextPage());

  




}

float moveTo(float setpoint){ // moves the servo to an input position
  float error, out;

  static float lastError = 0;
  static float integral = 0;


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
  

  float Kp = 2.0;  
  float Ki = 0.01;  
  float Kd = 3.5;   

  integral += error;  
  integral = constrain(integral, -50, 50);  // Prevent integral windup

  float derivative = error - lastError;
  out = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  out = constrain(out, -255, 255);

  Serial.print("error");
  Serial.println(error);

  out = 10*error;

  if(abs(error) > 1){
    if(error > 0){
    out = max(out, 50); // Set a minimum threshold for forward
    }else if (error < 0){
    out = min(out, -50); // Set a minimum threshold for reverse
    }
    Serial.print("-correcting error-");
    Serial.println(error);
    Serial.print(out);
  }else{
  out = 0;

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

//DISPLAY
void drawServoInfo(int pos_x,int pos_y,String message)
{
u8g.setFont(u8g_font_04b_03b);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);
}

void drawThrottle(int pos_x,int pos_y,String message)
{

//u8g_font_helvB24n
u8g.setFont(u8g_font_helvB24n);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);


//u8g.drawBox(10, 50, 100 ,50);


}

void drawThrottleTriangle(int throttle) {
    int max_width = 128;  // Display width
    int max_height = 64;  // Display height
    int base_y = max_height;  // Bottom of the screen

    // Map throttle (0-1023) to screen width
    int triangle_width = map(throttle, 0, 100, 0, max_width);

    // Draw the triangle
    u8g.drawTriangle(0, base_y,  // Bottom-left
                         triangle_width, base_y,  // Bottom-right
                         triangle_width, base_y - (triangle_width / 2)); // Peak

}
