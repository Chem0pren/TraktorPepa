#include <U8glib.h>
#include <EEPROM.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);

//pins
const int ENA = 3; // enable pin 
const int pwm[2] = {4, 5}; // motor driver pwm pins 
const int encoder = A6; // encoder pin
const int buttonPin = 2;

#define pinY A2
#define pinX A3

int PEDAL_IN1 = A0;
//constants
float offset = 0;
String input;

//eprom data
int Kpaddress = 0;  // float takes 4 bytes
int Kiaddress = Kpaddress + 4;
int Kdaddress = Kiaddress + 4;
int AngleAdress = Kdaddress + 4;

//default data
/*
float KpDefault = 2.0;  
float KiDefault = 0.01;  
float KdDefault = 3.5;   
*/
float default_angle = 90;

float Kp;  
float Ki;  
float Kd;   
float max_angle;

//button state 
int buttonState = 0;  // variable for reading the pushbutton status
//hold button
unsigned long buttonHoldStart = 0;
bool buttonHeld = false;
const unsigned long holdTime = 3000; // 3 seconds

int PedalAngle;
int PedalInput1;

//menu state
int displayMenu = 1;

unsigned long lastMoveTime = 0;
const int debounceDelay = 500;

//menu item
struct MenuItem {
  const char* name;
  float* value;
  float step;
};

//menu list
MenuItem menu[] = {
  {"Kp", &Kp, 0.1},
  {"Ki", &Ki, 0.01},
  {"Kd", &Kd, 0.1},
  {"MaxAngle", &max_angle, 1.0}
};

const int menuLength = sizeof(menu) / sizeof(menu[0]);
int selectedItem = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  //nulaX = analogRead(pinX);
  Serial.begin(9600);
  moveTo(0);
  u8g.setColorIndex(1); // display draws with pixel on

  Serial.println("read from EEPROM.");
  delay(1000); // Short delay before reading
  //load EEPROM data
  loadFromEEPROM();

}

void loop() {
  //pedal ops
  PedalInput1 = analogRead(PEDAL_IN1);
  delay(1);  // delay in between reads for stability
  PedalAngle = map(PedalInput1, 75, 472, 0, max_angle);
  moveTo(PedalAngle);

  int yVal = analogRead(pinY);
  int xVal = analogRead(pinX);

  //hold button check
  if (digitalRead(buttonPin) == LOW) 
  {
    if (!buttonHeld) {
      buttonHeld = true;
      buttonHoldStart = millis();
    } else if (millis() - buttonHoldStart >= holdTime) {
      if(displayMenu == 0)
      {
        Serial.println("entering menu");
        displayMenu = 1;
      }else
      {
        displayMenu = 3;
        Serial.println("save data");
        saveToEEPROM();
        //drawInfo(40,40,"Data");
        delay(500);
        displayMenu = 0;
      }
     
      buttonHeld = false;
    }
  } else {
    buttonHeld = false;
  }


  if(displayMenu == 1)
  {
    if (millis() - lastMoveTime > debounceDelay) 
    {
      // Navigation
      if (yVal < 400) 
      {
        selectedItem--;
        if (selectedItem < 0) selectedItem = menuLength - 1;
        lastMoveTime = millis();
       // drawMenu();   
        
      } else if (yVal > 600) 
      {
        selectedItem++;
        if (selectedItem >= menuLength) selectedItem = 0;
        lastMoveTime = millis();
       // drawMenu();   
        
      }

      // Adjust value left/right
      if (xVal < 400) {
        adjustValue(-1);
        lastMoveTime = millis();
       // drawMenu();   
       
      } else if (xVal > 600) {
        adjustValue(1);
        lastMoveTime = millis();
       // drawMenu();  
      }
    
    drawMenu();   
    }
  
 

  //Serial.println(selectedItem);
  //Serial.println(*menu[selectedItem].value);
  //Serial.println(menu[selectedItem].name);

  }

  if(displayMenu == 0)
  {
    drawThrottle(40,40,String(100));
  }
  

  /*
  // front end
  u8g.firstPage();
  do {
   
  } while (u8g.nextPage());
  */
 
  
}


void drawMenu() {

  u8g.setFont(u8g_font_04b_03br);
 
  u8g.firstPage();
  do {
    for (int i = 0; i < menuLength; i++) {
      int y = 7 + i * 7;
      if (i == selectedItem) {
        //u8g.drawFrame(0, y - 10, 100, 10);
        
        //u8g.setColorIndex(0); // text in black
        u8g.setPrintPos(2, y);
        u8g.print(">");
        u8g.print(menu[i].name);
        u8g.print(": ");
        u8g.print(*menu[i].value, 2);
        //u8g.setColorIndex(1); // reset to white
      } else {
        u8g.setPrintPos(2, y);
        u8g.print(menu[i].name);
        u8g.print(": ");
        u8g.print(*menu[i].value, 2);
      }
    }

    u8g.setPrintPos(2, 50);
    u8g.print("PedalAngle: ");
    u8g.print(String(PedalAngle));

    u8g.setPrintPos(2, 57);
    u8g.print("PedalInput: ");
    u8g.print(String(PedalInput1));

    u8g.setPrintPos(2, 63);
    u8g.print("current angle: ");
    u8g.print(String(getPos()));

  } while (u8g.nextPage());
}

// === Adjust Current Value ===
void adjustValue(int direction) {
  *menu[selectedItem].value += menu[selectedItem].step * direction;
}


void saveToEEPROM() {
  for (int i = 0; i < menuLength; i++) {
    EEPROM.put(i * sizeof(float), *menu[i].value);
  }
}

void loadFromEEPROM() {
  for (int i = 0; i < menuLength; i++) {
    EEPROM.get(i * sizeof(float), *menu[i].value);
  }
}

float readFloatFromEEPROM(int address) {
  union {
    byte b[4];
    float f;
  } data;

  for (int i = 0; i < 4; i++) {
    data.b[i] = EEPROM.read(address + i);
  }

  return data.f;
}

float moveTo(float setpoint){ // moves the servo to an input position
  float error, out;

  float CW_error = setpoint - getPos(); //calculates the error if moving CW
  if(CW_error < 0){ //if CW_error is less than 0
    CW_error = CW_error+360;
  }
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

  out = 5*error;
  out = constrain(out, -255, 255); //constrains output to have maximum magnitude of 255 
  if(abs(out) < 50){ //if output is less than 25 make it 0
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


void drawInfo(int pos_x,int pos_y,String message)
{
u8g.firstPage();
do {
  u8g.setFont(u8g_font_helvB24n);
  u8g.setPrintPos(pos_x, pos_y);
  u8g.print(message);
  } while (u8g.nextPage());
}


void drawThrottle(int pos_x,int pos_y,String message)
{
u8g.firstPage();
do {
  u8g.setFont(u8g_font_helvB24n);
  u8g.setPrintPos(pos_x, pos_y);
  u8g.print(message);
  } while (u8g.nextPage());
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
