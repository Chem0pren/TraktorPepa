#include <U8glib.h>
#include <EEPROM.h>
#include "A4988.h"

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);

#define pinY A2
#define pinX A3
#define MOTOR_STEPS 200
#define DIR 3
#define STEP 4
#define ENBL 8
#define MS1 7
#define MS2 6
#define MS3 5
#define GEAR_RATIO 3.0
#define MICROSTEPS 2
#define STEPPPER_ON 10
#define INPUTPEDALPIN A0 
#define ENCODERPIN A6
#define BUTTONPIN 2


A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

//current jostick value
int yVal = 0;
int xVal = 0;

//reading 
int previous = 0;
int v = 0;
const int tolerance = 0;
int lastInputAngle = 0;
long currentPosition = 0;  // Manual step tracking

const int POT_ZERO_THRESHOLD = 10;    // Treat potentiometer as "zero" below this value
float smoothedValue = 0;

bool error = false;                 // Global or static error flag
unsigned long movementStartTime = 0;
bool movementInProgress = false;
const int angleTolerance = 10;       // Degrees tolerance

bool init_done = false;

float max_angle;
float minAlpha = 0.1;  // Smoothing when changes are small
float maxAlpha = 0.8;   // Smoothing when changes are large
float maxWaitTime = 3000; // Max wait in ms
float HOME_TOLERANCE_DEG = 1;     // Acceptable range around 0°

int buttonState = 0;  // variable for reading the pushbutton status
unsigned long buttonHoldStart = 0;
bool buttonHeld = false;
const unsigned long holdTime = 3000; // 3 seconds

//display state
int CurrentDisplay = 1;

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
  {"Maximalni uhel", &max_angle, 1.0},
  {"Citlivost min", &minAlpha, 0.01},
  {"Citlivost max", &maxAlpha, 0.1},
  {"Cas Erroru", &maxWaitTime, 1.0},
  {"Home tolerance", &HOME_TOLERANCE_DEG, 1.0}
};

const int menuLength = sizeof(menu) / sizeof(menu[0]);
int selectedItem = 0;

int smoothedPedalValue = 0;

void setup() {
  stepper.setRPM(200);
  stepper.setMicrostep(MICROSTEPS);
  pinMode(BUTTONPIN, INPUT_PULLUP);
  Serial.begin(9600);
  u8g.setColorIndex(1); // display draws with pixel on
  Serial.println("read from EEPROM.");
  delay(1000); // Short delay before reading
  //load EEPROM data
  loadFromEEPROM();

}

void loop() {

  //handle button state
  buttonStateHandle();
  
  //jostick read
  yVal = analogRead(pinY);
  xVal = analogRead(pinX);

  if(!init_done){
    if(seekHome()){
      init_done = true;
    }
  }

  smoothedPedalValue = GetPedalSmoothInput(minAlpha,maxAlpha);

  if(!error){
    turnToAngle(smoothedPedalValue);
  }

  //ensure always zero when idle
  if(smoothedPedalValue == 0 &&  stepper.getStepsCompleted() == 0)
  {
    //Serial.println(getEncoderAngle());
    if(getEncoderAngle() > 1){
      seekHome();
    }
  }

  EncoderResponseCheck(smoothedPedalValue,getEncoderAngle());
  delay(10);

  

  if(CurrentDisplay==0)
  {
    drawThrottle(40,40,String(smoothedPedalValue));
  }

  if(CurrentDisplay==1)
  {
   // InteractiveMenu();
    drawInfo();
  }

  if(CurrentDisplay==2)
  {
    // print error
  }
 


}

void buttonStateHandle()
{
  
  //hold button check
  if (digitalRead(BUTTONPIN) == LOW) 
  {
    Serial.println("button on");
    if (!buttonHeld) {
      buttonHeld = true;
      buttonHoldStart = millis();
    } else if (millis() - buttonHoldStart >= holdTime) {
      if(CurrentDisplay == 0)
      {
        Serial.println("entering menu");
        CurrentDisplay = 1;
      }else
      {
        CurrentDisplay = 3;
        Serial.println("save data");
        saveToEEPROM();
        //drawInfo(40,40,"Data");
        delay(500);
        CurrentDisplay = 0;
      }
      buttonHeld = false;
    }
  } else {
    buttonHeld = false;
  }

}

void InteractiveMenu()
{
  
  
    // Navigation
    if (yVal < 400) 
    {
      selectedItem--;
      if (selectedItem < 0) selectedItem = menuLength - 1;
      lastMoveTime = millis();
      drawMenu();   
      
    } else if (yVal > 600) 
    {
      selectedItem++;
      if (selectedItem >= menuLength) selectedItem = 0;
      lastMoveTime = millis();
      drawMenu();   
      
    }

    // Adjust value left/right
    if (xVal < 400) {
      adjustValue(-1);
      lastMoveTime = millis();
      drawMenu();   
      
    } else if (xVal > 600) {
      adjustValue(1);
      lastMoveTime = millis();
      drawMenu();  
    }
  
 // drawMenu();   
  
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

//DISPLAY
void drawServoInfo(int pos_x,int pos_y,String message)
{
u8g.setFont(u8g_font_04b_03b);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);
}



void drawInfo()
{
u8g.setFont(u8g_font_04b_03br);
u8g.firstPage();
do {
   u8g.setPrintPos(2, 50);
    u8g.print("pedal vstup: ");
    u8g.print(String(smoothedPedalValue));

   // u8g.setPrintPos(2, 57);
   // u8g.print("uhel encoder: ");
   // u8g.print(String(getEncoderAngle()));
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

void turnToAngle(int angle_to_move)
{
    if(angle_to_move < 0){
      angle_to_move = 0;
    }

    float currentAngle = getEncoderAngle();
    v = map(abs(angle_to_move), 0, 360, 0, 600);
    int stepsToMove = (v - previous) * MICROSTEPS;
       
    stepper.move(stepsToMove);
    currentPosition += stepsToMove;

    lastInputAngle = angle_to_move;
    previous = v;

    float finalAngle = getEncoderAngle();

    Serial.print("Target: "); Serial.print(angle_to_move);
    Serial.print(" | Reached: "); Serial.println(finalAngle);

}

bool EncoderResponseCheck(int targetAngle, float currentAngle)
{
    //float error = targetAngle - currentAngle;

    float errorAngle = targetAngle - currentAngle;


    if (errorAngle > 180) errorAngle -= 360;
    if (errorAngle < -180) errorAngle += 360;

    if (!movementInProgress) {
      movementInProgress = true;
      movementStartTime = millis();
      //error = false;
    }

    // Check if angle difference is within tolerance
    if (abs(errorAngle) <= angleTolerance) {
        movementInProgress = false;
        //error = false;
        //Serial.println("Movement complete.");
        return;
    }

    // Check timeout
    if (millis() - movementStartTime > maxWaitTime) {
        movementInProgress = false;
        error = true;
        Serial.println("ERROR: Encoder failed to reach target angle in time!");
        return;
    }

}

int GetPedalSmoothInput(float minAlpha,float maxAlpha)
{
  int currentReading = analogRead(INPUTPEDALPIN);
  int input_angle = map(currentReading, 75, 472, 0, max_angle);
  int rawValue = input_angle;
  float difference = abs(rawValue - smoothedValue);
  // Normalize difference (0–1023) to a 0–1 scale
  float normDiff = constrain(difference / max_angle, 0, 1);
  // Interpolate alpha between min and max based on difference
  float alpha = minAlpha + (maxAlpha - minAlpha) * normDiff;
  // Apply smoothing
  smoothedValue = alpha * rawValue + (1 - alpha) * smoothedValue;
  return round(smoothedValue);
}

float getEncoderAngle() {
  int raw = analogRead(ENCODERPIN);
  return map(raw, 0, 1023, 0, 360);
}

bool seekHome() {
  Serial.println("Seeking home position...");
  float angle = getEncoderAngle();

  while (!(angle < HOME_TOLERANCE_DEG || angle > (360 - HOME_TOLERANCE_DEG))) {
    if(angle > 0 && angle < 180){
      stepper.move(-1);  // Move slowly backward
    }
    else{
      stepper.move(1);  // Move slowly backward

    }
    currentPosition--;
    delay(5);
    angle = getEncoderAngle();
    Serial.print("Current Angle: ");
    Serial.println(angle);
  }

  Serial.println("Home position reached.");
  currentPosition = 0;
  return true;
  
}


