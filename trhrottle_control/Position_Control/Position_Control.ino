// this is final code

#include <EEPROM.h>
#include <AccelStepper.h>

#define pinY A2
#define pinX A3
#define MOTOR_STEPS 200
#define DIR 4
#define STEP 3
#define ENBL 5
#define GEAR_RATIO 3.0
#define MICROSTEPS 4
#define STEPPPER_ON 10
#define INPUTPEDALPIN A0 
#define ENCODERPIN A6
#define BUTTONPIN 2


// Create stepper instance with DRIVER mode and custom pins
AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);

unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 100; // ms

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
float smoothedServoValue = 0;
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
int CurrentDisplay = 0;

unsigned long lastMoveTime = 0;
const int debounceDelay = 500;
unsigned long lastPrintTime = 0;

//menu item
struct MenuItem {
  const char* name;
  float* value;
  float step;
};

// Add this to send runtime data like throttle and errors
struct StatusInfo {
  float throttleValue;
  const char* errorMessage; // nullptr or "" if no error
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

  stepper.setMaxSpeed(3000);       // Try something between 1000–3000
  stepper.setAcceleration(1000);
  pinMode(BUTTONPIN, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("read from EEPROM.");
  delay(1000); // Short delay before reading
  
  //load EEPROM data
  loadFromEEPROM();

    // Global status info (can be updated dynamically)
  StatusInfo status = {
    0.0,      // throttleValue
    nullptr   // errorMessage (null = no error)
  };

 // stepper.move(400*3);
}

void loop() {

  //handle button state
  buttonStateHandle();
  
  //jostick read
  yVal = analogRead(pinY);
  xVal = analogRead(pinX);

  if(!init_done && CurrentDisplay == 0){
    if(seekHome()){
      init_done = true;
      
    }
  }

  

  //Serial.println(smoothedPedalValue);

  if(CurrentDisplay == 0){
      smoothedPedalValue = GetPedalSmoothInput(minAlpha,maxAlpha);
      turnToAngle(smoothedPedalValue);

      //EncoderResponseCheck(smoothedPedalValue,getEncoderAngle());
    
      if(error){
          CurrentDisplay=2;
      }
    
      // if (millis() - lastPrintTime > 200) {  // print every 200ms
      //     Serial.print("smoothedPedalValue: ");
      //     Serial.println(smoothedPedalValue);
      //     Serial.print("currentAngle: ");
      //     Serial.println(getEncoderAngle());
      //     lastPrintTime = millis();
      // }
    //  tepper.runSpeedToPosition();  // VERY important for smooth motion
  }
  


  //ensure always zero when idle
  if(!error){
    if(smoothedPedalValue == 0 &&  stepper.distanceToGo() == 0)
    {
      //Serial.println(getEncoderAngle());
      if(getEncoderAngle() > 1){
        seekHome();
      }
    }
  }
 
  if(CurrentDisplay==0){
    if (millis() - lastDisplayUpdate > displayInterval) {
      lastDisplayUpdate = millis();
      
      // SendValues();
      
    }
  }
  if(CurrentDisplay==0)
  {
   // SendValues();
  }
  if(CurrentDisplay==1)
  {
    InteractiveMenu();
    stepper.stop();
  }

  //encoder error
  if(CurrentDisplay==2)
  {
    sendError("Encoder a vstup maji jine hodnoty");
    stepper.stop();
  }
 
  if(CurrentDisplay==3)
  {
    sendError("chyba pri hledani pozice 0");
    stepper.stop();
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
      sendMenu();   
      
    } else if (yVal > 600) 
    {
      selectedItem++;
      if (selectedItem >= menuLength) selectedItem = 0;
      lastMoveTime = millis();
      //sendMenu();   
      
    }

    // Adjust value left/right
    if (xVal < 400) {
      adjustValue(-1);
      lastMoveTime = millis();
      //sendMenu();   
      
    } else if (xVal > 600) {
      adjustValue(1);
      lastMoveTime = millis();
      //sendMenu();  
    }
  
  sendMenu();   
}


void SendValues()
{
    //Serial.println("DISPLAY:");
    Serial.print("THROTTLE:");
    Serial.println(smoothedPedalValue);
   // Serial.print("END\n");
    
}


void sendMenu() {
    Serial.println("MENU:");
    for (int i = 0; i < sizeof(menu) / sizeof(menu[0]); i++) {
      if (i == selectedItem) {
        Serial.print("> ");   // add cursor mark before selected item
      } else {
        Serial.print("  ");   // indent others for alignment
      }
      Serial.print(menu[i].name);
      Serial.print(":");
      Serial.print(*menu[i].value, 4);
      Serial.print(":");
      Serial.print("\n");
    }
    Serial.print("END\n");
    delay(200);
}

void sendError(String Message) {

    Serial.println("MENU:");
    Serial.println(Message);
    Serial.print("END\n");
    delay(200);
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


void turnToAngle(float angle_to_move)
{   
    
    //angle_to_move = constrain(angle_to_move, 0.0, 360.0);  // clamp input

    stepper.setSpeed(2000);

    // Map angle to stepper position: 0°–360° → 0–1200 steps
    float v = (angle_to_move / 360.0) * 1200.0;
    int targetSteps = round(v);  // absolute position

    float smoothedValue = GetServoSmoothInput(targetSteps,1,0.01);
    // Only update if position changes
    if (stepper.targetPosition() != smoothedValue) {
        stepper.moveTo(smoothedValue);
    }

    lastInputAngle = angle_to_move;
    previous = v;
    
    stepper.runSpeedToPosition();

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
       // Serial.println("ERROR: Encoder failed to reach target angle in time!");
        return;
    }

}

int GetPedalSmoothInput(float minAlpha,float maxAlpha)
{
 // max_angle = 1024/4;

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

int GetServoSmoothInput(float input,float minAlpha,float maxAlpha)
{
 // max_angle = 1024/4;

  //int currentReading = analogRead(INPUTPEDALPIN);
  //int input_angle = map(currentReading, 75, 472, 0, max_angle);
  int rawValue = input;
  float difference = abs(input - smoothedServoValue);
  // Normalize difference (0–1023) to a 0–1 scale
  float normDiff = constrain(difference / max_angle, 0, 1);
  // Interpolate alpha between min and max based on difference
  float alpha = minAlpha + (maxAlpha - minAlpha) * normDiff;
  // Apply smoothing
  smoothedServoValue = alpha * rawValue + (1 - alpha) * smoothedServoValue;
  return round(smoothedServoValue);
}


float getEncoderAngle() {
  int raw = analogRead(ENCODERPIN);
  //return raw;
  return map(raw, 0, 1023, 0, 360);
}

bool seekHome() {
  Serial.println("Seeking home position...");

  //stepper.setSpeed(-200);  // Negative = backward direction

  while (true) {
    float angle = getEncoderAngle();

    // Check if within tolerance of 0 degrees
    if (angle < HOME_TOLERANCE_DEG || angle > (360.0 - HOME_TOLERANCE_DEG)) {
      break;
    }

    if(angle > 0 && angle < 180){
      stepper.setSpeed(-200);  // Move slowly backward
    }
    else{
      stepper.setSpeed(200);   // Move slowly backward
    }

    stepper.runSpeed();  // Keeps moving at set speed
    delay(5);            // Optional slow down
    Serial.print("Current Angle: ");
    Serial.println(angle);
    
    EncoderResponseCheck(0,angle);
    if(error){
      CurrentDisplay = 3;
      break;
    }


  }

  stepper.stop();  // Stop stepper
  Serial.println("Home position reached.");
  stepper.setCurrentPosition(0);  // Reset stepper internal position to 0
  return true;
}


