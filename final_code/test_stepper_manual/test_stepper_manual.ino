//this is final code v2
#include <EEPROM.h>

const int stepPin = 3;
const int dirPin = 4;
const int INPUTPEDALPIN = A0;
const int ENCODERPIN = A6;
const int pinY = A2;
const int pinX = A3;
const int BUTTONPIN  = 2;
const int RELAYPIN = 12;

//system display
//int currentDisplay = 0; //0 - large number, 1 - menu, 2 - error 
bool InitPhase = true;

// error check
//bool error = false;                 // Global or static error flag
unsigned long movementStartTime = 0;
bool movementInProgress = false;
const int angleTolerance = 10;       // Degrees tolerance
float maxWaitTime = 3000; // Max wait in ms

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // ms

//joystick button
int buttonState = 0;  // variable for reading the pushbutton status
unsigned long buttonHoldStart = 0;
bool buttonHeld = false;
const unsigned long holdTime = 3000; // 3 seconds

//joystick throttle control
int throttleJoystick = 0;     // Global throttle value
const int AngleSwitchTreshold = 5;

int currentTargetAngle = 0;
float smoothedTargetAngle = 0.0;

//to be eprom
float MAX_ANGLE = 90;
float ZERO_OFFSET = -20;
// Steps per full rotation of the motor
float MAX_STEPS = 1200;
int joystickDeadzone = 30;  // Around center (512)
float throttleStep = 5;
float smoothingFactor = 0.8; // Between 0.0 (slow) and 1.0 (instant)
float MAX_HOMING_STEPS = 300;

//global properties
float stepDifference = 0;

// Store current motor step position
int currentStepPos = 0;

//menu item
struct MenuItem {
  const char* name;
  float* value;
  float step;
};

//menu list
MenuItem menu[] = {
  {"Maximalni uhel", &MAX_ANGLE, 1.0},
  {"Offset Nula", &ZERO_OFFSET, 1.0},
  {"Citlivost", &smoothingFactor, 0.1},
  {"Citlivost Yoy osa Y", &throttleStep, 0.1},
  {"Homing cas max", &MAX_HOMING_STEPS, 1.0}
};

const int menuLength = sizeof(menu) / sizeof(menu[0]);
int selectedItem = 0;
unsigned long lastMoveTime = 0;
const int debounceDelay = 500;

const int JOY_CENTER = 512;   // typical mid point (calibrate if needed)
const int DEADZONE   = 60;    // +/- dead zone, tweak (40..100)
const unsigned long MOVE_DELAY = 10; // ms between moves

int8_t lastYDir = 0; // -1 up, 0 center, +1 down
int8_t lastXDir = 0; // -1 left, 0 center, +1 right

bool isMenuInit = false; 

//nsigned long lastMoveTime = 0; // you already had this


// Possible states
enum SystemState {
  STATE_INIT,
  STATE_RUN,
  STATE_ERROR,
  STATE_MENU,
  STATE_SAVING
};

const char* getStateName(SystemState state) {
  switch (state) {
    case STATE_INIT:  return "INIT";
    case STATE_RUN:   return "RUN";
    case STATE_ERROR: return "ERROR";
    case STATE_MENU:  return "MENU";
    case STATE_SAVING:  return "Saving";
    default:          return "UNKNOWN";
  }
}

SystemState currentState = STATE_INIT;
SystemState previousState = STATE_INIT; // global or static variable

void setup() {
  pinMode(RELAYPIN, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(INPUTPEDALPIN, INPUT);
  pinMode(ENCODERPIN, INPUT);
  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(pinY, INPUT);
  pinMode(pinX, INPUT);
  digitalWrite(RELAYPIN, HIGH);  // LOW = off, assuming active HIGH relay
  Serial.begin(19200);
  
  //load eeprom data
  loadFromEEPROM();
 // seekHome();

}

void loop() {


 if (currentState != previousState) {
       // Serial.print("System State: ");
       // Serial.println(getStateName(currentState));
        previousState = currentState;
    }

  // Always check buttons first (this can change currentState)
  buttonStateHandle();

  switch (currentState) {

    case STATE_INIT:
      seekHome();
      currentState = STATE_RUN;
      break;

    case STATE_RUN:
      // Joystick input allowed
      updateThrottleFromJoystick();
      handlePedalAndMovement(true); // true = joystick enabled
      break;

    case STATE_ERROR:
      digitalWrite(RELAYPIN, LOW);
      break;

    case STATE_MENU:
     // Serial.println("menu");
      InteractiveMenu();
      // Pedal control still active, joystick ignored
      handlePedalAndMovement(false); // false = joystick disabled
      break;
  }
}

void handlePedalAndMovement(bool joystickEnabled) {
  
  if (!joystickEnabled) {
    throttleJoystick = 0; // force joystick to zero
  }

  int pedalAngle = analogRead(INPUTPEDALPIN);
  int targetAngleJoystick = joystickEnabled ? 
      map(throttleJoystick, 0, 100, 0, MAX_ANGLE) : 0;
  int targetAnglePedal = map(pedalAngle, 75, 472, 0, MAX_ANGLE);

  if (targetAnglePedal > AngleSwitchTreshold) {
    currentTargetAngle = targetAnglePedal;
    if (joystickEnabled) throttleJoystick = 0;
  } else {
    currentTargetAngle = targetAngleJoystick;
  }

  updateSmoothedAngle();
  turnToAngle(smoothedTargetAngle, 600, throttleJoystick);

  static unsigned long lastSeekTime = 0;
  if (smoothedTargetAngle <= 1 && stepDifference < 0 &&
      millis() - lastSeekTime > 2000) {
    seekHome();
    lastSeekTime = millis();
  }

  float encoderAngle = getEncoderAngle() - ZERO_OFFSET;
  EncoderResponseCheck(smoothedTargetAngle, encoderAngle);
}

//loop end


void updateSmoothedAngle() {
  smoothedTargetAngle = smoothedTargetAngle * (1.0 - smoothingFactor) + currentTargetAngle * smoothingFactor;
}

void turnToAngle(float angle_to_move,int speed,float throttleJoystick)
{  

  // Map analog value to motor steps
  int targetStepPos = map(angle_to_move, 0, 360, 0, MAX_STEPS);
  stepDifference = targetStepPos - currentStepPos;

  // Print debug info
  float encoderAngle = getEncoderAngle() - ZERO_OFFSET;

  float pedalPercent =  map(angle_to_move, 0, MAX_ANGLE, 0, 100);

  //encoderAngle = constrain(encoderAngle,0,MAX_ANGLE);

  float EncoderPercent =  map(encoderAngle, 0 , MAX_ANGLE , 0, 100);

  EncoderPercent = constrain(EncoderPercent,0,100);
  pedalPercent = constrain(pedalPercent,0,100);

  if (millis() - lastPrintTime >= printInterval && currentState == STATE_RUN) {
    lastPrintTime = millis();

    Serial.print("THROTTLE: PedalPercent=");
    Serial.print(pedalPercent);
    Serial.print(" | EncoderPercent=");
    Serial.print(EncoderPercent);
    Serial.print(" | ThrottleJoystick=");
    Serial.println(throttleJoystick);
  }else{
    //compensate delay for stability
    delay(10);

  }
  
  if (stepDifference != 0) {
    // Set direction based on sign of difference
    digitalWrite(dirPin, stepDifference > 0 ? HIGH : LOW);

    // Move motor step-by-step
    int stepsToMove = abs(stepDifference);
    for (int x = 0; x < stepsToMove; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speed);  // Speed control
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speed);
    }

    // Update current position
    currentStepPos = targetStepPos;
  }

  delay(10);  // Small delay to reduce flickering or excessive updates

}


float getEncoderAngle() {
  int raw = analogRead(ENCODERPIN);
  //return raw;
  return map(raw, 0, 1023, 0, 360);
}

void seekHome() {
  //Serial.println("Seeking home with encoder...");

  const float HOME_ANGLE_THRESHOLD = 1.0; // Allow ±1° tolerance
  const int homingSpeed = 700;
  bool showMessage = true;
  int stepsTaken = 0;

  while (true) {
    float angle = getEncoderAngle() - ZERO_OFFSET;
    angle = fmod(angle + 360.0, 360.0); // Normalize

    if(currentState==STATE_INIT)
    {
      Serial.print("INFO:");
      Serial.print("Hledam pozici 0: ");
      Serial.println(angle);
    }

    float distanceToZero = min(angle, 360.0 - angle);
    if (distanceToZero <= HOME_ANGLE_THRESHOLD) {
    //  Serial.println("Reached home angle.");
     // error = false;
     // currentDisplay=0;
     // InitPhase = false;
      break;
    }

    // 🔁 Recalculate best direction to rotate
    bool goClockwise = angle > 180.0;
    digitalWrite(dirPin, goClockwise ? HIGH : LOW);

    // Step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(homingSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(homingSpeed);
    
    delay(10);

    stepsTaken++;
    if (stepsTaken >= MAX_HOMING_STEPS) {
      currentState = STATE_ERROR;
      Serial.print("ERROR:");
      Serial.print("| vychozi pozice nanalezena");
      Serial.print("| Vypinam zapalovani!");
      Serial.println("| Podrz tlacitko | pro reset");
      InitPhase = false;
      break;
    }
  }

  currentStepPos = 0;
  delay(200); // optional pause
}

void buttonStateHandle()
{
  if (digitalRead(BUTTONPIN) == LOW) 
  {
    if (!buttonHeld) {
      buttonHeld = true;
      buttonHoldStart = millis();
    } 
    else if (millis() - buttonHoldStart >= holdTime) {
      
      // Enter menu from RUN
      if (currentState == STATE_RUN) {
        isMenuInit = false;
        currentState = STATE_MENU;
       // isMenuInit = true; // optional init flag
      } 
      // Reset from ERROR
      else if (currentState == STATE_ERROR) {
        currentState = STATE_INIT;
        resetSystem();
      } 
      // Save and exit only if already in MENU
      else if (currentState == STATE_MENU) {
        currentState = STATE_SAVING;
        Serial.println("INFO:");
        Serial.println("Nastaveni ulozeno");
        saveToEEPROM();
        delay(2000);
        isMenuInit = false;
        currentState = STATE_RUN;
       
      }

      buttonHeld = false;
    }
  } 
  else {
    buttonHeld = false;
  }
}


void updateThrottleFromJoystick() {
  int yValue = analogRead(pinY);
  int centeredY = yValue - 512;

  if (centeredY > joystickDeadzone) {
    // Joystick forward
    throttleJoystick -= throttleStep;
  } else if (centeredY < -joystickDeadzone) {
    // Joystick backward
    throttleJoystick += throttleStep;
  }

  // Clamp throttle
  throttleJoystick = constrain(throttleJoystick, 0, 100);

  // Optional debug
  //Serial.print("Joystick Y: ");
  //Serial.print(yValue);
  //Serial.print(" | throttleJoystick: ");
  //Serial.println(throttleJoystick);
}

void EncoderResponseCheck(int targetAngle, float currentAngle)
{


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
      //  Serial.println("Movement complete.");
       // return;
    }

    // Check timeout
    if (millis() - movementStartTime > maxWaitTime) {
        movementInProgress = false;
        currentState = STATE_ERROR;
        Serial.print("ERROR:");
        Serial.print("Rozdil mezi hodnotou | plynu a encoderu");
        Serial.print(" | Uhel encoderu: ");
        Serial.print(currentAngle);
        Serial.print(" | Nastaveny uhel: ");
        Serial.print(targetAngle);
        Serial.print(" | Vypinam zapalovani!");
        Serial.println("| Podrz tlacitko | pro reset");
        currentState = STATE_ERROR;

       // Serial.println("ERROR: Encoder failed to reach target angle in time!");
       // return;
    }

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

void InteractiveMenu()
{
  if(!isMenuInit)
  {
    delay(10);
    sendMenu();
    isMenuInit = true;
  }

  // Read joystick (optionally average several samples if noisy)
  int yVal = analogRead(pinY);
  int xVal = analogRead(pinX);

  // Determine direction with dead zone
  int8_t yDir = 0;
  if (yVal < JOY_CENTER - DEADZONE) yDir = -1;       // up
  else if (yVal > JOY_CENTER + DEADZONE) yDir = +1;  // down

  int8_t xDir = 0;
  if (xVal < JOY_CENTER - DEADZONE) xDir = -1;       // left
  else if (xVal > JOY_CENTER + DEADZONE) xDir = +1;  // right

  unsigned long now = millis();
  bool changed = false;

  // NAVIGATION (Y)
  // Only act when we go from center (lastYDir == 0) -> moved (yDir != 0)
  if (yDir != 0 && lastYDir == 0 && (now - lastMoveTime) > MOVE_DELAY) {
    if (yDir == -1) {
      selectedItem--;
      if (selectedItem < 0) selectedItem = menuLength - 1;
    } else {
      selectedItem++;
      if (selectedItem >= menuLength) selectedItem = 0;
    }
    lastMoveTime = now;
    changed = true;
  }
  lastYDir = yDir;

  // ADJUST VALUE (X)
  // Same edge logic for left/right
  if (xDir != 0 && lastXDir == 0 && (now - lastMoveTime) > MOVE_DELAY) {
    adjustValue(xDir); // passes -1 or +1; adjustValue should accept this
    lastMoveTime = now;
    changed = true;
  }
  lastXDir = xDir;

  // Only send menu when something actually changed
  if (changed) {
    sendMenu();
    seekHome();
  }
}

void sendMenu() 
{
    Serial.print("INFO:");
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
      //Serial.print("\n");
      Serial.print("|");
    }
    Serial.println("END\n");
}

void resetSystem()
{
    currentState = STATE_INIT;
    //InitPhase = true;
    isMenuInit = false;
    //currentDisplay = 0;
}
