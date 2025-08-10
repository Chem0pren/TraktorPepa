//this is final code v2

const int stepPin = 3;
const int dirPin = 4;
const int INPUTPEDALPIN = A0;
const int ENCODERPIN = A6;
const int pinY = A2;
const int pinX = A3;
const int BUTTONPIN  = 2;
const int RELAYPIN = 12;

//system display
int currentDisplay = 0; //0 - large number, 1 - menu, 2 - error 

// error check
bool error = false;                 // Global or static error flag
unsigned long movementStartTime = 0;
bool movementInProgress = false;
const int angleTolerance = 10;       // Degrees tolerance
float maxWaitTime = 3000; // Max wait in ms

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
const int MAX_ANGLE = 90;
const int ZERO_OFFSET = -20;
// Steps per full rotation of the motor
const int MAX_STEPS = 1200;
const int joystickDeadzone = 30;  // Around center (512)
const int throttleStep = 5;
const float smoothingFactor = 0.8; // Between 0.0 (slow) and 1.0 (instant)

//global properties
float stepDifference = 0;

// Store current motor step position
int currentStepPos = 0;

void setup() {
  pinMode(RELAYPIN, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(INPUTPEDALPIN, INPUT);
  pinMode(ENCODERPIN, INPUT);
  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(pinY, INPUT);
  pinMode(pinX, INPUT);
  digitalWrite(RELAYPIN, LOW);  // LOW = off, assuming active HIGH relay
  Serial.begin(9600);
  seekHome();
}

void loop() {

  if(!error){  
    buttonStateHandle();
    updateThrottleFromJoystick();

    int pedalAngle = analogRead(INPUTPEDALPIN);

    int targetAngleJoystick = map(throttleJoystick, 0, 100, 0, MAX_ANGLE);
    int targetAnglePedal = map(pedalAngle, 75, 472, 0, MAX_ANGLE);
    updateSmoothedAngle();


    if(targetAnglePedal > AngleSwitchTreshold){
      currentTargetAngle = targetAnglePedal;
      throttleJoystick = 0;
    }else{
      currentTargetAngle = targetAngleJoystick;
    }

    turnToAngle(smoothedTargetAngle,600);

    //zero correction
    if(smoothedTargetAngle <=1 && stepDifference < 0){
      seekHome();
    }

    //check for error
    float encoderAngle = getEncoderAngle() - ZERO_OFFSET;
    EncoderResponseCheck(encoderAngle,smoothedTargetAngle);
  }else{
    Serial.println("switch off system");
    digitalWrite(RELAYPIN, LOW);  // Turn relay ON
  }

}
//loop end





void updateSmoothedAngle() {
  smoothedTargetAngle = smoothedTargetAngle * (1.0 - smoothingFactor) + currentTargetAngle * smoothingFactor;
}

void turnToAngle(float angle_to_move,int speed)
{  

  // Map analog value to motor steps
  int targetStepPos = map(angle_to_move, 0, 360, 0, MAX_STEPS);
  stepDifference = targetStepPos - currentStepPos;

  // Print debug info
  float encoderAngle = getEncoderAngle() - ZERO_OFFSET;

  Serial.print("Pedal: ");
  Serial.print(angle_to_move);
  Serial.print(" | Target: ");
  Serial.print(targetStepPos);
  Serial.print(" | Current: ");
  Serial.print(currentStepPos);
  Serial.print(" | Delta: ");
  Serial.print(stepDifference);
  Serial.print(" | Encoder: ");
  Serial.println(encoderAngle);
  
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
  Serial.println("Seeking home with encoder...");

  const float HOME_ANGLE_THRESHOLD = 1.0; // Allow ±1° tolerance
  const int MAX_HOMING_STEPS = 300;
  const int homingSpeed = 500;

  int stepsTaken = 0;

  while (true) {
    float angle = getEncoderAngle() - ZERO_OFFSET;
    angle = fmod(angle + 360.0, 360.0); // Normalize

    Serial.print("Encoder angle: ");
    Serial.print(angle);
    Serial.print(" | Zero offset: ");
    Serial.println(ZERO_OFFSET);

    float distanceToZero = min(angle, 360.0 - angle);
    if (distanceToZero <= HOME_ANGLE_THRESHOLD) {
      Serial.println("Reached home angle.");
      break;
    }

    // 🔁 Recalculate best direction to rotate
    bool goClockwise = angle > 180.0;
    digitalWrite(dirPin, goClockwise ? HIGH : LOW);

    Serial.print("Step ");
    Serial.print(stepsTaken);
    Serial.print(" | Direction: ");
    Serial.println(goClockwise ? "CW" : "CCW");

    // Step
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(homingSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(homingSpeed);

    stepsTaken++;
    if (stepsTaken >= MAX_HOMING_STEPS) {
      error = true;
      Serial.println("Homing failed: step limit reached.");
      break;
    }
  }

  currentStepPos = 0;
  delay(200); // optional pause
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
      if(currentDisplay == 0)
      {
        Serial.println("entering menu");
        currentDisplay = 1;
      }else
      {
        currentDisplay = 3;
        Serial.println("save data");
        //saveToEEPROM();
        //drawInfo(40,40,"Data");
        delay(500);
        currentDisplay = 0;
      }
      buttonHeld = false;
    }
  } else {
    buttonHeld = false;
  }

}

void updateThrottleFromJoystick() {
  int yValue = analogRead(pinY);
  int centeredY = yValue - 512;

  if (centeredY > joystickDeadzone) {
    // Joystick forward
    throttleJoystick += throttleStep;
  } else if (centeredY < -joystickDeadzone) {
    // Joystick backward
    throttleJoystick -= throttleStep;
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
        error = true;
        Serial.println("ERROR: Encoder failed to reach target angle in time!");
       // return;
    }

}

