#include <Arduino.h>
#include "A4988.h"

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


A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

int previous = 0;
int v = 0;
const int tolerance = 0;
int lastInputAngle = 0;

int inputPin = A0;
int encoderPin = A6;

long currentPosition = 0;  // Manual step tracking
const int STEPS_PER_REV = 100;

const int HOME_TOLERANCE_DEG = 1;     // Acceptable range around 0°
const int POT_ZERO_THRESHOLD = 10;    // Treat potentiometer as "zero" below this value

float smoothedValue = 0;
float minAlpha = 0.1;  // Smoothing when changes are small
float maxAlpha = 0.8;   // Smoothing when changes are large

bool error = false;                 // Global or static error flag
unsigned long movementStartTime = 0;
bool movementInProgress = false;
const unsigned long maxWaitTime = 3000; // Max wait in ms
const int angleTolerance = 10;       // Degrees tolerance

bool init_done = false;

float getEncoderAngle() {
  int raw = analogRead(encoderPin);
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


void setup() {
 // pinMode(10, OUTPUT);
  //pinMode(9, OUTPUT);
  Serial.begin(9600);
  stepper.setRPM(200);
  stepper.setMicrostep(MICROSTEPS);
}

void loop() {

  if(!init_done){
    if(seekHome()){
      init_done = true;
    }
  }

  int smoothedPedalValue = GetPedalSmoothInput();

  if(!error){
    turnToAngle(smoothedPedalValue);
  }

  //ensure always zero when idle
  if(round(smoothedValue) == 0 &&  stepper.getStepsCompleted() == 0)
  {
    //Serial.println(getEncoderAngle());
    if(getEncoderAngle() > 1){
   // seekHome();
    }
  }

  EncoderResponseCheck(smoothedValue,getEncoderAngle());
  delay(10);
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

int GetPedalSmoothInput()
{
  int currentReading = analogRead(inputPin);
  int input_angle = map(currentReading, 75, 472, 0, 90);
  int rawValue = input_angle;
  float difference = abs(rawValue - smoothedValue);
  // Normalize difference (0–1023) to a 0–1 scale
  float normDiff = constrain(difference / 90.0, 0, 1);
  // Interpolate alpha between min and max based on difference
  float alpha = minAlpha + (maxAlpha - minAlpha) * normDiff;
  // Apply smoothing
  smoothedValue = alpha * rawValue + (1 - alpha) * smoothedValue;

  return round(smoothedValue);

}
