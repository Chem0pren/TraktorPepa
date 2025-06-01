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

float getEncoderAngle() {
  int raw = analogRead(encoderPin);
  return map(raw, 0, 1023, 0, 360);
}

void seekHome() {
  Serial.println("Seeking home position...");
  float angle = getEncoderAngle();

  while (!(angle < HOME_TOLERANCE_DEG || angle > (360 - HOME_TOLERANCE_DEG))) {
    if(angle > 0 && angle < 100){
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
}

void correctPos() {
  Serial.println("Seeking home position...");
  float angle = getEncoderAngle();

  while (!(angle < HOME_TOLERANCE_DEG || angle > (360 - HOME_TOLERANCE_DEG))) {
    if(angle > 0 && angle < 100){
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
}




void setup() {
 // pinMode(10, OUTPUT);
  //pinMode(9, OUTPUT);
  Serial.begin(9600);
  stepper.setRPM(200);
  stepper.setMicrostep(MICROSTEPS);

  seekHome();  // Home at startup

}

void loop() {

  //digitalWrite(10, HIGH);
  //digitalWrite(9, HIGH);

  int currentReading = analogRead(inputPin);
  
  //if something happend with cable
 


  int input_angle = map(currentReading, 75, 472, 0, 90);

  int rawValue = input_angle;

  float difference = abs(rawValue - smoothedValue);

  // Normalize difference (0–1023) to a 0–1 scale
  float normDiff = constrain(difference / 90.0, 0, 1);

  // Interpolate alpha between min and max based on difference
  float alpha = minAlpha + (maxAlpha - minAlpha) * normDiff;

  // Apply smoothing
  smoothedValue = alpha * rawValue + (1 - alpha) * smoothedValue;


  turnToAngle(round(smoothedValue));

  //enshure always zero when idle
  if(round(smoothedValue) == 0 &&  stepper.getStepsCompleted() == 0)
  {
    //Serial.println(getEncoderAngle());
    if(getEncoderAngle() > 1){
    seekHome();
    }
  }

  delay(10);
}


void turnToAngle(int angle_to_move)
{
    if(angle_to_move < 0){
      angle_to_move = 0;
    }
    // Normal movement based on potentiometer
  //if (abs(angle_to_move - lastInputAngle) > tolerance) {
    v = map(abs(angle_to_move), 0, 360, 0, 600);
    int stepsToMove = (v - previous) * MICROSTEPS;

    stepper.move(stepsToMove);
    currentPosition += stepsToMove;

    lastInputAngle = angle_to_move;
    previous = v;

    float finalAngle = getEncoderAngle();
    Serial.print("Target: "); Serial.print(angle_to_move);
    Serial.print(" | Reached: "); Serial.println(finalAngle);
  //}
}


