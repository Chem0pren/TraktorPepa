#include <Arduino.h>
#include "A4988.h"

#define MOTOR_STEPS 200
#define DIR 3
#define STEP 4
#define ENBL 8
#define MS1 5
#define MS2 6
#define MS3 7
#define GEAR_RATIO 3.0
#define MICROSTEPS 2

A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

int previous = 0;
int v = 0;
const int tolerance = 5;
int lastInputAngle = 0;

int inputPin = A0;
int encoderPin = A6;

long currentPosition = 0;  // Manual step tracking
const int STEPS_PER_REV = 100;

const int HOME_TOLERANCE_DEG = 1;     // Acceptable range around 0°
const int POT_ZERO_THRESHOLD = 10;    // Treat potentiometer as "zero" below this value

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

void setup() {
  Serial.begin(9600);
  stepper.setRPM(200);
  stepper.setMicrostep(MICROSTEPS);

  seekHome();  // Home at startup

}

void loop() {

  int currentReading = analogRead(inputPin);

  //int pedal_angle = analogRead(inputPin);

  int input_angle = map(currentReading, 75, 472, 0, 360);




  int angle = getPos();

  

  //Serial.print("engle error: ");
 // Serial.println(angle - abs(input_angle));
 // Serial.print("encoder Angle: ");
 // Serial.println(angle);


  //moveToAngle(input_angle);
  
  // Normal movement based on potentiometer
  if (abs(input_angle - lastInputAngle) > tolerance) {
    v = map(input_angle, 0, 360, 0, 600);
    int stepsToMove = (v - previous) * MICROSTEPS;

    stepper.move(stepsToMove);
    currentPosition += stepsToMove;

    lastInputAngle = input_angle;
    previous = v;

    float finalAngle = getEncoderAngle();
    Serial.print("Target: "); Serial.print(input_angle);
    Serial.print(" | Reached: "); Serial.println(finalAngle);
    }
    
  delay(1);
}

void moveToAngle(float targetAngle) {
  float currentAngle = getEncoderAngle();
  float angleDelta = targetAngle - currentAngle;

  // Normalize angle to -180 to +180 range
  //if (angleDelta > 180) angleDelta -= 360;
  //if (angleDelta < -180) angleDelta += 360;

  // Steps = angleDelta / 360 * stepsPerRevBigPulley
  int stepsToMove = angleDelta * (MOTOR_STEPS * GEAR_RATIO);


  stepper.move(stepsToMove);

  // Optional: verify angle achieved (can loop + recheck if needed)
  delay(500); // wait for motor to settle
  float finalAngle = getEncoderAngle();
  Serial.print("Target: "); Serial.print(targetAngle);
  Serial.print(" | Reached: "); Serial.println(finalAngle);
}

float getPos(){ // gets and returns encoder position
  // gets raw 10 bit position from the encoder and maps it to angle value
  // adds offset value
  int offset = 0;
  float pos = map(analogRead(encoderPin), 0, 1023, 0, 359) + offset;
  //corrects position if needed
  if (pos < 0) {
    pos = 359 + pos;
  }

  else if (pos > 359) {
    pos = pos - 359;
  }

  return pos;
}

/*
  // Normal movement based on potentiometer
  if (abs(currentReading - lastReading) > tolerance) {
    v = map(currentReading, 75, 472, 0, 200);
    int stepsToMove = (v - previous) * 2;

    currentPosition += stepsToMove;
    
    Serial.print("Potentiometer Value: ");
    Serial.println(currentReading);
    Serial.print("Stepper Position (steps): ");
    Serial.println(currentPosition);

    

    lastReading = currentReading;
    previous = v;
  }
  */
