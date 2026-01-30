#include <Servo.h>

Servo myServo;

const int servoPin = 4;
const int buzzerPin = 6;
const int buttonLeftPin  = 10;
const int buttonRightPin = 11;

// Calibration
const int servoCenter = 90;
const int maxOffset   = 45;

const int minPos = servoCenter - maxOffset;
const int maxPos = servoCenter + maxOffset;

int servoPos = servoCenter;
int lastServoPos = servoCenter;

const int step = 1;
const int moveDelayMs = 15;

// Button edge tracking
bool lastLeftPressed  = false;
bool lastRightPressed = false;

void beep(int durationMs)
{
    digitalWrite(buzzerPin, HIGH);
    delay(durationMs);
    digitalWrite(buzzerPin, LOW);
}

void doubleBeep()
{
    beep(60);
    delay(60);
    beep(60);
}

void setup()
{
    Serial.begin(9600);
    Serial.println("Servo ±45 with beeper");

    myServo.attach(servoPin);
    myServo.write(servoPos);

    pinMode(buttonLeftPin, INPUT_PULLUP);
    pinMode(buttonRightPin, INPUT_PULLUP);

    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
}

void loop()
{
    bool leftPressed  = digitalRead(buttonLeftPin)  == LOW;
    bool rightPressed = digitalRead(buttonRightPin) == LOW;

    // Beep on button press (edge)
    if (leftPressed && !lastLeftPressed)
        beep(40);

    if (rightPressed && !lastRightPressed)
        beep(40);

    // Servo movement
    if (leftPressed && !rightPressed)
        servoPos -= step;
    else if (rightPressed && !leftPressed)
        servoPos += step;

    servoPos = constrain(servoPos, minPos, maxPos);
    myServo.write(servoPos);

    // Beep when reaching center
    if ((lastServoPos < servoCenter && servoPos >= servoCenter) ||
        (lastServoPos > servoCenter && servoPos <= servoCenter))
    {
        doubleBeep();
    }

    lastServoPos = servoPos;
    lastLeftPressed  = leftPressed;
    lastRightPressed = rightPressed;

    delay(moveDelayMs);
}
