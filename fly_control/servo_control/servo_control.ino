#include <Servo.h>

Servo myServo;

/* ------------------ PINS ------------------ */
const int servoPin = 17;
const int buzzerPin = 14;
const int buttonLeftPin  = 18;
const int buttonRightPin = 19;

// LED bar: 4 left, 1 center, 4 right
int ledPins[9] = {4, 5, 6, 7, 8, 9, 10, 11, 12};

/* ---------------- CALIBRATION ------------- */
const int servoCenter = 90;   // adjust if needed
const int maxOffset   = 45;   // ±45°

const int minPos = servoCenter - maxOffset;
const int maxPos = servoCenter + maxOffset;

/* ----------------- STATE ------------------ */
int servoPos = servoCenter;
int lastServoPos = servoCenter;

const int step = 1;
const int moveDelayMs = 50;

bool lastLeftPressed  = false;
bool lastRightPressed = false;

/* ----------------- BUZZER ----------------- */
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

void updateLedBar()
{
    // Clear all LEDs
    for (int i = 0; i < 9; i++)
        digitalWrite(ledPins[i], LOW);

    int offset = servoPos - servoCenter;

    // EXACT center only
    if (offset == 0)
    {
        digitalWrite(ledPins[4], HIGH); // center LED (pin 8)
        return;
    }

    // Magnitude: 1..4 LEDs
    int magnitude = map(abs(offset), 1, maxOffset, 1, 4);
    magnitude = constrain(magnitude, 1, 4);

    if (offset > 0)
    {
        // Right side LEDs
        for (int i = 5; i < 5 + magnitude; i++)
            digitalWrite(ledPins[i], HIGH);
    }
    else
    {
        // Left side LEDs
        for (int i = 3; i > 3 - magnitude; i--)
            digitalWrite(ledPins[i], HIGH);
    }
}

/* ------------------ SETUP ----------------- */
void setup()
{
    Serial.begin(9600);
    Serial.println("Servo trim controller started");

    myServo.attach(servoPin);
    myServo.write(servoCenter);

    pinMode(buttonLeftPin, INPUT_PULLUP);
    pinMode(buttonRightPin, INPUT_PULLUP);

    for (int i = 0; i < 9; i++)
    {
        pinMode(ledPins[i], OUTPUT);
        digitalWrite(ledPins[i], LOW);
    }

    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);

    updateLedBar();   // show CENTER at startup
    doubleBeep();
}

/* ------------------- LOOP ----------------- */
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

    // Beep on center crossing
    if ((lastServoPos < servoCenter && servoPos >= servoCenter) ||
        (lastServoPos > servoCenter && servoPos <= servoCenter))
    {
        doubleBeep();
    }

    lastServoPos = servoPos;
    lastLeftPressed  = leftPressed;
    lastRightPressed = rightPressed;

    updateLedBar();

    delay(moveDelayMs);
}