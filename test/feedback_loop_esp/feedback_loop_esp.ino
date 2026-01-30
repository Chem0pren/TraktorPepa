#include <Wire.h>

// ================== INPUT ==================
#define PEDAL_POT_PIN   34 // magnetic encoder analog output

// ================== CONFIG ==================
#define PEDAL_MIN 250 //ohms
#define PEDAL_MAX 2480 //ohms
#define GEAR_RATIO 3.0f 

#define DEADZONE_DEG 0.5f

#define MIN_STEP_DELAY 200
#define MAX_STEP_DELAY 5000
#define MT6701_ADDR 0x06  // Default I2C Address

// ================== STEPPER PID ==================
float Kp1 = 1.5;
float Ki1 = 0.0;
float Kd1 = 0.05;

// ================== STATE ==================
float integral = 0;
float lastError = 0;

unsigned long lastMicros = 0;
unsigned long lastDebug = 0;


void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  
  // -------- Target (0–360°) --------
  int rawTarget = readADC(PEDAL_POT_PIN);
  rawTarget = constrain(rawTarget, PEDAL_MIN, PEDAL_MAX);
  
  
  delay(10);
}

// ================== ADC AVERAGING ==================
int readADC(int pin) {
  int sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(pin);
  }
  return sum >> 3;
}


float GetEncoderAngle()
{
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03);       // Point to Angle High Register
  Wire.endTransmission();
  
  Wire.requestFrom(MT6701_ADDR, 2); // Request 2 bytes

  if (Wire.available() >= 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();

    int rawValue = (highByte << 6) | (lowByte >> 2); 
    // 4. Convert to Degrees (0-360)
    float degrees = (float)rawValue * 360.0 / 16384.0;

    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Degrees: ");
    Serial.println(degrees);
    return degrees;
  }
}

