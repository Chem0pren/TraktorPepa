#include <Wire.h>
#include <PID_v1.h> // Ensure you install "PID" by Brett Beauregard via Library Manager

// ================== PINS ===================
#define STEP_PIN      12
#define DIR_PIN       14
#define PEDAL_POT_PIN 34 

// ================== CONFIG ==================
#define PEDAL_MIN     250 
#define PEDAL_MAX     2480 
#define GEAR_RATIO    3.0f 
#define DEADZONE_DEG  1.0f

#define MT6701_ADDR   0x06 

unsigned long lastDebug = 0;

// ================== PID VARIABLES =============
double setpoint, input, output;
// Tune these: Kp handles snap, Kd handles oscillation
double Kp = 1.0, Ki = 0.01, Kd = 0.02; 

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  Wire.begin();
  Serial.begin(115200);

  // Configure PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // We'll map this to step frequency
  myPID.SetSampleTime(10);          // 100Hz refresh rate
}

void loop() {
  // 1. Get Target from Pedal (0 - 360 degrees)
  int rawTarget = readADC(PEDAL_POT_PIN);
  rawTarget = constrain(rawTarget, PEDAL_MIN, PEDAL_MAX);
  setpoint = map(rawTarget, PEDAL_MIN, PEDAL_MAX, 0, 360);

  // 2. Get Current Position from MT6701
  input = GetEncoderAngle();

  // 3. Compute PID
  float error = setpoint - input;

  // Handle circular wrap-around (shortest path)
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  // Manual deadzone to prevent "jitter" at rest
  if (abs(error) < DEADZONE_DEG) {
    output = 0;
  } else {
    myPID.Compute();
  }

  // 4. Drive Stepper
  handleStepper(output);

  // Debugging
  if (millis() - lastDebug > 100) {
    Serial.printf("Target: %.2f | Current: %.2f | Out: %.2f\n", setpoint, input, output);
    lastDebug = millis();
  }
}

void handleStepper(double pidOut) {
  if (pidOut == 0) return;

  // Set Direction
  digitalWrite(DIR_PIN, pidOut > 0 ? HIGH : LOW);

  // Map PID magnitude to speed (Lower delay = Higher speed)
  // Adjust 2000 and 200 based on your motor's torque capabilities
  int speedDelay = map(abs(pidOut), 0, 255, 2000, 200); 

  // Pulse the motor
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10); // Minimum pulse width
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(speedDelay); 
}

// ================== HELPER FUNCTIONS ==================

int readADC(int pin) {
  int sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return sum >> 3;
}

float GetEncoderAngle() {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03);
  if (Wire.endTransmission() != 0) return input; // Return last known if I2C fails

  Wire.requestFrom(MT6701_ADDR, 2);
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    uint16_t rawValue = (highByte << 6) | (lowByte >> 2);
    return (float)rawValue * 360.0 / 16384.0;
  }
  return input; 
}