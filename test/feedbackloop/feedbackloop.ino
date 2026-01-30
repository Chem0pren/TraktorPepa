// ================== STEPPER ==================
#define STEP_PIN 18
#define DIR_PIN  16
#define EN_PIN   27

// ================== INPUT ==================
#define TARGET_POT_PIN   34
#define ENCODER_PIN      35   // magnetic encoder analog output

// ================== CONFIG ==================
#define ADC_MIN 250
#define ADC_MAX 2480

#define ENC_MIN 0
#define ENC_MAX 4095   // ESP32 ADC full range

#define GEAR_RATIO 3.0f

#define DEADZONE_DEG 0.5f

#define MIN_STEP_DELAY 200
#define MAX_STEP_DELAY 5000

// ================== PID ==================
float Kp = 1.5;
float Ki = 0.0;
float Kd = 0.05;

// ================== STATE ==================
float integral = 0;
float lastError = 0;

unsigned long lastMicros = 0;
unsigned long lastDebug = 0;

// ================== ADC AVERAGING ==================
int readADC(int pin) {
  int sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(pin);
  }
  return sum >> 3;
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  Serial.println("Stepper Servo | Analog Encoder | 12-bit ADC");

  lastMicros = micros();
}

void loop() {

  // -------- Target (0–360°) --------
  int rawTarget = readADC(TARGET_POT_PIN);
  rawTarget = constrain(rawTarget, ADC_MIN, ADC_MAX);

  float targetDeg =
    map(rawTarget, ADC_MIN, ADC_MAX, 0, 360);

  // -------- Encoder (0–360°) --------
  int rawEnc = readADC(ENCODER_PIN);
  rawEnc = constrain(rawEnc, ENC_MIN, ENC_MAX);

  float positionDeg =
    map(rawEnc, ENC_MIN, ENC_MAX, 0, 360);

  // -------- Error (circular) --------
  float error = targetDeg - positionDeg;

  // Wrap-around handling (0–360° loop)
  if (error > 180)  error -= 360;
  if (error < -180) error += 360;

  // -------- Deadzone --------
  if (abs(error) < DEADZONE_DEG) {
    integral = 0;
    debugPrint(targetDeg, positionDeg, error, 0);
    delay(5);
    return;
  }

  // -------- PID timing --------
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  lastMicros = now;

  if (dt <= 0) return;

  // -------- PID --------
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  lastError = error;

  float pid = Kp * error + Ki * integral + Kd * derivative;

  // -------- Direction --------
  digitalWrite(DIR_PIN, pid > 0 ? HIGH : LOW);

  // -------- Speed (gear compensated) --------
  float speed = abs(pid) * GEAR_RATIO;
  speed = constrain(speed, 0, 360 * GEAR_RATIO);

  int stepDelay =
    map(speed, 0, 360 * GEAR_RATIO,
        MAX_STEP_DELAY, MIN_STEP_DELAY);

  // -------- Step --------
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepDelay);

  // -------- Debug --------
  debugPrint(targetDeg, positionDeg, error, pid);
}

// ================== DEBUG ==================
void debugPrint(float target, float pos,
                float err, float pid) {

  unsigned long now = millis();
  if (now - lastDebug < 100) return;
  lastDebug = now;

  Serial.print("Target:");
  Serial.print(target, 1);

  Serial.print(" | Pos:");
  Serial.print(pos, 1);

  Serial.print(" | Err:");
  Serial.print(err, 2);

  Serial.print(" | PID:");
  Serial.println(pid, 2);
}
