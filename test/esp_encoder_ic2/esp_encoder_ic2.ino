#include <Wire.h>

#define MT6701_ADDR 0x06  // Default I2C Address

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void loop() {
  // 1. Request 2 bytes starting from register 0x03
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03);       // Point to Angle High Register
  Wire.endTransmission();
  
  Wire.requestFrom(MT6701_ADDR, 2); // Request 2 bytes

  if (Wire.available() >= 2) {
    // 2. Read the bytes
    byte highByte = Wire.read();
    byte lowByte = Wire.read();

    // 3. Combine them into a 14-bit integer
    // High byte contains the top 8 bits. Shift it left by 6.
    // Low byte contains the bottom 6 bits (bits 5-0).
    int rawValue = (highByte << 6) | (lowByte >> 2); 
    // Note: Some datasheets/libs suggest lowByte is right-aligned (bits 0-5), 
    // others suggest left-aligned (bits 2-7). 
    // If the above gives noisy data, try: int rawValue = (highByte << 6) | (lowByte & 0x3F);

    // 4. Convert to Degrees (0-360)
    float degrees = (float)rawValue * 360.0 / 16384.0;

    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print(" | Degrees: ");
    Serial.println(degrees);
  }
  
  delay(10);
}