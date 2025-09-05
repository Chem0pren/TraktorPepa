#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

//COMM
#define PKT_STATE 0x01   // State change (RUN, MENU, ERROR, INFO)
#define PKT_DATA  0x02   // Live data (pedal, encoder, joystick)
#define PKT_MENU  0x03   // Menu text (item1|item2|...)

// keep DataPacket same as sender
struct DataPacket { float pedalPercent, encoderPercent, joystick; };

enum ParseState { WAIT_START, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM };

ParseState parseState = WAIT_START;
byte pktType = 0;
byte pktLen = 0;
byte pktBuf[32];
byte pktIndex = 0;
byte checksum = 0;

// parser timeout/resync
unsigned long lastByteMs = 0;
const unsigned long PARSE_TIMEOUT_MS = 100; // if payload stalls >100ms -> resync

// small read-burst limit per loop
const uint8_t MAX_READS_PER_LOOP = 64;

// display / state variables (kept)
int pedalPercent = 0;
int encoderPercent = 0;
int joystickPercent = 0;

const int maxLines = 6;  // Max lines we can display
String menuLines[maxLines];
int lineCount = 0;

String currentError = "";
int currentThrottle = 0;

bool hasReceivedData = false;

int displayDrawState = 0;
int previousDisplayDrawState = 0;

enum SystemState : byte {
  STATE_INIT   = 0,
  STATE_RUN    = 1,
  STATE_ERROR  = 2,
  STATE_MENU   = 3,
  STATE_SAVING = 4
};

void setup() {
  Serial.begin(19200);
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_tf);

  Serial.println("Setup");           // DEBUG
  Serial.println("DEBUG: Display start");
  showReadyMessage();
}

void loop() {
  // read up to MAX_READS_PER_LOOP bytes so loop() can run other tasks
  uint8_t reads = 0;
  while (Serial.available() && reads < MAX_READS_PER_LOOP) {
    byte b = Serial.read();
    parseByte(b);
    lastByteMs = millis();
    reads++;
  }

  // if we're stuck waiting for payload and no new bytes recently -> resync
  if (parseState == WAIT_PAYLOAD && (millis() - lastByteMs) > PARSE_TIMEOUT_MS) {
    Serial.println(F("PARSER: timeout while waiting payload -> resync"));
    parseState = WAIT_START;
  }

  // Heartbeat
  static unsigned long last = 0;
  if (millis() - last > 1000) {
    Serial.println(F("still alive"));
    last = millis();
  }

  // small idle
  delay(5);
}

void parseByte(byte b) {
  switch (parseState) {
    case WAIT_START:
      if (b == 0xAA) {
        parseState = WAIT_TYPE;
        pktType = 0;
        pktLen = 0;
        pktIndex = 0;
        checksum = 0;
      }
      break;

    case WAIT_TYPE:
      pktType = b;
      checksum = b; // initialize checksum with type
      parseState = WAIT_LEN;
      break;

    case WAIT_LEN:
      pktLen = b;
      pktIndex = 0;
      checksum ^= b;
      // sanity check: length must fit our buffer and be > 0
      if (pktLen == 0 || pktLen > sizeof(pktBuf)) {
        Serial.print(F("PARSER: bad length="));
        Serial.println(pktLen);
        parseState = WAIT_START; // bad length -> resync
      } else {
        parseState = WAIT_PAYLOAD;
      }
      break;

    case WAIT_PAYLOAD:
      pktBuf[pktIndex++] = b;
      checksum ^= b;
      if (pktIndex >= pktLen) {
        parseState = WAIT_CHECKSUM;
      }
      break;

    case WAIT_CHECKSUM:
      if (checksum == b) {
        // ✅ Packet OK
        if (pktType == PKT_STATE && pktLen == 1) {
          SystemState st = (SystemState)pktBuf[0];
          handleState(st);
        }
        else if (pktType == PKT_DATA && pktLen == sizeof(DataPacket)) {
          DataPacket dp;
          memcpy(&dp, pktBuf, sizeof(dp));
          handleData(dp.pedalPercent, dp.encoderPercent, dp.joystick);
        } else if (pktType == PKT_MENU) {
          // optional: treat payload as string (ensure null-termination)
          char tmp[33];
          byte copyLen = min((int)pktLen, 32);
          memcpy(tmp, pktBuf, copyLen);
          tmp[copyLen] = '\0';
          Serial.print(F("PARSER: MENU payload: "));
          Serial.println(tmp);
        } else {
          Serial.print(F("PARSER: unknown pktType="));
          Serial.print(pktType);
          Serial.print(F(" len="));
          Serial.println(pktLen);
        }
      } else {
        Serial.print(F("PARSER: checksum fail (calc=0x"));
        Serial.print(checksum, HEX);
        Serial.print(F(" recv=0x"));
        Serial.print(b, HEX);
        Serial.print(F(" type="));
        Serial.print(pktType);
        Serial.print(F(" len="));
        Serial.println(pktLen);
      }
      // always resync to start after checksum (success or fail)
      parseState = WAIT_START;
      break;
  }
}

// ---------------- display / handlers (kept mostly unchanged) ----------------

void displayMenu() {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.firstPage();
  do {
    if (currentError.length() > 0) {
      u8g2.drawStr(0, 10, "!!! ERROR !!!");
      u8g2.drawStr(0, 20, currentError.c_str());
    } else {
      for (int i = 0; i < lineCount; i++) {
        int y = (i + 1) * 8;
        u8g2.drawStr(0, y, menuLines[i].c_str());
      }
    }
  } while (u8g2.nextPage());
}

void showReadyMessage() {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.firstPage();
  do {
    u8g2.drawStr(10, 30, "Display Ready...");
  } while (u8g2.nextPage());
}

void handleState(SystemState st) {
  switch (st) {
    case STATE_INIT:
      Serial.println("STATE: INIT");
      break;
    case STATE_RUN:
      Serial.println("STATE: RUN");
     // drawThrottle(40,40,String(pedalPercent));
      break;
    case STATE_ERROR:
      Serial.println("STATE: ERROR");
      break;
    case STATE_MENU:
      Serial.println("STATE: MENU");
      break;
    case STATE_SAVING:
      Serial.println("STATE: SAVING");
      break;
  }
}

void handleData(float pedal, float encoder, float joy) {
  currentThrottle  = static_cast<int>(pedal);
  pedalPercent = pedal;
  encoderPercent = encoder;
  joystickPercent = joy;

  drawThrottle(40,40,String(pedalPercent));

  Serial.print("DATA -> Pedal: ");
  Serial.print(pedal, 2);
  Serial.print(" | Encoder: ");
  Serial.print(encoder, 2);
  Serial.print(" | Joystick: ");
  Serial.println(joy, 2);
}

void drawThrottle(int pos_x,int pos_y,String message)
{
 // Serial.print("DEBUG: Drawing throttle: ");  // DEBUG
 // Serial.println(message);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB24_tr);
    u8g2.drawStr(pos_x, pos_y, message.c_str());
  } while (u8g2.nextPage());
  delay(10);
}

