#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

//COMM
#define PKT_STATE 0x01   // State change (RUN, MENU, ERROR, INFO)
#define PKT_DATA  0x02   // Live data (pedal, encoder, joystick)
#define PKT_MENU  0x03   // Menu text (item1|item2|...)

struct StatePacket { byte stateId; };
struct DataPacket { float pedalPercent, encoderPercent, joystick; };

enum ParseState { WAIT_START, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM };

ParseState parseState = WAIT_START;
byte pktType, pktLen, pktBuf[32], pktIndex;
byte checksum;
//




const int maxLines = 6;  // Max lines we can display (7 * 8px = 56px + 8px for throttle)
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

  Serial.println("Setup");  // DEBUG
  Serial.println("DEBUG: Display start");  // DEBUG
 // u8g2.clearDisplay();
  //u8g2.initDisplay();
  //u8g2.clearBuffer();
  showReadyMessage();
  
}

void loop() {
  while (Serial.available()) {
    parseByte(Serial.read());
  }
}

/*  
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    Serial.println(line);  // DEBUG
    //if(previousDisplayDrawState != displayDrawState){
   // u8g2.clearDisplay();
   // u8g2.clearBuffer();
    //}
     if (line.startsWith("THROTTLE:")){
        currentThrottle = line.substring(22).toInt();
        drawThrottle(40, 40, String(currentThrottle));
        Serial.println("Show throttle");  // DEBUG
       // u8g2.clearBuffer();
     }else if (line.startsWith("INFO:")){
        u8g2.clearDisplay();
        u8g2.clearBuffer();
        Serial.println("Show info");  // DEBUG
        Serial.println(line);  // DEBUG
        showInfoFromString(line);
     }
    /*
     }else if (line.startsWith("ERROR:")){
        Serial.println("Show error");  // DEBUG
        showInfoFromString(line);
     }else if (line.startsWith("MENU:")){
        //Serial.println("Show info");  // DEBUG
        showInfoFromString(line);

     }
    */
  //  }
   // delay(200);

    /*
    if (line == "END") {
        if(displayDrawState == 1)
        {
        displayMenu();
        lineCount = 0;
        }

      }
      else if (line.startsWith("THROTTLE:")) {

        //just for obtain throttle value
        currentThrottle = line.substring(9).toInt();
       // Serial.println(currentThrottle);
        

      }
      else if (line.startsWith("DISPLAY:")) {
        displayDrawState = 0;
        if(displayDrawState == 0){
        //Serial.println("DEBUG:draw throttle"); 
        drawThrottle(40, 40, String(currentThrottle));
         }
      }
      else if (line.startsWith("MENU:")) {
        displayDrawState = 1;
    
      }

      else if(displayDrawState == 1) {
      if (lineCount < maxLines) {
        menuLines[lineCount++] = line;
      } else {
       // Serial.println("DEBUG: Menu line count exceeded maxLines");     
      }

    }
  }
  previousDisplayDrawState = displayDrawState;
*/

//}


void displayMenu() {
 // Serial.println("DEBUG: Drawing menu on display");  // DEBUG
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

      // char buf[20];
      // snprintf(buf, sizeof(buf), "Throttle: %.2f", currentThrottle);
      // u8g2.drawStr(0, 64, buf);  // bottom line
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

void showMessage(String message) {
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.firstPage();
  do {
    u8g2.drawStr(10, 30, message.c_str());
  } while (u8g2.nextPage());
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

void showInfoFromString(String menuStr) {
  u8g2.setFont(u8g2_font_5x8_tf);
  const int lineHeight = u8g2.getMaxCharHeight();

  // Remove "MENU:" at the start and "END" at the end if they exist
  if (menuStr.startsWith("MENU:")) {
    menuStr.remove(0, 5); // remove "MENU:"
  }
  int endIndex = menuStr.indexOf("END");
  if (endIndex != -1) {
    menuStr.remove(endIndex);
  }

  u8g2.firstPage();
  do {
    int y = lineHeight;
    int start = 0;
    while (true) {
      int sepIndex = menuStr.indexOf('|', start);
      if (sepIndex == -1) break; // no more separators

      String line = menuStr.substring(start, sepIndex);
      line.trim(); // remove extra spaces

      if (line.length() > 0) {
        u8g2.drawStr(0, y, line.c_str());
        y += lineHeight;
      }

      start = sepIndex + 1; // move past separator
    }
  } while (u8g2.nextPage());
}

//COMM
void parseByte(byte b) {
  switch (parseState) {
    case WAIT_START:
      if (b == 0xAA) {
        parseState = WAIT_TYPE;
      }
      break;

    case WAIT_TYPE:
      pktType = b;
      checksum = b;
      parseState = WAIT_LEN;
      break;

    case WAIT_LEN:
      pktLen = b;
      pktIndex = 0;
      checksum ^= b;
      parseState = WAIT_PAYLOAD;
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
        }
      } else {
        // ❌ checksum error → discard
      }
      parseState = WAIT_START;
      break;
  }
}

// Example handlers

void handleState(SystemState st) {
  switch (st) {
    case STATE_INIT:
      Serial.println("STATE: INIT");
      break;

    case STATE_RUN:
      Serial.println("STATE: RUN");
      // show big number
      break;

    case STATE_ERROR:
      Serial.println("STATE: ERROR");
      // show error msg
      break;

    case STATE_MENU:
      Serial.println("STATE: MENU");
      // show menu items
      break;

    case STATE_SAVING:
      Serial.println("STATE: SAVING");
      // show saving screen
      break;
  }
}



void handleData(float pedal, float encoder, float joy) {
 // updateRunDisplay(pedal, encoder, joy);
 /*
  Serial.print("DATA -> Pedal: ");
  Serial.print(pedal, 2);   // 2 decimal places
  Serial.print(" | Encoder: ");
  Serial.print(encoder, 2);
  Serial.print(" | Joystick: ");
  Serial.println(joy, 2);
  */
}
