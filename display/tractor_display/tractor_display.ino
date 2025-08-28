#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

const int maxLines = 6;  // Max lines we can display (7 * 8px = 56px + 8px for throttle)
String menuLines[maxLines];
int lineCount = 0;

String currentError = "";
int currentThrottle = 0;

bool hasReceivedData = false;

int displayDrawState = 0;
int previousDisplayDrawState = 0;

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
    }
    delay(200);

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

}


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
