#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

const int maxLines = 7;  // Max lines we can display (7 * 8px = 56px + 8px for throttle)
String menuLines[maxLines];
int lineCount = 0;

String currentError = "";
int currentThrottle = 0;

bool hasReceivedData = false;

void setup() {
  Serial.begin(9600);
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_tf);

  Serial.println("Setup");  // DEBUG
  Serial.println("DEBUG: Display start");  // DEBUG
  //showReadyMessage();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    Serial.print("DEBUG: Received line: ");
    Serial.println(line);

    if (line == "END") {
      
      }
      else if (line.startsWith("THROTTLE:")) {
      currentThrottle = line.substring(9).toInt();
      
      Serial.print("DEBUG: Throttle updated to ");
      Serial.println(currentThrottle,2);
      //drawThrottle(40, 40, String(currentThrottle,2));

      }
      else if (line.startsWith("DISPLAY:")) {
      Serial.println("DEBUG: DISPLAY=0 received, drawing throttle only");
      //currentThrottle = line.substring(9).toFloat();
      drawThrottle(40, 40, String(currentThrottle));
    }
  }
}
  

//     if (line == "END") {
//       hasReceivedData = true;
//       Serial.println("DEBUG: End of data received, displaying menu");
//       displayMenu();
//       lineCount = 0;
//     } 
//     else if (line.startsWith("THROTTLE:")) {
//       currentThrottle = line.substring(9).toFloat();
//       Serial.print("DEBUG: Throttle updated to ");
//       Serial.println(currentThrottle);
//     } 
//     else if (line.startsWith("ERROR:")) {
//       currentError = line.substring(6);
//       Serial.print("DEBUG: Error message set to: ");
//       Serial.println(currentError);
//     } 
//     else if (line.startsWith("DISPLAY:")) {
//       Serial.println("DEBUG: DISPLAY=0 received, drawing throttle only");
//       drawThrottle(40, 40, String(currentThrottle));
//     }
//     else {
//       if (lineCount < maxLines) {
//         menuLines[lineCount++] = line;
//         Serial.print("DEBUG: Menu line stored: ");
//         Serial.println(line);
//       } else {
//         Serial.println("DEBUG: Menu line count exceeded maxLines");
//       }
//     }
//   }
//   delay(1000);
// }


void displayMenu() {
  Serial.println("DEBUG: Drawing menu on display");  // DEBUG
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

      char buf[20];
      snprintf(buf, sizeof(buf), "Throttle: %.2f", currentThrottle);
      u8g2.drawStr(0, 64, buf);  // bottom line
    }
  } while (u8g2.nextPage());
}

void showReadyMessage() {
  u8g2.firstPage();
  do {
    u8g2.drawStr(10, 30, "Display Ready...");
  } while (u8g2.nextPage());
}

void drawThrottle(int pos_x,int pos_y,String message)
{
  Serial.print("DEBUG: Drawing throttle: ");  // DEBUG
  Serial.println(message);
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvB24_tr);
    u8g2.drawStr(pos_x, pos_y, message.c_str());
  } while (u8g2.nextPage());
}
