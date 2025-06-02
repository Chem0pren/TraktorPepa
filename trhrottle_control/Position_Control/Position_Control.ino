#include <U8glib.h>
#include <EEPROM.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);

//pins
const int buttonPin = 2;

//current jostick value
int yVal = 0;
int xVal = 0;

#define pinY A2
#define pinX A3


//eprom data
int Kpaddress = 0;  // float takes 4 bytes
int Kiaddress = Kpaddress + 4;
int Kdaddress = Kiaddress + 4;
int AngleAdress = Kdaddress + 4;

//default data
/*
float KpDefault = 2.0;  
float KiDefault = 0.01;  
float KdDefault = 3.5;   
*/
float default_angle = 90;

float Kp;  
float Ki;  
float Kd;   
float max_angle;

//button state 
int buttonState = 0;  // variable for reading the pushbutton status
//hold button
unsigned long buttonHoldStart = 0;
bool buttonHeld = false;
const unsigned long holdTime = 3000; // 3 seconds

//display state
int CurrentDisplay = 1;

unsigned long lastMoveTime = 0;
const int debounceDelay = 500;

//menu item
struct MenuItem {
  const char* name;
  float* value;
  float step;
};

//menu list
MenuItem menu[] = {
  {"Kp", &Kp, 0.1},
  {"Ki", &Ki, 0.01},
  {"Kd", &Kd, 0.1},
  {"MaxAngle", &max_angle, 1.0}
};

const int menuLength = sizeof(menu) / sizeof(menu[0]);
int selectedItem = 0;

void setup() {

  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
  u8g.setColorIndex(1); // display draws with pixel on
  Serial.println("read from EEPROM.");
  delay(1000); // Short delay before reading
  //load EEPROM data
  loadFromEEPROM();

}

void loop() {
  
  //jostick read
  yVal = analogRead(pinY);
  xVal = analogRead(pinX);

  //handle button state
  void buttonStateHandle();

  
  if(CurrentDisplay==1)
  {
    InteractiveMenu();
  }

}

void buttonStateHandle()
{
    //hold button check
  if (digitalRead(buttonPin) == LOW) 
  {
    if (!buttonHeld) {
      buttonHeld = true;
      buttonHoldStart = millis();
    } else if (millis() - buttonHoldStart >= holdTime) {
      if(CurrentDisplay == 0)
      {
        Serial.println("entering menu");
        CurrentDisplay = 1;
      }else
      {
        CurrentDisplay = 3;
        Serial.println("save data");
        saveToEEPROM();
        //drawInfo(40,40,"Data");
        delay(500);
        CurrentDisplay = 0;
      }
      buttonHeld = false;
    }
  } else {
    buttonHeld = false;
  }

}


void InteractiveMenu()
{
  if (millis() - lastMoveTime > debounceDelay) 
  {
    // Navigation
    if (yVal < 400) 
    {
      selectedItem--;
      if (selectedItem < 0) selectedItem = menuLength - 1;
      lastMoveTime = millis();
      // drawMenu();   
      
    } else if (yVal > 600) 
    {
      selectedItem++;
      if (selectedItem >= menuLength) selectedItem = 0;
      lastMoveTime = millis();
      // drawMenu();   
      
    }

    // Adjust value left/right
    if (xVal < 400) {
      adjustValue(-1);
      lastMoveTime = millis();
      // drawMenu();   
      
    } else if (xVal > 600) {
      adjustValue(1);
      lastMoveTime = millis();
      // drawMenu();  
    }
  
  drawMenu();   
  }
}


  /*
  if(displayMenu == 0)
  {
    drawThrottle(40,40,String(100));
  } 
  */

void drawMenu() {

  u8g.setFont(u8g_font_04b_03br);
 
  u8g.firstPage();
  do {
    for (int i = 0; i < menuLength; i++) {
      int y = 7 + i * 7;
      if (i == selectedItem) {
        //u8g.drawFrame(0, y - 10, 100, 10);
        
        //u8g.setColorIndex(0); // text in black
        u8g.setPrintPos(2, y);
        u8g.print(">");
        u8g.print(menu[i].name);
        u8g.print(": ");
        u8g.print(*menu[i].value, 2);
        //u8g.setColorIndex(1); // reset to white
      } else {
        u8g.setPrintPos(2, y);
        u8g.print(menu[i].name);
        u8g.print(": ");
        u8g.print(*menu[i].value, 2);
      }
    }

    u8g.setPrintPos(2, 50);
   // u8g.print("PedalAngle: ");
   // u8g.print(String(PedalAngle));

    u8g.setPrintPos(2, 57);
   // u8g.print("PedalInput: ");
   // u8g.print(String(PedalInput1));

    u8g.setPrintPos(2, 63);
  //  u8g.print("current angle: ");
   // u8g.print(String(getPos()));

  } while (u8g.nextPage());
}

// === Adjust Current Value ===
void adjustValue(int direction) {
  *menu[selectedItem].value += menu[selectedItem].step * direction;
}


void saveToEEPROM() {
  for (int i = 0; i < menuLength; i++) {
    EEPROM.put(i * sizeof(float), *menu[i].value);
  }
}

void loadFromEEPROM() {
  for (int i = 0; i < menuLength; i++) {
    EEPROM.get(i * sizeof(float), *menu[i].value);
  }
}

float readFloatFromEEPROM(int address) {
  union {
    byte b[4];
    float f;
  } data;

  for (int i = 0; i < 4; i++) {
    data.b[i] = EEPROM.read(address + i);
  }

  return data.f;
}

//DISPLAY
void drawServoInfo(int pos_x,int pos_y,String message)
{
u8g.setFont(u8g_font_04b_03b);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);
}


void drawInfo(int pos_x,int pos_y,String message)
{
u8g.firstPage();
do {
  u8g.setFont(u8g_font_helvB24n);
  u8g.setPrintPos(pos_x, pos_y);
  u8g.print(message);
  } while (u8g.nextPage());
}


void drawThrottle(int pos_x,int pos_y,String message)
{
u8g.firstPage();
do {
  u8g.setFont(u8g_font_helvB24n);
  u8g.setPrintPos(pos_x, pos_y);
  u8g.print(message);
  } while (u8g.nextPage());
//u8g.drawBox(10, 50, 100 ,50);
}


