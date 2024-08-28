#include <U8glib.h>
#include <Servo.h>
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE | U8G_I2C_OPT_DEV_0);

//variables for stepper 
uint8_t txBuffer[20];      //Send data array
uint8_t rxBuffer[20];      //Receive data array
uint8_t rxCnt=0;          //Receive data count

uint8_t getCheckSum(uint8_t *buffer,uint8_t len);
void readRealTimeLocation(uint8_t slaveAddr);
bool waitingForACK(uint8_t len);
void printToMonitor(uint8_t *value);
void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absoluteAxis);

String test = "init";

int32_t absoluteAxis = 200;           //absolute coordinates 

//throttle potenciometers
int potPin = A0;
float potVal = 0;
int inputResMin = 516;
int inputResMax = 938;

float throttlePercent = 0;



void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  u8g.setColorIndex(1); // display draws with pixel on
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  // The power-on delay is 3000 milliseconds, waiting for the motor to be initialized
  delay(3000);
  }

}

void loop() {
  bool ackStatus = true;
  // put your main code here, to run repeatedly:
  
  //readRealTimeLocation(1); 
  //ackStatus = waitingForACK(10);  

  //positionMode3Run(1,500,0, (throttlePercent/1000)*4125); 
  // 4125 
  // 8250 = 180 
  //1 degree = 45.83
  //positionMode3Run(1,100,200,absoluteAxis); //Slave address=1, speed=100RPM, acceleration=200, absolute coordinates

  //calc potencimeter percent
  potVal = analogRead(potPin);
  throttlePercent = map(potVal, inputResMin, inputResMax, 0, 4125);

  if(throttlePercent>0){
    positionMode3Run(1,500,200, throttlePercent); 
  }else{
    positionMode3Run(1,500,200, 0); 
  }

  //Slave address=1, speed=100RPM, acceleration=200, absolute coordinates

  if(ackStatus == true)        //Received location information
  {
    test = "motor ok";
   // positionMode3Run(1,200,200,throttlePercent*100); 
   // printToMonitor(&rxBuffer[5]); // The lower 32 bits output the real-time position value to the serial monitor
    
  }
  else                      //Failed to receive location information (1. Check the connection of the serial cable; 2. Check whether the motor is powered on; 3. Check the slave address and baud rate)
  {
  
   test = "motor error";

  }


  delay(1); 

// front end
u8g.firstPage();
do {

  

  drawServoInfo(10,10,String(test));
  drawThrottle(10,20,String(potVal));


//draw(0,10,TimeString);
} while (u8g.nextPage());

}





void positionMode3Run(uint8_t slaveAddr,uint16_t speed,uint8_t acc,int32_t absAxis)
{
  int i;
  uint16_t checkSum = 0;

  txBuffer[0] = 0xFA;       //frame header
  txBuffer[1] = slaveAddr;  //slave address
  txBuffer[2] = 0xF5;       //function code
  txBuffer[3] = (speed>>8)&0x00FF; //8 bit higher speed
  txBuffer[4] = speed&0x00FF;     //8 bits lower
  txBuffer[5] = acc;            //acceleration
  txBuffer[6] = (absAxis >> 24)&0xFF;  //Absolute coordinates bit31 - bit24
  txBuffer[7] = (absAxis >> 16)&0xFF;  //Absolute coordinates bit23 - bit16
  txBuffer[8] = (absAxis >> 8)&0xFF;   //Absolute coordinates bit15 - bit8
  txBuffer[9] = (absAxis >> 0)&0xFF;   //Absolute coordinates bit7 - bit0
  txBuffer[10] = getCheckSum(txBuffer,10);  //Calculate checksum

  Serial.write(txBuffer,11);
}


void printToMonitor(uint8_t *value)
{
  int32_t iValue;
  String  tStr;
  iValue = (int32_t)(
                      ((uint32_t)value[0] << 24)    |
                      ((uint32_t)value[1] << 16)    |
                      ((uint32_t)value[2] << 8)     |
                      ((uint32_t)value[3] << 0)
                    );

  
  test = String(iValue);
}

void readRealTimeLocation(uint8_t slaveAddr)
{
 
  txBuffer[0] = 0xFA;       //frame header
  txBuffer[1] = slaveAddr;  //slave address
  txBuffer[2] = 0x31;       //function code
  txBuffer[3] = getCheckSum(txBuffer,3);  //Calculate checksum
  Serial.write(txBuffer,4);   //The serial port issues a command to read the real-time position
}

uint8_t getCheckSum(uint8_t *buffer,uint8_t size)
{
  uint8_t i;
  uint16_t sum=0;
  for(i=0;i<size;i++)
    {
      sum += buffer[i];  //Calculate accumulated value
    }
  return(sum&0xFF);     //return checksum
}

bool waitingForACK(uint8_t len)
{
  bool retVal;       //return value
  unsigned long sTime;  //timing start time
  unsigned long time;  //current moment
  uint8_t rxByte;      

  sTime = millis();    //get the current moment
  rxCnt = 0;           //Receive count value set to 0
  while(1)
  {
    if (Serial.available() > 0)     //The serial port receives data
    {
      rxByte = Serial.read();       //read 1 byte data
      if(rxCnt != 0)
      {
        rxBuffer[rxCnt++] = rxByte; //Storing data
      }
      else if(rxByte == 0xFB)       //Determine whether the frame header
      {
        rxBuffer[rxCnt++] = rxByte;   //store frame header
      }
    }

    if(rxCnt == len)    //Receive complete
    {
      if(rxBuffer[len-1] == getCheckSum(rxBuffer,len-1))
      {
        retVal = true;   //checksum correct
        break;                  //exit while(1)
      }
      else
      {
        rxCnt = 0;  //Verification error, re-receive the response
      }
    }

    time = millis();
    if((time - sTime) > 3000)   //Judging whether to time out
    {
      retVal = false;
      break;                    //timeout, exit while(1)
    }
  }
  return(retVal);
}




void drawServoInfo(int pos_x,int pos_y,String message)
{
u8g.setFont(u8g_font_04b_03b);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);
}

void drawThrottle(int pos_x,int pos_y,String message)
{
u8g.setFont(u8g_font_04b_03b);
u8g.setPrintPos(pos_x, pos_y);
u8g.print(message);
}

