#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

/////////////////
// SETTINGS
#define verbose true                     // Debug output to Serial

#define jogBase_mm 1.0                    // X jog increment for each press of the button
#define jogFeed_mmSec 4.0                 // X jog feed rate
#define spindleIncr_ms 400                // Time to run spindle for each command. Should be long enough to execute all other functions between sends, and longer than the interval between status requests.
#define spindleMaxSpeed_TurnsSec 5.0

// Pin assignments
#define grblRxPin 2
#define grblTxPin 3
#define startPin 5
#define stopPin 4
#define modePin 6 // TODO: set mode enum
#define zeroPin 7
#define jogNegPin 8
#define jogPosPin 9
#define dirPin 10
#define jog01Pin 12 // 0.1 mm/sec
#define jog10Pin 11 // 10 mm/sec
int speedPin = A1;

/////////////////

bool jogPosVal = false;
bool jogPosValPrev = false;
bool joggingPos = false;
bool jogNegVal = false;
bool jogNegValPrev = false;
bool joggingNeg = false;
bool dirVal = false;
bool initLathe = true;
double jog_mm = jogBase_mm;
double spindleSpeed_TurnsSec = 0;
bool running = false;
unsigned long lastDisplay = millis();
unsigned long lastStatusReq = millis();
double spindleIncr_Turns = 0;
double spindleIncrNew_Turns = 0;
double spindleCmd_Turns = 0;
double spindleCur_Turns = 0;
double xCmd_mm = 0;
double xCur_mm = 0;

enum modes {
  lathe,
  wind
};
modes mode = lathe;

// G Code and GRBL
#define GRBL_BUFFER_SIZE 128
#define STRLEN 64
#define maxCommands 5
SoftwareSerial grblSerial(grblRxPin, grblTxPin);
int charCount = 0; // Bytes sent to GRBL
char gCodeStr[STRLEN];

// Status
char errorStr[STRLEN];
char respStr[STRLEN];
char speedStr[STRLEN];
char posStr[STRLEN];
char tempStr[STRLEN];

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
SSD1306AsciiAvrI2c display;
#define I2C_ADDRESS 0x3C
 
bool SendGCode(const char* gCode)
{
  // Send command if GRBL buffer is not full
  int len = strlen(gCode);

  if (charCount + len <= GRBL_BUFFER_SIZE-1 ) // && grblSerial.available() == 0
  {
    grblSerial.write(gCode);
    grblSerial.write('\n');
    charCount += len + 1;
    grblSerial.flush();

    if (verbose) Serial.println(gCode);

    return true;
  }

  return false;
}

void RecvResp()
{
  while (grblSerial.available() > 0)
  {
    char recvChar = grblSerial.read();

    // Read characters until new line
    if (recvChar == '\n')
    {
      // Status message
      // NOTE: This requires $10=0
      if (strchr(respStr,'<'))
      {
        // WPos:0.000,0.000,0.000
        const char* iX = strchr(respStr,':') + 1; 
        const char* iY = strchr(iX,',') + 1; 
        const char* iZ = strchr(iY,',') + 1; 

        int tempLen = iY-iX-1;
        memcpy(tempStr, iX, tempLen);
        tempStr[tempLen] = '\0';
        xCur_mm = atof(tempStr);

        tempLen = iZ-iY-1;
        memcpy(tempStr, iY, tempLen);
        tempStr[tempLen] = '\0';
        spindleCur_Turns = atof(tempStr);
      }
      // Normal response
      else if (strstr(respStr,"ok"))
      {
        charCount = 0;
      }
      // Error
      else if (strstr(respStr,"error"))
      {
        strcpy(errorStr,respStr);
        charCount = 0;

        display.setRow(0);
        display.println(""); display.println(""); display.println("");
        display.clearToEOL();
        display.println(errorStr); 
      }
      if (verbose) Serial.println(respStr);
      strcpy(respStr, "");
    }
    else if (strlen(respStr) < STRLEN)
    {
      strncat(respStr, &recvChar, 1);
    }
    else
    {
      // Expected response was never received
      sprintf(errorStr,"Unknown resp");
      charCount = 0;
    }
  }
}

void InitDisplay()
{
  display.begin(&Adafruit128x64, I2C_ADDRESS);
  display.setFont(X11fixed7x14);
  display.clear();
  display.print("Stopped");
}

void setup() 
{
  // Set inputs
  pinMode(jogPosPin, INPUT_PULLUP);
  pinMode(jogNegPin, INPUT_PULLUP);
  pinMode(dirPin, INPUT_PULLUP);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(zeroPin, INPUT_PULLUP);
  pinMode(jog01Pin, INPUT_PULLUP);
  pinMode(jog10Pin, INPUT_PULLUP);
  pinMode(modePin, INPUT_PULLUP);

  pinMode(grblRxPin, INPUT);
  pinMode(grblTxPin, OUTPUT);

  Serial.begin(9600);
  grblSerial.begin(9600);
  
  InitDisplay();

  // Initialize GRBL
  SendGCode("G91");
  SendGCode("G10 L20 P1 Y0"); // Reset spindle work position
  grblSerial.write("~"); // Return to IDLE state

  strcpy(respStr, "");

  if (verbose) Serial.println("Initialized");
}

void GetInputs()
{
  // Some commands latch true until they are able to be sent to GRBL

  jogPosVal = !digitalRead(jogPosPin);
  joggingPos |= (jogPosVal && !jogPosValPrev);
  jogPosValPrev = jogPosVal;

  jogNegVal = !digitalRead(jogNegPin);
  joggingNeg |= (jogNegVal && !jogNegValPrev);
  jogNegValPrev = jogNegVal;

  if (!digitalRead(jog01Pin))
  {
    jog_mm = 0.1 * jogBase_mm;
  }
  else if (!digitalRead(jog10Pin))
  {
    jog_mm = 10.0 * jogBase_mm;
  }
  else
  {
    jog_mm = jogBase_mm;
  }
  
  dirVal = digitalRead(dirPin);
  
  if (!digitalRead(stopPin))
  { 
    if (running)
    {
      grblSerial.write("!"); // Cancel any existing movements
      grblSerial.write("~"); // Return to IDLE state
      if (verbose) Serial.println("Stop");
    }
    running = false;
    display.setRow(0);
    display.clearToEOL();
    display.println("Stopped");
  }
  else if (!digitalRead(startPin)) 
  {
    if (!running)
    {
      if (verbose) Serial.println("Run");
    }
    running = true;
    display.setRow(0);
    display.clearToEOL();
    display.println("Running");
  }
  
  spindleSpeed_TurnsSec = float(analogRead(speedPin)) * (spindleMaxSpeed_TurnsSec-1.0) / 1024.0 + 1.0/1024.0; // Min 1 turn per second

  if (!digitalRead(zeroPin))
  SendGCode("G10 L20 P1 X0 Y0"); // Reset  work position
}

void UpdateDisplay()
{
  unsigned long curTime = millis();
  if ((curTime - lastDisplay) < 200) return;
  lastDisplay = curTime;

  display.setRow(0);
  display.println("");
  sprintf(speedStr,"%2.0f RPM",spindleSpeed_TurnsSec * 60.0);
  display.println(speedStr);

  sprintf(posStr,"%4.1fmm %6.0f t",xCur_mm,spindleCur_Turns);
  display.println(posStr);
}

void JogX()
{
  // Add jogging command if there is room. Otherwise try again later.
  if (joggingPos)
  {
    sprintf(gCodeStr, "$J=X%f F%f", jog_mm, jogFeed_mmSec * 60.0);
    if (SendGCode(gCodeStr)) joggingPos = false;
  }
  else if (joggingNeg)
  {
    sprintf(gCodeStr, "$J=X-%f F%f", jog_mm, jogFeed_mmSec * 60.0);
    if (SendGCode(gCodeStr)) joggingNeg = false;
  }
}

void RunLathe()
{
  if (!running)
  {
    spindleCmd_Turns = spindleCur_Turns;
    return;
  }

  if ((dirVal && ((spindleCmd_Turns - spindleCur_Turns) <= spindleIncr_Turns)) || (!dirVal && ((spindleCmd_Turns - spindleCur_Turns) >= spindleIncr_Turns)))
  {
    spindleIncrNew_Turns = spindleIncr_ms * spindleSpeed_TurnsSec / 1000.0;
    if (!dirVal) spindleIncrNew_Turns *= -1.0;
    sprintf(gCodeStr, "$J=Y%f F%f", spindleIncrNew_Turns, spindleSpeed_TurnsSec * 60.0);
    if (SendGCode(gCodeStr))
    {
      spindleIncr_Turns = spindleIncrNew_Turns;
      spindleCmd_Turns += spindleIncr_Turns;
    }
    //if (verbose) Serial.println("Lathe: Incr" + String(spindleIncr_Turns));
  }
  else
  {
    //if (verbose) Serial.println("Lathe: " + String(spindleCur_Turns) + ", waiting for " + String(spindleCmd_Turns));
  }
}

void loop() 
{
  GetInputs();
  JogX();

  switch (mode)
  {
    case lathe:
      RunLathe();
    break;

    case wind:
      //TODO
    break;
  }

  // Request status
  unsigned long curTime = millis();
  if ((curTime - lastStatusReq) > 200 ) //&& grblSerial.available() == 0
  {
    grblSerial.write("?");
    lastStatusReq = curTime;
  }

  RecvResp();
  UpdateDisplay();
}

