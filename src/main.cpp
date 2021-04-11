/* TODO:
- Longer moves for winder
- Recv buffer freezing? (flush and/or check available)
*/

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

/////////////////
// SETTINGS
#define verbose false                     // Debug output to Serial

#define jogBase_mm 1.0                    // X jog increment for each press of the button
#define jogFeed_mmSec 5.0                 // X jog feed rate
#define spindleIncr_ms 500                // Time to run spindle for each command (lathe mode). Should be long enough to execute all other functions between sends, and longer than the interval between status requests.
#define spindleMaxSpeed_TurnsSec 8.0

// Pin assignments
#define grblRxPin 2
#define grblTxPin 3
#define startPin 5
#define stopPin 4
#define modePin 6
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
bool startVal = false;
bool startValPrev = false;
double jog_mm = jogBase_mm;
double spindleSpeed_TurnsSec = 0;
bool running = false;
unsigned long lastDisplay = millis();
unsigned long lastStatusReq = millis();
double spindleIncr_Turns = 0;
double spindleIncrNew_Turns = 0;
double spindleCmd_Turns = 0;
double newSpindleCmd_Turns = 0;
double spindleCur_Turns = 0;
double xCmd_mm = 0;
double newXCmd_mm = 0;
double xCur_mm = 0;
double xIncr_mm = 0;
double winderxMin_mm = 0;
double winderxMax_mm = 0;
double wireDia_mm = 0;

enum modes 
{
  undefined,
  lathe,
  wind
};
modes mode = undefined;
modes modePrev = undefined;

enum winderStages
{
  initEndX,
  initStartX,
  initDiameter,
  winding
};
winderStages winderStage = initEndX;

// G Code and GRBL
#define GRBL_BUFFER_SIZE 128
#define STRLEN 128
SoftwareSerial grblSerial(grblRxPin, grblTxPin);
int charCount = 0; // Bytes sent to GRBL
char gCodeStr[STRLEN];

// Status
char errorStr[STRLEN];
char respStr[STRLEN];
char speedStr[STRLEN];
char posStr[STRLEN];
char tempStr[STRLEN];
char statusStr[STRLEN];

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
SSD1306AsciiAvrI2c display;
#define I2C_ADDRESS 0x3C
 
bool SendGCode(const char* gCode)
{
  // Send command if GRBL buffer is not full
  int len = strlen(gCode);

  if (charCount + len <= GRBL_BUFFER_SIZE-1 && grblSerial.available() == 0)
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
  SendGCode("G90"); // Absolute mode by default. Jogs will override with relative.
  SendGCode("G10 L20 P1 Y0"); // Reset spindle work position
  grblSerial.write("~"); // Return to IDLE state

  strcpy(respStr, "");

  if (verbose) Serial.println("Initialized");
}

void StartSpindle()
{
  if (!running)
  {
    grblSerial.write("~"); // Resume
    if (verbose) Serial.println("Run");
  }
  running = true;

  if (mode == lathe) sprintf(statusStr, "Lathe Running");
  else sprintf(statusStr, "Winder Running");
}

void StopSpindle()
{
  if (running)
  {
    grblSerial.write("!"); // Cancel any existing jogs
    grblSerial.write("~"); // Return to IDLE if jogging
    if (verbose) Serial.println("Stop");
  }
  running = false;
  spindleCmd_Turns = spindleCur_Turns;

  if (mode == lathe) sprintf(statusStr, "Lathe Stopped");
  else sprintf(statusStr, "Winder Stopped");
}

void GetInputs()
{
  // Some commands latch true until they are able to be sent to GRBL

  if (!digitalRead(modePin))
  {
    mode = lathe;
  }
  else 
  {
    mode = wind;
  }

  if (mode != modePrev)
  {
    StopSpindle();
    winderStage = initEndX;
  }
  modePrev = mode;

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
  
  startVal = !digitalRead(startPin);
  if (startVal && !startValPrev)
  {
    if (mode == wind)
    {
      switch (winderStage)
      {
        case initEndX:
          winderxMax_mm = xCur_mm;
          winderStage = initStartX;
          break;

        case initStartX:
          winderxMin_mm = xCur_mm;

          // Initialize work position
          if (winderxMax_mm < winderxMin_mm)
          {
            double tempx = winderxMax_mm;
            winderxMax_mm = winderxMin_mm;
            winderxMin_mm = tempx;
            xCmd_mm = winderxMax_mm-winderxMin_mm; // At end
          }
          else
          {
            xCmd_mm = 0.0; // At start
          }
          winderxMax_mm -= winderxMin_mm;
          winderxMin_mm = 0.0;
          sprintf(gCodeStr, "G10 L20 P1 X%f Y0",xCmd_mm);
          SendGCode(gCodeStr);

          winderStage = initDiameter;
          break;

        case initDiameter:
          xIncr_mm = wireDia_mm;
          if (xCmd_mm > 0.0) xIncr_mm *= -1.0;
          winderStage = winding;

          sprintf(tempStr,"Min Max Incr: %f %f %f",winderxMin_mm,winderxMax_mm,xIncr_mm);
          if (verbose) Serial.println(tempStr);

          StartSpindle();
          break;

        case winding:
          StartSpindle();
          break;
      } 
    }

    else StartSpindle();
  }
  startValPrev = startVal;

  if (!digitalRead(stopPin)) StopSpindle();

  spindleSpeed_TurnsSec = float(analogRead(speedPin)) * (spindleMaxSpeed_TurnsSec-1.0) / 1024.0 + 1.0/1024.0; // Min 1 turn per second

  if (!digitalRead(zeroPin))
  SendGCode("G10 L20 P1 X0 Y0"); // Reset  work position
}

void UpdateDisplay()
{
  unsigned long curTime = millis();
  if ((curTime - lastDisplay) < 500) return;
  lastDisplay = curTime;

  display.clear();

  display.setRow(0);
  display.println(statusStr);

  display.setRow(2);
  sprintf(speedStr,"%2.0f RPM",spindleSpeed_TurnsSec * 60.0);
  display.println(speedStr);

  sprintf(posStr,"%4.1fmm %6.0f t",xCur_mm,spindleCur_Turns);
  display.println(posStr);
}

void JogX()
{
  // Don't jog if currently setting wire diameter or winding
  if (mode == wind && (winderStage == initDiameter || winderStage == winding))
  {
    if (joggingPos)
    {
      wireDia_mm += jog_mm * 0.1;
      joggingPos = false;
    }
    else if (joggingNeg)
    {
      wireDia_mm -= jog_mm * 0.1;
      joggingNeg = false;
    }
  }

  // Add jogging command if there is room. Otherwise try again later.
  else if (joggingPos)
  {
    sprintf(gCodeStr, "$J=G91 X%f F%f", jog_mm, jogFeed_mmSec * 60.0);
    if (SendGCode(gCodeStr)) joggingPos = false;
  }
  else if (joggingNeg)
  {
    sprintf(gCodeStr, "$J=G91 X-%f F%f", jog_mm, jogFeed_mmSec * 60.0);
    if (SendGCode(gCodeStr)) joggingNeg = false;
  }
}

void RunLathe()
{
  if (!running)
  {
    return;
  }

  if ((dirVal && ((spindleCmd_Turns - spindleCur_Turns) <= spindleIncr_Turns)) || (!dirVal && ((spindleCmd_Turns - spindleCur_Turns) >= spindleIncr_Turns)))
  {
    spindleIncrNew_Turns = spindleIncr_ms * spindleSpeed_TurnsSec / 1000.0;
    if (!dirVal) spindleIncrNew_Turns *= -1.0;
    sprintf(gCodeStr, "$J=G91 Y%f F%f", spindleIncrNew_Turns, spindleSpeed_TurnsSec * 60.0);
    if (SendGCode(gCodeStr))
    {
      spindleIncr_Turns = spindleIncrNew_Turns;
      spindleCmd_Turns += spindleIncr_Turns;
    }
  }
}

void Wind()
{
  // Start button advances stage
  switch (winderStage)
  {
    case initEndX:
      sprintf(statusStr, "Jog to end");
    break;

    case initStartX:
      sprintf(statusStr, "Jog to start");
    break;

    case initDiameter:
      sprintf(statusStr, "Set dia: %2.2f mm", wireDia_mm);
    break;

    case winding:
      if (running)
      {
        if ((dirVal && ((spindleCmd_Turns - spindleCur_Turns) <= spindleIncr_Turns)) || (!dirVal && ((spindleCmd_Turns - spindleCur_Turns) >= spindleIncr_Turns)))
        {
          spindleIncrNew_Turns = 1;
          if (!dirVal) spindleIncrNew_Turns *= -1.0;

          newSpindleCmd_Turns = spindleCmd_Turns + spindleIncr_Turns;
          newXCmd_mm = xCmd_mm + xIncr_mm;
          sprintf(gCodeStr, "G1 X%f Y%f F%f", newXCmd_mm, newSpindleCmd_Turns, spindleSpeed_TurnsSec * 60.0);
          if (SendGCode(gCodeStr))
          {
            spindleIncr_Turns = spindleIncrNew_Turns;
            spindleCmd_Turns = newSpindleCmd_Turns;
            xCmd_mm = newXCmd_mm;
            if (xCmd_mm >= winderxMax_mm || xCmd_mm <= winderxMin_mm) xIncr_mm *= -1;
          }
        }
      }
    break;
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
      Wind();
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

