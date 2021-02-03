#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

/////////////////
// SETTINGS
#define jogBase_mm 1.0                    // X jog increment for each press of the button
#define jogFeed_mmSec 4.0                 // X jog feed rate
#define spindleIncr_Turns 0.5             // Spindle increment for each command
#define spindleMaxSpeed_TurnsSec 5.0

#define verbose false                     // Debug output to Serial

/////////////////

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels 
#define I2C_ADDRESS 0x3C
 
// Buttons
#define startPin 5
#define stopPin 4
#define modePin 6 // TODO: set mode enum
#define zeroPin 7

#define jogPosPin 9
bool jogPosVal = false;
bool jogPosValPrev = false;
bool joggingPos = false;

#define jogNegPin 8
bool jogNegVal = false;
bool jogNegValPrev = false;
bool joggingNeg = false;

#define dirPin 10
bool dirVal = false;

#define jog01Pin 12 // 0.1 mm/sec
#define jog10Pin 11 // 10 mm/sec
double jog_mm = jogBase_mm;

int speedPin = A1;
double speedVal = 0; // Turns per sec

bool running = false;

unsigned long lastDisplay = millis();
unsigned long lastStatusReq = millis();

enum modes {
  lathe,
  wind
};
modes mode = lathe;

// G Code
#define GRBL_BUFFER_SIZE 64//128
#define STRLEN 64
#define maxCommands 5
int charCount = 0; // Bytes sent to GRBL
int iNextCommand = 0; // Index of next command to send to GRBL
int pendCommands = 0; // Number of commands pending to be sent
char commandQ[maxCommands][STRLEN]; // Queue of gcode to send to GRBL
char nextCommand[STRLEN];
char gCodeStr[STRLEN];

// Status
char errorStr[STRLEN];
char respStr[STRLEN];
char speedStr[STRLEN];
char posStr[STRLEN];
char tempStr[STRLEN];

// Commanbded and current positions
double spindleCmd_Turns = 0;
double spindleCur_Turns = 0;
double xCmd_mm = 0;
double xCur_mm = 0;

// Talking to GRBL over software serial
#define grblRxPin 2
#define grblTxPin 3
SoftwareSerial grblSerial(grblRxPin, grblTxPin);

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1//4 // Reset pin # (or -1 if sharing Arduino reset pin)
SSD1306AsciiAvrI2c display;
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool AppendGCode(const char* gCode)
{
  // No room for a new command
  if (pendCommands >= maxCommands) return false;

  // Add command to internal buffer
  strcpy(commandQ[(iNextCommand + pendCommands) % maxCommands], gCode);
  pendCommands++;
//  if (verbose) Serial.println(gCode);

  return true;
}

void SendGCode()
{
  // Send commands until GRBL buffer is full
  while (pendCommands > 0)
  {
    strcpy(nextCommand, commandQ[iNextCommand]);
    int len = strlen(nextCommand);

    if (charCount + len <= GRBL_BUFFER_SIZE-1 && grblSerial.available() == 0)
    {
      grblSerial.write(nextCommand);
      grblSerial.write('\n');
      charCount += len + 1;
      if (verbose) Serial.println(nextCommand);

      pendCommands--;
      iNextCommand++;

      if (iNextCommand == maxCommands) iNextCommand = 0;
    }
    else
    {
      break;
    } 
  }

  // Request status
  unsigned long curTime = millis();
  if ((curTime - lastStatusReq) > 500 && grblSerial.available() == 0)
  {
    grblSerial.write("?");
    lastStatusReq = curTime;
  }

  grblSerial.flush();
}

void RecvResp()
{
  delay(50); // Necessary to prevent corruption in SoftwareSerial for some reason
 
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
  AppendGCode("G91");
  AppendGCode("G10 L20 P1 Y0"); // Reset spindle work position

  strcpy(respStr, "");
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
    if (verbose && running) Serial.println("Stop");
    running = false;
    display.setRow(0);
    display.clearToEOL();
    display.println("Stopped");
  }
  else if (!digitalRead(startPin)) 
  {
    if (verbose && !running) Serial.println("Run");
    running = true;
    display.setRow(0);
    display.clearToEOL();
    display.println("Running");
  }
  
  speedVal = float(analogRead(speedPin)) * (spindleMaxSpeed_TurnsSec-1.0) / 1024.0 + 1.0/1024.0; // Min 1 turn per second

  if (!digitalRead(zeroPin))
  AppendGCode("G10 L20 P1 X0 Y0"); // Reset  work position
}

void UpdateDisplay()
{
  unsigned long curTime = millis();
  if ((curTime - lastDisplay) < 200) return;
  lastDisplay = curTime;

  display.setRow(0);
  //display.setRow(1);
  display.println("");
  sprintf(speedStr,"%2.0f RPM",speedVal * 60.0);
  display.println(speedStr);

  //display.setRow(2);
  sprintf(posStr,"%4.1fmm %6.0f t",xCur_mm,spindleCur_Turns);
  display.println(posStr);
}

void JogX()
{
  // Add jogging command if there is room
  if (joggingPos)
  {
    sprintf(gCodeStr, "G1 X%f F%f", jog_mm, jogFeed_mmSec * 60.0);
    if (AppendGCode(gCodeStr)) joggingPos = false;
  }
  else if (joggingNeg)
  {
    sprintf(gCodeStr, "G1 X-%f F%f", jog_mm, jogFeed_mmSec * 60.0);
    if (AppendGCode(gCodeStr)) joggingNeg = false;
  }
}

void RunLathe()
{
  // Send another turn command if the queue is getting empty and we have few turns to go. Leave room for other commands.
  // TODO: Base this on time, not number of turns
  // TODO: Getting stuck at low RPM
  if (pendCommands >= maxCommands-1 || fabs(spindleCmd_Turns - spindleCur_Turns) > spindleIncr_Turns * 5.0) return;

Serial.println(dirVal);
  if (dirVal) 
  {
    sprintf(gCodeStr, "G1 Y%f F%f", spindleIncr_Turns, speedVal * 60.0);
    spindleCmd_Turns += spindleIncr_Turns;
  }
  else
  {
    sprintf(gCodeStr, "G1 Y-%f F%f", spindleIncr_Turns, speedVal * 60.0);
    spindleCmd_Turns -= spindleIncr_Turns;
  }
  AppendGCode(gCodeStr);
}

void loop() 
{
  GetInputs();
  JogX();

  switch (mode)
  {
    case lathe:
      if (running) RunLathe();
    break;

    case wind:
    break;
  }

  SendGCode();
  RecvResp();
  UpdateDisplay();
}

