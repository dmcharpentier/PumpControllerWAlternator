#include <SD.h>
//#include <ModbusMaster.h>
//#include <SoftwareSerial.h>
#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long
//Pin Definitions
//********************************************************
//Pins to read 8 button inputs
#define LOAD_165_PIN 4
#define DATA_165_PIN 2
#define CLK_165_PIN 3
//Relay and 4 Digit 7 Segment Display Pins
//ALWAYS draw OE_595_PIN LOW during setup to enable Relays and Tube.
#define LATCH_PIN 11
#define CLOCK_PIN 12
#define DATA_PIN 9
#define OE_595_PIN 10
#define RS485_RD 13
//Set pins for onboard buttons
const int BUTTON_PINS[] = { 5, 6, 7, 8 };
//Variables for program operation
uchar dat_buf[8] = { 0 };
uchar setRelay;
uchar setRemoteRelay = { 0 };
uchar displayData;
uchar comNum;
//Raw input from buttons
byte dataFrom165;
//Pump and Hand button logic
byte pumpAuto;
byte pumpHand;
//Pump Logic
int activePump;
int lastPump;
int run = 0;
int disChoice;
int running;
//Timers for logic
ulong lastRunTime = 0;
//Set tube segment hex characters
/*       NO.:0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28
/*Character :0,1,2,3,4,5,6,7,8,9,A ,b ,C ,c ,d ,E ,F ,H ,h ,L ,n ,N ,o ,P ,r ,t ,U ,- ,  ,*/
uchar tubeSegments[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x58, 0x5e, 0x79, 0x71, 0x76, 0x74, 0x38, 0x54, 0x37, 0x5c, 0x73, 0x50, 0x78, 0x3e, 0x40, 0x00 };
uchar tubeNumbers[] = { 0xfe, 0xff, 0xfd, 0xff, 0xfb, 0xff, 0xf7, 0xff };

//Read 8 button inputs
byte readByteFrom165() {
  uchar byte_temp = 0;
  uchar byte_;

  // Write pulse to load pin
  digitalWrite(LOAD_165_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(LOAD_165_PIN, HIGH);

  // Get data from 74HC165
  for (int i = 7; i >= 0; i--) {
    byte_ = digitalRead(DATA_165_PIN);
    byte_temp |= (byte_ << i);
    digitalWrite(CLK_165_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLK_165_PIN, LOW);
  }

  return ~byte_temp;
}

//Iterate through relay and tube display bits
void updateDisplayAndRelay() {
  comNum = (comNum < 7) ? comNum + 1 : 0;
  displayData = dat_buf[comNum];
  bitRelayUpdate();
}

//Output Relay and current segment information to 165
void bitRelayUpdate() {
  uchar tubeData = tubeSegments[displayData];
  uchar bitNum = tubeNumbers[comNum];

  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, setRelay);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bitNum);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, tubeData);
  digitalWrite(LATCH_PIN, HIGH);
}

int checkRunTimeElapsed(int delay) {
  // Get the current time
  ulong currentTime = millis();

  // Check if 0.25 seconds have passed since the last time the function was called
  if (currentTime - lastRunTime >= delay) {
    // Update the last time the function was called
    lastRunTime = currentTime;
    return 1;
  } else {
    return 0;
  }
}

void inputLogic(byte inputs) {
  int ctime;
  //enable when on
  for (int i = 0; i < 8; ++i) {
    if (bitRead(inputs, i) == 1) {
      // Execute commands based on the bit position (i)
      switch (i) {
        case 0:
          //P1A - Set pump1 Auto
          bitSet(pumpAuto, 0);
          break;
        case 1:
          //P1H - Set pump1 Hand
          bitSet(pumpHand, 0);
          //extChoice = 4;
          break;
        case 2:
          //P2A - Set pump2 Auto
          bitSet(pumpAuto, 1);
          break;
        case 3:
          //P2H - Set pump2 Hand
          bitSet(pumpHand, 1);
          //extChoice = 5;
          break;
        case 7:
          ctime = checkRunTimeElapsed(150);
          if (!run && ctime) {
            run = 1;
          }
          break;
        default:

          break;
      }
    }
    if (bitRead(inputs, i) == 0) {
      // Execute commands based on the bit position (i)
      switch (i) {
        case 0:
          // Set pump1 Active
          bitClear(pumpAuto, 0);
          break;
        case 1:
          // Command for bit 1
          bitClear(pumpHand, 0);
          break;
        case 2:
          // Set pump1 Active
          bitClear(pumpAuto, 1);
          break;
        case 3:
          // Command for bit 1
          bitClear(pumpHand, 1);
          break;
        case 7:
          ctime = checkRunTimeElapsed(150);
          // Command for bit 1
          if (run && ctime) {
            activePump = setActivePump(activePump);
            run = 0;
          }
          break;
        default:

          break;
      }
    }
  }
}

int setActivePump(int currentPump) {
  if (!bitRead(pumpAuto, 0) && !bitRead(pumpAuto, 1)) {
    //return currentPump;
  } else if (bitRead(pumpAuto, 0) && !bitRead(pumpAuto, 1)) {
    return 0;
  } else if (!bitRead(pumpAuto, 0) && bitRead(pumpAuto, 1)) {
    return 1;
  } else {
    lastPump = currentPump;
    return (currentPump == 0) ? 1 : 0;
  }
}

void pumpLogic() {
  if (!run) {
    if (bitRead(pumpAuto, 0) || bitRead(pumpAuto, 1)) {
      disChoice = 1;
      bitClear(setRelay, 0);
      bitClear(setRelay, 1);
      bitClear(setRelay, 6);
      bitClear(setRelay, 7);
      running = 0;
    } else {
      disChoice = 0;
    }
  } else if (run) {
    //Pump1
    if (bitRead(pumpAuto, 0) && !activePump && !bitRead(pumpHand, 0)) {
      if (run && !activePump) {
        bitSet(setRelay, 0);
        bitSet(setRelay, 7);
        disChoice = 2;
        running = 1;
      }
    } else {
      bitClear(setRelay, 0);
      bitClear(setRelay, 7);
    }

    //Pump2
    if (bitRead(pumpAuto, 1) && activePump && !bitRead(pumpHand, 1)) {
      if (run && activePump) {
        bitSet(setRelay, 1);
        bitSet(setRelay, 6);
        disChoice = 3;
        running = 2;
      }
    } else {
      bitClear(setRelay, 1);
      bitClear(setRelay, 6);
    }
  }
  if (bitRead(pumpHand, 0)) {
    bitSet(setRelay, 0);
    disChoice = 4;
    running = 1;
  }
  if (bitRead(pumpHand, 1)) {
    bitSet(setRelay, 1);
    disChoice = 5;
    running = 2;
  }
}

void setDisplay(int intChoice) {
  const int maxStatusMessages = 7;
  const int statusMessages[][4] = {
    { 28, 27, 27, 28 },  // " -- "
    { 10, 26, 25, 22 },  //"AUto"
    { 23, 27, 1, 10 },   // "P-1A"
    { 23, 27, 2, 10 },   // "P-2A"
    { 23, 27, 1, 17 },   // "P-1H"
    { 23, 27, 2, 17 },   // "P-2H"
    { 15, 24, 24, 28 }   // "Err "
  };
  setDisplayBits(statusMessages[intChoice]);
}

void setDisplayBits(const int displayData[4]) {
  for (int i = 0; i < 4; i++) {
    dat_buf[2 * i] = displayData[i];
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(OE_595_PIN, OUTPUT);
  pinMode(RS485_RD, OUTPUT);

  for (int i = 0; i < 4; i++) {
    pinMode(BUTTON_PINS[i], INPUT);
    digitalWrite(BUTTON_PINS[i], HIGH);
  }

  pinMode(LOAD_165_PIN, OUTPUT);
  pinMode(DATA_165_PIN, INPUT);
  pinMode(CLK_165_PIN, OUTPUT);

  digitalWrite(OE_595_PIN, LOW);
  Serial.println("System Ready");
  Serial.println("Pump Controller V1.3");
  Serial.println();
}

void loop() {

  dataFrom165 = readByteFrom165();
  inputLogic(dataFrom165);
  pumpLogic();
  setDisplay(disChoice);
  updateDisplayAndRelay();
}
