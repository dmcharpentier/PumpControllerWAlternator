#include <SD.h>
// #include "MessageHandler.h"
//  #include <SoftwareSerial.h>

#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long

// Pin Definitions
//********************************************************
// Pins to read 8 button inputs
#define LOAD_165_PIN 4
#define DATA_165_PIN 2
#define CLK_165_PIN 3

// Relay and 4 Digit 7 Segment Display Pins
#define LATCH_PIN 11
#define CLOCK_PIN 12
#define DATA_PIN 9
#define OE_595_PIN 10
#define RS485_RD 13

// Set pins for onboard buttons
const int BUTTON_PINS[] = {5, 6, 7, 8};

// Variables for program operation
uchar dat_buf[8] = {0};   // Initialize all elements to 0
byte setRelay = 0b0000;   // Represents relay states (4 relays)
uchar setRemoteRelay = 0; // Initialize to 0
uchar displayData = 0;    // Initialize to 0
uchar comNum = 0;         // Initialize to 0

// Raw input from buttons
byte dataFrom165;

// Pump and Hand button logic
byte pumpAuto = 0b0000;    // Represents pump auto settings (4 pumps)
byte pumpHand = 0b0000;    // Represents pump manual settings (4 pumps)
byte msg1to8 = 0b00000000; // Initialize to 0

// Pump Logic
int activePump = 0; // Initialize to 0
int lastPump = 0;   // Initialize to 0
int run = 0;        // Initialize to 0
int disChoice = 0;  // Initialize to 0
int running = 0;    // Initialize to 0

// Timers for logic
ulong lastRunTime = 0; // Initialize to 0

// Arrays for display Message Handling
int messageQty = 0;
int maxMsg = 8;
int activeMessages[10][4] = {0}; // Initialize all elements to 0, adjust the size as needed
int currentMessage[4] = {0};     // Initialize all elements to 0

unsigned long lastDisplayChangeTime = 0;
const int displayChangeInterval = 1000;
int currentMessageIndex = 0;

// Set tube segment hex characters
/*       NO.:0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28
/*Character :0,1,2,3,4,5,6,7,8,9,A ,b ,C ,c ,d ,E ,F ,H ,h ,L ,n ,N ,o ,P ,r ,t ,U ,- ,  ,*/
const int statusMessages[][4] = {
    {28, 27, 27, 28}, // " -- " 0
    {10, 26, 25, 22}, //"AUto"  1
    {23, 27, 1, 10},  // "P-1A" 2
    {23, 27, 1, 17},  // "P-1H" 3
    {23, 27, 2, 10},  // "P-2A" 4
    {23, 27, 2, 17},  // "P-2H" 5
    {23, 27, 3, 10},  // "P-3A" 6
    {23, 27, 3, 17},  // "P-3H" 7
    {15, 24, 24, 28}  // "Err " 8
};

uchar tubeSegments[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x58, 0x5e, 0x79, 0x71, 0x76, 0x74, 0x38, 0x54, 0x37, 0x5c, 0x73, 0x50, 0x78, 0x3e, 0x40, 0x00};
uchar tubeNumbers[] = {0xfe, 0xff, 0xfd, 0xff, 0xfb, 0xff, 0xf7, 0xff};

// Read 8 button inputs
byte readByteFrom165()
{
  uchar byte_temp = 0;
  uchar byte_;

  // Write pulse to load pin
  digitalWrite(LOAD_165_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(LOAD_165_PIN, HIGH);

  // Get data from 74HC165
  for (int i = 7; i >= 0; i--)
  {
    byte_ = digitalRead(DATA_165_PIN);
    byte_temp |= (byte_ << i);
    digitalWrite(CLK_165_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLK_165_PIN, LOW);
  }

  return ~byte_temp;
}

/**
 * @brief Updates the display and relay based on the current communication number.
 *
 * This function increments the communication number and updates the display data
 * based on the new communication number. It then calls the bitRelayUpdate() function
 * to update the relay.
 */
void updateDisplayAndRelay()
{
  comNum = (comNum < 7) ? comNum + 1 : 0;
  displayData = dat_buf[comNum];
  bitRelayUpdate();
}

/**
 * Updates the bit relay by shifting out the relay state, bit number, and tube data.
 * This function is responsible for controlling the relay operation.
 */
void bitRelayUpdate()
{
  uchar tubeData = tubeSegments[displayData];
  uchar bitNum = tubeNumbers[comNum];

  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, setRelay);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bitNum);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, tubeData);
  digitalWrite(LATCH_PIN, HIGH);
}

/**
 * Checks if a specified delay has elapsed since the last time the function was called.
 *
 * @param delay The delay in milliseconds.
 * @return Returns 1 if the specified delay has elapsed, otherwise returns 0.
 */
int checkRunTimeElapsed(int delay)
{
  // Get the current time
  ulong currentTime = millis();

  // Check if 0.25 seconds have passed since the last time the function was called
  if (currentTime - lastRunTime >= delay)
  {
    // Update the last time the function was called
    lastRunTime = currentTime;
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * Sets the active pump based on the pumpAuto flags.
 *
 * @param currentPump The current active pump.
 * @return The new active pump.
 */
int setActivePump(int currentPump)
{
  if (!bitRead(pumpAuto, 0) && !bitRead(pumpAuto, 1))
  {
    // return currentPump;
  }
  else if (bitRead(pumpAuto, 0) && !bitRead(pumpAuto, 1))
  {
    return 0;
  }
  else if (!bitRead(pumpAuto, 0) && bitRead(pumpAuto, 1))
  {
    return 1;
  }
  else
  {
    lastPump = currentPump;
    return (currentPump == 0) ? 1 : 0;
  }
}

/**
 * This function handles the logic for the input signals.
 * It takes a byte parameter 'inputs' which represents the input signals.
 * The function executes commands based on the bit position of the input signals.
 * It sets or clears certain flags and variables accordingly.
 */
void inputLogic(byte inputs)
{
  int ctime;
  // enable when on
  for (int i = 0; i < 8; ++i)
  {
    if (bitRead(inputs, i))
    {
      // Execute commands based on the bit position (i)
      switch (i)
      {
      case 0:
        // P1A - Set pump1 Auto
        bitSet(pumpAuto, 0);
        break;
      case 1:
        // P1H - Set pump1 Hand
        bitSet(pumpHand, 0);
        // extChoice = 4;
        break;
      case 2:
        // P2A - Set pump2 Auto
        bitSet(pumpAuto, 1);
        break;
      case 3:
        // P2H - Set pump2 Hand
        bitSet(pumpHand, 1);
        // extChoice = 5;
        break;
      case 4:
        // P3A - Set pump3 Auto
        bitSet(pumpAuto, 2);
        break;
      case 5:
        // P3H - Set pump3 Hand
        bitSet(pumpHand, 2);
        break;
      case 7:
        ctime = checkRunTimeElapsed(150);
        if (!run && ctime)
        {
          run = 1;
        }
        break;
      default:

        break;
      }
    }
    if (!bitRead(inputs, i))
    {
      // Execute commands based on the bit position (i)
      switch (i)
      {
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
        if (run && ctime)
        {
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

int handActive()
{
  if (bitRead(pumpHand, 0) || bitRead(pumpHand, 1) || bitRead(pumpHand, 2))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

int autoActive()
{
  if (bitRead(pumpAuto, 0) || bitRead(pumpAuto, 1) || bitRead(pumpAuto, 2))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void setMessage(int msg)
{
  if (!bitRead(msg1to8, msg))
  {
    // Serial.println(bitRead(msg1to8, msg));
    addMessage(msg);
    bitSet(msg1to8, msg);
  }
}

void delMessage(int msg)
{
  if (bitRead(msg1to8, msg))
  {
    removeMessage(msg);
    bitClear(msg1to8, msg);
  }
}

void handLogic()
{
  if (bitRead(pumpHand, 0))
  {
    bitSet(setRelay, 0);
    setMessage(3);
    running = 1;
  }
  else
  {
    bitClear(setRelay, 0);
    delMessage(3);
  }

  if (bitRead(pumpHand, 1))
  {
    bitSet(setRelay, 1);
    setMessage(5);
    running = 2;
  }
  else
  {
    bitClear(setRelay, 1);
    delMessage(5);
  }
}

void clearAuto()
{
  delMessage(1);
  delMessage(2);
  delMessage(4);
  delMessage(6);
}

void clearHand()
{
  delMessage(3);
  delMessage(5);
  delMessage(7);
}

void pumpLogic()
{
  if (handActive)
  {
    clearAuto();
    handLogic();
  }
  else
  {
    clearHand();
    handLogic();
  }

  if (autoActive() == 1 && handActive() == 0)
  {
    if (!run)
    {
      // delMessage(0);
      setMessage(1);
      setRelay = 0b00000000;
      running = 0;
    }

    if (run)
    {
      // Pump1
      if (bitRead(pumpAuto, 0) && !activePump)
      {
        if (run && !activePump)
        {
          bitSet(setRelay, 0);
          bitSet(setRelay, 7);
          setMessage(2);
          running = 1;
        }
      }
      else
      {
        bitClear(setRelay, 0);
        bitClear(setRelay, 7);
        delMessage(2);
      }

      // Pump2
      if (bitRead(pumpAuto, 1) && activePump)
      {
        if (run && activePump)
        {
          bitSet(setRelay, 1);
          bitSet(setRelay, 6);
          setMessage(4);
          running = 2;
        }
      }
      else if (!activePump)
      {
        bitClear(setRelay, 1);
        bitClear(setRelay, 6);
        delMessage(4);
      }
    }
  }
  else
  {
    // clearAuto();
    // setMessage(0);
  }
}

/**
 * Sets the display bits for a 4-digit display.
 *
 * @param displayChar An array of integers representing the bits to be displayed on each digit.
 */
void setDisplayBits(const int displayChar[4])
{
  for (int i = 0; i < 4; i++)
  {
    dat_buf[2 * i] = displayChar[i];
  }
}

/**
 * Adds a new message to the activeMessages array.
 *
 * @param option The index of the status message to be added.
 */
void addMessage(int option)
{
  int newData[4];

  // Copy the status message to newData
  for (int i = 0; i < 4; i++)
  {
    newData[i] = statusMessages[option][i];
  }

  // Check if there is space in the activeMessages array
  if (messageQty < maxMsg)
  {
    // Add the new message to activeMessages
    for (int i = 0; i < 4; i++)
    {
      activeMessages[messageQty][i] = newData[i];
    }
    /**
    Serial.print("Adding new message: ");
    Serial.print(option);
    Serial.print(",");
    for (int i = 0; i < 4; i++)
    {
      Serial.print(newData[i]);
    }
    Serial.println();
    **/
    messageQty++;
  }
  else
  {
    // Serial.println("Message queue is full");
  }
}

/**
 * @brief Deletes a message from the statusMessages array based on the given option.
 *
 * This function removes a message from the statusMessages array by comparing it with the targetData.
 * If a matching message is found, it is removed from the activeMessages array.
 *
 * @param option The index of the message to be removed.
 */
void removeMessage(int option)
{
  // Get the message to be removed
  int targetData[4];
  for (int i = 0; i < 4; i++)
  {
    targetData[i] = statusMessages[option][i];
  }
  /**
  Serial.print("Deleting message: ");
  Serial.print(option);
  Serial.print(",");
  for (int i = 0; i < 4; i++)
  {
    Serial.print(targetData[i]);
  }
  Serial.println();
  **/

  for (int i = 0; i < messageQty; i++)
  {
    // Serial.println(compareArrays(activeMessages[i], targetData));
    if (compareArrays(activeMessages[i], targetData))
    {
      // If the current message matches the targetData, remove it
      for (int j = i; j < messageQty - 1; j++)
      {
        for (int k = 0; k < 4; k++)
        {
          activeMessages[j][k] = activeMessages[j + 1][k];
        }
      }
      // Serial.println("Compare found an equal");
      messageQty--;
      break; // Break out of the loop after removing the first matching message
    }
    else
    {
      // Serial.println("Compare found no equal");
    }
  }
}

/**
 * Compares two arrays of integers.
 *
 * @param array1 The first array to compare.
 * @param array2 The second array to compare.
 * @return True if the arrays are identical, false otherwise.
 */
bool compareArrays(const int array1[4], const int array2[4])
{
  for (int i = 0; i < 4; i++)
  {
    if (array1[i] != array2[i])
    {
      return false; // Arrays are different
    }
  }
  return true; // Arrays are identical
}

/**
 * Rotates the messages displayed on the screen at a specified interval.
 * The messages are stored in the activeMessages array and are cycled through
 * using the currentMessageIndex variable. The displayChangeInterval determines
 * how often the messages are rotated.
 */
void rotateMessages()
{
  unsigned long currentTime = millis();

  // Check if the display change interval has passed
  if (currentTime - lastDisplayChangeTime >= displayChangeInterval)
  {
    // Increment the error message index
    currentMessageIndex = (currentMessageIndex + 1) % messageQty;

    // Set the display with the new error message
    for (int i = 0; i < 4; i++)
    {
      // Serial.println(activeMessages[currentMessageIndex][i]);
    }
    setDisplayBits(activeMessages[currentMessageIndex]);

    // Update the last display change time
    lastDisplayChangeTime = currentTime;
  }
}

void setup()
{
  Serial.begin(9600);

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(OE_595_PIN, OUTPUT);
  pinMode(RS485_RD, OUTPUT);

  for (int i = 0; i < 4; i++)
  {
    pinMode(BUTTON_PINS[i], INPUT);
    digitalWrite(BUTTON_PINS[i], HIGH);
  }

  pinMode(LOAD_165_PIN, OUTPUT);
  pinMode(DATA_165_PIN, INPUT);
  pinMode(CLK_165_PIN, OUTPUT);

  digitalWrite(OE_595_PIN, LOW);
  Serial.println("System Ready");
}

void loop()
{
  dataFrom165 = readByteFrom165();
  inputLogic(dataFrom165);
  pumpLogic();
  // setDisplay(disChoice);
  rotateMessages();
  updateDisplayAndRelay();
}