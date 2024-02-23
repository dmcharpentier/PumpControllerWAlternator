#include <SD.h>
#include <inttypes.h>
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
byte pumpAuto = 0b0000;                 // Represents pump auto settings (4 pumps)
byte pumpHand = 0b0000;                 // Represents pump manual settings (4 pumps)
uint16_t msg1to16 = 0b0000000000000000; // Initialize to 0

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
int currentMessage[4] = {};      // Initialize all elements to 0
uint8_t Dot = 0x80;              // Dot for display

unsigned long lastDisplayChangeTime = 0; // Stores the time when the display was last changed
const int displayChangeInterval = 750;   // The interval (in ms) at which the display should change
int currentMessageIndex = 0;

// Set tube segment hex characters
/*       NO.:0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28
/*Character :0,1,2,3,4,5,6,7,8,9,A ,b ,C ,c ,d ,E ,F ,H ,h ,L ,n ,N ,o ,P ,r ,t ,U ,- ,  ,*/
const int statusMessages[][4] = {
    {28, 27, 27, 28}, // " -- " 0
    {10, 26, 25, 22}, //"AUto"  1
    {23, 27, 1, 10},  // "P-1A" 2
    {23, 27, 2, 10},  // "P-2A" 3
    {23, 27, 3, 10},  // "P-3A" 4
    {23, 27, 1, 17},  // "P-1H" 5
    {23, 27, 2, 17},  // "P-2H" 6
    {23, 27, 3, 17},  // "P-3H" 7
    {26, 2, 27, 3}    // V-23 8
};

uchar tubeSegments[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x58, 0x5e, 0x79, 0x71, 0x76, 0x74, 0x38, 0x54, 0x37, 0x5c, 0x73, 0x50, 0x78, 0x3e, 0x40, 0x00};
uchar tubeNumbers[] = {0xfe, 0xff, 0xfd, 0xff, 0xfb, 0xff, 0xf7, 0xff};

// Read 8 button inputs

uint8_t Read_Inputs(void)
{
  uint8_t i;
  uint8_t Temp = 0;
  // Write pulse to load pin
  digitalWrite(LOAD_165_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(LOAD_165_PIN, HIGH);
  for (i = 0; i < 8; i++)
  {
    Temp <<= 1;
    digitalWrite(CLK_165_PIN, HIGH);
    Temp |= digitalRead(DATA_165_PIN);
    digitalWrite(CLK_165_PIN, LOW);
  }
  return Temp;
}

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

void ForceSetDisplay(int option)
{
  dat_buf[0] = activeMessages[option][0];
  dat_buf[2] = activeMessages[option][1];
  dat_buf[4] = activeMessages[option][2];
  dat_buf[6] = activeMessages[option][3];
  for (int comNum = 0; comNum < 8; comNum++)
  {
    displayData = dat_buf[comNum];
    uchar tubeData = tubeSegments[displayData];
    uchar bitNum = tubeNumbers[comNum];

    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, setRelay);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, bitNum);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, tubeData);
    digitalWrite(LATCH_PIN, HIGH);
  }
}

/**
 * Updates the relay by shifting out the relay state, bit number, and tube data.
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
int setActivePump()
{
  if (bitRead(pumpAuto, 0) == 1 && bitRead(pumpAuto, 1) == 0 && bitRead(pumpAuto, 2) == 0)
  {
    lastPump = activePump;
    return 0;
  }
  else if (bitRead(pumpAuto, 0) == 0 && bitRead(pumpAuto, 1) == 1 && bitRead(pumpAuto, 2) == 0)
  {
    lastPump = activePump;
    return 1;
  }
  else if (bitRead(pumpAuto, 0) == 0 && bitRead(pumpAuto, 1) == 0 && bitRead(pumpAuto, 2) == 1)
  {
    lastPump = activePump;
    return 2;
  }
  else if (bitRead(pumpAuto, 0) == 1 && bitRead(pumpAuto, 1) == 1 && bitRead(pumpAuto, 2) == 0)
  {
    lastPump = activePump;
    return activePump = 1 - activePump;
  }
  else if (bitRead(pumpAuto, 0) == 0 && bitRead(pumpAuto, 1) == 1 && bitRead(pumpAuto, 2) == 1)
  {
    lastPump = activePump;
    if (lastPump == 2)
    {
      return 1;
    }
    else
    {
      return 2;
    }
  }
  else if (bitRead(pumpAuto, 0) == 1 && bitRead(pumpAuto, 1) == 0 && bitRead(pumpAuto, 2) == 1)
  {
    lastPump = activePump;
    if (lastPump == 2)
    {
      return 0;
    }
    else
    {
      return 2;
    }
  }
  else if (bitRead(pumpAuto, 0) == 1 && bitRead(pumpAuto, 1) == 1 && bitRead(pumpAuto, 2) == 1)
  {
    switch (activePump)
    {
    case 0:
      return 1;
    case 1:
      return 2;
    case 2:
      return 0;
    }
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
        // Set PressureSwitch run request
        ctime = checkRunTimeElapsed(150);
        if (bitRead(pumpAuto, 0) || bitRead(pumpAuto, 1) || bitRead(pumpAuto, 2))
        {
          if (!run && ctime)
          {
            activePump = setActivePump();
            run = 1;
          }
        }
        else
        {
          run = 0;
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
        // P1A - Clear pump1 Auto
        bitClear(pumpAuto, 0);
        break;
      case 1:
        // P1H - Clear pump1 Hand
        bitClear(pumpHand, 0);
        break;
      case 2:
        // P2A - Clear pump2 Auto
        bitClear(pumpAuto, 1);
        break;
      case 3:
        // P2H - Clear pump2 Hand
        bitClear(pumpHand, 1);
        break;
      case 4:
        // P3A - Clear pump3 Auto
        bitClear(pumpAuto, 2);
        break;
      case 5:
        // P3H - Clear pump3 Hand
        bitClear(pumpHand, 2);
        break;
      case 7:
        // Clear PressureSwitch run request
        ctime = checkRunTimeElapsed(150);
        if (run && ctime)
        {
          run = 0;
        }
        break;
      default:

        break;
      }
    }
  }
}

/**
 * Checks if a pump is in hand mode.
 *
 * @return 1 if the hand mode is active on any pump, 0 otherwise.
 */
int handActive()
{
  if (bitRead(pumpHand, 0) || bitRead(pumpHand, 1) || bitRead(pumpHand, 2))
  {
    delMessage(0);
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * Checks if the automatic pump control is active.
 * @return 1 if the automatic pump control is active, 0 otherwise.
 */
int autoActive()
{
  if (bitRead(pumpAuto, 0) || bitRead(pumpAuto, 1) || bitRead(pumpAuto, 2))
  {
    delMessage(0);
    return 1;
  }
  else
  {
    return 0;
  }
}

/**
 * Sets the message for the given index.
 * If the message at the given index is not already set, it adds the message and sets the corresponding bit.
 *
 * @param msg The index of the message to be set.
 */
void setMessage(int msg)
{
  if (!bitRead(msg1to16, msg))
  {
    addMessage(msg);
    bitSet(msg1to16, msg);
  }
}

/**
 * @brief Deletes a message from the message list and clears the corresponding bit in msg1to8.
 *
 * @param msg The message to be deleted.
 */
void delMessage(int msg)
{
  if (bitRead(msg1to16, msg))
  {
    removeMessage(msg);
    bitClear(msg1to16, msg);
  }
}

/**
 * @brief Clears the auto messages.
 *
 * This function deletes the auto messages with IDs 1, 2, 4, and 6.
 */
void clearAuto()
{
  delMessage(1);
  delMessage(2);
  delMessage(3);
  delMessage(4);
}

/**
 * @brief Clears the hand by deleting specific messages.
 *
 * This function deletes messages with IDs 3, 5, and 7.
 */
void clearHand()
{
  delMessage(5);
  delMessage(6);
  delMessage(7);
}

/**
 * @brief This function handles the logic for controlling the pump based on the hand input.
 *
 * It checks the status of the pumpHand variable and sets or clears the corresponding relay and message.
 * If the pumpHand bit 0 is set, it sets the relay 0, displays message 3, and sets the running variable to 1.
 * If the pumpHand bit 0 is clear, it clears the relay 0 and deletes message 3.
 * If the pumpHand bit 1 is set, it sets the relay 1, displays message 5, and sets the running variable to 2.
 * If the pumpHand bit 1 is clear, it clears the relay 1 and deletes message 5.
 */
void handLogic()
{
  // Hand Pump1
  if (bitRead(pumpHand, 0))
  {
    bitSet(setRelay, 0);
    bitSet(setRelay, 7);
    setMessage(5);
    running = 1;
  }
  else
  {
    bitClear(setRelay, 0);
    bitClear(setRelay, 7);
    delMessage(5);
  }

  // Hand Pump2
  if (bitRead(pumpHand, 1))
  {
    bitSet(setRelay, 1);
    bitSet(setRelay, 3);
    setMessage(6);
    running = 2;
  }
  else
  {
    bitClear(setRelay, 1);
    bitClear(setRelay, 3);
    delMessage(6);
  }

  // Hand Pump3
  if (bitRead(pumpHand, 2))
  {
    bitSet(setRelay, 2);
    bitSet(setRelay, 5);
    setMessage(7);
    running = 2;
  }
  else
  {
    bitClear(setRelay, 2);
    bitClear(setRelay, 5);
    delMessage(7);
  }
}

/**
 * Performs the automatic logic for controlling the pumps based on the run and activePump variables.
 * If run is false, it sets the relay and running variables accordingly.
 * If run is true, it checks the pumpAuto and activePump variables to determine which pumps to activate.
 * It sets the relay, displays messages, and updates the running variable accordingly.
 */
void autoLogic()
{
  if (!run && handActive() == 0)
  {
    setMessage(1);
    setRelay = 0b00000000;
    running = 0;
  }

  // Pump1
  if (bitRead(pumpAuto, 0) && activePump == 0 && run)
  {
    bitSet(setRelay, 0);
    bitSet(setRelay, 7);
    setMessage(2);
    setMessage(1);
    running = 1;
  }
  else
  {
    bitClear(setRelay, 0);
    bitClear(setRelay, 7);
    delMessage(2);
  }

  // Pump2
  if (bitRead(pumpAuto, 1) && activePump == 1 && run)
  {
    bitSet(setRelay, 1);
    bitSet(setRelay, 6);
    setMessage(3);
    setMessage(1);
    running = 2;
  }
  else
  {
    bitClear(setRelay, 1);
    bitClear(setRelay, 6);
    delMessage(3);
  }

  // Pump3
  if (bitRead(pumpAuto, 2) && activePump == 2 && run)
  {
    bitSet(setRelay, 2);
    bitSet(setRelay, 5);
    setMessage(4);
    setMessage(1);
    running = 3;
  }
  else
  {
    bitClear(setRelay, 2);
    bitClear(setRelay, 5);
    delMessage(4);
  }
}

/**
 * Executes the logic for controlling the pumps based on the active mode (hand or auto).
 * If hand mode is active, it calls the handLogic function.
 * If auto mode is active and hand mode is inactive, it controls the pumps based on the pumpAuto settings.
 * @note This function assumes that the variables handActive, pumpAuto, activePump, run, setRelay, running are properly initialized and defined.
 */
void pumpLogic()
{
  //
  if (handActive() == 1 || (autoActive() == 1 && run == 1))
  {
    bitSet(setRelay, 4);
  }
  else
  {
    bitClear(setRelay, 4);
  }

  // handLogic
  if (handActive() == 1)
  {
    clearAuto();
    handLogic();
  }
  else
  {
    clearHand();
    handLogic();
  }

  // autoLogic
  if (autoActive() == 1 && handActive() == 0)
  {
    delMessage(0);
    autoLogic();
  }
  else
  {
    clearAuto();
  }

  // set default display
  if (autoActive() == 0 && handActive() == 0)
  {
    delMessage(1);
    setMessage(0);
  }
  else
  {
    delMessage(0);
  }
}

/**
 * Adds a new message to the activeMessages array.
 *
 * @param option The index of the status message to be added.
 */
void addMessage(int option)
{
  // Check if there is space in the activeMessages array
  if (messageQty < maxMsg)
  {
    // Add the new message to activeMessages
    for (int i = 0; i < 4; i++)
    {
      activeMessages[messageQty][i] = statusMessages[option][i];
    }
    messageQty++;
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
  for (int i = 0; i < messageQty; i++)
  {
    if (compareArrays(activeMessages[i], statusMessages[option]))
    {
      // If the current message matches the targetData, remove it
      for (int j = i; j < messageQty - 1; j++)
      {
        for (int k = 0; k < 4; k++)
        {
          activeMessages[j][k] = activeMessages[j + 1][k];
        }
      }
      messageQty--;
      break; // Break out of the loop after removing the first matching message
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
    // Increment the message index
    currentMessageIndex = (currentMessageIndex + 1) % messageQty;
    // Set the display with the new message
    for (int i = 0; i < 4; i++)
    {
      dat_buf[2 * i] = activeMessages[currentMessageIndex][i];
    }
    // Update the last display change time
    lastDisplayChangeTime = currentTime;
  }
}

void setup()
{
  Serial.begin(115200);

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
  activePump = setActivePump();
  Serial.println("System Ready");
  Serial.println("PumpControllerV2.3");
  Serial.println("WellWorksLLC-DMC");
  int start = 1;
  int count = 0;
  setMessage(8);
  while (start == 1)
  {
    rotateMessages();
    updateDisplayAndRelay();
    if (count == 10000)
    {
      delMessage(8);
      start = 0;
    }
    else
    {
      count++;
    }
  }
}

void loop()
{
  dataFrom165 = readByteFrom165();
  inputLogic(dataFrom165);
  pumpLogic();
  rotateMessages();
  updateDisplayAndRelay();
}