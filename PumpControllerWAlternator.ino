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
uint8_t loopVar = 0;      // Initialize to 0
uint8_t activePump = 0; // Initialize to 0
uint8_t lagPump = 0; // Initialize to 0
uint8_t lagRun = 0; // Initialize to 0
uint8_t lastPump = 0;   // Initialize to 0
uint8_t run = 0;        // Initialize to 0
uint8_t disChoice = 0;  // Initialize to 0

// Timers for logic
ulong lastRunTime = 0; // Initialize to 0
ulong lastLagTime = 0; // Initialize to 0

// Arrays for display Message Handling
#define MAX_MSGS 10                 // Adjust as needed
#define MSG_SIZE 5                  // Number of segments per message
int messageQty = 0;
uint8_t activeMessages[MAX_MSGS][4]; // Initialize all elements to 0, adjust the size as needed
uint8_t Dot = 0x80;              // Dot for display
uint8_t curDisplay[5];

unsigned long lastDisplayChangeTime = 0; // Stores the time when the display was last changed
const int displayChangeInterval = 750;   // The interval (in ms) at which the display should change
int currentMessageIndex = 0;

/**
 * @brief Array of status messages.
 * 
 * This array stores status messages as a 2-dimensional array of uint8_t values.
 * Each status message is represented by a row in the array, where each element
 * in the row represents a character code. The last element in each row is the
 * position of the dot in the message (1-4) or 0 if no dot is present.
 * 
 * The structure of each row is as follows:
 * - Element 0: First character code of the message
 * - Element 1: Second character code of the message
 * - Element 2: Third character code of the message
 * - Element 3: Fourth character code of the message
 * - Element 4: Dot position (1-4) or 0 if no dot is present
 *
 * Set tube segment hex characters
 *        NO.:0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28
 * Character :0,1,2,3,4,5,6,7,8,9,A ,b ,C ,c ,d ,E ,F ,H ,h ,L ,n ,N ,o ,P ,r ,t ,U ,- ,  ,
 * */
const uint8_t statusMessages[][5] = {
    {28, 27,27, 28, 0}, // " -- " 0
    {10, 26,25, 22, 0}, //"AUto"  1
    {23, 27, 1, 10, 0},  // "P-1A" 2
    {23, 27, 2, 10, 0},  // "P-2A" 3
    {23, 27, 3, 10, 0},  // "P-3A" 4
    {23, 27, 1, 17, 0},  // "P-1H" 5
    {23, 27, 2, 17, 0},  // "P-2H" 6
    {23, 27, 3, 17, 0},  // "P-3H" 7
    {26, 28, 2, 6, 3}    // "V 2.6" 8
};

uint8_t  SEG8Code[] = 
{0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x58,0x5e,0x79,0x71,0x76,0x74,0x38,0x54,0x37,0x5c,0x73,0x50,0x78,0x3e,0x40,0x00};//Common anode Digital Tube Character Gallery
uint8_t BitsSelection[4] = {0xFE, 0xFD, 0xFB, 0xF7};
uint8_t BitsSele = 0;

// Read 8 button inputs
byte readByteFrom165()
{
  uchar byte_temp = 0;
  uchar byte_;

  // Write pulse to load pin
  digitalWrite(LOAD_165_PIN, LOW);
  digitalWrite(LOAD_165_PIN, HIGH);

  // Get data from 74HC165
  for (int i = 7; i >= 0; i--)
  {
    byte_ = digitalRead(DATA_165_PIN);
    byte_temp |= (byte_ << i);
    digitalWrite(CLK_165_PIN, HIGH);
    digitalWrite(CLK_165_PIN, LOW);
  }

  return ~byte_temp;
}

/**************************************************************************
*                                                                         *
*                        Messaging System Functions                       *
*                                                                         *
***************************************************************************
*     Adds a new message to the activeMessages array.                     *
*                                                                         *
* @param option The index of the statusMessages array to retrieve         *
*               the message from.                                         *
**************************************************************************/
void addMessage(int option)
{
  // Check if there is space in the activeMessages array
  if (messageQty < MAX_MSGS)
  {
    // Add the new message to activeMessages
    for (int i = 0; i < 5; i++)
    {
      activeMessages[messageQty][i] = statusMessages[option][i];
    }
    messageQty++;
  }
}

/**
 * Removes a message from the activeMessages array.
 * 
 * @param option The index of the statusMessages array to retrieve the message from.
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
        for (int k = 0; k < 5; k++)
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
bool compareArrays(const uint8_t array1[4], const uint8_t array2[4])
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
 * Function to rotate through a list of messages and update the Active display with the new message.
 * The rotation interval is determined by the displayChangeInterval variable.
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
    for (int i = 0; i < 5; i++)
    {
      curDisplay[i] = activeMessages[currentMessageIndex][i];
    }
    // Update the last display change time
    lastDisplayChangeTime = currentTime;
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
 * @brief Deletes a message from the message list and clears the corresponding bit in msg1to16.
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
 * This function deletes the auto messages with IDs 1, 2, 3, and 4.
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
 * This function deletes messages with IDs 5, 6, and 7.
 */
void clearHand()
{
  delMessage(5);
  delMessage(6);
  delMessage(7);
}

/**************************************************************************
*                     END MESSAGING SYSTEM FUNCTIONS                      *
***************************************************************************
*                                                                         *
*                     Begin Output System Functions                       *
*                                                                         *
*****  *****  *****  *****  *****  *****  *****  *****  *****  *****  *****
* Sends data to a 74HC595 shift register.                                 *
*                                                                         *
* @param Num The number to be sent to the shift register.                 *
* @param Seg The segment to be sent to the shift register.                *
* @param out The output value to be sent to the shift register.           *
**************************************************************************/
void Send_74HC595(uint8_t Num, uint8_t Seg, uint8_t out)
{
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, out);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, Seg);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, Num);
    digitalWrite(LATCH_PIN, LOW);
    digitalWrite(LATCH_PIN, HIGH);
}

void outputSend(void)
{
  uint8_t tempPx[4] = {0};
  for (int i = 0; i < 4; i++)
  {
    if (curDisplay[4] == i + 1)
    {
      tempPx[i] = SEG8Code[curDisplay[i]] | Dot;
    }
    else
    {
      tempPx[i] = SEG8Code[curDisplay[i]];
    }
  }
  Send_74HC595(tempPx[BitsSele], BitsSelection[BitsSele], setRelay);
  BitsSele = (BitsSele + 1) % 4;
}

/***************************************************************************
*                      END OUTPUT SYSTEM FUNCTIONS                         *
****************************************************************************
*                                                                          *
*                     Begin Logic System Functions                         *
*                                                                          *
***************************************************************************/

/**
 * Checks if a specified delay has elapsed since the last time the function was called.
 *
 * @param delay The delay in milliseconds.
 * @return Returns 1 if the specified delay has elapsed, otherwise returns 0.
 */
int checkRunTimeElapsed(int delay, uint8_t trigger)
{
  // Get the current time
  ulong currentTime = millis();
  // Check if 0.25 seconds have passed since the last time the function was called
  switch (trigger)
  {
  case 1:
    if (currentTime - lastRunTime >= delay)
    {
      // Update the last time the function was called
      // Set PS Last Active Time
      lastRunTime = currentTime;
      return 1;
    }
    else
    {
      return 0;
    }
    break;
  case 2:
    if (currentTime - lastLagTime >= delay)
    {
      // Update the last time the function was called
      // Set LagPS Last Active Time
      lastLagTime = currentTime;
      return 1;
    }
    else
    {
      return 0;
    }
    break;
  }
}

/**
 * Sets the active pump based on the pumpAuto flags.
 *
 * @param currentPump The current active pump.
 * @return The new active pump.
 */
void setActivePump()
{
  bool pump1 = bitRead(pumpAuto, 0);
  bool pump2 = bitRead(pumpAuto, 1);
  bool pump3 = bitRead(pumpAuto, 2);
  if (pump1 == 1 && pump2 == 0 && pump3 == 0)
  {
    lastPump = activePump;
    activePump = 0;
    lagPump = 1;
    return;
  }
  if (pump1 == 0 && pump2 == 1 && pump3 == 0)
  {
    lastPump = activePump;
    activePump = 1;
    lagPump = 2;
    return;
  }
  if (pump1 == 0 && pump2 == 0 && pump3 == 1)
  {
    lastPump = activePump;
    activePump = 2;
    lagPump = 3;
    return;
  }
  if (pump1 == 1 && pump2 == 1 && pump3 == 0)
  {
    lastPump = activePump;
    if (lastPump == 0)
    {
      activePump = 1;
      lagPump = 1;
      return;
    }
    else
    {
      activePump = 0;
      lagPump = 2;
      return;
    }
    return;
  }
  if (pump1 == 0 && pump2 == 1 && pump3 == 1)
  {
    lastPump = activePump;
    if (lastPump == 2)
    {
      activePump = 1;
      lagPump = 3;
      return;
    }
    else
    {
      activePump = 2;
      lagPump = 2;
      return;
    }
  }
  if (pump1 == 1 && pump2 == 0 && pump3 == 1)
  {
    lastPump = activePump;
    if (lastPump == 2)
    {
      activePump = 0;
      lagPump = 3;
      return;
    }
    else
    {
      activePump = 2;
      lagPump = 1;
      return;
    }
  }
  if (pump1 == 1 && pump2 == 1 && pump3 == 1)
  {
    switch (activePump)
    {
    case 0:
      activePump = 1;
      lagPump = 3;
      return;
    case 1:
      activePump = 2;
      lagPump = 1;
      return;
    case 2:
      activePump = 0;
      lagPump = 2;
      return;
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
  uint8_t ctime;
  uint8_t ctime2;
  for (int i = 0; i < 8; ++i)
  {
    bool isBitSet = bitRead(inputs, i);

    if (isBitSet)
    {
      handleBitSet(i, ctime, ctime2);
    }
    else
    {
      handleBitClear(i, ctime);
    }
  }
}

/**
 * @brief Handles the bit set operation based on the given bit position.
 * 
 * This function sets the corresponding bits in the pumpAuto and pumpHand variables
 * based on the provided bit position. It also checks the run time elapsed and sets
 * the lagRun and run variables accordingly.
 * 
 * @param bitPosition The position of the bit to be set.
 * @param ctime Reference to the ctime variable.
 * @param ctime2 Reference to the ctime2 variable.
 */
void handleBitSet(int bitPosition, uint8_t& ctime,uint8_t& ctime2)
{
  switch (bitPosition)
  {
    case 0:
      // P1A - Set pump1 Auto
      bitSet(pumpAuto, 0);
      break;
    case 1:
      // P1H - Set pump1 Hand
      bitSet(pumpHand, 0);
      break;
    case 2:
      // P2A - Set pump2 Auto
      bitSet(pumpAuto, 1);
      break;
    case 3:
      // P2H - Set pump2 Hand
      bitSet(pumpHand, 1);
      break;
    case 4:
      // P3A - Set pump3 Auto
      bitSet(pumpAuto, 2);
      break;
    case 5:
      // P3H - Set pump3 Hand
      bitSet(pumpHand, 2);
      break;
    case 6:
      // Set LagPressureSwitch run request
      ctime2 = checkRunTimeElapsed(250,2);
      if (run)
      {
        if (ctime2)
        {
          lagRun = 1;
        }
      }
      else
      {
        lagRun = 0;
      }
      break;
    case 7:
      // Set PressureSwitch run request
      ctime = checkRunTimeElapsed(300,1);
      if (bitRead(pumpAuto, 0) || bitRead(pumpAuto, 1) || bitRead(pumpAuto, 2))
        {
          if (!run && ctime)
          {
            setActivePump();
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

/**
 * @brief Clears the specified bit position in the given variable and performs additional actions based on the bit position.
 * 
 * @param bitPosition The position of the bit to be cleared.
 * @param ctime The reference to the variable to be modified.
 */
void handleBitClear(int bitPosition, uint8_t& ctime)
{
  switch (bitPosition)
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
    case 6:
      // Clear LagPressureSwitch run request
      lagRun = 0;
      break;
    case 7:
      // Clear PressureSwitch run request
      ctime = checkRunTimeElapsed(150,1);
      run = run && ctime;
      break;
    default:
      break;
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
 * @brief Performs logic for controlling the hand pumps.
 * 
 * This function iterates through the hand pumps and updates the relay and message bits based on the state of each pump.
 * If a pump is active, the corresponding relay bits are set and the message bit is set.
 * If a pump is inactive, the corresponding relay bits are cleared and the message bit is cleared.
 */
void handLogic()
{
  for (int pump = 0; pump < 3; pump++)
  {
    int relayBit = pump;
    int messageBit = pump + 5;

    if (bitRead(pumpHand, pump))
    {
      setRelay |= 1 << relayBit;
      setRelay |= 1 << (7 - relayBit);
      setMessage(messageBit);
    }
    else
    {
      setRelay &= ~(1 << relayBit);
      setRelay &= ~(1 << (7 - relayBit));
      delMessage(messageBit);
    }
  }
}

/**
 * Executes the logic for automatically controlling the pumps based on the current state and settings.
 * This function checks the active pump, run status, and lag pump status to determine which pumps to activate or deactivate.
 * It also updates the relay settings and messages accordingly.
 */
void autoLogic()
{
  uint8_t ap = activePump;
  if (!run && handActive() == 0)
  {
    setMessage(1);
    setRelay = 0b00000000;
  }

  // Pump1
  if (bitRead(pumpAuto, 0) && ((ap == 0 && run) || (lagPump == 1 && lagRun)))
  {
    setRelay |= 1 << 0;
    setRelay |= 1 << 7;
    setMessage(2);
    setMessage(1);
  }
  else 
  {
    setRelay &= ~(1 << 0);
    setRelay &= ~(1 << 7);
    delMessage(2);
  }

  // Pump2
  if (bitRead(pumpAuto, 1) && ((ap == 1 && run) || (lagPump == 2 && lagRun == 1)))
  {
    setRelay |= 1 << 1;
    setRelay |= 1 << 6;
    setMessage(3);
    setMessage(1);
  }
  else
  {
    setRelay &= ~(1 << 1);
    setRelay &= ~(1 << 6);
    delMessage(3);
  }

  // Pump3
  if (bitRead(pumpAuto, 2) && ((ap == 2 && run) || (lagPump == 3 && lagRun == 1)))
  {
    setRelay |= 1 << 2;
    setRelay |= 1 << 5;
    setMessage(4);
    setMessage(1);
  }
  else
  {
    setRelay &= ~(1 << 2);
    setRelay &= ~(1 << 5);
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
  //Set relay for chemical pumps or accessory pumps
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

/**************************************************************************
*                       END LOGIC SYSTEM FUNCTIONS                        *
**************************************************************************/

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
  setActivePump();
  Serial.println("System Ready");
  Serial.println("PumpControllerV2.6");
  Serial.println("WellWorksLLC-DMC");
  int start = 1;
  int count = 0;
  setMessage(8);
  while (start == 1)
  {
    rotateMessages();
    outputSend();
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
  inputLogic(readByteFrom165());
  pumpLogic();
  rotateMessages();
  if (loopVar == 15)
  {
    outputSend();
    loopVar = 0;
  }
  loopVar++;
}