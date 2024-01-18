// Adjusting built-in bit macros to work with 64-bit data types
// Thanks, Ron B! https://forum.dronebotworkshop.com/c-plus-plus/extended-arduino-bit-operator-macros/

#undef bitRead
#undef bitSet
#undef bitClear
#undef bitWrite
#define bitRead(value,  bit) (((value) >> (bit)) & 1ULL)
#define bitSet(value,   bit) ((value) |=  (1ULL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1ULL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value,  bit))

// To do list: LED Circut & timing code, auto flush stuck keys, prune and clean code.

#include "MIDIUSB.h"
#define NUM_GREAT 61
#define NUM_SWELL 61
#define NUM_PISTON 15
#define NUM_PEDALS 31
// For the record, manuals will be refered to with a 'char' - as follows:
// Great = 'g'
// Swell = 's'
// Pedal = 'p'
// Pistons = 'b'
// Test = 't'
//const char manuals [] = "gspb";
const int swellInPin[] = {28, 26, 24, 22};
const int greatInPin[] = {36, 34, 32, 30};
const int pedalInPin[] = {42};
const int pistonInPin[] = {23, 31}; //Add 31 after pull-up resistors are added
const int muxCtrlPin[] = {4, 5, 6, 7}; //Multiplexer control pins A, B, C, and D
const int numOfMuxChannels = 16;  // Number of channels in the multiplexer
//const int numOfBanks = 1;         // Number of Banks(Multiplexers) to poll - Note: All polled via the same control pins
//int currentChannel = 0;           // Variable to store the current channel being read
bool keyPress[16];                    // Boolean to store if a keypress is being registered
uint64_t activeGreat = 0x8000000000000000;
uint64_t standbyGreat = 0x8000000000000000;
uint64_t pastGreat = 0x8000000000000000;
uint64_t activeSwell = 0x8000000000000000;
uint64_t standbySwell = 0x8000000000000000;
uint64_t pastSwell = 0x8000000000000000;
uint32_t activePedal = 0x80000000;
uint32_t standbyPedal = 0x80000000;
uint32_t pastPedal = 0x80000000;
uint64_t activePistons = 0x8000000000000000;
uint64_t standbyPistons = 0x8000000000000000;
uint64_t pastPistons = 0x8000000000000000;
//Trying some debouce stuff here
uint64_t debounceSwell = 0x8000000000000000;
uint64_t debounceGreat = 0x8000000000000000;
uint64_t debouncePedal = 0x8000000000000000;
uint64_t debouncePistons = 0x8000000000000000;
int debounceArray[4][63];

int byteLength;
byte checkActive;
byte checkPast;
const int connectionLED[] = {13, 13};
const int activeLED[] = {73, 72};
bool isManualActive[3];
uint32_t manualActiveTime[3];
bool keyEvent;
uint32_t lastPush;
int refreshInterval = 1000; //Milliseconds

void setup() {
  for (int s = 0; s < 4; s++)  // For 0-3...
  {
    pinMode(muxCtrlPin[s], OUTPUT);
    pinMode(swellInPin[s], INPUT);
    pinMode(greatInPin[s], INPUT);
  }
  for (int s = 0; s < 2; s++) //For 0-1...
  {
    pinMode(pistonInPin[s], INPUT);
    pinMode(activeLED[s], OUTPUT);
    digitalWrite(activeLED[s], HIGH);
  }
  pinMode(13, OUTPUT);

  Serial.begin(115200);
  // Some other setup
}
void loop() {
  readKeys();
  checkForKeyChanges();
//  flushMIDIbuffer();
//  refresh();
  statusController();
}

void readKeys()
{
  int reversed_i;
  const int debounceThreshold = 2;
  for (int i = 0; i < numOfMuxChannels; i++)  // For 0-15...
  {
    setMuxChannel(i);  // Set the multiplexer to the current channel. For Input 0-15...
    reversed_i = map(i, 0, 15, 15, 0);  // Some multiplexers are reversed - this accommodates that.
    for (int m = 1; m < 5; m++) //Each keyboard has 4 multiplexers (plus 1 for pistons, but that's seperate). Pedals will be different
    {
      keyPress[i] = digitalRead(swellInPin[m-1]);
//    ============ SWELL ============
      if (keyPress[i] == LOW)
      {
        if (debounceArray[0][i + (m*16) - 16] < debounceThreshold) 
        {
          debounceArray[0][i + (m*16) - 16]++;
        } 
        else if (debounceArray[0][i + (m*16) - 16] >= debounceThreshold) 
        {
          bitWrite(standbySwell, i + (m * 16) - 16, 1);
          debounceArray[0][i + (m*16) - 16]++;
        }
      } 
      else {
        debounceArray[0][i + (m*16) - 16] = 0; // Reset the counter when the key is released
        bitWrite(standbySwell, i + (m * 16) - 16, 0);  // And flip the active bit for the swell.
      }

//      if (keyPress[i] == LOW && keyPress[i] != bitRead(debounceSwell, i + (m*16) - 16))
//      {
//        bitWrite(standbySwell, i + (m*16) - 16, 1);      //  Swell On
//      }
//      if (keyPress[i] == HIGH && keyPress[i] != bitRead(debounceSwell, i + (m*16) - 16))
//      {
//        bitWrite(standbySwell, i + (m*16) - 16, 0);      //  Swell Off
//      }
//      if (keyPress[i] == LOW)
//      {
//        bitWrite(debounceSwell, i + (m*16) - 16, 1);     //  Debounce Logic. Set the first of two values.
//      }                                                  //  If it's still LOW when we get back to it above, after a cycle,
//                                                         //  then set the actual standby value HIGH
//      if (keyPress[i] == HIGH)
//      {
//        bitWrite(debounceSwell, i + (m*16) - 16, 0);
//      }
//    ============ GREAT ============
      keyPress[reversed_i] = digitalRead(greatInPin[m-1]);
//      if (keyPress[reversed_i] == LOW && keyPress[reversed_i] != bitRead(debounceGreat, reversed_i + (m*16) - 16))
//      {
//        bitWrite(standbyGreat, reversed_i + (m*16) - 16, 1);      //  Great On
//      }
//      if (keyPress[reversed_i] == HIGH && keyPress[reversed_i] != bitRead(debounceGreat, reversed_i + (m*16) - 16))
//      {
//        bitWrite(standbyGreat, reversed_i + (m*16) - 16, 0);      //  Great Off
//      }
//      if (keyPress[reversed_i] == LOW)
//      {
//        bitWrite(debounceGreat, reversed_i + (m*16) - 16, 1);     //  Debounce Logic. Set the first of two values.
//      }                                                  //  If it's still LOW when we get back to it above, after a cycle,
//                                                         //  then set the actual standby value HIGH
//      if (keyPress[reversed_i] == HIGH)
//      {
//        bitWrite(debounceGreat, reversed_i + (m*16) - 16, 0);
//      }
      if (keyPress[reversed_i] == LOW)
      {
        if (debounceArray[1][reversed_i + (m*16) - 16] < debounceThreshold) 
        {
          debounceArray[1][reversed_i + (m*16) - 16]++;
        } 
        else if (debounceArray[1][reversed_i + (m*16) - 16] >= debounceThreshold) 
        {
          bitWrite(standbyGreat, reversed_i + (m * 16) - 16, 1);
          debounceArray[1][reversed_i + (m*16) - 16]++;
        }
      } 
      else {
        debounceArray[1][reversed_i + (m*16) - 16] = 0; // Reset the counter when the key is released
        bitWrite(standbyGreat, reversed_i + (m * 16) - 16, 0);  // And flip the active bit for the swell.
      }
    }
    //PISTONS
    for (int m = 1; m < 3; m++) //Repeat for each set of banks - 4 per manual (pedals will be different?)
    {
      keyPress[i] = digitalRead(pistonInPin[m-1]);
//      if (keyPress[i] == LOW && keyPress[i] != bitRead(debouncePistons, i + (m*16) - 16))
//      {
      if (1 == 1)  {
        bitWrite(standbyPistons, i + (m*16) - 16, 1);      //  Piston On
      }
      if (keyPress[i] == HIGH && keyPress[i] != bitRead(debouncePistons, i + (m*16) - 16))
      {
        bitWrite(standbyPistons, i + (m*16) - 16, 0);      //  Piston Off
      }
      if (keyPress[i] == LOW)
      {
        bitWrite(debouncePistons, i + (m*16) - 16, 1);     //  Debounce Logic. Set the first of two values.
      }                                                        //  If it's still LOW when we get back to it above, after a cycle,
      if (keyPress[i] == HIGH)                                 //  then set the actual standby value HIGH
      {
        bitWrite(debouncePistons, i + (m*16) - 16, 0);
      }
      keyPress[i] == HIGH; //Reset the keyPress for the next use. I could probably get rid of this?
    }
  }
  activeGreat = standbyGreat;
  activeSwell = standbySwell;
  activePedal = standbyPedal;
  activePistons = standbyPistons;
  
}

void setMuxChannel(int channel) {
  digitalWrite(muxCtrlPin[0], bitRead(channel, 0)); // A
  digitalWrite(muxCtrlPin[1], bitRead(channel, 1)); // B
  digitalWrite(muxCtrlPin[2], bitRead(channel, 2)); // C
  digitalWrite(muxCtrlPin[3], bitRead(channel, 3)); // D
}

void checkForKeyChanges()
{
  if (activeSwell != pastSwell)
  {
    byteLength = 62; // 64 bits but accounting for the 0-index and the preset 64th bit
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activeSwell, i);
      checkPast = bitRead(pastSwell, i);
      controller(checkPast, checkActive, i + 24, 's', 0);
//      bitWrite(pastSwell, i, checkActive);
    }
    isManualActive[0] = true;
    manualActiveTime[0] = millis();
    pastSwell = activeSwell;
  }
  
  if (activeGreat != pastGreat)
  {
    byteLength = 62; // 64 bits but accounting for the 0-index
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activeGreat, i);
      checkPast = bitRead(pastGreat, i);
      controller(checkPast, checkActive, i + 24, 'g', 1);
    }
    isManualActive[1] = true;
    manualActiveTime[1] = millis();
    pastGreat = activeGreat;
  }
  
  if (activePedal != pastPedal)
  {
    byteLength = 31; // 32 bits but accounting for the 0-index
    for (byte i = 0; i <= byteLength; i++)
    {
      checkActive = bitRead(activePedal, i);
      checkPast = bitRead(pastPedal, i);
      controller(checkPast, checkActive, i + 24, 'p', 2);
    }
    isManualActive[2] = true;
    manualActiveTime[2] = millis();
    pastPedal = activePedal;
  }
  if (activePistons != pastPistons)
  {
    byteLength = 62; // 64 bits but accounting for the 0-index
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activePistons, i);
      checkPast = bitRead(pastPistons, i);
      controller(checkPast, checkActive, i + 24, 'b', 3);
    }
    pastPistons = activePistons;
  }
}

void controller(byte checkPast, byte checkActive, byte key, char reg, byte reg_channel)
{
  if (checkActive == 1 && checkPast == 0)     //If a key is pressed
  {
    midiEventPacket_t noteOn = {0x09, 0x90 | reg_channel, key, 127};  // Might need to set velocity to 64?
    MidiUSB.sendMIDI(noteOn); 
    keyEvent = true;
    delayMicroseconds(70);
  }
  if (checkActive == 0 && checkPast == 1)     //If a key is realeased
  {
    midiEventPacket_t noteOff = {0x08, 0x80 | reg_channel, key, 127};  // Might need to set velocity to 64?
    MidiUSB.sendMIDI(noteOff);  
    keyEvent = true;
    delayMicroseconds(70);  
  }
MidiUSB.flush();
}

void flushMIDIbuffer()
{
  if (keyEvent == true)
  {
    MidiUSB.flush();
    keyEvent = false;
  }
}

void refresh()
{
  bool isSwellActive, isGreatActive, isPedalActive;
  if (millis() - lastPush >= refreshInterval)
  {
    lastPush = millis();
    for (byte i = 0; i < 62; i++)
    {
      isSwellActive = bitRead(activeSwell, i);
      if (isSwellActive)
      {
        controller(0, bitRead(activeSwell, i), i + 24, 's', 0);
      }
      if (!isSwellActive)
      {
        controller(1, bitRead(activeSwell, i), i + 24, 's', 0);
      }
      isGreatActive = bitRead(activeGreat, i);
      if (isGreatActive)
      {
        controller(0, bitRead(activeGreat, i), i + 24, 'g', 1);
      }
      if (!isGreatActive)
      {
        controller(1, bitRead(activeGreat, i), i + 24, 'g', 1);
      }
    }
    for (byte i = 0; i < 31; i++)
    {
      isPedalActive = bitRead(activePedal, i);
      if (isPedalActive)
      {
        controller(0, bitRead(activePedal, i), i + 24, 'p', 2);
      }
      if (!isPedalActive)
      {
        controller(1, bitRead(activePedal, i), i + 24, 'p', 2);
      }
    }
   MidiUSB.flush();
  }
}

void statusController() // Controls status and activity LEDs
{
  for (int man = 0; man < 2; man++) // For manuals 0 and 1...
  {
    if (((millis()/1000) % 2 == 0) && digitalRead(connectionLED[man]) == LOW) // If off and an even second
    {
      digitalWrite(connectionLED[man], HIGH);
    }
    if (((millis()/1000) % 2 != 0) && digitalRead(connectionLED[man]) == HIGH) // If on and an odd second
    {
      digitalWrite(connectionLED[man], LOW);
    }

    if (isManualActive[man] == true)
    {
      if(millis() > manualActiveTime[man] + 1000)
      {
        digitalWrite(activeLED[man], HIGH);
        isManualActive[man] = false;
      }
      else
      {
        if ((millis() / 100) % 2 == 0 && digitalRead(activeLED[man]) == HIGH)
        {
          digitalWrite(activeLED[man], LOW);
        }
        if ((millis() / 100) % 2 != 0 && digitalRead(activeLED[man]) == LOW)
        {
          digitalWrite(activeLED[man], HIGH);
        }
      }
    }
  }
}

//void resetActive(char man_sel)
//{
//  //Serial.println("reset called");
//  if (man_sel == 's')
//  {
//    activeSwell = 0x8000000000000000;
//  }
//    if (man_sel == 'g')
//  {
//    activeGreat = 0x8000000000000000;
//  }
//    if (man_sel == 'p')
//  {
//    activePedal = 0x80000000;
//  }
//    if (man_sel == 'b')
//  {
//    activePistons = 0x8000000000000000;
//  }
//}
