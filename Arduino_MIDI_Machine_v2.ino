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
// Just for reference...
#define NUM_KEYS 61
#define NUM_PISTON 15
#define NUM_PEDALS 32
// For the record, manuals will be refered to with a 'char' - as follows:
// Great = 'g'
// Swell = 's'
// Pedal = 'p'
// Pistons = 'b'
// Test = 't'
//const char manuals [] = "gscpb";
const int swellInPin[] = {28, 26, 24, 22};
const int greatInPin[] = {29, 27, 25, 23};
const int choirInPin[] = {38, 36, 34, 32};
const int pedalInPin[] = {35, 33};
const int pistonInPin[] = {39, 37};
const int stopPin[] = {40, 41, 42, 43};
const int ledPin[] = {44, 45, 46, 47};
const int muxCtrlPin[] = {4, 5, 6, 7}; //Multiplexer control pins A, B, C, and D
const int numOfMuxChannels = 16;  // Number of channels in the multiplexer
//const int numOfBanks = 1;         // Number of Banks(Multiplexers) to poll - Note: All polled via the same control pins
//int currentChannel = 0;           // Variable to store the current channel being read
bool keyPress[16];                    // Boolean to store if a keypress is being registered
uint64_t activeSwell = 0x8000000000000000;
uint64_t standbySwell = 0x8000000000000000;
uint64_t pastSwell = 0x8000000000000000;
uint64_t activeGreat = 0x8000000000000000;
uint64_t standbyGreat = 0x8000000000000000;
uint64_t pastGreat = 0x8000000000000000;
uint64_t activeChoir = 0x8000000000000000;
uint64_t standbyChoir = 0x8000000000000000;
uint64_t pastChoir = 0x8000000000000000;
uint64_t activePedal = 0x8000000000000000;
uint64_t standbyPedal = 0x8000000000000000;
uint64_t pastPedal = 0x8000000000000000;
uint64_t activePistons = 0x8000000000000000;
uint64_t standbyPistons = 0x8000000000000000;
uint64_t pastPistons = 0x8000000000000000;
uint64_t activeStops = 0x8000000000000000;
uint64_t standbyStops = 0x8000000000000000;
uint64_t pastStops = 0x8000000000000000;
//Trying some debouce stuff here
uint64_t debounceSwell = 0x8000000000000000;
uint64_t debounceGreat = 0x8000000000000000;
uint64_t debounceChoir = 0x8000000000000000;
uint64_t debouncePedal = 0x8000000000000000;
uint64_t debouncePistons = 0x8000000000000000;
uint64_t debounceStops = 0x8000000000000000;
int debounceArray[10][64];

int byteLength;
byte checkActive;
byte checkPast;
const int connectionLED[] = {13, 13, 13, 13};
const int activeLED[] = {73, 72, 73, 72};
bool isManualActive[4];
uint32_t manualActiveTime[4];
bool keyEvent;
uint32_t lastPush;
int refreshInterval = 1000; //Milliseconds

void setup() {
  for (int s = 0; s < 4; s++)  // For 0-3...
  {
    pinMode(muxCtrlPin[s], OUTPUT);
    pinMode(swellInPin[s], INPUT_PULLUP);
    pinMode(greatInPin[s], INPUT_PULLUP);
    pinMode(choirInPin[s], INPUT_PULLUP);
    pinMode(stopPin[s], INPUT_PULLUP);
    pinMode(ledPin[s], OUTPUT);
    digitalWrite(ledPin[s], HIGH);
  }
  for (int s = 0; s < 2; s++) //For 0-1...
  {
    pinMode(pistonInPin[s], INPUT_PULLUP);
    pinMode(activeLED[s], OUTPUT);
    pinMode(pedalInPin[s], INPUT_PULLUP);
    digitalWrite(activeLED[s], HIGH);
  }
  pinMode(13, OUTPUT);

  Serial.begin(9600); //115200
//  delay(1000);
//  Serial.print("Hello!");
  // Some other setup
}
void loop() {
  readKeys();
 checkForKeyChanges();
//  flushMIDIbuffer(); // OLD
//  refresh(); // OLD
  statusController();
  midiReadTest();
}

void readKeys()
{
  int reversed_i;
  int offset = 1;
  const int debounceThreshold = 10;
  for (int i = 0; i < numOfMuxChannels; i++)  // For 0-15...
  {
    setMuxChannel(i);  // Set the multiplexer to the current channel. For Input 0-15...
    reversed_i = map(i, 0, 15, 15, 0);  // Some multiplexers are reversed - this accommodates that.
    for (int m = 1; m < 5; m++) //Each keyboard has 4 multiplexers (plus 2 for pistons, but that's seperate). Pedals will be different
    {
//    ============ SWELL ============
      keyPress[reversed_i] = digitalRead(swellInPin[m-1]);
      if (keyPress[reversed_i] == LOW)
      {
        if (debounceArray[0][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[0][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[0][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[1][reversed_i + (m*16) - 16 - offset] = 0; // Reset the release counter when the key is pressed
          bitWrite(standbySwell, reversed_i + (m * 16) - 16 - offset, 1);
          debounceArray[0][reversed_i + (m*16) - 16 - offset]++;
        };
      } 
      else {
        if (debounceArray[1][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[1][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[1][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[0][reversed_i + (m*16) - 16 - offset] = 0; // Reset the press counter when the key is released
          bitWrite(standbySwell, reversed_i + (m * 16) - 16 - offset, 0); // Flip the active bit for the swell
          debounceArray[1][reversed_i + (m*16) - 16 - offset]++;
        }        
      }

//    ============ GREAT ============
      keyPress[reversed_i] = digitalRead(greatInPin[m-1]);
      if (keyPress[reversed_i] == LOW)
      {
        if (debounceArray[2][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[2][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[2][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[3][reversed_i + (m*16) - 16 - offset] = 0; // Reset the release counter when the key is pressed
          bitWrite(standbyGreat, reversed_i + (m * 16) - 16 - offset, 1);
          debounceArray[2][reversed_i + (m*16) - 16 - offset]++;
        }
      } 
      else {
        if (debounceArray[3][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[3][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[3][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[2][reversed_i + (m*16) - 16 - offset] = 0; // Reset the press counter when the key is released
          bitWrite(standbyGreat, reversed_i + (m * 16) - 16 - offset, 0); // Flip the active bit for the great
          debounceArray[3][reversed_i + (m*16) - 16 - offset]++;
        }
      }

//    ============ CHOIR ============
      keyPress[reversed_i] = digitalRead(choirInPin[m-1]);
      if (keyPress[reversed_i] == LOW)
      {
        if (debounceArray[4][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[4][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[4][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[5][reversed_i + (m*16) - 16 - offset] = 0; // Reset the release counter when the key is pressed 
          bitWrite(standbyChoir, reversed_i + (m * 16) - 16 - offset, 1);
          debounceArray[4][reversed_i + (m*16) - 16 - offset]++;
        };
      } 
      else {
        if (debounceArray[5][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[5][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[5][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[4][reversed_i + (m*16) - 16 - offset] = 0; // Reset the press counter when the key is released
          bitWrite(standbyChoir, reversed_i + (m * 16) - 16 - offset, 0); // Flip the active bit for the choir
          debounceArray[5][reversed_i + (m*16) - 16 - offset]++;
        }
      }
//    ========= STOPS ==========
      keyPress[reversed_i] = digitalRead(stopPin[m-1]);
      if (keyPress[reversed_i] == LOW)
      {
        if (debounceArray[8][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[8][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[8][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[9][reversed_i + (m*16) - 16 - offset] = 0; // Reset the release counter when the key is pressed
          bitWrite(standbyStops, reversed_i + (m * 16) - 16 - offset, 1);
          debounceArray[8][reversed_i + (m*16) - 16 - offset]++;
        }
      } 
      else {
        if (debounceArray[9][reversed_i + (m*16) - 16 - offset] < debounceThreshold) 
        {
          debounceArray[9][reversed_i + (m*16) - 16 - offset]++;
        } 
        else if (debounceArray[9][reversed_i + (m*16) - 16 - offset] >= debounceThreshold) 
        {
          debounceArray[8][reversed_i + (m*16) - 16 - offset] = 0; // Reset the press counter when the key is released
          bitWrite(standbyStops, reversed_i + (m * 16) - 16 - offset, 0); // Flip the active bit for the great
          debounceArray[9][reversed_i + (m*16) - 16 - offset]++;
        }
      }
    }

//    ============= PEDALS ===============
    for (int m = 1; m < 3; m++)
    {
      keyPress[i] = digitalRead(pedalInPin[m-1]);
      if (keyPress[i] == LOW)
      {
        if (debounceArray[6][i + (m*16) - 16] < debounceThreshold) 
        {
          debounceArray[6][i + (m*16) - 16]++;
        } 
        else if (debounceArray[6][i + (m*16) - 16] >= debounceThreshold) 
        {
          debounceArray[7][i + (m*16) - 16] = 0; // Reset the release counter when the key is pressed 
          bitWrite(standbyPedal, i + (m * 16) - 16, 1);
          debounceArray[6][i + (m*16) - 16]++;
        }
      } 
      else {
       if (debounceArray[7][i + (m*16) - 16] < debounceThreshold) 
        {
          debounceArray[7][i + (m*16) - 16]++;
        } 
        else if (debounceArray[7][i + (m*16) - 16] >= debounceThreshold) 
        {
          debounceArray[6][i + (m*16) - 16] = 0; // Reset the press counter when the key is released
          bitWrite(standbyPedal, i + (m * 16) - 16, 0); // Flip the active bit for the pedal
          debounceArray[7][i + (m*16) - 16]++;
        }
      }
//    ========== PISTONS ===========
      keyPress[i] = digitalRead(pistonInPin[m-1]);
      if (keyPress[i] == LOW && keyPress[i] != bitRead(debouncePistons, i + (m*16) - 16))
      {
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
  activeChoir = standbyChoir;
  activePedal = standbyPedal;
  activePistons = standbyPistons;
  activeStops = standbyStops;
  
}

void setMuxChannel(int channel) {
  digitalWrite(muxCtrlPin[0], bitRead(channel, 0)); // A
  digitalWrite(muxCtrlPin[1], bitRead(channel, 1)); // B
  digitalWrite(muxCtrlPin[2], bitRead(channel, 2)); // C
  digitalWrite(muxCtrlPin[3], bitRead(channel, 3)); // D
//  delay(100);
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
  
  if (activeChoir != pastChoir)
  {
    byteLength = 62; // 64 bits but accounting for the 0-index and the preset 64th bit
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activeChoir, i);
      checkPast = bitRead(pastChoir, i);
      controller(checkPast, checkActive, i + 24, 'c', 2);
    }
    isManualActive[2] = true;
    manualActiveTime[2] = millis();
    pastChoir = activeChoir;
  }
  
  if (activePedal != pastPedal)
  {
    byteLength = 32;
    for (byte i = 0; i <= byteLength; i++)
    {
      checkActive = bitRead(activePedal, i);
      checkPast = bitRead(pastPedal, i);
      controller(checkPast, checkActive, i + 24, 'p', 3);
    }
    isManualActive[3] = true;
    manualActiveTime[3] = millis();
    pastPedal = activePedal;
  }
  if (activePistons != pastPistons)
  {
    byteLength = 62; // 64 bits but accounting for the 0-index
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activePistons, i);
      checkPast = bitRead(pastPistons, i);
      controller(checkPast, checkActive, i + 24, 'b', 4);
    }
    pastPistons = activePistons;
  }
  if (activeStops != pastStops)
  {
    byteLength = 62; // 64 bits but accounting for the 0-index and the preset 64th bit
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activeStops, i);
      checkPast = bitRead(pastStops, i);
      controller(checkPast, checkActive, i + 24, 'v', 5);
    }
    isManualActive[2] = true;
    manualActiveTime[2] = millis();
    pastChoir = activeChoir;
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

void midiReadTest()
{
  midiEventPacket_t rx;
  do {
    rx = MidiUSB.read();
    if (rx.header != 0) {
      Serial.print("Received: ");
      Serial.print(rx.header, HEX);
      Serial.print("-");
      Serial.print(rx.byte1, HEX);
      Serial.print("-");
      Serial.print(rx.byte2, HEX);
      Serial.print("-");
      Serial.println(rx.byte3, HEX);
    }
  } while (rx.header != 0);
}
//
//void flushMIDIbuffer()
//{
//  if (keyEvent == true)
//  {
//    MidiUSB.flush();
//    keyEvent = false;
//  }
//}

//void refresh()
//{
//  bool isSwellActive, isGreatActive, isChoirActive, isPedalActive;
//  if (millis() - lastPush >= refreshInterval)
//  {
//    lastPush = millis();
//    for (byte i = 0; i < 62; i++)
//    {
//      isSwellActive = bitRead(activeSwell, i);
//      if (isSwellActive)
//      {
//        controller(0, bitRead(activeSwell, i), i + 24, 's', 0);
//      }
//      if (!isSwellActive)
//      {
//        controller(1, bitRead(activeSwell, i), i + 24, 's', 0);
//      }
//      isGreatActive = bitRead(activeGreat, i);
//      if (isGreatActive)
//      {
//        controller(0, bitRead(activeGreat, i), i + 24, 'g', 1);
//      }
//      if (!isGreatActive)
//      {
//        controller(1, bitRead(activeGreat, i), i + 24, 'g', 1);
//      }
//      isChoirActive = bitRead(activeChoir, i);
//      if (isChoirActive)
//      {
//        controller(0, bitRead(activeChoir, i), i + 24, 'c', 2);
//      }
//      if (!isChoirActive)
//      {
//        controller(1, bitRead(activeChoir, i), i + 24, 'c', 2);
//      }
//    }
//    for (byte i = 0; i < 31; i++)
//    {
//      isPedalActive = bitRead(activePedal, i);
//      if (isPedalActive)
//      {
//        controller(0, bitRead(activePedal, i), i + 24, 'p', 3);
//      }
//      if (!isPedalActive)
//      {
//        controller(1, bitRead(activePedal, i), i + 24, 'p', 3);
//      }
//    }
//   MidiUSB.flush();
//  }
//}

void statusController() // Controls status and activity LEDs
{
  for (int man = 0; man < 3; man++) // For manuals 0 and 1...
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
