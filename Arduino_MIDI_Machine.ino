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

#include "MIDIUSB.h"
#define NUM_GREAT 61
#define NUM_SWELL 61
#define NUM_PISTON 15
#define NUM_BUTTONS 16 //Testing
// For the record, manuals will be refered to with a 'char' - as follows:
// Great = 'g'
// Swell = 's'
// Pedal = 'p'
// Pistons = 'b'
// Test = 't'
const char manuals [] = "gspb";
//const int digitalInPin1 = 52;       // Analog input pin connected to the multiplexer
const int swellInPin[] = {47, 49, 50, 52};
//const int swellInPin1[ = 47;
//const int swellInPin2 = 49;
//const int swellInPin3 = 51;
//const int swellInPin4 = 53;
const int greatInPin[] = {41};
const int pedalInPin[] = {42};
const int pistonInPin[] = {35};
const int muxCtrlPinA = 4;        // Multiplexer control pin A
const int muxCtrlPinB = 5;        // Multiplexer control pin B
const int muxCtrlPinC = 6;        // Multiplexer control pin C
const int muxCtrlPinD = 7;        // Multiplexer control pin D
const int numOfMuxChannels = 16;  // Number of channels in the multiplexer
const int numOfBanks = 1;         // Number of Banks(Multiplexers) to poll - Note: All polled via the same control pins
int currentChannel = 0;           // Variable to store the current channel being read
bool keyPress[20];                    // Boolean to store if a keypress is being registered
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

int byteLength;
byte checkActive;
byte checkPast;
int numMan = String(manuals).length();
int lastKeyPressed = 0;
int lastKeyPressedAddr = 0;

void setup() {
  pinMode(muxCtrlPinA, OUTPUT);
  pinMode(muxCtrlPinB, OUTPUT);
  pinMode(muxCtrlPinC, OUTPUT);
  pinMode(muxCtrlPinD, OUTPUT);
  pinMode(swellInPin[0], INPUT);
  pinMode(swellInPin[1], INPUT);
  pinMode(swellInPin[2], INPUT);
  pinMode(swellInPin[3], INPUT);

  Serial.begin(115200);
  // Some other setup
}
void loop() {
//  Serial.println(micros()); // For benchmarking
  readKeys();
  checkForKeyChanges();
}

void resetActive(char man_sel)
{
  //Serial.println("reset called");
  if (man_sel == 's')
  {
    activeSwell = 0x8000000000000000;
  }
    if (man_sel == 'g')
  {
    activeGreat = 0x8000000000000000;
  }
    if (man_sel == 'p')
  {
    activePedal = 0x80000000;
  }
    if (man_sel == 'b')
  {
    activePistons = 0x8000000000000000;
  }
}

void resetStandby()
{
  standbyGreat = 0x8000000000000000;
  standbySwell = 0x8000000000000000;
  standbyPedal = 0x80000000;
  standbyPistons = 0x8000000000000000;
}

void readKeys()
{
  //char readKeyPlaceholder;
  for (int i = 0; i < numOfMuxChannels; i++)  // For 0-15...
  {
    setMuxChannel(i);  // Set the multiplexer to the current channel. For 1-4...
    for (int m = 1; m < 5; m++) //Repeat for each set of banks - 4 per manual (pedals will be different?)
    {
      keyPress[i] = digitalRead(swellInPin[m-1]);
      if (keyPress[i] == LOW && keyPress[i] != bitRead(debounceSwell, i + (m*16) - 16))
      {
        bitWrite(standbySwell, i + (m*16) - 16, 1);      //  Swell On
      }
      if (keyPress[i] == HIGH && keyPress[i] != bitRead(debounceSwell, i + (m*16) - 16))
      {
        bitWrite(standbySwell, i + (m*16) - 16, 0);      //  Swell Off
      }
      if (keyPress[i] == LOW)
      {
        bitWrite(debounceSwell, i + (m*16) - 16, 1);     //  Debounce Logic. Set the first of two values.
      }                                                  //  If it's still LOW when we get back to it above, after a cycle,
                                                         //  then set the actual standby value HIGH
      if (keyPress[i] == HIGH)
      {
        bitWrite(debounceSwell, i + (m*16) - 16, 0);
      }
      keyPress[i] == HIGH; //Reset the keyPress for the next use. I could probably get rid of this?
    }
//    for (int m = 6; m < 10; m++)
//    {
//      keyPress[m] = digitalRead(digitalInPin1);
//      if (keyPress[m] == LOW)
//      {
//        bitWrite(standbyGreat, i + (m*16) - 96, 1);    // This is the GREAT
//        keyPress[m] = HIGH;
////        Serial.print(standbyGreat, BIN);
//      }
//    }
//    for (int m = 11; m < 13; m++) // Note: 2 Banks/Multiplexers only! Only 31 Pedal notes
//    {
//      keyPress[m] = digitalRead(digitalInPin1);
//      if (keyPress[m] == LOW)
//      {
//        bitWrite(standbyPedal, i + (m*16) - 176, 1);    //This is the PEDAL
//        keyPress[m] = HIGH;
//      }
//    }
    for (int m = 1; m < 2; m++) //Repeat for each set of banks - 4 per manual (pedals will be different?)
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
//    for (int m = 5; m <= 15; m += 5)
//    {
//      keyPress[m] = digitalRead(digitalInPin1);
//      if (keyPress[m] == LOW)
//      {
//        bitWrite(standbyPistons, i + ((m/5)*16) - 16, 1);    //These are the PISTONS
//        keyPress[m] = HIGH;
//      }
  }
  activeGreat = standbyGreat;
  activeSwell = standbySwell;
  activePedal = standbyPedal;
  activePistons = standbyPistons;
//  resetStandby();
  
}

void setMuxChannel(int channel) {
  digitalWrite(muxCtrlPinA, bitRead(channel, 0));
  digitalWrite(muxCtrlPinB, bitRead(channel, 1));
  digitalWrite(muxCtrlPinC, bitRead(channel, 2));
  digitalWrite(muxCtrlPinD, bitRead(channel, 3));
}

void checkForKeyChanges()
{
//  Serial.println(keyPress[0]);
//  Serial.print("Active - ");
//  Serial.print(activeSwell, HEX);
//  Serial.print(" - ");
//  Serial.print(activeGreat, HEX);
//  Serial.print(" - ");
//  Serial.print(activePedal, HEX);
//  Serial.print(" - ");
//  Serial.println(activePistons, HEX);
//  Serial.print("Past   - ");
//  Serial.println(pastSwell, HEX);
//  delay(10);
  if (activeSwell != pastSwell)
  {
    // This is broken... But, becuase the byte is always the same length (as the 64th bit is set high), I don't think I need this?
//    byteLength = String(activeSwell, BIN).length() - 1;   // Iterate through the uintx_t
    byteLength = 62; // 64 bits but accounting for the 0-index and the preset 64th bit
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activeSwell, i);
      checkPast = bitRead(pastSwell, i);
      controller(checkPast, checkActive, i + 24, 's', 0);
    }
    pastSwell = activeSwell;
    resetActive('s');
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
    pastGreat = activeGreat;
    resetActive('g');
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
    pastPedal = activePedal;
    resetActive('p');
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
    resetActive('b');
  }
}

void controller(byte checkPast, byte checkActive, byte key, char reg, byte reg_channel)
{
  if (checkActive == 1 && checkPast == 0)     //If a key is pressed
  {
//    sendStartSignal(key, reg);       //Updated - Key should now reflect the midi key value. (old: This bit should already be the number of the key + 1- e.g. the 0th bit is the 1st key))
    midiEventPacket_t noteOn = {0x09, 0x90 | reg_channel, key, 127};  // Might need to set velocity to 64?
    MidiUSB.sendMIDI(noteOn); 
  }
  if (checkActive == 0 && checkPast == 1)     //If a key is realeased
  {
//    sendStopSignal( key, reg);  
    midiEventPacket_t noteOff = {0x08, 0x80 | reg_channel, key, 127};  // Might need to set velocity to 64?
    MidiUSB.sendMIDI(noteOff);  
  }
MidiUSB.flush();
}



void sendStartSignal(int key, char man_sel)
{
  Serial.print("Called the Start Signal");
  Serial.print(": ");
  Serial.print(key, DEC);
  Serial.print(" - ");
  Serial.println(man_sel);
}

void sendStopSignal(int key, char man_sel)
{
  Serial.print("Called the Stop Signal");
  Serial.print(": ");
  Serial.print(key, DEC);
  Serial.print(" - ");
  Serial.println(man_sel);
}

// void graveyard ()
//
//    keyPress1[] = digitalRead(digitalInPin1);
//    // keyPressGreat = .... etc....
//    if (keyPress1[0] == LOW)
//    {
//      bitWrite(activeTest, i + (numOfBanks*16) - 16 , 1);     // Need to iterate through numofBanks
//    }
//    // Repeat for each manual
//  }
//  unsigned long x = micros();
//  if (1000000 <= x && x <= 2000000)
//  {
//   Serial.println(x); 
//  }
//  Serial.println(micros());
//  checkForKeyChanges();
