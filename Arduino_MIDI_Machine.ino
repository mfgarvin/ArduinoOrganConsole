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

//const char manuals [] = "t";
const char manuals [] = "gspb";
//char translator[] = {"activeTest"};
//const char *translator[] = {"activeGreat", "activeSwell", "activePedal", "activePistons"};
const int digitalInPin1 = 7;       // Analog input pin connected to the multiplexer
const int muxCtrlPinA = 2;        // Multiplexer control pin A
const int muxCtrlPinB = 3;        // Multiplexer control pin B
const int muxCtrlPinC = 4;        // Multiplexer control pin C
const int muxCtrlPinD = 5;        // Multiplexer control pin D
const int numOfMuxChannels = 16;  // Number of channels in the multiplexer
const int numOfBanks = 1;         // Number of Banks(Multiplexers) to poll - Note: All polled via the same control pins
int currentChannel = 0;           // Variable to store the current channel being read
bool keyPress[10];                    // Boolean to store if a keypress is being registered
uint32_t activeGreat = 0x80000000;
uint32_t standbyGreat = 0x80000000;
uint32_t pastGreat = 0x80000000;
uint32_t activeSwell = 0x80000000;
uint32_t standbySwell = 0x80000000;
uint32_t pastSwell = 0x80000000;
uint32_t activePedal = 0x80000000;
uint32_t standbyPedal = 0x80000000;
uint32_t pastPedal = 0x80000000;
uint32_t activePistons = 0x80000000;
uint32_t standbyPistons = 0x80000000;
uint32_t pastPistons = 0x80000000;
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
  pinMode(digitalInPin1, INPUT);
  Serial.begin(115200);
  // Some other setup
}
void loop() {
//  Serial.println(micros()); // For benchmarking
  readKeys();
  checkForKeyChanges();
}

void readKeys()
{
  //char readKeyPlaceholder;
  for (int i = 0; i < numOfMuxChannels; i++)
  {
    setMuxChannel(i);  // Set the multiplexer to the current channel
    for (int m = 1; m < 5; m++) //Repeat for each set of banks - 5 per manual (pedals will be different?)
    {
      keyPress[m] = digitalRead(digitalInPin1); //gonna have to adjust this...
      if (keyPress[m] == LOW)
      {
        bitWrite(standbySwell, i + (m*16) - 16, 1);      //  This is the SWELL
        keyPress[m] = HIGH;
      }
    }
    for (int m = 6; m < 10; m++)
    {
      keyPress[m] = digitalRead(digitalInPin1);
      if (keyPress[m] == LOW)
      {
        bitWrite(standbyGreat, i + (m*16) - 96, 1);    // This is the GREAT
        keyPress[m] = HIGH;
//        Serial.print(standbyGreat, BIN);
      }
    }
    for (int m = 11; m < 15; m++) // This won't need 4 banks, adjust later.
    {
      keyPress[m] = digitalRead(digitalInPin1);
      if (keyPress[m] == LOW)
      {
        bitWrite(standbyPedal, i + (m*16) - 176, 1);    //This is the PEDAL
        keyPress[m] = HIGH;
      }
    }
    for (int m = 5; m <= 15; m += 5)
    {
      keyPress[m] = digitalRead(digitalInPin1);
      if (keyPress[m] == LOW)
      {
        bitWrite(standbyPistons, i + ((m/5)*16) - 16, 1);    //These are the PISTONS
        keyPress[m] = HIGH;
      }
    }
  }
  activeGreat = standbyGreat;
  activeSwell = standbySwell;
  activePedal = standbyPedal;
  activePistons = standbyPistons;
  resetStandby();
  
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
//delay(1000);
  if (activeSwell != pastSwell)
  {
    byteLength = String(activeSwell, BIN).length() - 1;   // Iterate through the uintx_t
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activeSwell, i);
      checkPast = bitRead(pastSwell, i);
      controller(checkPast, checkActive, i, 's');
    }
    pastSwell = activeSwell;
    resetActive('s');
  }
  if (activeGreat != pastGreat)
  {
    byteLength = String(activeGreat, BIN).length() - 1;   // Iterate through the uintx_t
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activeGreat, i);
      checkPast = bitRead(pastGreat, i);
      controller(checkPast, checkActive, i, 'g');
    }
    pastGreat = activeGreat;
    resetActive('g');
  }
  if (activePedal != pastPedal)
  {
    byteLength = String(activePedal, BIN).length() - 1;   // Iterate through the uintx_t
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activePedal, i);
      checkPast = bitRead(pastPedal, i);
      controller(checkPast, checkActive, i, 'p');
    }
    pastPedal = activePedal;
    resetActive('p');
  }
  if (activePistons != pastPistons)
  {
    byteLength = String(activePistons, BIN).length() - 1;   // Iterate through the uintx_t
    for (byte i = 0; i < byteLength; i++)
    {
      checkActive = bitRead(activePistons, i);
      checkPast = bitRead(pastPistons, i);
      controller(checkPast, checkActive, i, 'b');
    }
    pastPistons = activePistons;
    resetActive('b');
  }
}

void controller(byte checkPast, byte checkActive, byte key, char reg)
{
  if (checkActive == 1 && checkPast == 0)     //If a key is pressed
  {
    sendStartSignal(key + 1, reg);       //This bit should already be the number of the key + 1- e.g. the 0th bit is the 1st key)
  }
  if (checkActive == 0 && checkPast == 1)     //If a key is realeased
  {
    sendStopSignal( key+ 1, reg);
  }
}

void resetActive(char man_sel)
{
  Serial.println("reset called");
  if (man_sel == 's')
  {
    activeSwell = 0x80000000;
  }
    if (man_sel == 'g')
  {
    activeGreat = 0x80000000;
  }
    if (man_sel == 'p')
  {
    activePedal = 0x80000000;
  }
    if (man_sel == 'b')
  {
    activePistons = 0x80000000;
  }
}

void resetStandby()
{
  standbyGreat = 0x80000000;
  standbySwell = 0x80000000;
  standbyPedal = 0x80000000;
  standbyPistons = 0x80000000;
}

void sendStartSignal(int key, char man_sel)
{
  Serial.print("Called the Start Signal");
  Serial.print(" - ");
  Serial.println(key, DEC);
  //First practice over serial, then impliment SPI, then impliment MIDI
}

void sendStopSignal(int key, char man_sel)
{
  Serial.print("Called the Stop Signal");
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
