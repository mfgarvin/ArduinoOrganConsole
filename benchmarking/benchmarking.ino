bool someval = false;
unsigned long duration = 0;
unsigned long x = 0;
unsigned long y = 0;
uint16_t storage = 0b00;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  x = micros();
  y = micros();
  duration = y - x;
  Serial.print("Control Duration: ");
  Serial.println(duration);
  Serial.println("");
  x = micros();
  if (someval == true)
  {
    char words [] = {"Something here"};
  }
  y = micros();
  duration = y - x;
  Serial.print("Logic Duration: ");
  Serial.println(duration);
  Serial.println("");
  x = micros();
  bitWrite(storage, 4, 1);
  y = micros();
  duration = y - x;
  Serial.print("bitWrite Duration: ");
  Serial.println(duration); 
  Serial.println("");
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
