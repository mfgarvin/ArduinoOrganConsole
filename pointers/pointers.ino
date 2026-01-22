void setup() {
  Serial.begin(9600);
  uint16_t test = 0x00;
  char myVar[] = "test";
  char sometext[] = "myVar";
  //= 'H';  // Initialize a variable.
  
  Serial.print("myVar's lvalue: ");
  Serial.println((long) &myVar, DEC);  // Grab myVar's lvalue
  Serial.print("myVar's rvalue: ");
  Serial.println(myVar);
  Serial.println();
  
  char *myPointer;   // Declare your pointer.
  myPointer = &myVar[0]; //Assign myVar's memory address to pointer.
  uint16_t *myDataPointer;
  myDataPointer = &test;
  
  Serial.print("myPointer's lvalue: ");
  Serial.println((long) &myPointer, DEC);  //myPointer's lvalue
  Serial.print("myPointer's rvalue: ");
  Serial.println((long) myPointer);  //myPointer's rvalue
  Serial.print("printing what rvalue is referencing?");
//  Serial.println(*(myPointer + ));
  for (int i=0; i < sizeof(myVar)/sizeof(myVar[0]); i++)
  {
    Serial.print(*(myPointer + i));
  }
  Serial.println("----");
    Serial.println(*(&myVar));
//  Serial.println(*myDataPointer, BIN);
//  Serial.println(sizeof(myVar)/sizeof(myVar[0]));
  bitWrite(*(&myVar), 0, 1);
}

void loop() {
}
