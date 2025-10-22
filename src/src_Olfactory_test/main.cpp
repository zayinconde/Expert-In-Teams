#include <Arduino.h>
#include "BluetoothSerial.h"


BluetoothSerial serialBT; //global object for bluetooth
char cmd;


// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  //int result = myFunction(2, 3);
  serialBT.begin("ESP32-BT");
  pinMode(2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(serialBT.available()){
    cmd = serialBT.read();
  }
  if(cmd == '1'){
    digitalWrite(2, HIGH);
  }
  if(cmd == '0'){
    digitalWrite(2, LOW);
  }
}

// put function definitions here:
//int myFunction(int x, int y) {
//return x + y;
//}