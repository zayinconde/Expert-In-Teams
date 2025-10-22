#include <Arduino.h>
#include <Wire.h> // allows communication for I2C devices
//#include "BluetoothSerial.h"

static const uint8_t SDA_PIN = 21; // defining SDA Pin
static const uint8_t SCL_PIN = 22; // defining SCL pin
static uint8_t DA728X_ADDR = 0x4A; // try 0x4B if scan shows that

static const char *SCRIPT = 
R"(WRITE DA728x 0x03 0x00
WRITE DA728x 0x04 0x00
WRITE DA728x 0x05 0x00
WRITE DA728x 0x07 0x00
WRITE DA728x 0x08 0x40
WRITE DA728x 0x0A 0x22
WRITE DA728x 0x0B 0x3C
WRITE DA728x 0x0C 0x6B
WRITE DA728x 0x0D 0x6B
WRITE DA728x 0x0E 0x1F
WRITE DA728x 0x0F 0x00
WRITE DA728x 0x10 0x98
WRITE DA728x 0x13 0x18
WRITE DA728x 0x14 0x11
WRITE DA728x 0x15 0x03
WRITE DA728x 0x16 0x40
WRITE DA728x 0x17 0x81
WRITE DA728x 0x1C 0x0E
WRITE DA728x 0x1D 0x20
WRITE DA728x 0x1E 0x03
WRITE DA728x 0x1F 0x20
WRITE DA728x 0x20 0x43
WRITE DA728x 0x22 0x00
WRITE DA728x 0x23 0x36
WRITE DA728x 0x24 0x08
WRITE DA728x 0x25 0x61
WRITE DA728x 0x26 0xB4
WRITE DA728x 0x27 0xEC
WRITE DA728x 0x28 0x00
WRITE DA728x 0x29 0x00
WRITE DA728x 0x2A 0x08
WRITE DA728x 0x2B 0x10
WRITE DA728x 0x2D 0x80
WRITE DA728x 0x48 0x1D
WRITE DA728x 0x49 0x04
WRITE DA728x 0x4C 0x02
WRITE DA728x 0x5F 0x0E
WRITE DA728x 0x60 0x9D
WRITE DA728x 0x62 0x6F
WRITE DA728x 0x6E 0x00
WRITE DA728x 0x81 0x04
WRITE DA728x 0x83 0x00
WRITE DA728x 0x84 0x00
WRITE DA728x 0x85 0x00
WRITE DA728x 0x86 0x00
WRITE DA728x 0x87 0x00
WRITE DA728x 0x88 0x00
WRITE DA728x 0x89 0x00
WRITE DA728x 0x8A 0x00
WRITE DA728x 0x8B 0x00
WRITE DA728x 0x8C 0x00
WRITE DA728x 0x8D 0x00
WRITE DA728x 0x8E 0x00
WRITE DA728x 0x8F 0x00
WRITE DA728x 0x90 0x00
WRITE DA728x 0x91 0x00
WRITE DA728x 0x92 0x00
WRITE DA728x 0x93 0x00
WRITE DA728x 0x94 0x00
WRITE DA728x 0x95 0x00
WRITE DA728x 0x96 0x00
WRITE DA728x 0x97 0x00
WRITE DA728x 0x98 0x00
WRITE DA728x 0x99 0x00
WRITE DA728x 0x9A 0x00
WRITE DA728x 0x9B 0x00
WRITE DA728x 0x9C 0x00
WRITE DA728x 0x9D 0x00
WRITE DA728x 0x9E 0x00
WRITE DA728x 0x9F 0x00
WRITE DA728x 0xA0 0x00
WRITE DA728x 0xA1 0x00
WRITE DA728x 0xA2 0x00
WRITE DA728x 0xA3 0x00
WRITE DA728x 0xA4 0x00
WRITE DA728x 0xA5 0x00
WRITE DA728x 0xA6 0x00
WRITE DA728x 0xA7 0x00
WRITE DA728x 0xA8 0x00
WRITE DA728x 0xA9 0x00
WRITE DA728x 0xAA 0x00
WRITE DA728x 0xAB 0x00
WRITE DA728x 0xAC 0x00
WRITE DA728x 0xAD 0x00
WRITE DA728x 0xAE 0x00
WRITE DA728x 0xAF 0x00
WRITE DA728x 0xB0 0x00
WRITE DA728x 0xB1 0x00
WRITE DA728x 0xB2 0x00
WRITE DA728x 0xB3 0x00
WRITE DA728x 0xB4 0x00
WRITE DA728x 0xB5 0x00
WRITE DA728x 0xB6 0x00
WRITE DA728x 0xB7 0x00
WRITE DA728x 0xB8 0x00
WRITE DA728x 0xB9 0x00
WRITE DA728x 0xBA 0x00
WRITE DA728x 0xBB 0x00
WRITE DA728x 0xBC 0x00
WRITE DA728x 0xBD 0x00
WRITE DA728x 0xBE 0x00
WRITE DA728x 0xBF 0x00
WRITE DA728x 0xC0 0x00
WRITE DA728x 0xC1 0x00
WRITE DA728x 0xC2 0x00
WRITE DA728x 0xC3 0x00
WRITE DA728x 0xC4 0x00
WRITE DA728x 0xC5 0x00
WRITE DA728x 0xC6 0x00
WRITE DA728x 0xC7 0x00
WRITE DA728x 0xC8 0x00
WRITE DA728x 0xC9 0x00
WRITE DA728x 0xCA 0x00
WRITE DA728x 0xCB 0x00
WRITE DA728x 0xCC 0x00
WRITE DA728x 0xCD 0x00
WRITE DA728x 0xCE 0x00
WRITE DA728x 0xCF 0x00
WRITE DA728x 0xD0 0x00
WRITE DA728x 0xD1 0x00
WRITE DA728x 0xD2 0x00
WRITE DA728x 0xD3 0x00
WRITE DA728x 0xD4 0x00
WRITE DA728x 0xD5 0x00
WRITE DA728x 0xD6 0x00
WRITE DA728x 0xD7 0x00
WRITE DA728x 0xD8 0x00
WRITE DA728x 0xD9 0x00
WRITE DA728x 0xDA 0x00
WRITE DA728x 0xDB 0x00
WRITE DA728x 0xDC 0x00
WRITE DA728x 0xDD 0x00
WRITE DA728x 0xDE 0x00
WRITE DA728x 0xDF 0x00
WRITE DA728x 0xE0 0x00
WRITE DA728x 0xE1 0x00
WRITE DA728x 0xE2 0x00
WRITE DA728x 0xE3 0x00
WRITE DA728x 0xE4 0x00
WRITE DA728x 0xE5 0x00
WRITE DA728x 0xE6 0x00
WRITE DA728x 0xE7 0x00
)";

const char *cursor = SCRIPT; // where we are in the step through

// need to create a two bool function (true or false), one function is for writing over I2C and the other is for Reading over I2C
//the first function begins the transmission and sends the register it wants to write to (reg)
// after that it sends the val to write into that register (val).
// if this is successful then it returns "true" if the transmission failed it returns (false)
bool wr(uint8_t reg, uint8_t val){
  Wire.beginTransmission(DA728X_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool rd(uint8_t reg, uint8_t &val) {        // <- pass by reference
  Wire.beginTransmission(DA728X_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)      // <- repeated START, no STOP
    return false;
  if (Wire.requestFrom((int)DA728X_ADDR, 1) != 1)
    return false;
  val = Wire.read();
  return true;
}

void scan(){
  Serial.println("scanning I2C");
  for(uint8_t a=1; a<127; a++){
    Wire.beginTransmission(a);
    if(Wire.endTransmission()==0) Serial.printf(" 0x%02X\n", a); // if transmission is successful, it prints the value in hexadecimal format (a)
  }
}

bool nextScriptCmd(uint8_t &reg, uint8_t &val) { // next script command address and value ?
  // read next non-empty line from SCRIPT and parse it 
  char line[64];
  size_t n = 0;
  while(*cursor && *cursor!='\n' && n<sizeof(line)-1) line[n++] = *cursor++;
  if(!*cursor) return false;

  //copy line
  while(*cursor && *cursor!='\n' && n<sizeof(line)-1) line[n++] = *cursor++;
  if (*cursor == '\n') cursor++;
  line[n] = 0;

  unsigned r,v;
  if(sscanf(line, "WRITE DA728x 0x%X 0x%X", &r,&v)==2){
    reg = (uint8_t)r;
    val = (uint8_t)v;
    return true;
  }
  return false; // skip unexpected lines
}


void help(){ // help function for printing the commands in serial monitor
  Serial.println("Commands:");
  Serial.println("  s                 : scan I2C bus");
  Serial.println("  a                 : apply ALL remaining script lines");
  Serial.println("  n                 : apply NEXT script line");
  Serial.println("  r RR              : read reg (hex), e.g. r 08");
  Serial.println("  w RR VV           : write reg/value (hex), e.g. w 08 40");
  Serial.println("  addr XX           : set I2C address (hex), e.g. addr 4B");
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(200);
  Serial.println("\nDA7280 Driver Ready!");
  Wire.begin(SDA_PIN,SCL_PIN);
  Wire.setClock(400000); // sets clock frequency of I2C to 400kHz
  help(); //triggers help function with commands

}

void loop() {
  // put your main code here, to run repeatedly:
  static String cmd;
  while(Serial.available()){
    char c = (char)Serial.read();
    if(c=='\r' || c=='\n'){
      if(cmd.length()==0) continue; // checks if cmd string is empty if it is then continue on and skip this if loop
    
      if (cmd == "s"){
        scan();
      } else if (cmd == "a"){
        uint8_t r,v;
        size_t count=0, ok=0;
        while(nextScriptCmd(r,v)){
          count++;
          if (wr(r,v)) {
            uint8_t rb=0xFF;
            if (rd(r, rb)) {
              Serial.printf("  [%02u] W 0x%02X=0x%02X  (RB=0x%02X)\n", (unsigned)count, r, v, rb);
              ok++;
            } else {
              Serial.printf("  [%02u] W 0x%02X=0x%02X  (RB FAIL)\n", (unsigned)count, r, v);
            }
          } else {
            Serial.printf("  [%02u] WRITE FAIL at reg 0x%02X\n", (unsigned)count, r);
            break;
          }
          delay(1);
        }
        Serial.printf("Applied %u lines, %u OK.\n", (unsigned)count, (unsigned)ok);
      } else if (cmd == "rewind") {
        cursor = SCRIPT;
        Serial.println("Script cursor reset to start.");
      
      } else if (cmd=="n") {
        uint8_t r,v;
        while (*cursor && (*cursor=='\r' || *cursor=='\n')) cursor++; // skip blanks
        if (!*cursor) { Serial.println("Script finished."); }
        else if (nextScriptCmd(r,v)) {
          if (wr(r,v)) {
            uint8_t rb;
            if (rd(r,rb)) Serial.printf("  W 0x%02X=0x%02X  (RB=0x%02X)\n", r, v, rb);
            else          Serial.printf("  W 0x%02X=0x%02X  (RB FAIL)\n", r, v);
          } else {
            Serial.printf("  WRITE FAIL at 0x%02X\n", r);
          }
        } else {
          Serial.println("Skipped non-command line.");
        }
      } else if (cmd.startsWith("r ")) {
        unsigned r;
        if (sscanf(cmd.c_str()+2, "%X", &r)==1) {
          uint8_t v;
          if (rd((uint8_t)r, v)) Serial.printf("  R 0x%02X -> 0x%02X\n", (uint8_t)r, v);
          else Serial.println("  Read failed.");
        }
      } else if (cmd.startsWith("w ")) {
        unsigned r,v;
        if (sscanf(cmd.c_str()+2, "%X %X", &r, &v)==2) {
          if (wr((uint8_t)r, (uint8_t)v)) Serial.printf("  W 0x%02X=0x%02X\n", (uint8_t)r, (uint8_t)v);
          else Serial.println("  Write failed.");
        }
      } else if (cmd.startsWith("addr ")) {
        unsigned a;
        if (sscanf(cmd.c_str()+5, "%X", &a)==1) {
          DA728X_ADDR = (uint8_t)a;
          Serial.printf("  I2C address set to 0x%02X\n", DA728X_ADDR);
        }
      } else {
        help();
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }
}


// put function definitions here:
//int myFunction(int x, int y) {
//return x + y;
//}