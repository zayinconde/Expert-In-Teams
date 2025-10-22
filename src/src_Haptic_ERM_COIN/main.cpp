#include <Arduino.h>
#include <Wire.h>

#define PIN_IRQ             27
#define DA7280_ADDR_PRIMARY 0x4A
#define DA7280_ADDR_ALT     0x4B
uint8_t DA7280_ADDR = DA7280_ADDR_PRIMARY;

#define REG_CHIP_REV        0x00
#define REG_INT_MASK        0x09
#define REG_INT_STATUS      0x0A
#define REG_ENABLE          0x0B
#define REG_RTP_INPUT       0x0F
#define REG_TOP_CTL1        0x22
#define REG_ACTUATOR1       0x2B

#define MODE_STANDBY        0x00
#define MODE_RTP            0x01

volatile bool g_irqFlag = false;

bool writeReg_raw(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DA7280_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool readReg_raw(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(DA7280_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  delayMicroseconds(50);
  if (Wire.requestFrom((int)DA7280_ADDR, 1, (int)true) != 1) return false;
  val = Wire.read();
  return true;
}

bool readReg(uint8_t reg, uint8_t &val) {
  for (int i=0; i<5; i++) {
    if (readReg_raw(reg, val)) return true;
    delay(2);
  }
  return false;
}

void IRAM_ATTR onIRQ() { g_irqFlag = true; }

bool setTopCtl1(uint8_t op_mode, bool standby_en, bool seq_start){
  uint8_t v = ((seq_start?1:0)<<4) | ((standby_en?1:0)<<1) | (op_mode & 0x07);
  return writeReg_raw(REG_TOP_CTL1, v);
}

bool enterStandby(){ return setTopCtl1(MODE_STANDBY, true, false); }
bool enterRTP()    { return setTopCtl1(MODE_RTP, false, false); }

void scanI2C() {
  Serial.println("\n=== I2C Scan ===");
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", addr);
      if (addr == 0x4A) Serial.print(" (DA7280 primary)");
      if (addr == 0x4B) Serial.print(" (DA7280 alt)");
      Serial.println();
    }
  }
  Serial.println();
}

void diagRegisters() {
  Serial.println("=== Registers ===");
  uint8_t val;
  const uint8_t regs[] = {0x00, 0x09, 0x0A, 0x0B, 0x0F, 0x22, 0x23, 0x27, 0x28, 0x2B};
  const char* names[] = {"CHIP_REV", "INT_MASK", "INT_STATUS", "ENABLE", 
                         "RTP_INPUT", "TOP_CTL1", "TOP_CTL2", "V2I_H", "V2I_L", "ACTUATOR1"};
  
  for (int i=0; i<10; i++) {
    if (readReg(regs[i], val)) {
      Serial.printf("  0x%02X %-10s = 0x%02X", regs[i], names[i], val);
      if (regs[i] == 0x0A && val != 0x00) {
        Serial.print(" [");
        if (val & 0x20) Serial.print("UVLO ");
        if (val & 0x10) Serial.print("OVT ");
        if (val & 0x08) Serial.print("OVD ");
        if (val & 0x04) Serial.print("OC ");
        if (val & 0x01) Serial.print("UVLO2");
        Serial.print("]");
      }
      Serial.println();
    }
  }
  Serial.println();
}

bool rampVibrate(uint8_t target, uint16_t hold_ms){
  Serial.println("  Stopping...");
  writeReg_raw(REG_ENABLE, 0x00);
  enterStandby();
  delay(10);

  Serial.println("  Entering RTP...");
  if (!enterRTP()) return false;
  
  Serial.println("  Enabling...");
  if (!writeReg_raw(REG_ENABLE, 0x01)) return false;

  Serial.printf("  Ramping to %d...\n", target);
  uint8_t amp = 12;
  while (amp < target){
    writeReg_raw(REG_RTP_INPUT, amp);
    delay(35);
    if (g_irqFlag) {
      writeReg_raw(REG_ENABLE, 0x00);
      enterStandby();
      g_irqFlag = false;
      Serial.println("  IRQ during ramp!");
      return false;
    }
    int next = amp + 8;
    amp = (next > target) ? target : (uint8_t)next;
  }

  Serial.printf("  Holding %d ms...\n", hold_ms);
  writeReg_raw(REG_RTP_INPUT, target);
  uint32_t t0 = millis();
  while ((millis()-t0) < hold_ms){
    if (g_irqFlag) {
      writeReg_raw(REG_ENABLE, 0x00);
      enterStandby();
      g_irqFlag = false;
      Serial.println("  IRQ during hold!");
      return false;
    }
    delay(5);
  }

  Serial.println("  Done.");
  writeReg_raw(REG_ENABLE, 0x00);
  enterStandby();
  return true;
}

void setup(){
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== DA7280 Diagnostic ===\n");

  Wire.begin(21, 22);
  Wire.setClock(100000);
  Serial.println("I2C: 100kHz, SDA=21, SCL=22");

  scanI2C();

  bool found = false;
  for (uint8_t addr : {DA7280_ADDR_PRIMARY, DA7280_ADDR_ALT}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      DA7280_ADDR = addr;
      Serial.printf("DA7280 at 0x%02X\n\n", addr);
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println("ERROR: No DA7280!");
    while(1) delay(1000);
  }

  uint8_t rev;
  readReg(REG_CHIP_REV, rev);
  Serial.printf("Chip Rev: 0x%02X\n", rev);

  diagRegisters();

  // === RECOVERY SEQUENCE ===
  Serial.println("=== RECOVERY ===");
  
  Serial.println("1. Force power-down sequence...");
  writeReg_raw(REG_ENABLE, 0x00);
  writeReg_raw(REG_TOP_CTL1, 0x00);  // All bits off
  delay(100);
  
  Serial.println("2. Software reset...");
  writeReg_raw(0x01, 0x80);  // Reset bit
  delay(100);
  
  Serial.println("3. Clear all interrupt events...");
  writeReg_raw(0x01, 0xFF);  // Clear all event bits
  writeReg_raw(0x02, 0xFF);  // IRQ_EVENT2
  delay(50);
  
  uint8_t stat;
  readReg(REG_INT_STATUS, stat);
  Serial.printf("   Status after clear: 0x%02X\n", stat);
  
  Serial.println("4. Try writing SNP_MEM registers (0x1C-0x1E)...");
  // Some chips need these written to exit fault state
  writeReg_raw(0x1C, 0x00);
  writeReg_raw(0x1D, 0x00);
  writeReg_raw(0x1E, 0x00);
  delay(50);
  
  Serial.println("5. Clear TOP_CTL2...");
  uint8_t tc2;
  readReg(0x23, tc2);
  Serial.printf("   Was: 0x%02X\n", tc2);
  writeReg_raw(0x23, 0x00);
  delay(50);
  readReg(0x23, tc2);
  Serial.printf("   Now: 0x%02X\n", tc2);
  
  Serial.println("6. Clear INT_STATUS by reading...");
  for (int i=0; i<5; i++) {
    readReg(REG_INT_STATUS, stat);
    Serial.printf("   %d: 0x%02X\n", i+1, stat);
    if (stat == 0) {
      Serial.println("   *** CLEARED! ***");
      break;
    }
    delay(20);
  }
  
  Serial.println("4. Force standby...");
  enterStandby();
  delay(50);
  
  Serial.println("5. Set ERM mode...");
  uint8_t act;
  readReg(REG_ACTUATOR1, act);
  act &= ~0x01;
  writeReg_raw(REG_ACTUATOR1, act);
  
  Serial.println("6. Unmask interrupts...");
  writeReg_raw(REG_INT_MASK, 0x00);
  
  Serial.println();
  diagRegisters();

  pinMode(PIN_IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), onIRQ, FALLING);

  Serial.println("Ready! Commands:");
  Serial.println("  <intensity> <duration>  (e.g., 64 800)");
  Serial.println("  scan / diag\n");
}

void loop(){
  if (Serial.available()){
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "scan") {
      scanI2C();
    } 
    else if (input == "diag") {
      diagRegisters();
    }
    else {
      int intensity = input.toInt();
      int duration = 0;
      int sp = input.indexOf(' ');
      if (sp > 0) duration = input.substring(sp+1).toInt();

      if (intensity > 0 && intensity <= 255 && duration > 0){
        Serial.printf("\n=== Vibrate %d @ %dms ===\n", intensity, duration);
        rampVibrate((uint8_t)intensity, (uint16_t)duration);
      }
    }
  }

  if (g_irqFlag) {
    g_irqFlag = false;
    Serial.println("\n! IRQ !");
    diagRegisters();
  }
}