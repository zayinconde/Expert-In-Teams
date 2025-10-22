#include <Arduino.h>
#include <Wire.h>

// ---------- Pins ----------
#define PIN_IRQ             27      // DA7280 nIRQ -> ESP32 GPIO (active LOW)

// ---------- DA7280 I2C + Registers ----------
#define DA7280_ADDR_PRIMARY 0x4A    // AD0 = VDD
#define DA7280_ADDR_ALT     0x4B    // AD0 = GND
uint8_t DA7280_ADDR = DA7280_ADDR_PRIMARY; // Will be auto-detected

#define REG_CHIP_REV        0x00
#define REG_INT_MASK        0x09
#define REG_INT_STATUS      0x0A
#define REG_ENABLE          0x0B
#define REG_RTP_INPUT       0x0F
#define REG_TOP_CTL1        0x22
#define REG_ACTUATOR1       0x2B

// OPERATION_MODE
#define MODE_STANDBY        0x00
#define MODE_RTP            0x01

volatile bool g_irqFlag = false;

// ---------- Robust I2C helpers ----------
static const uint8_t I2C_RETRIES = 5;

bool writeReg_raw(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DA7280_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool readReg_raw(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(DA7280_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // Repeated START
  delayMicroseconds(50);
  if (Wire.requestFrom((int)DA7280_ADDR, 1, (int)true) != 1) return false;
  val = Wire.read();
  return true;
}

bool writeReg(uint8_t reg, uint8_t val) {
  for (uint8_t i=0; i<I2C_RETRIES; i++) {
    if (writeReg_raw(reg, val)) {
      // Verify write
      uint8_t verify;
      if (readReg_raw(reg, verify) && verify == val) {
        return true;
      }
    }
    delay(2);
  }
  Serial.printf("  [FAIL] Write 0x%02X to reg 0x%02X failed\n", val, reg);
  return false;
}

bool readReg(uint8_t reg, uint8_t &val) {
  for (uint8_t i=0; i<I2C_RETRIES; i++) {
    if (readReg_raw(reg, val)) return true;
    delay(2);
  }
  Serial.printf("  [FAIL] Read reg 0x%02X failed\n", reg);
  return false;
}

// ---------- IRQ ----------
void IRAM_ATTR onIRQ() { g_irqFlag = true; }

// ---------- INT utils ----------
uint8_t clearIntStatusOnce(){ 
  uint8_t s=0; 
  readReg(REG_INT_STATUS, s); 
  return s; 
}

void clearAllLatchedInterrupts(){
  for (int i=0; i<5; ++i){ 
    if (clearIntStatusOnce()==0x00) break; 
    delay(2); 
  }
}

// ---------- TOP_CTL1 helpers ----------
bool setTopCtl1(uint8_t op_mode, bool standby_en, bool seq_start){
  uint8_t v = ((seq_start?1:0)<<4) | ((standby_en?1:0)<<1) | (op_mode & 0x07);
  return writeReg(REG_TOP_CTL1, v);
}

bool enterStandby(){ return setTopCtl1(MODE_STANDBY, true, false); }
bool enterRTP()    { return setTopCtl1(MODE_RTP, false, false); }

// ---------- Status dump ----------
void dumpInterruptStatus(const char* tag){
  uint8_t s=0, m=0;
  readReg(REG_INT_STATUS, s);
  readReg(REG_INT_MASK, m);
  Serial.printf("%s: STATUS=0x%02X, MASK=0x%02X\n", tag, s, m);
}

// ---------- I2C Scanner ----------
void scanI2C() {
  Serial.println("\n=== I2C Bus Scan ===");
  bool found = false;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Device found at 0x%02X", addr);
      if (addr == 0x4A) Serial.print(" (DA7280 primary)");
      if (addr == 0x4B) Serial.print(" (DA7280 alt)");
      Serial.println();
      found = true;
    }
  }
  if (!found) Serial.println("  No devices found!");
  Serial.println();
}

// ---------- Diagnostic registers ----------
void diagRegisters() {
  Serial.println("=== Register Diagnostics ===");
  uint8_t val;
  
  const uint8_t regs[] = {0x00, 0x09, 0x0A, 0x0B, 0x0F, 0x22, 0x23, 0x27, 0x28, 0x2B};
  const char* names[] = {"CHIP_REV", "INT_MASK", "INT_STATUS", "ENABLE", 
                         "RTP_INPUT", "TOP_CTL1", "TOP_CTL2", "CALIB_V2I_H", "CALIB_V2I_L", "ACTUATOR1"};
  
  for (int i=0; i<10; i++) {
    if (readReg(regs[i], val)) {
      Serial.printf("  0x%02X %-12s = 0x%02X", regs[i], names[i], val);
      if (regs[i] == 0x0A && val != 0x00) {
        Serial.print(" [");
        if (val & 0x20) Serial.print("UVLO ");
        if (val & 0x10) Serial.print("OVT ");
        if (val & 0x08) Serial.print("OVD ");
        if (val & 0x04) Serial.print("OC ");
        if (val & 0x02) Serial.print("SEQ ");
        if (val & 0x01) Serial.print("UVLO2");
        Serial.print("]");
      }
      Serial.println();
    } else {
      Serial.printf("  0x%02X %-12s = READ FAILED\n", regs[i], names[i]);
    }
  }
  Serial.println();
}

// ---------- Motor control ----------
bool rampVibrate(uint8_t target, uint16_t hold_ms){
  Serial.println("  Entering standby...");
  writeReg(REG_ENABLE, 0x00);
  enterStandby();
  clearAllLatchedInterrupts();

  Serial.println("  Entering RTP mode...");
  if (!enterRTP()) {
    Serial.println("  [ERROR] Failed to enter RTP mode");
    return false;
  }
  
  Serial.println("  Enabling output...");
  if (!writeReg(REG_ENABLE, 0x01)) {
    Serial.println("  [ERROR] Failed to enable output");
    return false;
  }

  // Verify enable worked
  uint8_t enCheck;
  readReg(REG_ENABLE, enCheck);
  Serial.printf("  ENABLE register readback: 0x%02X\n", enCheck);

  Serial.printf("  Ramping to %d...\n", target);
  uint8_t amp = 12;
  while (amp < target){
    if (!writeReg(REG_RTP_INPUT, amp)) {
      Serial.println("  [ERROR] RTP write failed during ramp");
    }
    delay(35);
    if (g_irqFlag) {
      writeReg(REG_ENABLE, 0x00);
      enterStandby();
      delay(2);
      g_irqFlag = false;
      dumpInterruptStatus("IRQ during ramp");
      return false;
    }
    amp = (uint8_t)min<int>(amp + 8, target);
  }

  Serial.printf("  Holding at %d for %d ms...\n", target, hold_ms);
  writeReg(REG_RTP_INPUT, target);
  uint32_t t0 = millis();
  while ((millis()-t0) < hold_ms){
    if (g_irqFlag) {
      writeReg(REG_ENABLE, 0x00);
      enterStandby();
      delay(2);
      g_irqFlag = false;
      dumpInterruptStatus("IRQ during hold");
      return false;
    }
    delay(5);
  }

  Serial.println("  Stopping...");
  writeReg(REG_ENABLE, 0x00);
  enterStandby();
  delay(2);
  dumpInterruptStatus("Post");
  return true;
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200);
  delay(2000); // Give time to open serial monitor
  
  Serial.println("\n\n=== DA7280 Haptic Driver Diagnostic ===\n");

  // Initialize I2C with explicit pins
  Wire.begin(21, 22);  // SDA=21, SCL=22 (ESP32 default)
  Wire.setClock(100000); // 100 kHz standard mode
  Serial.println("I2C initialized: SDA=21, SCL=22, 100kHz");

  // Scan bus
  scanI2C();

  // Try to detect DA7280 at both addresses
  bool found = false;
  for (uint8_t addr : {DA7280_ADDR_PRIMARY, DA7280_ADDR_ALT}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      DA7280_ADDR = addr;
      Serial.printf("DA7280 detected at 0x%02X\n\n", addr);
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.println("ERROR: No DA7280 found at 0x4A or 0x4B!");
    Serial.println("Check:");
    Serial.println("  - Power supply (3.3V)");
    Serial.println("  - I2C connections (SDA=21, SCL=22)");
    Serial.println("  - Pull-up resistors (4.7k to 3.3V)");
    Serial.println("  - Common ground");
    while(1) delay(1000);
  }

  // Read chip revision
  uint8_t chipRev;
  if (readReg(REG_CHIP_REV, chipRev)) {
    Serial.printf("Chip Revision: 0x%02X\n", chipRev);
  } else {
    Serial.println("WARNING: Cannot read chip revision!");
  }

  // Initial register dump
  diagRegisters();

  // Force standby and reset
  Serial.println("=== Initialization Sequence ===");
  Serial.println("Forcing standby mode...");
  enterStandby();
  delay(10);

  // Force ERM mode
  uint8_t act;
  if (readReg(REG_ACTUATOR1, act)){
    act &= ~0x01; // Clear bit 0 for ERM
    if (writeReg(REG_ACTUATOR1, act)) {
      Serial.printf("ACTUATOR1 set to 0x%02X (ERM mode)\n", act);
    }
  }

  // Setup IRQ pin
  pinMode(PIN_IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), onIRQ, FALLING);
  Serial.println("IRQ pin configured on GPIO27");

  // Check for fault conditions
  uint8_t status;
  if (readReg(REG_INT_STATUS, status)) {
    Serial.printf("Initial INT_STATUS: 0x%02X\n", status);
    if (status & 0x20) Serial.println("  WARNING: UVLO (Under-Voltage) detected!");
    if (status & 0x10) Serial.println("  WARNING: OVT (Over-Temperature) detected!");
    if (status & 0x08) Serial.println("  WARNING: OVD (Over-Voltage) detected!");
    if (status & 0x04) Serial.println("  WARNING: OC (Over-Current) detected - check motor!");
    if (status & 0x01) Serial.println("  WARNING: UVLO2 (Under-Voltage 2) detected!");
    
    if (status != 0x00) {
      Serial.println("\n*** FAULT CONDITION ACTIVE - Chip is protecting itself ***");
      Serial.println("The DA7280 will refuse writes until faults are cleared.\n");
      Serial.println("Try:");
      Serial.println("  1. Disconnect motor and power cycle");
      Serial.println("  2. Check VDD voltage (should be stable 3.3V)");
      Serial.println("  3. Ensure motor is 8-16 ohms ERM type");
      Serial.println("  4. Check all power connections\n");
    }
  }

  // Try to clear status by reading multiple times
  Serial.println("Attempting to clear fault status...");
  for (int i=0; i<10; i++) {
    readReg(REG_INT_STATUS, status);
    if (status == 0x00) {
      Serial.println("Faults cleared!");
      break;
    }
    delay(10);
  }
  
  dumpInterruptStatus("After clear attempt");

  // Unmask all interrupts
  Serial.println("Attempting to unmask interrupts...");
  if (writeReg(REG_INT_MASK, 0x00)) {
    Serial.println("Interrupts unmasked successfully");
  } else {
    Serial.println("WARNING: Failed to unmask interrupts (chip in fault state)");
  }

  // Final register dump
  Serial.println("\n=== Post-Init Register State ===");
  diagRegisters();

  Serial.println("=== Ready ===");
  Serial.println("Commands:");
  Serial.println("  <intensity> <duration>  - Vibrate (e.g., '64 800')");
  Serial.println("  scan                     - Rescan I2C bus");
  Serial.println("  diag                     - Dump registers");
  Serial.println();
}

// ---------- Loop ----------
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
      int spacePos = input.indexOf(' ');
      if (spacePos > 0) {
        duration = input.substring(spacePos+1).toInt();
      }

      if (intensity >= 0 && intensity <= 255 && duration > 0){
        Serial.printf("\n=== RTP Vibrate: %d intensity, %d ms ===\n", intensity, duration);
        bool ok = rampVibrate((uint8_t)intensity, (uint16_t)duration);
        Serial.println(ok ? "=== Done ===\n" : "=== Aborted ===\n");
      } else {
        Serial.println("Invalid input. Example: 64 800");
      }
    }
  }

  // Check for spontaneous IRQs
  if (g_irqFlag) {
    g_irqFlag = false;
    Serial.println("\n! Unexpected IRQ detected !");
    dumpInterruptStatus("Unexpected");
  }
}