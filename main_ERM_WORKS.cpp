#include <Arduino.h>
#include <Wire.h>
#include <Haptic_Driver.h>

Haptic_Driver haptic;          // address fixed to 0x4A by this lib
// const int PIN_INT = 27;     // not needed for this test

static void assertMsg(bool ok, const char* msg) {
  Serial.print(msg); Serial.println(ok ? " OK" : " FAIL");
}

static void printStatus() {
  status_t s = haptic.getIrqStatus();    // status flags
  event_t  e = haptic.getIrqEvent();     // event/irq flags
  diag_status_t d = haptic.getEventDiag(); // diag flags

  Serial.print("STATUS: 0x"); Serial.println((uint8_t)s, HEX);
  Serial.print("EVENT : 0x"); Serial.println((uint8_t)e, HEX);
  Serial.print("DIAG  : 0x"); Serial.println((uint8_t)d, HEX);
  // Clear everything so next read shows fresh flags
  haptic.clearIrq(0xFF);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(21, 22);

  if (!haptic.begin(Wire)) {
    Serial.println("begin() FAIL — check I2C/wiring");
    while (1) {}
  }
  Serial.println("begin() OK");

  // Configure the driver conservatively for your ERM coin
  assertMsg(haptic.setActuatorType(ERM_TYPE), "setActuatorType(ERM)");
  // or: assertMsg(haptic.enableCoinERM(), "enableCoinERM()");

  // Safe limits (tweak if needed)
  assertMsg(haptic.setActuatorNOMVolt(2.7f), "setActuatorNOMVolt(2.7V)");
  assertMsg(haptic.setActuatorABSVolt(3.0f), "setActuatorABSVolt(3.0V)");
  assertMsg(haptic.setActuatorIMAX(0.10f),   "setActuatorIMAX(0.10A)");
  assertMsg(haptic.setActuatorImpedance(26.0f), "setActuatorImpedance(26Ω)");

  // Helpers for ERM
  assertMsg(haptic.enableRapidStop(true), "enableRapidStop(true)");
  assertMsg(haptic.enableAmpPid(true),    "enableAmpPid(true)");
  assertMsg(haptic.enableFreqTrack(false),"enableFreqTrack(false)");

  // Go to RTWM (real-time amplitude control)
  assertMsg(haptic.setOperationMode(RTWM_MODE), "setOperationMode(RTWM)");

  Serial.println("=== Initial RTWM pulse ===");
  haptic.setVibrate(200); delay(200);
  haptic.setVibrate(0);   delay(200);

  printStatus();

  Serial.println("=== RTWM ramp 0..200..0 ===");
  for (int a = 0; a <= 200; a += 10) { haptic.setVibrate(a); delay(50); }
  for (int a = 200; a >= 0; a -= 10) { haptic.setVibrate(a); delay(40); }
  haptic.setVibrate(0);
  printStatus();

  // As a fallback, try a tiny memory pattern (DRO mode play)
  Serial.println("=== Memory play fallback ===");
  assertMsg(haptic.setOperationMode(DRO_MODE), "setOperationMode(DRO)");
  haptic.eraseWaveformMemory(0xFF);     // clear all
  haptic.createHeader(0x01, 0x01);      // sequence 1, 1 frame
  // snippet: ramp, amplitude=5, timeBase=3 (adjust amplitude up if weak)
  assertMsg(haptic.addSnippet(RAMP, 5, 3), "addSnippet(RAMP,5,3)");
  // frame: (seqId, startSnippet, endSnippet)
  (void)haptic.addFrame(0x01, 0x01, 0x01);
  assertMsg(haptic.playFromMemory(true), "playFromMemory(true)");

  delay(400);
  printStatus();

  Serial.println("=== TEST DONE ===");
}

void loop() {
  // stay in RTWM; drive hard for 8 seconds so you can test
  haptic.setVibrate(255);   // strongest
  delay(8000);
  haptic.setVibrate(0);
  delay(2000);
  Serial.println("Done with long test");

    // Switch to DRO and play a short built-in pattern
  haptic.setOperationMode(DRO_MODE);
  haptic.eraseWaveformMemory(0xFF);

  // Sequence 1 with 1 frame
  haptic.createHeader(0x01, 0x01);

  // One snippet: ramp, higher amplitude, moderate timebase
  haptic.addSnippet(RAMP, 15, 3);     // try amplitudes 10..25 if needed

  // Frame: (sequenceId, startSnippetIdx, endSnippetIdx)
  haptic.addFrame(0x01, 0x01, 0x01);

  // Loop once and play
  haptic.setSeqControl(0x01, 0x01);
  haptic.playFromMemory(true);

  delay(1500);
  Serial.println("Done with DRO sequence test");
}


