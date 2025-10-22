#include <Arduino.h>
#include <NimBLEDevice.h>

#define LED 2

class CtrlCallbacks : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic* ch) {        // no 'override' here
    const std::string &v = ch->getValue();
    if (!v.empty()) digitalWrite(LED, v[0] == '1' ? HIGH : LOW);
  }
};

static CtrlCallbacks ctrlCB;

void setup() {
  pinMode(LED, OUTPUT);

  NimBLEDevice::init("Hey Cutie");
  auto server  = NimBLEDevice::createServer();
  auto service = server->createService("1234");
  auto chr     = service->createCharacteristic(
                   "ABCD",
                   NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

  chr->setCallbacks(&ctrlCB);
  service->start();

  auto adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID("1234");
  adv->start();
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
