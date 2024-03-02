#include <Arduino.h>
#include "BLEDevice.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"

// HID report descriptor for a simple gamepad with 21 buttons
// Adjust this descriptor according to your specific needs
static const uint8_t REPORT_MAP[] = {
  USAGE_PAGE(1), 0x01,  // Generic Desktop Ctrls
  USAGE(1), 0x05,       // Game Pad
  COLLECTION(1), 0x01,  // Application
  REPORT_ID(1), 0x01,   //   Report ID (1)
  USAGE_PAGE(1), 0x09,  // Button
  USAGE_MINIMUM(1), 0x01,
  USAGE_MAXIMUM(1), 0x15,  // 21 Buttons
  LOGICAL_MINIMUM(1), 0x00,
  LOGICAL_MAXIMUM(1), 0x01,
  REPORT_COUNT(1), 0x15,  // 21 buttons
  REPORT_SIZE(1), 0x01,
  HIDINPUT(1), 0x02,      // Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position
  REPORT_COUNT(1), 0x03,  // 3 bits (padding)
  REPORT_SIZE(1), 0x01,
  HIDINPUT(1), 0x03,  // Cnst,Var,Abs,No Wrap,Linear,Preferred State,No Null Position
  END_COLLECTION(0)
};

// Device name
#define DEVICE_NAME "KayO Bluetooth"

// Report ID
#define REPORT_ID 1

BLEHIDDevice* hid;
BLECharacteristic* input;

bool isBleConnected = false;
uint8_t uart_msg []= {0,0,0,0};
bool is_section = true;
bool is_button = false;
int section = 0;
bool is_return = false;
uint8_t lookup[] = {0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };


uint8_t reverse_byte(uint8_t b) {
  return ((lookup[b & 0x0f] << 4) | lookup[b >> 4]);
}
// Structure for gamepad input report
// Adjust the structure according to your specific needs
struct InputReport {
  uint8_t buttons[3];  // 3 bytes to hold the state of 21 buttons
};

// No buttons pressed state
InputReport report = { 0 };
const InputReport NO_BUTTON_PRESSED = { 0, 0, 0 };

class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    isBleConnected = true;
    BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    cccDesc->setNotifications(true);
    Serial.println("OK:CONN");
  };

  void onDisconnect(BLEServer* pServer) {
    isBleConnected = false;
    BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
    cccDesc->setNotifications(false);
    Serial.println("OK:DISC");
  }
};

void setup() {
  Serial.begin(115200);
  BLEDevice::init(DEVICE_NAME);
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacks());

  hid = new BLEHIDDevice(pServer);
  input = hid->inputReport(REPORT_ID);  // Report ID
  hid->manufacturer()->setValue("Maker Community");
  // set USB vendor and product ID
  hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
  // information about HID device: device is not localized, device can be connected
  hid->hidInfo(0x00, 0x02);

  // Setup the security
  BLESecurity* pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

  // Set the HID report map
  hid->reportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
  hid->startServices();

  // Start advertising
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(HID_GAMEPAD);
  pAdvertising->addServiceUUID(hid->hidService()->getUUID());
  pAdvertising->addServiceUUID(hid->deviceInfo()->getUUID());
  pAdvertising->start();

  Serial.println("Waiting for a client connection to notify...");
}

void loop() {

  if (isBleConnected) {
    // Buttons 1, 2, and 3 pressed
    if (Serial.available()) {
      for (int i = 0; i < 4; i++) {
        uart_msg[i] = Serial.read();
        Serial.print(i);
        Serial.println(uart_msg[i], HEX);
      }
      if (uart_msg[0] == 0xFE) {
        if (is_return == true) {
          is_return = false;
          Serial.println("OK:off");
        } else {
          is_return = true;
          Serial.println("OK:on");
        }
      } else if (uart_msg[0] == 0xFF) {
        report.buttons[0] = 0;
        report.buttons[1] = 0;
        report.buttons[2] = 0;
        if (is_return == true) {
          Serial.println("Reset");
        }
      } else if (uart_msg[0] == 0xFD) {
        report.buttons[0] = reverse_byte(uart_msg[1]);
        report.buttons[1] = reverse_byte(uart_msg[2]);
        report.buttons[2] = reverse_byte(uart_msg[3]);
        if (is_return == true) {
          Serial.print("OK1_8:");
          Serial.println(report.buttons[0], HEX);
          Serial.print("OK9_16:");
          Serial.println(report.buttons[1], HEX);
          Serial.print("OK17_21:");
          Serial.print(uart_msg[3], HEX);
          Serial.println(report.buttons[2], HEX);
        }
      }
    }

    input->setValue((uint8_t*)&report, sizeof(report));
    input->notify();
  }

  // Send the input report
  delay(5);  // Delay to simulate button press duration and frequency
}
