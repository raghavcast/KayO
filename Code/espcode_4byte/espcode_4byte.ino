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
uint8_t uart_msg = 0;
bool is_section = true;
bool is_button = false;
int section = 0;
bool is_return = false;
int lookup[16] = {0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf };
int lookup_id[4] = {0x0, 0x2, 0x1, 0x3};

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
      uart_msg = Serial.read();
      if (uart_msg == '?') {
        if (is_return == true) {
          is_return = false;
          Serial.println("OK:off");
        } else {
          is_return = true;
          Serial.println("OK:on");
        }
      } else if (uart_msg == '/') {
        report.buttons[0] = 0;
        report.buttons[1] = 0;
        report.buttons[2] = 0;
        if (is_return == true) {
          Serial.println("Reset");
        }
      } else if ((uart_msg & 0x3) == 0) {
        report.buttons[0] = (report.buttons[0] & ~0x3f) | (uart_msg >> 2);
        if (is_return == true) {
          Serial.print("OK1_6:");
          Serial.println(report.buttons[0], HEX);
        }
      } else if ((uart_msg & 0x3) == 1) {
        report.buttons[0] = (report.buttons[0] & ~0xc0) | ((uart_msg >> 2) << 6);
        report.buttons[1] = (report.buttons[1] & ~0xf) | (uart_msg >> 4);
        if (is_return == true) {
          Serial.print("OK7_12:");
          Serial.println((report.buttons[0] >> 6 | ((report.buttons[1] & 0x3f) << 2)), HEX);
        }
      } else if ((uart_msg & 0x3) == 2) {
        report.buttons[1] = (report.buttons[1] & ~0xf0) | ((uart_msg >> 2) << 4);
        report.buttons[2] = (report.buttons[2] & ~0x3) | (uart_msg >> 6);
        if (is_return == true) {
          Serial.print("OK13_18:");
          Serial.println((report.buttons[1] >> 4 | (report.buttons[2] & 0x3) << 4), HEX);
        }
      } else if ((uart_msg & 0x3) == 3 && uart_msg != '?') {
        report.buttons[2] = (report.buttons[2] & ~0x1c) | (uart_msg & 0x1c);
        if (is_return == true) {
          Serial.print("OK18_21:");
          Serial.println(report.buttons[2] >> 2, HEX);
        }
      }

      uart_msg = 0;
    }

    input->setValue((uint8_t*)&report, sizeof(report));
    input->notify();
  }

  // Send the input report
  delay(5);  // Delay to simulate button press duration and frequency
}
