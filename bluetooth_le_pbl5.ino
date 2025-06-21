#include <Preferences.h>               // NVS
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>                   //  BLE2902 descriptor
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_AHTX0.h>
#include "ScioSense_ENS160.h"  

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PIR_front    6
#define PIR_backward 5
#define LED_PIN      4
#define LED_PIN_R    3
#define BTN_UP       2
#define BTN_SELECT   1
#define BTN_DOWN     0

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET   -1

Preferences prefs;
Adafruit_AHTX0 aht;
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);
BLECharacteristic *pCharacteristic;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int MAX_CONTACTS = 40;           
int contactCount   = 0;               // how many are stored now
String contacts[MAX_CONTACTS];
String phones  [MAX_CONTACTS];
volatile bool callInProgress = false;
volatile bool callAnswered = false;

String    urcBuffer      = "";

uint8_t indexFlag;
int8_t humidity;
int8_t tempC;
int8_t thresholdTempC = 38;
uint16_t thresholdCO2 = 1600;
bool deviceConnected = false;

const unsigned long DEBOUNCE_MS = 120;
const unsigned long MULTI_PRESS_WINDOW_MS = 1000;

volatile unsigned long lastISRTime = 0;
volatile unsigned long lastPressTime = 0;
volatile uint8_t pressCount = 0;

int selectedIndex = 0;
int lastSelectedIndex = -1;
uint8_t selectPressCount = 0;
unsigned long lastSelectPressTime = 0;

volatile bool upPressed = false;
volatile bool downPressed = false;


//— Hàm lưu danh bạ vào NVS — 
void saveContacts() {
  prefs.begin("contacts", false);
  prefs.putUInt("count", contactCount);
  for (int i = 0; i < contactCount; i++) {
    char key[8]; sprintf(key, "c%d", i);
    prefs.putString(key, contacts[i]);
  }
  prefs.end();
}

//— Hàm đọc danh bạ từ NVS — 
void loadContacts() {
  // mặc định nếu NVS rỗng
  contactCount = 0;
  prefs.begin("contacts", true);
  int stored = prefs.getUInt("count", 0);
  if (stored > 0 && stored <= MAX_CONTACTS) contactCount = stored;
  for (int i = 0; i < contactCount; i++) {
    char key[8]; sprintf(key, "c%d", i);
    contacts[i] = prefs.getString(key, "");
    int dash = contacts[i].lastIndexOf('-');
    if (dash > 0) phones[i] = contacts[i].substring(dash + 2);
  }
  prefs.end();
}

void makeCall(const String &phone) {
  Serial.printf("Calling %s...\n", phone.c_str());
  Serial1.print("ATD" + phone + ";\r\n");
  // check phone state during  a call
  /* 0: the call is active
    1: 
    2:
  */
  display.clearDisplay(); 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 15);
  display.println("Calling...");
  display.display();
  callInProgress = true;
  urcBuffer      = "";
//   //if(oncall){
//   display.setTextSize(1);
//   display.setTextColor(WHITE);
//   display.setCursor(20, 15);
//   display.println("Calling...");
//   display.display();
//   }else if (the call is ended){
//     drawMenu();
//   }

}
String getPhoneByKeyword(const String& keyword) {
  for (int i = 0; i < contactCount; i++) {
    String lower = contacts[i];
    lower.toLowerCase();
    if (lower.indexOf(keyword) >= 0) {
      return phones[i];
    }
  }
  return "";
}

void callDriver() {
  String number = getPhoneByKeyword("driver");
  if (number.length() > 0) {
    makeCall(number);
  } else {
    Serial.println("[Error] Driver contact not found.");
  }
}

void callParent() {
  String number = getPhoneByKeyword("parent");
  if (number.length() > 0) {
    makeCall(number);
  } else {
    Serial.println("[Error] Parent contact not found.");
  }
}

void callDriverAndParent() {
  String driver = getPhoneByKeyword("driver");
  String parent = getPhoneByKeyword("parent");

  if (driver.length() > 0) makeCall(driver);
  delay(3000);  // optional pause between calls
  if (parent.length() > 0) makeCall(parent);

  if (driver.length() == 0 && parent.length() == 0) {
    Serial.println("[Error] No driver or parent contact found.");
  }
}

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *p) override {
    String v = p->getValue();
    int dash = v.lastIndexOf('-');
    if (dash > 0 && contactCount < MAX_CONTACTS) {
      String name  = v.substring(0, dash - 1);
      String phone = v.substring(dash + 2);
      contacts[contactCount] = name + " - " + phone;
      phones[contactCount]   = phone;
      contactCount++;
      saveContacts();
      drawMenu();
      Serial.printf("Appended: %s\n", contacts[contactCount-1].c_str());
    } else {
      Serial.println("List full or format invalid");
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    digitalWrite(LED_PIN, LOW);
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    Serial.println("BLE disconnected");
    s->startAdvertising();
  }
};

void IRAM_ATTR button_select() {
  unsigned long now = millis();
  if (now - lastISRTime > DEBOUNCE_MS) {
    if (selectedIndex != lastSelectedIndex || (now - lastSelectPressTime > MULTI_PRESS_WINDOW_MS)) {
      selectPressCount = 1;
    } else {
      selectPressCount++;
    }
    lastSelectPressTime = now;
    lastSelectedIndex = selectedIndex;
    pressCount++;
    lastPressTime = now;
  }
  lastISRTime = now;
}

void IRAM_ATTR button_up() {
  unsigned long now = millis();
  if (now - lastISRTime > DEBOUNCE_MS) { upPressed = true; lastPressTime = now; }
  lastISRTime = now;
}

void IRAM_ATTR button_down() {
  unsigned long now = millis();
  if (now - lastISRTime > DEBOUNCE_MS) { downPressed = true; lastPressTime = now; }
  lastISRTime = now;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  loadContacts();
  setCpuFrequencyMhz(80);
  pinMode(PIR_front,INPUT);
   pinMode(PIR_backward,INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BTN_UP), button_up, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_SELECT), button_select, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_DOWN), button_down, FALLING);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(0, 15);
  display.println("L42Y");
  display.display();

  delay(4000);  // replace by setup SIM
  //ìf setup done ? drawMenu();

  display.setTextSize(1);
  drawMenu();

  BLEDevice::init("L42Y_BLT");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Hello from Lazy Team");
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE advertising…");

  ens160.begin();
  if (ens160.available()) ens160.setMode(ENS160_OPMODE_STD);
  aht.begin();
}

void loop() {
  static unsigned long lastToggle = 0, lastScan = 0;
  unsigned long now = millis();
  const unsigned long interval = 500;
  handleCallResponse();
  if (deviceConnected && (now - lastScan >= 10000)) {
    readSensor();
    lastScan = now;
    char buffer[30];
    sprintf(buffer, "T:%d - H:%d - C:%d", tempC, humidity, ens160.geteCO2());
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();

  }else if (!deviceConnected) {
    if(digitalRead(PIR_front) || digitalRead(PIR_backward)){
      callDriver();
    } else {
        if(now - lastToggle >= interval){
          lastToggle = now;
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }

        if(now - lastScan >= 20000){
          readSensor();
          if(indexFlag == 1 || indexFlag == 2){
            callDriver();
          }else if (indexFlag == 3){
            callDriverAndParent();
          }
        }
    }
  }


  if (!callInProgress && upPressed) {
    upPressed = false;
    if (digitalRead(BTN_UP) == LOW && contactCount>0) {
      selectedIndex = (selectedIndex - 1 + contactCount) % contactCount;
      drawMenu();
    }
  }

  if (!callInProgress &&  downPressed) {
    downPressed = false;
    if (digitalRead(BTN_DOWN) == LOW && contactCount>0) {
      selectedIndex = (selectedIndex + 1) % contactCount;
      drawMenu();
    }
  }
// Child press 9 times in row to call their parent
  if (pressCount > 0 && (now - lastPressTime > MULTI_PRESS_WINDOW_MS)) {
    pressCount = 0;
    if (selectPressCount >= 9 && contactCount>0) {
      makeCall(phones[selectedIndex]);
      selectPressCount = 0;
    }
  }
}

void drawMenu() {
  const int lineHeight = 10;
  const int visibleLines = SCREEN_HEIGHT / lineHeight; 

  int windowStart = 0;
  if (selectedIndex >= visibleLines) {
    windowStart = selectedIndex - (visibleLines - 1);
    if (windowStart + visibleLines > contactCount) {
      windowStart = max(0, contactCount - visibleLines);
    }
  }

  display.clearDisplay();
  for (int line = 0; line < visibleLines; line++) {
    int idx = windowStart + line;
    if (idx >= contactCount) break;

    if (idx == selectedIndex) {
      display.setTextColor(BLACK, WHITE); 
    } else {
      display.setTextColor(WHITE);
    }
    display.setCursor(0, line * lineHeight);
    display.println(contacts[idx]);
  }
  display.display();
}

void readSensor() {
  sensors_event_t hEvent, tEvent;
  aht.getEvent(&hEvent, &tEvent);
  tempC = (int8_t)tEvent.temperature;
  humidity = (int8_t)hEvent.relative_humidity;

  if (ens160.available()) {
    ens160.set_envdata(tempC, humidity);
    ens160.measure(true);
    ens160.measureRaw(true);
  }

  if (tempC > thresholdTempC) setbit(1); else clearbit(1);
  if (ens160.geteCO2() > thresholdCO2) setbit(0); else clearbit(0);

}
void handleCallResponse() {
  if (!callInProgress) return;

  while (Serial1.available()) {
    char c = Serial1.read();
    urcBuffer += c;

    if (c == '\n') {
      urcBuffer.trim();

      // --- Call answered ---
      if (urcBuffer.startsWith("CONNECT") ||
          urcBuffer.indexOf("VOICE CALL: BEGIN") >= 0 ||
          urcBuffer.indexOf("+CLCC:") >= 0) {
        Serial.println("Call answered.");
        callAnswered = true;

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 10);
        display.println("Call in progress...");
        display.display();
      }

      // --- Call ended ---
      else if (urcBuffer.startsWith("NO CARRIER") ||
               urcBuffer.startsWith("BUSY") ||
               urcBuffer.startsWith("NO ANSWER")) {
        callInProgress = false;

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 10);

        if (callAnswered) {
          display.println("Call ended.");
          Serial.println("Call ended normally.");
        } else {
          display.println("Call failed.");
          Serial.println("Call failed before answer.");
        }

        display.display();
        delay(2000);  // show message for 2 sec
        callAnswered = false;  // reset for next call
        drawMenu();            // back to main menu
      }

      urcBuffer = ""; 
    }
  }
}


void setbit(uint8_t index) {
  indexFlag |= (1 << index);
}
void clearbit(uint8_t index) {
  indexFlag &= ~(1 << index);
}
