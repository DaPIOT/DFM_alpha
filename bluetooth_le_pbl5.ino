#include <Preferences.h>               // NVS
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>                   // BLE2902 descriptor
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_AHTX0.h>
#include "ScioSense_ENS160.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PIR_FRONT    6
#define PIR_BACK     5
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
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BLECharacteristic *pCharacteristic;

// Display State Machine
enum DisplayState : uint8_t { MENU_STATE = 0, CALL_STATE = 1, WAIT_STATE = 2 };
volatile DisplayState displayState = MENU_STATE;

// Contacts
static const int MAX_CONTACTS = 40;
int contactCount = 0;
String contacts[MAX_CONTACTS];
String phones[MAX_CONTACTS];

// Call flags
volatile bool callInProgress = false;
volatile bool callAnswered   = false;
String urcBuffer;

// Driver response flags
volatile bool driverResponded    = false;
volatile bool reminderSent       = false;
unsigned long driverDisconnectTs = 0;
static const unsigned long ACK_WINDOW_MS = 2UL * 60UL * 1000UL;

// Sensor thresholds
uint8_t indexFlag;
int8_t humidity;
int8_t tempC;
int8_t thresholdTempC = 38;
uint16_t thresholdCO2 = 1600;
bool deviceConnected = false;

// Button debounce & multi-tap
typedef unsigned long ul;
static const ul DEBOUNCE_MS = 120;
static const ul MULTI_PRESS_WINDOW_MS = 1000;
volatile ul lastISRTime = 0;
volatile ul lastPressTime = 0;
volatile uint8_t pressCount = 0;
int selectedIndex = 0;
int lastSelectedIndex = -1;
volatile uint8_t selectPressCount = 0;
volatile ul lastSelectPressTime = 0;
volatile bool upPressed = false;
volatile bool downPressed = false;

// Forward declarations
void drawMenu();
void readSensor();
void makeCall(const String &phone);
void handleCallResponse();
void handleURCs();

// ----------------------------------
// NVS: Save & Load Contacts
void saveContacts() {
  prefs.begin("contacts", false);
  prefs.putUInt("count", contactCount);
  for (int i = 0; i < contactCount; ++i) {
    char key[8]; sprintf(key, "c%d", i);
    prefs.putString(key, contacts[i]);
  }
  prefs.end();
}

void loadContacts() {
  prefs.begin("contacts", true);
  contactCount = prefs.getUInt("count", 0);
  contactCount = min(contactCount, MAX_CONTACTS);
  for (int i = 0; i < contactCount; ++i) {
    char key[8]; sprintf(key, "c%d", i);
    contacts[i] = prefs.getString(key, "");
    int dash = contacts[i].lastIndexOf('-');
    if (dash > 0) phones[i] = contacts[i].substring(dash + 2);
  }
  prefs.end();
}

// ----------------------------------
// Call & Lookup
void makeCall(const String &phone) {
  Serial.printf("Calling %s...\n", phone.c_str());
  Serial1.print("ATD" + phone + ";\r\n");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 15);
  display.println("Calling...");
  display.display();
  callInProgress = true;
  displayState   = CALL_STATE;
  urcBuffer      = "";
}

String getPhoneByKeyword(const String &keyword) {
  for (int i = 0; i < contactCount; ++i) {
    String lower = contacts[i]; lower.toLowerCase();
    if (lower.indexOf(keyword) >= 0) return phones[i];
  }
  return "";
}

void callDriver() {
  String num = getPhoneByKeyword("driver");
  if (num.length()) makeCall(num);
  else Serial.println("[Error] driver not found");
}
void callParent() {
  String num = getPhoneByKeyword("parent");
  if (num.length()) makeCall(num);
  else Serial.println("[Error] parent not found");
}
void callDriverAndParent() {
  String d = getPhoneByKeyword("driver"), p = getPhoneByKeyword("parent");
  if (d.length()) makeCall(d); delay(3000);
  if (p.length()) makeCall(p);
}

// ----------------------------------
// Reminder SMS
void sendForgottenReminder() {
  String drv = getPhoneByKeyword("driver");
  if (drv.length()) {
    Serial.printf("[Reminder] SMS to %s\n", drv.c_str());
    Serial1.print("AT+CMGF=1\r\n"); delay(200);
    Serial1.print("AT+CMGS=\"" + drv + "\"\r\n"); delay(200);
    Serial1.print("Driver, did you forget someone in the car?\x1A\r\n");
  }
}

// ----------------------------------
// Handle SMS & URCs
void handleURCs() {
  while (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    if (line.startsWith("+CMT:")) {
      String msg = Serial1.readStringUntil('\n');
      msg.trim(); msg.toLowerCase();
      if (msg == "y" || msg == "ok") {
        driverResponded = true;
        Serial.println("[Info] driver responded");
      }
    }
  }
}

// ----------------------------------
// BLE Callbacks
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *p) override {
    String v = p->getValue(); int dash = v.lastIndexOf('-');
    if (dash > 0 && contactCount < MAX_CONTACTS) {
      String name = v.substring(0, dash - 1);
      String phone= v.substring(dash + 2);
      contacts[contactCount] = name + " - " + phone;
      phones[contactCount]   = phone;
      contactCount++;
      saveContacts(); drawMenu();
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected   = true;
    driverResponded  = false;
    reminderSent     = false;
    digitalWrite(LED_PIN, LOW);
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    Serial.println("BLE disconnected");
    s->startAdvertising();
    if (!reminderSent) {
      driverDisconnectTs = millis();
      reminderSent = true;
      sendForgottenReminder();
    }
  }
};

// ----------------------------------
// Button ISRs
void IRAM_ATTR button_select() {
  ul now = millis();
  if (now - lastISRTime > DEBOUNCE_MS) {
    if (now - lastSelectPressTime > MULTI_PRESS_WINDOW_MS) selectPressCount=1;
    else selectPressCount++;
    lastSelectPressTime=now; lastSelectedIndex=selectedIndex;
    pressCount++; lastPressTime=now;
  }
  lastISRTime=now;
}
void IRAM_ATTR button_up()   { upPressed   = true; }
void IRAM_ATTR button_down() { downPressed = true; }

// ----------------------------------
// Setup & Loop
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 20, 21);
  loadContacts();

  pinMode(PIR_FRONT, INPUT);
  pinMode(PIR_BACK, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_R, OUTPUT);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_SELECT,INPUT_PULLUP);
  pinMode(BTN_DOWN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_UP),button_up,   FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_SELECT),button_select,FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_DOWN),button_down,FALLING);

  display.begin(SSD1306_SWITCHCAPVCC,0x3C);
  display.setRotation(2);
  display.clearDisplay();
  display.setTextSize(4);
  display.setTextColor(WHITE);
  display.setCursor(0,15);
  display.println("L42Y");
  display.display();
  delay(2000); drawMenu();

  BLEDevice::init("L42Y_BLT");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ|
    BLECharacteristic::PROPERTY_WRITE|
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Hello from Lazy Team");
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE advertising...");

  ens160.begin(); if (ens160.available()) ens160.setMode(ENS160_OPMODE_STD);
  aht.begin();
}

void loop() {
  static unsigned long lastBleNotify = 0, lastSensorScan = 0, lastToggle=0;
  ul now = millis();

  handleCallResponse();
  handleURCs();

  // 1) BLE-connected: sensor notify
  if (deviceConnected && now - lastBleNotify >= 10000) {
    lastBleNotify = now;
    readSensor();
    char buf[32]; sprintf(buf,"T:%d H:%d C:%d",tempC,humidity,ens160.geteCO2());
    pCharacteristic->setValue(buf); pCharacteristic->notify();
  }

  // 2) BLE-disconnected → send reminder once
  if (!deviceConnected && !reminderSent) {
    driverDisconnectTs = now;
    reminderSent = true;
    sendForgottenReminder();
  }

  // 3) If reminder sent & driver responded → wait/sleep
  if (reminderSent && driverResponded && now - driverDisconnectTs<ACK_WINDOW_MS) {
    display.clearDisplay(); display.setTextSize(1);
    display.setCursor(0,10); display.println("Driver responded");
    display.display();
    esp_sleep_enable_timer_wakeup(5UL*1000UL*1000UL);
    esp_light_sleep_start();
    return;
  }

  // 4) Disconnected & no response → normal PIR/sensor
  if (!deviceConnected && !driverResponded) {
    if (digitalRead(PIR_FRONT)||digitalRead(PIR_BACK)) callDriver();
    else {
      if (now - lastToggle >= 500) { lastToggle=now; digitalWrite(LED_PIN,!digitalRead(LED_PIN)); }
      if (now - lastSensorScan >= 20000) {
        lastSensorScan = now;
        readSensor();
        if (indexFlag & 0x02) callDriver();
        else if (indexFlag & 0x04) callDriverAndParent();
      }
    }
  }

  // 5) Button navigation & child multi-tap
  if (!callInProgress && upPressed)    { upPressed=false;
    if (digitalRead(BTN_UP)==LOW&&contactCount) { selectedIndex=(selectedIndex-1+contactCount)%contactCount; drawMenu(); }}
  if (!callInProgress && downPressed)  { downPressed=false;
    if (digitalRead(BTN_DOWN)==LOW&&contactCount) { selectedIndex=(selectedIndex+1)%contactCount; drawMenu(); }}
  ul elapsed = now - lastPressTime;
  if (pressCount>0 && elapsed > MULTI_PRESS_WINDOW_MS) {
    pressCount=0;
    if (selectPressCount>=9 && contactCount) {
      makeCall(phones[selectedIndex]);
    }
    selectPressCount=0;
  }
}

// ----------------------------------
// Draw Menu (only in MENU_STATE)
void drawMenu() {
  if (displayState != MENU_STATE) return;
  const int lh = 10, lines=SCREEN_HEIGHT/lh;
  int winStart = max(0, selectedIndex - (lines-1));
  display.clearDisplay();
  for (int i=0;i<lines;i++) {
    int idx=winStart+i; if (idx>=contactCount) break;
    if (idx==selectedIndex) display.setTextColor(BLACK, WHITE);
    else display.setTextColor(WHITE);
    display.setCursor(0, i*lh);
    display.println(contacts[idx]);
  }
  display.display();
}

// ----------------------------------
// Sensor Read & Flagging
void readSensor() {
  sensors_event_t h,t; aht.getEvent(&h,&t);
  tempC = (int8_t)t.temperature;
  humidity = (int8_t)h.relative_humidity;
  if (ens160.available()) {
    ens160.set_envdata(tempC,humidity);
    ens160.measure(true); ens160.measureRaw(true);
  }
  indexFlag = 0;
  if (tempC>thresholdTempC) indexFlag |= 0x02;
  if (ens160.geteCO2()>thresholdCO2) indexFlag |= 0x04;
}

// ----------------------------------
// Call Response Handling
void handleCallResponse() {
  if (!callInProgress) return;
  while (Serial1.available()) {
    char c = Serial1.read(); urcBuffer+=c;
    if (c=='\n') {
      urcBuffer.trim();
      if (urcBuffer.startsWith("CONNECT")||urcBuffer.indexOf("VOICE CALL")>=0||urcBuffer.indexOf("+CLCC:")>=0) {
        callAnswered = true;
        display.clearDisplay(); display.setCursor(0,10);
        display.println("Call in progress..."); display.display();
      } else if (urcBuffer.startsWith("NO CARRIER")||urcBuffer.startsWith("BUSY")||urcBuffer.startsWith("NO ANSWER")) {
        callInProgress=false;
        display.clearDisplay(); display.setCursor(0,10);
        display.println(callAnswered?"Call ended.":"Call failed."); display.display();
        delay(2000);
        callAnswered=false;
        displayState=MENU_STATE; drawMenu();
      }
      urcBuffer="";
    }
  }
}


void setbit(uint8_t index) {
  indexFlag |= (1 << index);
}
void clearbit(uint8_t index) {
  indexFlag &= ~(1 << index);
}
