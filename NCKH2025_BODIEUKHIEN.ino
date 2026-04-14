/*
  ESP32 Controller for Autonomous Robot (Tank Control)
  - Hardware: ESP32 Dev Kit V1 + NRF24L01 + Joystick
  - Explicit SPI Pin definitions for VSPI
  - UPDATED: Added CALIBRATE_MAG (Command 8) via Triple Click
*/

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <OneButton.h>

// ==== SPI PIN CONFIG (ESP32 VSPI Default) ====
#define PIN_SPI_SCK   18
#define PIN_SPI_MISO  19
#define PIN_SPI_MOSI  23

// NRF Control Pins
#define PIN_CE    4
#define PIN_CSN   5

// Joystick & LED Pins
#define PIN_VRX   34  // ADC1
#define PIN_VRY   35  // ADC1
#define PIN_SW    32  // Joystick button
#define PIN_LED   2   // ACK LED (Dùng LED Onboard GPIO 2)

// Threshold
#define JOY_CENTER    2048
#define JOY_DEADZONE  400 

// Radio Addresses (Must match Robot)
const uint8_t ADDRESS_ROBOT[5] = {'0','0','0','0','1'}; 
const uint8_t ADDRESS_CTL[5]   = {'0','0','0','0','2'}; 

// Commands
enum Command : uint8_t {
  STOP = 0,
  FORWARD = 2,      // Bạn đã đảo chiều (ok giữ nguyên)
  BACKWARD = 1,     // Bạn đã đảo chiều (ok giữ nguyên)
  TURN_LEFT = 3,
  TURN_RIGHT = 4,
  RTH_ACTIVATE = 5,
  SET_HOME = 6,
  CLEAR_HOME = 7,
  CALIBRATE_MAG = 8 // <--- QUAN TRỌNG: Đã thêm lệnh số 8
};

// Objects
RF24 radio(PIN_CE, PIN_CSN);
OneButton btn(PIN_SW, true); 

// State
bool rthActive = false;
unsigned long lastLedTime = 0;

// ==== HELPER ====
void sendCommand(uint8_t cmd) {
  bool report = radio.write(&cmd, sizeof(cmd));
  if (report) {
    digitalWrite(PIN_LED, HIGH);
    lastLedTime = millis();
  }
}

// Button Callbacks
void handleClick() {
  if (!rthActive) {
    Serial.println("BTN: Activate RTH");
    sendCommand(RTH_ACTIVATE);
    rthActive = true;
  } else {
    Serial.println("BTN: Cancel RTH");
    sendCommand(STOP);
    rthActive = false;
  }
}

void handleDoubleClick() {
  Serial.println("BTN: Clear Home");
  sendCommand(CLEAR_HOME);
  // Nháy 3 lần
  for(int i=0; i<3; i++) { digitalWrite(PIN_LED, HIGH); delay(50); digitalWrite(PIN_LED, LOW); delay(50); }
}

void handleLongPress() {
  Serial.println("BTN: Set Home");
  sendCommand(SET_HOME);
  // Sáng dài
  digitalWrite(PIN_LED, HIGH); delay(500); digitalWrite(PIN_LED, LOW);
}

// --- HÀM MỚI: XỬ LÝ NHẤN 3 LẦN ---
void handleMultiClick() {
  if (btn.getNumberClicks() == 3) {
    Serial.println("BTN: SEND CALIBRATE MAG (CMD 8)");
    sendCommand(CALIBRATE_MAG);
    
    // Nháy đèn LED liên tục cảnh báo
    for(int i=0; i<10; i++) {
      digitalWrite(PIN_LED, HIGH); delay(50);
      digitalWrite(PIN_LED, LOW); delay(50);
    }
  }
}

// ==== SETUP ====
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  btn.attachClick(handleClick);
  btn.attachDoubleClick(handleDoubleClick);
  btn.attachLongPressStart(handleLongPress);
  
  // --- QUAN TRỌNG: Đăng ký hàm nhấn 3 lần ---
  btn.attachMultiClick(handleMultiClick); 
  btn.setPressTicks(1000);

  // SPI Bus
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_CSN);

  if (!radio.begin()) {
    Serial.println(F("NRF24 Hardware Error!"));
    while (1) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED)); 
      delay(200);
    }
  }

  radio.setChannel(108);         
  radio.setDataRate(RF24_1MBPS); 
  radio.setPALevel(RF24_PA_HIGH); 
  radio.setRetries(5, 15);       
  radio.setAutoAck(true);         
  radio.enableDynamicPayloads();
  
  radio.openWritingPipe(ADDRESS_ROBOT);
  radio.openReadingPipe(1, ADDRESS_CTL);
  radio.stopListening(); 

  Serial.println(F("Controller Ready (Added Calib CMD 8)"));
}

// ==== LOOP ====
void loop() {
  btn.tick(); // Lắng nghe nút bấm

  int rawX = analogRead(PIN_VRX);
  int rawY = analogRead(PIN_VRY);
  int valX = rawX - JOY_CENTER;
  int valY = rawY - JOY_CENTER;

  Command cmdToSend = STOP;
  bool manualInput = false;

  // Logic Tank Control
  if (abs(valY) > JOY_DEADZONE) {
    manualInput = true;
    if (valY < -JOY_DEADZONE) cmdToSend = FORWARD; 
    else cmdToSend = BACKWARD; 
  } 
  else if (abs(valX) > JOY_DEADZONE) {
    manualInput = true;
    if (valX < -JOY_DEADZONE) cmdToSend = TURN_LEFT;
    else cmdToSend = TURN_RIGHT;
  }

  if (manualInput && rthActive) {
    rthActive = false; 
    Serial.println("Manual Override -> RTH Cancelled");
  }

  static unsigned long lastTx = 0;
  if (!rthActive && millis() - lastTx > 50) {
    sendCommand(cmdToSend);
    
    // Debug in ra hướng
    if (cmdToSend != STOP) {
      if(cmdToSend == FORWARD) Serial.println("TIEN");
      else if(cmdToSend == BACKWARD) Serial.println("LUI");
      else if(cmdToSend == TURN_LEFT) Serial.println("TRAI");
      else if(cmdToSend == TURN_RIGHT) Serial.println("PHAI");
    }
    
    lastTx = millis();
  }

  if (millis() - lastLedTime > 30) {
    digitalWrite(PIN_LED, LOW);
  }
  
  delay(5);
}