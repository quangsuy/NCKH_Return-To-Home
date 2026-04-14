/*
  src/main.cpp

  ESP32-S3 robot firmware (FreeRTOS tasks) with improvements:
   - Single I2C bus (Wire) on SDA=21, SCL=15
   - Moving-average filters for sonar distances and circular moving-average for heading
   - Improved RTH avoidance logic (attempts, checks before returning to navigate)
   - prefs (NVS) protected by prefsMutex
   - radio.read(buf, len) fix; RF addresses 5 bytes
   - PID dt cap + anti-windup

  Note: All sensors and display share the same I2C bus Wire (SDA=21, SCL=15).
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Preferences.h>
#include <math.h>
#include <esp_task_wdt.h>
#include <QMC5883LCompass.h>
#include <float.h>

// ==== PIN / CONSTANTS ====
#define I2C_SDA         21
#define I2C_SCL         15

#define MOTOR_IN1       13
#define MOTOR_IN2       14
#define MOTOR_IN3       12
#define MOTOR_IN4       10
#define ENA_PIN         8
#define ENB_PIN         11
#define ENA_CHANNEL     0
#define ENB_CHANNEL     1
#define PWM_FREQ        5000
#define PWM_RESOLUTION  8
#define MAX_SPEED       255
#define BASE_SPEED      180

#define TRIG_FRONT      5
#define ECHO_FRONT      7
#define TRIG_BACK       4
#define ECHO_BACK       6
#define TRIG_LEFT       16
#define ECHO_LEFT       17
#define TRIG_RIGHT      18
#define ECHO_RIGHT      19
#define MAX_DISTANCE_CM 200

#define GPS_RX          2
#define GPS_TX          1

// Radio / SPI pins (adjusted for ESP32-S3 safe pins; CSN kept as requested)
#define CE_PIN          9
#define CSN_PIN         3
#define SPI_SCK         36
#define SPI_MISO        37
#define SPI_MOSI        35

// UI
#define SCREEN_SWITCH_INTERVAL 5000

// Failsafe
#define FAILSAFE_TIMEOUT 500UL  // ms
#define FAILSAFE_RTH_DISTANCE_METERS 10.0  // meters

// Watchdog (seconds)
#define TASK_WDT_TIMEOUT_S 30

// Radio addresses (5-byte) - must match controller
const uint8_t ADDRESS_ROBOT[5] = {'0','0','0','0','1'};
const uint8_t ADDRESS_CTL[5]   = {'0','0','0','0','2'};

// ==== OBJECTS ====
Adafruit_SSD1306 display(128, 64, &Wire, -1);
MPU9250_asukiaaa mpu;        // use accel+gyro only
QMC5883LCompass qmc;        // GY-271
TinyGPSPlus gps;
RF24 radio(CE_PIN, CSN_PIN);
Preferences prefs;

// ==== TYPES & SHARED DATA ====
enum RobotState { MANUAL, RTH_NAVIGATE, RTH_AVOID, RTH_ARRIVED };

enum Command : uint8_t {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  TURN_LEFT = 3,
  TURN_RIGHT = 4,
  RTH_ACTIVATE = 5,
  // controller uses 6 and 7 for SET_HOME/CLEAR_HOME
  CALIBRATE_MAG = 8  // <--- BẠN ĐANG THIẾU DÒNG NÀY
};

enum HomeOp : uint8_t { HOME_SAVE = 0, HOME_CLEAR = 1 };

struct SensorData {
  float distFront;
  float distBack;
  float distLeft;
  float distRight;
  float lat;
  float lon;
  float heading;
  int sats;
  unsigned long gpsAge;
};

struct HomeSave {
  HomeOp op;
  float lat;
  float lon;
};

// Shared state
static SensorData sensorData;
static double homeLat = 0.0;
static double homeLon = 0.0;
static bool homeValid = false;

// Magnetometer calibration (persisted)
static float magX_off = 0.0f, magY_off = 0.0f, magZ_off = 0.0f;
static float magScaleX = 1.0f, magScaleY = 1.0f, magScaleZ = 1.0f;

// PID
struct PID_Controller {
  double Kp, Ki, Kd;
  double setpoint, input, output;
  double integral, lastError;
  unsigned long lastTime;
};
static PID_Controller headingPID;

// State variables
static RobotState currentState = MANUAL;
static Command receivedCommand = STOP;

// FreeRTOS primitives
static QueueHandle_t commandQueue = NULL;
static QueueHandle_t storageQueue = NULL;
static SemaphoreHandle_t sensorMutex = NULL;
static SemaphoreHandle_t radioMutex = NULL;
static SemaphoreHandle_t prefsMutex = NULL; // protects Preferences (NVS)
// BIẾN QUẢN LÝ TASK HANDLE
static TaskHandle_t hControl = NULL;
static TaskHandle_t hRF = NULL;
static TaskHandle_t hSensor = NULL; 


// Timing / misc
static volatile unsigned long lastRadioTime = 0;
static int currentSonar = 0;
static unsigned long lastRthAction = 0;
static volatile unsigned long storageMessageUntil = 0;
static int currentScreen = 0;

// Avoidance improved trackers
static int avoidAttempts = 0;
static const int MAX_AVOID_ATTEMPTS = 3;
static unsigned long avoidClearCheckStart = 0;
static const unsigned long AVOID_CLEAR_CHECK_MS = 800; // require clear front for this duration before resume

// ===== Filter utilities =====
// Simple moving average for distances
template<int N>
struct MovingAverage {
  float buf[N];
  int idx = 0;
  int cnt = 0;
  float sum = 0.0f;
  void add(float v) {
    if (cnt < N) {
      buf[idx] = v; sum += v; cnt++;
    } else {
      sum -= buf[idx];
      buf[idx] = v;
      sum += v;
    }
    idx = (idx + 1) % N;
  }
  float get() const { return (cnt == 0) ? 0.0f : (sum / cnt); }
};

// Circular moving-average for heading: average on unit circle
template<int N>
struct HeadingAverage {
  float angBuf[N];
  int idx = 0;
  int cnt = 0;
  float sumX = 0.0f, sumY = 0.0f;
  void add(float degrees) {
    float rad = degrees * PI / 180.0f;
    float cx = cos(rad), sy = sin(rad);
    if (cnt < N) {
      angBuf[idx] = degrees;
      sumX += cx; sumY += sy;
      cnt++;
    } else {
      // remove oldest
      float oldRad = angBuf[idx] * PI / 180.0f;
      sumX -= cos(oldRad); sumY -= sin(oldRad);
      angBuf[idx] = degrees;
      sumX += cx; sumY += sy;
    }
    idx = (idx + 1) % N;
  }
  float get() const {
    if (cnt == 0) return 0.0f;
    float ax = atan2(sumY, sumX) * 180.0f / PI;
    if (ax < 0) ax += 360.0f;
    return ax;
  }
};

static MovingAverage<6> maFront, maBack, maLeft, maRight;
static HeadingAverage<8> haHeading;

// ===== Prototypes =====
long readUltrasonicCM(int trigPin, int echoPin);
void setupPID();
double computePID(PID_Controller* pid);
void controlMotorsPWM(int leftSpeed, int rightSpeed);
void move(Command cmd, int speed = BASE_SPEED);

void TaskSensor(void* pvParameters);
void TaskRF(void* pvParameters);
void TaskControl(void* pvParameters);
void TaskUI(void* pvParameters);
void TaskStorage(void* pvParameters);
void TaskConsole(void* pvParameters);

void loadMagCalibration();
void saveMagCalibrationToPrefs(float xoff, float yoff, float zoff, float sx, float sy, float sz);
void runMagCalibration(unsigned long durationMs = 20000);
void clearMagCalibration();

// ===== Implementation =====

long readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout (~4.3 m)
  long distance = duration / 29 / 2;
  if (distance == 0 || distance > MAX_DISTANCE_CM) return MAX_DISTANCE_CM;
  return distance;
}

void setupPID() {
  headingPID.Kp = 2.5;
  headingPID.Ki = 0.05;
  headingPID.Kd = 0.5;
  headingPID.integral = 0;
  headingPID.lastError = 0;
  headingPID.lastTime = millis();
  headingPID.output = 0;
  headingPID.setpoint = 0;
}

double computePID(PID_Controller* pid) {
  unsigned long now = millis();
  double dt = (now - pid->lastTime) / 1000.0;
  if (dt <= 0.0) return pid->output;
  if (dt > 0.5) dt = 0.5;
  double error = pid->setpoint - pid->input;
  pid->integral += error * dt;
  const double INTEGRAL_MAX = 1000.0;
  if (pid->integral > INTEGRAL_MAX) pid->integral = INTEGRAL_MAX;
  if (pid->integral < -INTEGRAL_MAX) pid->integral = -INTEGRAL_MAX;
  double derivative = (error - pid->lastError) / dt;
  double out = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
  if (out > MAX_SPEED) out = MAX_SPEED;
  if (out < -MAX_SPEED) out = -MAX_SPEED;
  if ((out >= MAX_SPEED && error > 0) || (out <= -MAX_SPEED && error < 0)) {
    pid->integral -= error * dt;
  }
  pid->lastError = error;
  pid->lastTime = now;
  pid->output = out;
  return out;
}

// Motor control
void controlMotorsPWM(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(ENA_CHANNEL, leftSpeed);
  } else {
    digitalWrite(MOTOR_IN1, LOW); digitalWrite(MOTOR_IN2, HIGH);
    ledcWrite(ENA_CHANNEL, abs(leftSpeed));
  }
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);
    ledcWrite(ENB_CHANNEL, rightSpeed);
  } else {
    digitalWrite(MOTOR_IN3, LOW); digitalWrite(MOTOR_IN4, HIGH);
    ledcWrite(ENB_CHANNEL, abs(rightSpeed));
  }
}

void move(Command cmd, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  switch (cmd) {
    case FORWARD: controlMotorsPWM(speed, speed); break;
    case BACKWARD: controlMotorsPWM(-speed, -speed); break;
    case TURN_LEFT: controlMotorsPWM(-speed, speed); break;
    case TURN_RIGHT: controlMotorsPWM(speed, -speed); break;
    case STOP:
    default: controlMotorsPWM(0, 0); break;
  }
}

// ===== Mag calibration persistence / utilities =====
void loadMagCalibration() {
  if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (prefs.begin("mag", true)) {
      magX_off = prefs.getFloat("magXoff", 0.0f);
      magY_off = prefs.getFloat("magYoff", 0.0f);
      magZ_off = prefs.getFloat("magZoff", 0.0f);
      magScaleX = prefs.getFloat("magSx", 1.0f);
      magScaleY = prefs.getFloat("magSy", 1.0f);
      magScaleZ = prefs.getFloat("magSz", 1.0f);
      prefs.end();
    }
    xSemaphoreGive(prefsMutex);
  }
}

void saveMagCalibrationToPrefs(float xoff, float yoff, float zoff, float sx, float sy, float sz) {
  if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (prefs.begin("mag", false)) {
      prefs.putFloat("magXoff", xoff);
      prefs.putFloat("magYoff", yoff);
      prefs.putFloat("magZoff", zoff);
      prefs.putFloat("magSx", sx);
      prefs.putFloat("magSy", sy);
      prefs.putFloat("magSz", sz);
      prefs.end();
    }
    xSemaphoreGive(prefsMutex);
  }
}

void clearMagCalibration() {
  if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (prefs.begin("mag", false)) {
      prefs.remove("magXoff"); prefs.remove("magYoff"); prefs.remove("magZoff");
      prefs.remove("magSx"); prefs.remove("magSy"); prefs.remove("magSz");
      prefs.end();
    }
    xSemaphoreGive(prefsMutex);
  }
  magX_off = magY_off = magZ_off = 0.0f;
  magScaleX = magScaleY = magScaleZ = 1.0f;
}

// Blocking calibration routine (called from TaskConsole)
// Hàm này sẽ chặn (block) người gọi nó trong 20 giây
void runMagCalibration(unsigned long durationMs) {
  Serial.println("=== BAT DAU HIEU CHUAN (20s) ===");
  move(STOP); // Dừng động cơ
  
  // Lưu quyền cũ và Nâng quyền lên mức 5 (Để ưu tiên chiếm I2C)
  UBaseType_t oldPrio = uxTaskPriorityGet(NULL);
  vTaskPrioritySet(NULL, 5); 

  // Reset Watchdog cho Task hiện tại
  esp_task_wdt_reset();

  float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
  float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;
  unsigned long start = millis();
  
  while (millis() - start < durationMs) {
    // 1. Cho Watchdog ăn (QUAN TRỌNG)
    esp_task_wdt_reset(); 

    // 2. Đọc cảm biến
    qmc.read();
    float mx = (float)qmc.getX();
    float my = (float)qmc.getY();
    float mz = (float)qmc.getZ();

    if (mx < minX) minX = mx; if (my < minY) minY = my; if (mz < minZ) minZ = mz;
    if (mx > maxX) maxX = mx; if (my > maxY) maxY = my; if (mz > maxZ) maxZ = mz;

    // 3. Nghỉ 20ms để TaskRF và TaskSensor kịp chạy nền
    // (Để không bị mất kết nối sóng và không bị reset)
    vTaskDelay(pdMS_TO_TICKS(20));
  }

  // Tính toán
  float offX = (maxX + minX) / 2.0f;
  float offY = (maxY + minY) / 2.0f;
  float offZ = (maxZ + minZ) / 2.0f;
  float rangeX = (maxX - minX) / 2.0f;
  float rangeY = (maxY - minY) / 2.0f;
  float rangeZ = (maxZ - minZ) / 2.0f;
  float avgRange = (rangeX + rangeY + rangeZ) / 3.0f;
  float sx = (rangeX > 0.001f) ? (avgRange / rangeX) : 1.0f;
  float sy = (rangeY > 0.001f) ? (avgRange / rangeY) : 1.0f;
  float sz = (rangeZ > 0.001f) ? (avgRange / rangeZ) : 1.0f;

  // Lưu NVS
  saveMagCalibrationToPrefs(offX, offY, offZ, sx, sy, sz);
  magX_off = offX; magY_off = offY; magZ_off = offZ;
  magScaleX = sx; magScaleY = sy; magScaleZ = sz;

  // Trả quyền
  vTaskPrioritySet(NULL, oldPrio);
  Serial.println("=== HOAN TAT HIEU CHUAN ===");
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Start I2C at 400k (single bus on pins requested)
  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init failed"));
    while (1) delay(1000);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // MPU accel+gyro on Wire
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();

  // QMC on Wire
  qmc.init();

  // init shared data & RTOS primitives
  sensorData = {MAX_DISTANCE_CM, MAX_DISTANCE_CM, MAX_DISTANCE_CM, MAX_DISTANCE_CM, 0.0f, 0.0f, 0.0f, 0, 0};
  commandQueue = xQueueCreate(8, sizeof(Command));
  storageQueue = xQueueCreate(4, sizeof(HomeSave));
  sensorMutex = xSemaphoreCreateMutex();
  radioMutex = xSemaphoreCreateMutex();
  prefsMutex = xSemaphoreCreateMutex();
  if (!commandQueue || !storageQueue || !sensorMutex || !radioMutex || !prefsMutex) {
    Serial.println("Failed to create RTOS primitives");
    while (1) delay(1000);
  }

  // load mag calibration if present
  loadMagCalibration();

  // motors
  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT); pinMode(MOTOR_IN4, OUTPUT);
  pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);
  ledcSetup(ENA_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(ENB_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA_PIN, ENA_CHANNEL);
  ledcAttachPin(ENB_PIN, ENB_CHANNEL);

  // ultrasonics
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);  pinMode(ECHO_BACK, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // radio: use safer VSPI pins; CSN left as you requested
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!radio.begin()) {
    Serial.println("NRF24 init failed - HALT!");
    while (1) delay(1000);
  }

  Serial.print("RF chip connected: "); Serial.println(radio.isChipConnected() ? "YES" : "NO");
  Serial.print("RF channel (default): "); Serial.println(radio.getChannel());

  // Configure radio - ensure these match controller settings
  radio.setChannel(108);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(5, 3);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();

  // Open pipes
  radio.openReadingPipe(0, ADDRESS_ROBOT);
  radio.openWritingPipe(ADDRESS_CTL);
  radio.startListening();

  Serial.println("Radio OK - Listening on channel 108");
  display.println("Radio OK");
  display.display();

  // Load stored home (robot)
  if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (prefs.begin("robot", true)) {
      bool storedHome = prefs.getBool("homeSet", false);
      if (storedHome) {
        float hlat = prefs.getFloat("homeLat", 0.0f);
        float hlon = prefs.getFloat("homeLon", 0.0f);
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
          homeLat = hlat; homeLon = hlon; homeValid = true;
          xSemaphoreGive(sensorMutex);
        } else {
          homeLat = hlat; homeLon = hlon; homeValid = true;
        }
        Serial.printf("Loaded home: %.6f, %.6f\n", homeLat, homeLon);
        display.println("Home loaded");
        display.display();
      }
      prefs.end();
    }
    xSemaphoreGive(prefsMutex);
  }

  // PID
  setupPID();

  // watchdog
  esp_task_wdt_init(TASK_WDT_TIMEOUT_S, true);

  // create tasks including console for calibration commands
  xTaskCreatePinnedToCore(TaskSensor,  "TaskSensor",  4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(TaskRF,      "TaskRF",      4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskControl, "TaskControl", 8192, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(TaskUI,      "TaskUI",      4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskStorage, "TaskStorage", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskConsole, "TaskConsole", 4096, NULL, 1, NULL, 0);

  display.println("Tasks started");
  display.display();
  lastRadioTime = millis();
}

// ===== TaskConsole: receive serial commands for calibration =====
void TaskConsole(void* pvParameters) {
  (void) pvParameters;
  for (;;) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n');
      s.trim();
      s.toLowerCase();
      if (s == "c" || s == "cal" || s == "calib") {
        Serial.println("Starting magnetometer calibration (20s)...");
        runMagCalibration(20000);
      } else if (s == "printcal") {
        Serial.printf("magX_off=%.3f magY_off=%.3f magZ_off=%.3f\n", magX_off, magY_off, magZ_off);
        Serial.printf("magScaleX=%.3f magScaleY=%.3f magScaleZ=%.3f\n", magScaleX, magScaleY, magScaleZ);
      } else if (s == "clearcal" || s == "clear" || s == "cc") {
        Serial.println("Clearing magnetometer calibration...");
        clearMagCalibration();
      } else {
        Serial.printf("Unknown command: %s\n", s.c_str());
        Serial.println("Commands: c|cal|calib -> run calibration, printcal -> show, clearcal|clear|cc -> delete");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ===== TaskSensor =====
void TaskSensor(void* pvParameters) {
  esp_task_wdt_add(NULL);
  (void) pvParameters;
  const TickType_t delayTicks = pdMS_TO_TICKS(20);
  for (;;) {
    long d = MAX_DISTANCE_CM;
    switch (currentSonar) {
      case 0:
        d = readUltrasonicCM(TRIG_FRONT, ECHO_FRONT);
        maFront.add((float)d);
        if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) { sensorData.distFront = maFront.get(); xSemaphoreGive(sensorMutex); }
        break;
      case 1:
        d = readUltrasonicCM(TRIG_BACK, ECHO_BACK);
        maBack.add((float)d);
        if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) { sensorData.distBack = maBack.get(); xSemaphoreGive(sensorMutex); }
        break;
      case 2:
        d = readUltrasonicCM(TRIG_LEFT, ECHO_LEFT);
        maLeft.add((float)d);
        if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) { sensorData.distLeft = maLeft.get(); xSemaphoreGive(sensorMutex); }
        break;
      case 3:
        d = readUltrasonicCM(TRIG_RIGHT, ECHO_RIGHT);
        maRight.add((float)d);
        if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) { sensorData.distRight = maRight.get(); xSemaphoreGive(sensorMutex); }
        break;
    }
    currentSonar = (currentSonar + 1) % 4;

    while (Serial2.available() > 0) {
      char c = (char)Serial2.read();
      if (gps.encode(c)) {
        if (gps.location.isValid()) {
          unsigned long age = gps.location.age();
          int sats = gps.satellites.value();
          if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            sensorData.lat = (float)gps.location.lat();
            sensorData.lon = (float)gps.location.lng();
            sensorData.sats = sats;
            sensorData.gpsAge = age;
            xSemaphoreGive(sensorMutex);
          } else {
            sensorData.lat = (float)gps.location.lat();
            sensorData.lon = (float)gps.location.lng();
            sensorData.sats = sats;
            sensorData.gpsAge = age;
          }

          bool doAutoHome = false;
          if (age < 2000 && sats >= 4) doAutoHome = true;

          if (!homeValid && doAutoHome) {
            HomeSave hs; hs.op = HOME_SAVE; hs.lat = (float)gps.location.lat(); hs.lon = (float)gps.location.lng();
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
              homeLat = hs.lat; homeLon = hs.lon; homeValid = true;
              xSemaphoreGive(sensorMutex);
            } else {
              homeLat = hs.lat; homeLon = hs.lon; homeValid = true;
            }
            xQueueSend(storageQueue, &hs, 0);
            Serial.println("Auto-home queued");
          }
        }
      }
    }

    if (mpu.accelUpdate() == 0) {
      float ax = mpu.accelX(), ay = mpu.accelY(), az = mpu.accelZ();
      qmc.read();
      float mx = ((float)qmc.getX() - magX_off) * magScaleX;
      float my = ((float)qmc.getY() - magY_off) * magScaleY;
      float mz = ((float)qmc.getZ() - magZ_off) * magScaleZ;

      float roll = atan2(ay, az);
      float pitch = atan2(-ax, sqrt(ay * ay + az * az));
      float magX_comp = mx * cos(pitch) - mz * sin(pitch);
      float magY_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
      float heading = atan2(magY_comp, magX_comp) * 180.0 / PI;
      if (heading < 0) heading += 360.0;
      haHeading.add(heading);
      float avgHeading = haHeading.get();

      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        sensorData.heading = avgHeading;
        xSemaphoreGive(sensorMutex);
      }
    }

    esp_task_wdt_reset();
    vTaskDelay(delayTicks);
  }
}

// ===== TaskRF =====
void TaskRF(void* pvParameters) {
  esp_task_wdt_add(NULL);
  (void) pvParameters;
  Command cmd;
  for (;;) {
    if (xSemaphoreTake(radioMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (radio.available()) {
        Serial.print("RF: chipConnected=");
        Serial.print(radio.isChipConnected() ? "YES" : "NO");
        Serial.print(" ch="); Serial.println(radio.getChannel());

        uint8_t len = radio.getDynamicPayloadSize();
        if (len > 0 && len <= 32) {
          uint8_t buf[32];
          radio.read(buf, len);
          Serial.print("RF RX len="); Serial.print(len); Serial.print(" bytes:");
          for (int i = 0; i < len; ++i) { Serial.print(' '); Serial.print((int)buf[i], DEC); }
          Serial.println();

          uint8_t rxcmd = buf[0];
          lastRadioTime = millis();

          if (rxcmd == 6) { // SET_HOME
            uint8_t ack = 0;
            if (gps.location.isValid() && gps.location.age() < 2000 && gps.satellites.value() >= 4) {
              double newLat = gps.location.lat(), newLon = gps.location.lng();
              if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) { homeLat = newLat; homeLon = newLon; homeValid = true; xSemaphoreGive(sensorMutex); }
              else { homeLat = newLat; homeLon = newLon; homeValid = true; }
              HomeSave hs; hs.op = HOME_SAVE; hs.lat = (float)newLat; hs.lon = (float)newLon;
              xQueueSend(storageQueue, &hs, 0);
              ack = 1;
              Serial.printf("RF: SET_HOME accepted %.6f, %.6f\n", newLat, newLon);
            } else {
              Serial.println("RF: SET_HOME denied (no fresh GPS)");
              ack = 0;
            }
            radio.stopListening();
            bool wrote = radio.write(&ack, sizeof(ack));
            radio.startListening();
            Serial.printf("RF: sent ACK=%d wrote=%s\n", ack, wrote ? "YES" : "NO");
          } else if (rxcmd == 7) { // CLEAR_HOME
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) { homeValid = false; homeLat = 0; homeLon = 0; xSemaphoreGive(sensorMutex); }
            else { homeValid = false; homeLat = 0; homeLon = 0; }
            HomeSave hs; hs.op = HOME_CLEAR; hs.lat = 0; hs.lon = 0; xQueueSend(storageQueue, &hs, 0);
            uint8_t ack = 1;
            radio.stopListening();
            bool wrote = radio.write(&ack, sizeof(ack));
            radio.startListening();
            Serial.printf("RF: CLEAR_HOME queued, ACK sent wrote=%s\n", wrote ? "YES" : "NO");
          } else {
            Command c = (Command)rxcmd;
            xQueueSend(commandQueue, &c, 0);
          }
        } else {
          if (len > 32) { radio.flush_rx(); Serial.println("RF: invalid payload len flushed"); }
        }
      }
      xSemaphoreGive(radioMutex);
    }
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

// ===== TaskControl =====
void TaskControl(void* pvParameters) {
  esp_task_wdt_add(NULL);
  (void) pvParameters;
  Command cmd;
  const TickType_t loopDelay = pdMS_TO_TICKS(40);
  for (;;) {
    if (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
      // TRƯỜNG HỢP LỆNH HIỆU CHUẨN (SỐ 8)
     // ... Bên trong TaskControl ...
      if (cmd == CALIBRATE_MAG) {
        Serial.println("CMD: Calib...");
        
        clearMagCalibration();
        vTaskDelay(pdMS_TO_TICKS(500));

        // 1. CHẶN 20 GIÂY ĐỂ XOAY
        runMagCalibration(20000); 
        
        // 2. SAU KHI XONG 20 GIÂY -> CODE CHẠY TIẾP XUỐNG ĐÂY
        Serial.println("-> Xong! Tro ve che do lai thuong.");

        // 3. QUAN TRỌNG: Xóa sạch các lệnh cũ bị tồn đọng trong lúc bận
        xQueueReset(commandQueue); 

        // 4. Đưa về trạng thái lái tay
        receivedCommand = STOP;
        currentState = MANUAL; 
      } 
      // ...
      else if (cmd == RTH_ACTIVATE) {
        bool hv = false;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) { hv = homeValid; xSemaphoreGive(sensorMutex); }
        else hv = homeValid;
        if (hv) { if (currentState == MANUAL) currentState = RTH_NAVIGATE; Serial.println("RTH accepted"); }
        else Serial.println("RTH denied: no home");
      } else if (cmd == STOP) { currentState = MANUAL; move(STOP); receivedCommand = STOP; }
      else receivedCommand = cmd;
    }

    SensorData s;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) { s = sensorData; xSemaphoreGive(sensorMutex); }
    else { vTaskDelay(loopDelay); continue; }

    if (currentState == MANUAL && (millis() - lastRadioTime > FAILSAFE_TIMEOUT)) {
      bool hv = false;
      double hlat = 0, hlon = 0;
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) { hv = homeValid; hlat = homeLat; hlon = homeLon; xSemaphoreGive(sensorMutex); }
      else { hv = homeValid; hlat = homeLat; hlon = homeLon; }

      if (hv && gps.location.isValid()) {
        double distanceToHome = TinyGPSPlus::distanceBetween((double)s.lat, (double)s.lon, hlat, hlon);
        if (distanceToHome > FAILSAFE_RTH_DISTANCE_METERS) { currentState = RTH_NAVIGATE; Serial.println("Failsafe -> start RTH"); }
        else { Serial.println("Failsafe -> near home, stop"); move(STOP); }
      } else { Serial.println("Failsafe -> no home/GPS, stop"); move(STOP); }
    }

    switch (currentState) {
      case MANUAL: move(receivedCommand); break;
      case RTH_NAVIGATE: {
        double hlat = 0, hlon = 0;
        bool hv = false;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(200)) == pdTRUE) { hv = homeValid; hlat = homeLat; hlon = homeLon; xSemaphoreGive(sensorMutex); }
        else { hv = homeValid; hlat = homeLat; hlon = homeLon; }

        if (!hv) { Serial.println("RTH aborted: home invalid"); currentState = MANUAL; move(STOP); break; }
        if (!gps.location.isValid()) { move(STOP); break; }

        double distanceToHome = TinyGPSPlus::distanceBetween((double)s.lat, (double)s.lon, hlat, hlon);
        if (distanceToHome < 1.0) { currentState = RTH_ARRIVED; move(STOP); break; }
        int rthSpeed = (distanceToHome < 3.0) ? 90 : BASE_SPEED;

        // obstacle ahead -> attempt avoidance
        if (s.distFront > 0 && s.distFront < 30) {
          currentState = RTH_AVOID;
          lastRthAction = millis();
          avoidAttempts = 0;
          avoidClearCheckStart = 0;
          Serial.println("RTH: obstacle ahead -> entering AVOID");
          move(STOP);
          break;
        }

        double bearingToHome = TinyGPSPlus::courseTo((double)s.lat, (double)s.lon, hlat, hlon);
        double bearingError = bearingToHome - s.heading;
        if (bearingError > 180) bearingError -= 360;
        if (bearingError < -180) bearingError += 360;
        headingPID.setpoint = 0.0; headingPID.input = bearingError;
        double turnOutput = computePID(&headingPID);
        int leftSpeed = constrain((int)round(rthSpeed + turnOutput), 0, MAX_SPEED);
        int rightSpeed = constrain((int)round(rthSpeed - turnOutput), 0, MAX_SPEED);
        controlMotorsPWM(leftSpeed, rightSpeed);
        break;
      }
      case RTH_AVOID: {
        unsigned long now = millis();
        if (avoidAttempts >= MAX_AVOID_ATTEMPTS) {
          Serial.println("RTH_AVOID: max attempts reached, try rotate to find path");
          move(TURN_LEFT, 120);
          vTaskDelay(pdMS_TO_TICKS(600));
          move(STOP);
          avoidAttempts = 0;
          lastRthAction = now;
        } else {
          if (now - lastRthAction < 400) {
            move(BACKWARD, 120);
          } else if (now - lastRthAction < 1200) {
            if (s.distLeft > s.distRight && s.distLeft > 35) move(TURN_LEFT, 100);
            else if (s.distRight > 35) move(TURN_RIGHT, 100);
            else move(BACKWARD, 80);
          } else {
            avoidAttempts++;
            lastRthAction = now;
            avoidClearCheckStart = (s.distFront > 40) ? now : 0;
          }
        }
        if (s.distFront > 45) {
          if (avoidClearCheckStart == 0) avoidClearCheckStart = now;
          else if (now - avoidClearCheckStart >= AVOID_CLEAR_CHECK_MS) {
            Serial.println("RTH_AVOID: path clear -> resume NAVIGATE");
            currentState = RTH_NAVIGATE;
            avoidAttempts = 0;
            move(STOP);
          }
        } else {
          avoidClearCheckStart = 0;
        }
        break;
      }
      case RTH_ARRIVED: move(STOP); break;
    }

    esp_task_wdt_reset();
    vTaskDelay(loopDelay);
  }
}

// ===== TaskUI (reset WDT before display.display()) =====
void TaskUI(void* pvParameters) {
  esp_task_wdt_add(NULL);
  (void) pvParameters;
  const TickType_t delayTicks = pdMS_TO_TICKS(600);
  unsigned long lastSwitch = 0;

  for (;;) {
    if (millis() - lastSwitch > SCREEN_SWITCH_INTERVAL) { currentScreen = (currentScreen + 1) % 2; lastSwitch = millis(); }

    SensorData s;
    double dispHomeLat = 0, dispHomeLon = 0;
    bool dispHomeValid = false;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      s = sensorData;
      dispHomeLat = homeLat; dispHomeLon = homeLon; dispHomeValid = homeValid;
      xSemaphoreGive(sensorMutex);
    } else {
      esp_task_wdt_reset();
      vTaskDelay(delayTicks);
      continue;
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    // Kiểm tra trạng thái Calib (để hiển thị ở Màn 2)
    bool isCalibrated = (magX_off != 0.0f || magY_off != 0.0f);
    if (currentScreen == 0) {
      display.setTextSize(2); display.print("MODE: ");
      switch (currentState) { case MANUAL: display.println("MANUAL"); break; case RTH_NAVIGATE: display.println("RTH"); break; case RTH_AVOID: display.println("AVOID"); break; case RTH_ARRIVED: display.println("HOME!"); break; default: display.println("?"); break; }
      display.setTextSize(1);
      display.printf("Head: %.1f deg\n", s.heading);
      if (!dispHomeValid) display.println("Home: NOT SET"); else display.printf("Home: %.5f, %.5f\n", dispHomeLat, dispHomeLon);
      if (currentState == RTH_NAVIGATE || currentState == RTH_AVOID) {
        double d = TinyGPSPlus::distanceBetween((double)s.lat, (double)s.lon, dispHomeLat, dispHomeLon);
        display.printf("ToHome: %.1fm\n", d);
      }
      display.printf("F:%.0f L:%.0f R:%.0f B:%.0f\n", s.distFront, s.distLeft, s.distRight, s.distBack);
      if (storageMessageUntil != 0 && millis() < storageMessageUntil) { display.println(); display.println("Home saved!"); }
    } else {
      display.setTextSize(2); display.println("GPS INFO"); display.setTextSize(1);
      display.printf("Lat: %.4f\n", s.lat); display.printf("Lon: %.4f\n", s.lon); display.printf("Sats: %d | Age: %lums\n", s.sats, s.gpsAge);
      display.print("MAG CALIB: ");
      if (isCalibrated) {
        display.printf("YES (OK):\n %4f ; %4f", magX_off, magY_off);
      } else {
        display.println("NO (RAW)!"); // Cảnh báo chưa calib
      }
    }

    Serial.println("UI: before display");
    esp_task_wdt_reset();
    display.display();
    esp_task_wdt_reset();
    Serial.println("UI: after display");

    vTaskDelay(delayTicks);
  }
}

// ===== TaskStorage =====
void TaskStorage(void* pvParameters) {
  (void) pvParameters;
  HomeSave hs;
  for (;;) {
    if (xQueueReceive(storageQueue, &hs, portMAX_DELAY) == pdTRUE) {
      if (hs.op == HOME_SAVE) {
        if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
          if (prefs.begin("robot", false)) {
            prefs.putFloat("homeLat", hs.lat); prefs.putFloat("homeLon", hs.lon); prefs.putBool("homeSet", true); prefs.end();
            Serial.printf("Home persisted %.6f, %.6f\n", hs.lat, hs.lon);
          }
          xSemaphoreGive(prefsMutex);
        }
        storageMessageUntil = millis() + 2000UL;
      } else if (hs.op == HOME_CLEAR) {
        if (xSemaphoreTake(prefsMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
          if (prefs.begin("robot", false)) { prefs.remove("homeLat"); prefs.remove("homeLon"); prefs.putBool("homeSet", false); prefs.end(); Serial.println("Home cleared"); }
          xSemaphoreGive(prefsMutex);
        }
      }
    }
  }
}

// ===== loop =====
void loop() {
  while (true) vTaskDelay(pdMS_TO_TICKS(1000));
}





