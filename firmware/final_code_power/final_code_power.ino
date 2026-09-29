#include "ICM42670P.h"
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>

// ======================================================
//                    WIFI SETTINGS
// ======================================================

const char* ssid = "ESP32-Data-Server";
const char* password = "12345678";

WebServer server(80);

// ======================================================
//                    BIKE CONSTANTS
// ======================================================

// in meters
const float crankLength = 0.06;
const float e = 2.7182818;

// ======================================================
//                    IMU PINS
// ======================================================

#define CS_pin 21
#define CLK_pin 14
#define MISO_pin 12
#define MOSI_pin 13

SPIClass* fspi = NULL;
ICM42670* IMU = NULL;

// ======================================================
//                    HX711 PINS
// ======================================================

const int hxClockPin = 48;
const int hxDataPin = 45;

// ======================================================
//                    GLOBAL VALUES
// ======================================================

float latestGyroX = 0;
float latestGyroY = 0;
float latestGyroZ = 0;

float latestAngularVelocity = 0;

float latestRawForce = 0;
float latestSlidingRaw = 0;    // 3-second sliding average raw force
float latestSlidingPower = 0;  // 3-second sliding average power

// Number of HX711 samples used for initial calibration
const int samples = 50;

float slope = 0;
float intercept = 0;

int initialForce = 0;

// ======================================================
//                3 SECOND TIMERS & WINDOWS
// ======================================================

const unsigned long measurementInterval = 1000;

// Time when the current measurement started
unsigned long measurementStartTime = 0;

// Sum of all HX711 readings during the period
long double hx711Sum = 0;

// Number of HX711 readings collected
unsigned long hx711Count = 0;

// IMU accumulation variables for 1-second averaging
float gyroSumX = 0;
float gyroSumY = 0;
float gyroSumZ = 0;
float angVelSum = 0;
unsigned long imuSampleCount = 0;

// 3-Second Sliding Window Buffers
const int windowSize = 3;
float powerWindow[windowSize] = { 0, 0, 0 };
int windowIndex = 0;

float rawWindow[windowSize] = { 0, 0, 0 };
int rawWindowIndex = 0;

// ======================================================
//                    HX711 FUNCTION
// ======================================================

long readHX711() {

  long count = 0;

  // Wait until HX711 is ready
  while (digitalRead(hxDataPin) == HIGH)
    ;

  // Read 24 bits
  for (int i = 0; i < 24; i++) {

    digitalWrite(hxClockPin, HIGH);

    count = count << 1;

    if (digitalRead(hxDataPin)) {
      count++;
    }

    digitalWrite(hxClockPin, LOW);
  }

  // 25th pulse → gain = 128
  digitalWrite(hxClockPin, HIGH);
  digitalWrite(hxClockPin, LOW);

  // Convert 24-bit signed value
  if (count & 0x800000) {
    count |= ~0xFFFFFF;
  }

  return count;
}

// ======================================================
//                    WEBSITE
// ======================================================

void handleRoot() {

  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>

    <head>
      <title>ESP32 Power Meter</title>

      <style>

        body {
          font-family: Arial;
          text-align: center;
          margin-top: 40px;
        }

        .box {
          font-size: 28px;
          margin: 15px;
        }

      </style>

      <script>

        function updateData() {

          fetch('/data')
            .then(response => response.json())
            .then(data => {

              document.getElementById('gyroX').textContent = data.gyroX;
              document.getElementById('gyroY').textContent = data.gyroY;
              document.getElementById('gyroZ').textContent = data.gyroZ;
              document.getElementById('angularVelocity').textContent = data.angularVelocity;
              document.getElementById('rawValue').textContent = data.raw;
              document.getElementById('slidingRaw').textContent = data.slidingRaw;
              document.getElementById('slidingPower').textContent = data.slidingPower;

            });

        }

        setInterval(updateData, 1000);

        // Get the first reading immediately
        updateData();

      </script>

    </head>

    <body>

      <h1>ESP32 Cycling Power Meter</h1>

      <div class="box">
        <b>Gyro X:</b>
        <span id="gyroX">0</span> rad/s
      </div>

      <div class="box">
        <b>Gyro Y:</b>
        <span id="gyroY">0</span> rad/s
      </div>

      <div class="box">
        <b>Gyro Z:</b>
        <span id="gyroZ">0</span> rad/s
      </div>

      <div class="box">
        <b>Angular Velocity:</b>
        <span id="angularVelocity">0</span> rad/s
      </div>

      <div class="box">
        <b>Average Raw HX711:</b>
        <span id="rawValue">0</span>
      </div>

      <div class="box">
        <b>3-Sec Sliding Raw HX711:</b>
        <span id="slidingRaw">0</span>
      </div>

      <div class="box">
        <b>3-Sec Average Power:</b> 
        <span id="slidingPower">0</span> W
      </div>

    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// ======================================================
//      SEND ALL LIVE DATA TO THE WEBSITE
// ======================================================

void handleData() {

  String json = "{";

  json += "\"gyroX\":" + String(latestGyroX, 3) + ",";
  json += "\"gyroY\":" + String(latestGyroY, 3) + ",";
  json += "\"gyroZ\":" + String(latestGyroZ, 3) + ",";
  json += "\"angularVelocity\":" + String(latestAngularVelocity, 3) + ",";
  json += "\"raw\":" + String(latestRawForce, 0) + ",";
  json += "\"slidingRaw\":" + String(latestSlidingRaw, 0) + ",";
  json += "\"slidingPower\":" + String(latestSlidingPower, 2);

  json += "}";

  server.send(200, "application/json", json);
}

// ======================================================
//                        SETUP
// ======================================================

void setup() {

  int ret;

  Serial.begin(115200);

  // ---------- IMU setup ----------

  fspi = new SPIClass(FSPI);

  fspi->begin(CLK_pin, MISO_pin, MOSI_pin, CS_pin);

  pinMode(CS_pin, OUTPUT);

  digitalWrite(CS_pin, HIGH);

  IMU = new ICM42670(*fspi, CS_pin);

  // Manual WHO_AM_I check
  digitalWrite(CS_pin, LOW);

  fspi->transfer(0x75 | 0x80);

  uint8_t who_am_i = fspi->transfer(0x00);

  digitalWrite(CS_pin, HIGH);

  Serial.print("Manual WHO_AM_I check: 0x");
  Serial.println(who_am_i, HEX);

  if (who_am_i != 0x67) {

    Serial.println("WARNING: Chip ID mismatch!");

  } else {

    Serial.println("IMU detected correctly.");
  }

  ret = IMU->begin();

  Serial.print("IMU begin returned: ");
  Serial.println(ret);

  if (ret != 0) {

    Serial.println("IMU initialization failed");

    while (1)
      ;
  }

  IMU->startAccel(100, 16);
  IMU->startGyro(100, 2000);

  // ---------- HX711 setup ----------

  pinMode(hxClockPin, OUTPUT);
  pinMode(hxDataPin, INPUT);

  delay(100);

  // ======================================================
  //                    INITIAL CALIBRATION
  // ======================================================


  for (int i = 0; i < samples; i++) {

    initialForce += readHX711();
  }

  initialForce /= samples;

  // ======================================================
  //                    WIFI SETUP
  // ======================================================

  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");

  IPAddress IP = WiFi.softAPIP();

  Serial.print("ESP32 IP Address: ");
  Serial.println(IP);

  // ======================================================
  //                    WEBSITE ENDPOINTS
  // ======================================================

  server.on("/", handleRoot);

  server.on("/data", handleData);

  server.begin();

  Serial.println("Web server started");

  // Start the first measurement period
  measurementStartTime = millis();

  Serial.print("Initial HX711 average: ");
  Serial.println(initialForce);
}

// ======================================================
//                        LOOP
// ======================================================

void loop() {

  // ======================================================
  //                    READ IMU
  // ======================================================

  inv_imu_sensor_event_t imu_event;

  IMU->getDataFromRegisters(imu_event);

  float gyroX = imu_event.gyro[0] / 16.4 * 0.0174533;
  float gyroY = imu_event.gyro[1] / 16.4 * 0.0174533;
  float gyroZ = imu_event.gyro[2] / 16.4 * 0.0174533;

  float angularVelocity = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  gyroSumX += gyroX;
  gyroSumY += gyroY;
  gyroSumZ += gyroZ;
  angVelSum += angularVelocity;
  imuSampleCount++;

  // ======================================================
  //                    READ HX711
  // ======================================================

  long measuredForce = readHX711();

  hx711Sum += abs(measuredForce - initialForce);
  hx711Count++;

  // Update every second
  if (millis() - measurementStartTime >= measurementInterval) {

    // 1. Calculate 1-second averages
    float avgGyroX = gyroSumX / imuSampleCount;
    float avgGyroY = gyroSumY / imuSampleCount;
    float avgGyroZ = gyroSumZ / imuSampleCount;
    float avgAngularVelocity = angVelSum / imuSampleCount;
    float averageHX711 = hx711Sum / hx711Count;

    // 2. Update 3-Second Sliding Window for Raw Force
    rawWindow[rawWindowIndex] = averageHX711;
    rawWindowIndex = (rawWindowIndex + 1) % windowSize;

    float rawSlidingSum = 0;
    for (int i = 0; i < windowSize; i++) {
      rawSlidingSum += rawWindow[i];
    }
    float slidingRaw = rawSlidingSum / windowSize;
    float torque = 0.000108 * slidingRaw - 0.744;

    // 3. Calculate 1-Second Power & Update 3-Second Sliding Power Window
    float oneSecondPower = torque * avgAngularVelocity;

    powerWindow[windowIndex] = oneSecondPower;
    windowIndex = (windowIndex + 1) % windowSize;

    float powerSlidingSum = 0;
    for (int i = 0; i < windowSize; i++) {
      powerSlidingSum += powerWindow[i];
    }
    float slidingPower = powerSlidingSum / windowSize;
    slidingPower -= 40;

    // 5. Save globals
    latestGyroX = avgGyroX;
    latestGyroY = avgGyroY;
    latestGyroZ = avgGyroZ;
    latestAngularVelocity = avgAngularVelocity;
    latestRawForce = averageHX711;
    latestSlidingRaw = slidingRaw;
    latestSlidingPower = slidingPower;

    Serial.print("lastestSlidingPower");
    Serial.println(latestSlidingPower);

    // 6. Reset ALL accumulators
    hx711Sum = 0;
    hx711Count = 0;
    gyroSumX = 0;
    gyroSumY = 0;
    gyroSumZ = 0;
    angVelSum = 0;
    imuSampleCount = 0;

    measurementStartTime = millis();
  }

  // ======================================================
  //                    HANDLE WEBSITE
  // ======================================================

  server.handleClient();
}