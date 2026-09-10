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
const float crankLength = 0.13;

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

float latestForce = 0;
float latestPower = 0;
float latestRawForce = 0;

//how many hx711 samples we will take
const int samples = 5;

// selecting which LOBF for calibration
float slope = 0;
int intercept = 0;

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


void calibrate(int rawForce) {

  if (rawForce < 7191500) {
    slope = 3.318678e-5;
    intercept = -238.398035;

  } else if (rawForce >= 7191500 && rawForce < 7217000) {
    slope = 3.809550e-5;
    intercept = -274.868529;

  } else if (rawForce >= 7217000 && rawForce < 7227000) {
    slope = 3.229836e-5;
    intercept = -233.323349;

  } else if (rawForce >= 7227000) {
    slope = 3.537439e-5;
    intercept = -255.456810;
  } else {
    slope = (2.35e-12 * rawForce) - 1.33e-5;
    intercept = -rawForce * slope;
  }
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

        setInterval(function() {

          fetch('/data')
            .then(response => response.json())
            .then(data => {

              document.getElementById('gyroX').textContent = data.gyroX;
              document.getElementById('gyroY').textContent = data.gyroY;
              document.getElementById('gyroZ').textContent = data.gyroZ;
              document.getElementById('angularVelocity').textContent = data.angularVelocity;
              document.getElementById('rawValue').textContent = data.raw;
              document.getElementById('power').textContent = data.power;

        });

        }, 50);

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
        <b>Latest Raw HX711:</b>
        <span id="rawValue">0</span>
      </div>

      <div class="box">
        <b>Power:</b>
        <span id="power">0</span> W
      </div>

    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}


// ======================================================
//       SEND ALL LIVE DATA TO THE WEBSITE
// ======================================================

void handleData() {

  String json = "{";

  json += "\"gyroX\":" + String(latestGyroX, 3) + ",";
  json += "\"gyroY\":" + String(latestGyroY, 3) + ",";
  json += "\"gyroZ\":" + String(latestGyroZ, 3) + ",";
  json += "\"angularVelocity\":" + String(latestAngularVelocity, 3) + ",";
  json += "\"raw\":" + String(latestRawForce, 0) + ",";
  json += "\"power\":" + String(latestPower);

  json += "}";

  server.send(200, "application/json", json);
}


// ======================================================
//                         SETUP
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
  //                    WIFI SETUP
  // ======================================================

  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");

  IPAddress IP = WiFi.softAPIP();

  Serial.print("ESP32 IP Address: ");
  Serial.println(IP);

  // ======================================================
  //                 WEBSITE ENDPOINTS
  // ======================================================

  server.on("/", handleRoot);

  // /data sends all live sensor values
  server.on("/data", handleData);

  server.begin();

  Serial.println("Web server started");

  int initialForce = 0;

  for (int i = 0; i < samples; i++) {
    initialForce += readHX711();
  }

  initialForce /= samples;

  calibrate(initialForce);
}


// ======================================================
//                         LOOP
// ======================================================

void loop() {

  // ===== Read IMU =====

  inv_imu_sensor_event_t imu_event;

  IMU->getDataFromRegisters(imu_event);

  //we have to convert to rad/s

  //16.4 is callibration according to datasheet
  //0.0174533 = pi/180 is converting from degrees/s to rad/s

  float gyroX = imu_event.gyro[0] / 16.4 * 0.0174533;
  float gyroY = imu_event.gyro[1] / 16.4 * 0.0174533;
  float gyroZ = imu_event.gyro[2] / 16.4 * 0.0174533;


  // ===== Read HX711 =====

  long measuredForce = 0;

  float actualForce;


  for (int i = 0; i < samples; i++) {
    measuredForce += readHX711();
  }

  measuredForce /= samples;


  //we're going to calibrate using a LOBF for the AVERAGED measured force values

  actualForce = slope * measuredForce + intercept + 161.2;


  // 1. Calculate Torque (N*m)

  double torque = actualForce * crankLength;


  // 2. Calculate the magnitude of the angular velocity vector from the gyro (rad/s)

  double angularVelocity =
    sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);


  // 3. Power = Torque * Angular Velocity (Watts)

  double totalPower = torque * angularVelocity;


  // ======================================================
  //          SAVE VALUES FOR WEBSITE DISPLAY
  // ======================================================

  latestGyroX = gyroX;
  latestGyroY = gyroY;
  latestGyroZ = gyroZ;

  latestAngularVelocity = angularVelocity;

  latestRawForce = measuredForce;

  latestForce = actualForce;

  latestPower = totalPower;


  // ======================================================
  //                 HANDLE WEBSITE
  // ======================================================

  server.handleClient();
}