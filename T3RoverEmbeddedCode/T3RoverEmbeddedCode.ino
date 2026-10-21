#include <string>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"
#include <hardware/watchdog.h>

// =========================================================================
// HARDWARE PIN DEFINITIONS
// =========================================================================

// I2C Bus Pins (I2C0)
#define I2C_SDA_PIN 2  // GP2
#define I2C_SCL_PIN 3  // GP3
#define WIRE Wire

// SPI SD Card Bus Pins (SPI0)
#define SD_PIN_MISO 16 // GP16 (SPI0 RX)
#define SD_PIN_CS   17 // GP17 (SPI0 CSn)
#define SD_PIN_SCK  18 // GP18 (SPI0 SCK)
#define SD_PIN_MOSI 19 // GP19 (SPI0 TX)

#define SEALEVELPRESSURE_HPA (1013.25)

// GM9 DC Motor Driver (Base Rotation)
#define MOTOR_DRIVER_BASE_IN1 7  // GP7
#define MOTOR_DRIVER_BASE_IN2 6  // GP6
#define MOTOR_DRIVER_BASE_PWM 11 // GP11

// Servo GPIO Assignment
// Servo 0: Smraza 45KG (Lower Elbow 270°) -> GP0
// Servo 1: Miuzei 25KG (Upper Joint 180°)  -> GP1
// Servo 2: MG996R     (Wrist Twist 180°)   -> GP4
// Servo 3: MG996R     (Claw Open/Close)    -> GP12
int servoPinNums[4] = {0, 1, 4, 12};
Servo servos[4];

// Servo Physical Angle Safety Boundaries
float maxVals[4] = {270.0, 180.0, 180.0, 180.0};
float minVals[4] = {0.0, 0.0, 0.0, 0.0};

// Timers
const uint32_t BLINK_INTERVAL = 1000;
const uint32_t READ_INTERVAL  = 100;
uint32_t lastBlink = 0;
uint32_t lastRead = 0;
bool ledState = false;

// Functions
String get_sensors();
String printEvent(sensors_event_t* event);
void write_to_SD(String filename, String dataline);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &WIRE);
Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !ledState);

  // Configure SPI0 Pins
  SPI.setRX(SD_PIN_MISO);
  SPI.setTX(SD_PIN_MOSI);
  SPI.setSCK(SD_PIN_SCK);

  // Base GM9 Motor Driver Pin Setup
  pinMode(MOTOR_DRIVER_BASE_PWM, OUTPUT);
  pinMode(MOTOR_DRIVER_BASE_IN1, OUTPUT);
  pinMode(MOTOR_DRIVER_BASE_IN2, OUTPUT);

  digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
  digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);

  Serial.println("log:Setup base motor driver");

  // Attach Servos
  // Servo 0: Smraza 45KG 270° Servo (500us to 2500us pulse)
  servos[0].attach(servoPinNums[0], 500, 2500);
  // Servos 1-3: Standard 180° Servos (500us to 2500us pulse)
  servos[1].attach(servoPinNums[1], 500, 2500);
  servos[2].attach(servoPinNums[2], 500, 2500);
  servos[3].attach(servoPinNums[3], 500, 2500);

  // Configure I2C Pins
  WIRE.setSCL(I2C_SCL_PIN);
  WIRE.setSDA(I2C_SDA_PIN);
  WIRE.begin();

  if (!bno.begin()) {
    Serial.println("log:Ooops, no BNO055 detected!");
  } else {
    Serial.println("log:BNO055 connected");
  }

  if (!bmp.begin_I2C(0x77, &WIRE)) {
    Serial.println("log:Could not find BMP3 sensor!");
  } else {
    Serial.println("log:BMP388 connected");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  if (!SD.begin(SD_PIN_CS)) {
    Serial.println("log:SD initialization failed!");
  } else {
    Serial.println("log:SD initialization done.");
    write_to_SD("data.csv", "ACCELERATION_X,ACCELERATION_Y,ACCELERATION_Z,ALTITUDE,PRESSURE,TEMPERATURE");
  }

  digitalWrite(LED_BUILTIN, ledState);
}

void loop() {
  // 1. Heartbeat
  if (millis() - lastBlink > BLINK_INTERVAL) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }

  // 2. Process Commands: motor:val0,val1,val2,val3,val4
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    String type = command.substring(0, command.indexOf(":"));

    if (type == "ping") {
      Serial.println("pong");
    } 
    else if (type == "reboot") {
      watchdog_reboot(0, 0, 0);
    } 
    else if (type == "motor") {
      String temp[5] = {""};
      String partsStr = command.substring(command.indexOf(":") + 1);

      for (int i = 0; i < 5; i++) {
        if (partsStr.indexOf(",") != std::string::npos) {
          temp[i] = partsStr.substring(0, partsStr.indexOf(","));
          partsStr = partsStr.substring(partsStr.indexOf(",") + 1);
        } else {
          temp[i] = partsStr;
          break;
        }
      }

      float values[5] = {0.0};
      for (int i = 0; i < 5; i++) {
        values[i] = temp[i].toFloat();
      }

      // Clamp Servos 0-3
      for (int i = 0; i < 4; i++) {
        if (values[i] > maxVals[i]) values[i] = maxVals[i];
        if (values[i] < minVals[i]) values[i] = minVals[i];
      }

      // 1. Smraza 45KG 270° Servo (Servo 0)
      int pulse0 = map((int)values[0], 0, 270, 500, 2500);
      servos[0].writeMicroseconds(pulse0);

      // 2. Standard 180° Servos (Servos 1, 2, 3)
      servos[1].write((int)values[1]);
      servos[2].write((int)values[2]);
      servos[3].write((int)values[3]);

      // 3. GM9 DC Base Motor Driver (Motor 4)
      if (values[4] > 0) {
        digitalWrite(MOTOR_DRIVER_BASE_IN2, HIGH);
        digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
      } else if (values[4] < 0) {
        digitalWrite(MOTOR_DRIVER_BASE_IN1, HIGH);
        digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);
      } else {
        digitalWrite(MOTOR_DRIVER_BASE_IN1, LOW);
        digitalWrite(MOTOR_DRIVER_BASE_IN2, LOW);
      }

      int pwmVal = (abs(values[4]) <= 1.0 && abs(values[4]) > 0) ? 255 : abs((int)values[4]);
      if (values[4] == 0) pwmVal = 0;
      analogWrite(MOTOR_DRIVER_BASE_PWM, pwmVal);
    }
  }

  // 3. 10 Hz Telemetry Loop
  if (millis() - lastRead > READ_INTERVAL) {
    lastRead = millis();
    String sensor_data = get_sensors();
    Serial.println("sensor_data:" + sensor_data);
    write_to_SD("data.csv", sensor_data);
  }
}

String get_sensors() {
  String dataline = "";
  sensors_event_t linearAccelData;
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  if (!bmp.performReading()) {
    Serial.println("log:Failed to perform BMP reading!");
  }

  dataline += printEvent(&linearAccelData);
  dataline += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  dataline += ",";
  dataline += String(bmp.pressure / 100.0);
  dataline += ",";
  dataline += String(bmp.temperature);
  return dataline;
}

String printEvent(sensors_event_t* event) {
  double x = 0, y = 0, z = 0;
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION || event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  return String(x) + "," + String(y) + "," + String(z) + ",";
}

void write_to_SD(String filename, String dataline) {
  File datafile = SD.open(filename, FILE_WRITE);
  if (datafile) {
    datafile.println(dataline);
    datafile.close();
  } else {
    Serial.println("log:Error opening SD file!");
  }
}