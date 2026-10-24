#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>

#define SEALEVELPRESSURE_HPA (1013.25)

// --- SD Card SPI Pins ---
const int _MISO = 16;
const int _MOSI = 19;
const int _CS = 17;
const int _SCK = 18;

// --- Timing ---
const unsigned long LOG_INTERVAL_MS = 250; 
const unsigned long FLUSH_INTERVAL_MS = 5000; // Flush data to SD card every 5 seconds
unsigned long lastLogTime = 0;
unsigned long lastFlushTime = 0;

// --- Objects ---
Servo wrist;
Servo claw;
Servo upperElbow;
Servo lowerElbow;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);
Adafruit_BMP3XX bmp;
File dataFile; // Move file object to global scope

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // Serial setup for ROS2
  Serial.begin(115200);
  Serial.setTimeout(2); // Lower timeout to prevent blocking if bytes are delayed

  // Servo setup
  wrist.attach(6, 500, 2500); wrist.write(135);
  claw.attach(8, 500, 2500); claw.write(45);
  upperElbow.attach(9, 500, 2500); upperElbow.write(90);
  lowerElbow.attach(12, 500, 2500); lowerElbow.write(90);

  // 1. Setup I2C on Wire1
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();

  // 2. Initialize BNO055
  if (bno.begin()) {
    bno.setExtCrystalUse(true);
  }

  // 3. Initialize BMP388
  if (bmp.begin_I2C(0x77, &Wire1) || bmp.begin_I2C(0x76, &Wire1)) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // 4. Setup SPI pins & initialize SD card
  SPI.setRX(_MISO);
  SPI.setTX(_MOSI);
  SPI.setSCK(_SCK);
  SPI.begin();

  if (SD.begin(_CS)) {
    bool fileExists = SD.exists("data.csv");
    
    // Open file persistently in write mode
    dataFile = SD.open("data.csv", FILE_WRITE);
    
    if (dataFile && !fileExists) {
      dataFile.println("Time_ms,Heading_deg,Pitch_deg,Roll_deg,Temp_C,Pressure_hPa,Altitude_m");
      dataFile.flush();
    }
  }
}

void loop() {
  // --- 1. ROS2 Servo Control (Parsing string characters safely) ---
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    int axis0, lower, upper, w, clawVal;
    if (sscanf(line.c_str(), "A,%d,%d,%d,%d,%d", &axis0, &lower, &upper, &w, &clawVal) == 5) {
      lowerElbow.write(constrain(lower, 0, 100));
      upperElbow.write(constrain(upper, 0, 180));
      wrist.write(constrain(w, 0, 180));
      claw.write(constrain(clawVal, 0, 180));
    }
  }

  // --- 2. Telemetry SD Logging ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentMillis;

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    bool bmpSuccess = bmp.performReading();
    float temp = bmpSuccess ? bmp.temperature : 0.0;
    float pressure = bmpSuccess ? (bmp.pressure / 100.0) : 0.0;
    float altitude = bmpSuccess ? bmp.readAltitude(SEALEVELPRESSURE_HPA) : 0.0;

    // Fast append without re-opening the file system cluster maps
    if (dataFile) {
      dataFile.print(currentMillis); dataFile.print(",");
      dataFile.print(orientationData.orientation.x); dataFile.print(",");
      dataFile.print(orientationData.orientation.y); dataFile.print(",");
      dataFile.print(orientationData.orientation.z); dataFile.print(",");
      dataFile.print(temp); dataFile.print(",");
      dataFile.print(pressure); dataFile.print(",");
      dataFile.println(altitude);
    }
  }

  // --- 3. Periodic Flush ---
  if (currentMillis - lastFlushTime >= FLUSH_INTERVAL_MS) {
    lastFlushTime = currentMillis;
    if (dataFile) {
      dataFile.flush(); // Forces writing data to physical SD card blocks seamlessly
    }
  }
}
