#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include "Adafruit_BMP3XX.h"
#include "BNO055_support.h"

#include <SD.h>
#include <RTClib.h>

unsigned long lastTime = 0;
static constexpr int I2CSDA = 4;
static constexpr int I2CSCL = 5;

#define PIN_SPI_CS 10
#define FILE_NAME "log.txt"
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;
struct bno055_t myBNO;
struct bno055_euler myEulerData;

RTC_DS3231 rtc;
File myFile;

void setup() {
  Serial.begin(115200);
  Wire.setSDA(I2CSDA);
  Wire.setSCL(I2CSCL);
  Wire.begin();

if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }
  if (!SD.begin(PIN_SPI_CS)) {
    Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    while (1);
  }
  Serial.println("SD CARD INITIALIZED.");
  Serial.println("--------------------");

  if (!bmp.begin_I2C()) {  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  BNO_Init(&myBNO); 
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  delay(10);

}

void loop() {
  if ((millis() - lastTime) >= 100) {

    lastTime = millis();

    myFile = SD.open(FILE_NAME, FILE_WRITE);

    Serial.print("Time Stamp: ");			
    Serial.println(lastTime);

    if(myFile){

      Serial.println("Writing log to SD Card");

      DateTime now = rtc.now();

      myFile.print(now.year()); myFile.print('/');
      myFile.print(now.month()); myFile.print('/');
      myFile.print(now.day()); myFile.print(" ");
      myFile.print(now.hour()); myFile.print(':');
      myFile.print(now.minute()); myFile.print(':');
      myFile.print(now.second()); myFile.print(", ");

      if (bmp.performReading()) {

        myFile.print("Temp: ");
        myFile.print(bmp.temperature);
        myFile.print(" *C, Press: ");
        myFile.print(bmp.pressure / 100.0);
        myFile.print(" hPa, Alt: ");
        myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
        myFile.print(" m, ");

        Serial.print("Temperature = ");
        Serial.print(bmp.temperature);
        Serial.println(" *C");

        Serial.print("Pressure = ");
        Serial.print(bmp.pressure / 100.0);
        Serial.println(" hPa");

        Serial.print("Approx. Altitude = ");
        Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");
      } else {
        Serial.println("Failed to perform reading :(");
      }

      bno055_read_euler_hrp(&myEulerData);		

      Serial.print("Heading(Yaw): ");				
      Serial.println(float(myEulerData.h) / 16.00);		

      Serial.print("Roll: ");					
      Serial.println(float(myEulerData.r) / 16.00);		

      Serial.print("Pitch: ");				
      Serial.println(float(myEulerData.p) / 16.00);		

      float yaw = float(myEulerData.h) / 16.00;
      float roll = float(myEulerData.r) / 16.00;
      float pitch = float(myEulerData.p) / 16.00;

      myFile.print("Yaw: "); myFile.print(yaw);
      myFile.print(", Roll: "); myFile.print(roll);
      myFile.print(", Pitch: "); myFile.println(pitch);

      myFile.close();

    } else {
    Serial.print("SD Card: error on opening file ");
    Serial.println(FILE_NAME);
    }
  }				
}




