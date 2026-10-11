//following tutorial:https://arduinogetstarted.com/tutorials/arduino-log-data-with-timestamp-to-sd-card
//All changes will have a precending comment "Sage" and seceeding comment "end"


#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include "Adafruit_BMP3XX.h"
#include "BNO055_support.h"

//Sage
#include <SD.h>
#include <RTClib.h>
//end

unsigned long lastTime = 0;
static constexpr int I2CSDA = 4;
static constexpr int I2CSCL = 5;


//Sage
//May need to change pin
#define PIN_SPI_CS 4

// Make sure the file on the sd matches file name
#define FILE_NAME "log.txt"
//end

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
  
  //Sage
  //RTC Module Setup 
  if (!rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    while (1);
  }



  if (!SD.begin(PIN_SPI_CS)){
    Serial.println(F("SD CARD FAILED, OR NOT PRESENT!"));
    while (1); // don't do anything else
  }

  Serial.println(F("SD CARD INITIALIZED."));
  Serial.println(F("--------------------"));
  //end


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
  
  //sage
  // open file for writing
  myFile = SD.open(FILE_NAME, FILE_WRITE);

  //should print to file
  if(myFILE){
    Serial.println(F("Writing log to SD Card"))

    if ((millis() - lastTime) >= 100) {

    lastTime = millis();
    Serial.print("Time Stamp: ");			
    Serial.println(lastTime);
    myFile.print("Time Stamp: ");			
    myFile.println(lastTime);
    myFile.print(" ")
    if (bmp.performReading()) {
      //print to  serial
      Serial.print("Temperature = ");
      Serial.print(bmp.temperature);
      Serial.println(" *C");

      Serial.print("Pressure = ");
      Serial.print(bmp.pressure / 100.0);
      Serial.println(" hPa");

      Serial.print("Approx. Altitude = ");
      Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");

      //Ptint to file
      myFile.print("Temperature = ");
      myFile.print(bmp.temperature);
      myFile.println(" *C");

      myFile.print("Pressure = ");
      myFile.print(bmp.pressure / 100.0);
      myFile.println(" hPa");

      myFile.print("Approx. Altitude = ");
      myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      myFile.println(" m");

      
    } 
    else {
      Serial.println("Failed to perform reading :(");
    }
    myFile.write("\n");

    myFile.close();
    }
    else{
      Serial.print(F("SD Card: error on opening file "));
      Serial.println(FILE_NAME)
    }
//end
  

    bno055_read_euler_hrp(&myEulerData);		

    Serial.print("Heading(Yaw): ");				
    Serial.println(float(myEulerData.h) / 16.00);		

    Serial.print("Roll: ");					
    Serial.println(float(myEulerData.r) / 16.00);		

    Serial.print("Pitch: ");				
    Serial.println(float(myEulerData.p) / 16.00);		

    Serial.println();					


      
    }
  }
}



