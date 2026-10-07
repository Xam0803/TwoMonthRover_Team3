#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include "Adafruit_BMP3XX.h"
#include "BNO055_support.h"

unsigned long lastTime = 0;
static constexpr int I2CSDA = 4;
static constexpr int I2CSCL = 5;

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;
struct bno055_t myBNO;
struct bno055_euler myEulerData;

void setup() {
  Serial.begin(115200);
  Wire.setSDA(I2CSDA);
  Wire.setSCL(I2CSCL);
  Wire.begin();

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
    Serial.print("Time Stamp: ");			
    Serial.println(lastTime);

    if (bmp.performReading()) {
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

    Serial.println();					
  }
}



