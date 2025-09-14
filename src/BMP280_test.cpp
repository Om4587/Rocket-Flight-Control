#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>


Adafruit_BMP280 bmp; 

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));

  if (!bmp.begin(0x76)) {   
    Serial.println(F("Could not find BMP280 sensor!"));
    while (1);
  }


  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
}

void loop() {
Serial.print(bmp.readTemperature());
Serial.print(",");         
Serial.print(bmp.readPressure()/100); 
Serial.print(",");
Serial.println(bmp.readAltitude(1013.23)); 

delay(500);
}