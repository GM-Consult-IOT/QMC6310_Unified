/*
Use this calibration sketch to calibrate your QMC6310 device/module.
After upload to the device, run the serial monitor and follow the directions.
When prompted, copy the last line into the `setup()` method of your project's 
actual sketch.

***************************************************************************
  This library is a [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) 
  driver for the [QMC6310 magnetometer/compass IC from QST Corporation](https://www.qstcorp.com/en_comp_prod/QMC6310). 
  
  The QMC6310 is a three-axis magnetic sensor, which integrates magnetic 
  sensors and signal condition ASIC into one silicon chip.  The QMC6310 
  enables 1° to 2° compass heading accuracy. The I²C serial bus allows for 
  easy interface.
  
  The library is based on the [Adafruit HMC5883L Driver](https://github.com/adafruit/Adafruit_QMC6310_Unified) 
  for the [Adafruit HMC5883 Breakout](http://www.adafruit.com/products/1746).
 
  The original HMC5883 driver was written by Kevin Townsend for Adafruit 
  Industries with some heading example from [Love Electronics](loveelectronics.co.uk). 
  The Adafruit library is open-source under the 
  [GPL-3.0 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

  The calibration methods are from the [QMC5883L Compass Arduino Library by 
  MPrograms](https://github.com/mprograms/QMC5883LCompass/), based on the 
  work of [Claus Näveke - TheNitek](https://github.com/TheNitek) and is 
  open-source under the [GPL-3.0 license](https://github.com/mprograms/QMC5883LCompass/blob/master/LICENSE).
  
  *** You will also need to install the Adafruit_Sensor library! ***

  *Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!*
 
  This library is open-source under the [BSD 3-Clause license](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE) 
  and redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the license conditions are met.

 ***************************************************************************/

#include <Arduino.h>
#include <inttypes.h>
#include <Wire.h> 
#include <QMC6310_Unified.h>

QMC6310_Unified compass;

int calibrationData[3][2];
bool changed = false;
bool done = false;
int t = 0;
int c = 0;

void setup() {
  Serial.begin(115200);
  compass.begin();
  
  Serial.println("This will provide calibration settings for your QMC5883L chip. When prompted, move the magnetometer in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);
  
}

void loop() {
  int x, y, z;
  
  sensors_event_t event; 
  compass.getEvent(&event);

  // Return XYZ readings
  x = event.magnetic.x;
  y = event.magnetic.y;
  z = event.magnetic.z;

  changed = false;

  if(x < calibrationData[0][0]) {
    calibrationData[0][0] = x;
    changed = true;
  }
  if(x > calibrationData[0][1]) {
    calibrationData[0][1] = x;
    changed = true;
  }

  if(y < calibrationData[1][0]) {
    calibrationData[1][0] = y;
    changed = true;
  }
  if(y > calibrationData[1][1]) {
    calibrationData[1][1] = y;
    changed = true;
  }

  if(z < calibrationData[2][0]) {
    calibrationData[2][0] = z;
    changed = true;
  }
  if(z > calibrationData[2][1]) {
    calibrationData[2][1] = z;
    changed = true;
  }

  if (changed && !done) {
    Serial.println("CALIBRATING... Keep moving your sensor around.");
    c = millis();
  }
    t = millis();
  
  
  if ( (t - c > 5000) && !done) {
    done = true;
    Serial.println("DONE. Copy the line below and paste it into your projects sketch.);");
    Serial.println();
      
    Serial.print("compass.setCalibration(");
    Serial.print(calibrationData[0][0]);
    Serial.print(", ");
    Serial.print(calibrationData[0][1]);
    Serial.print(", ");
    Serial.print(calibrationData[1][0]);
    Serial.print(", ");
    Serial.print(calibrationData[1][1]);
    Serial.print(", ");
    Serial.print(calibrationData[2][0]);
    Serial.print(", ");
    Serial.print(calibrationData[2][1]);
    Serial.println(");");
    }
  
 
}