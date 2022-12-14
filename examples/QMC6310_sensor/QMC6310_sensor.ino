/***************************************************************************
A Unified Sensor (1) driver for the QMC6310 magnetometer/compass IC (2) 
from QST Corporation. 

The QMC6310 is a three-axis magnetic sensor with I²C serial interface that 
integrates magnetic sensors and a signal condition ASIC into one silicon 
chip.

The library requires the `adafruit/Adafruit Unified Sensor@^1.1.6`.

The library is based on the Adafruit HMC5883L Driver (3) for the Adafruit 
HMC5883 Breakout (4). The original HMC5883 driver was written by Kevin 
Townsend for Adafruit Industries. The Adafruit HMC5883L Driver library is 
open-source under the GPL-3.0 license (5). Adafruit invests time and 
resources providing this open source code, please support Adafruit and 
open-source hardware by purchasing products from Adafruit.

This library is open-source under the BSD 3-Clause license (6) and 
redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the license conditions are met.

References:
1. https://github.com/adafruit/Adafruit_Sensor
2. https://www.qstcorp.com/en_comp_prod/QMC6310
4. https://github.com/adafruit/Adafruit_HMC5883_Unified
4. http://www.adafruit.com/products/1746
5. https://www.gnu.org/licenses/gpl-3.0.en.html
6. https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE

****************************************************************************/

// include the driver (it already includes <arduino.h> and <wire.h>)
#include <QMC6310_Unified.h>

// hydrate the sensor driver and assign a unique ID 
QMC6310_Unified mag_sensor = QMC6310_Unified(12345);

void setup() {
  
  // start i2c comms
  Wire.begin(); 
  
  // start serial port
  Serial.begin(115200); 

  // print a separator to serial terminal
  Serial.println(F("\n------------------------------------"));

  // initialise the sensor and handle status errors
  if (!mag_sensor.begin()){

    // echo the error to serial monitor
    Serial.println("Error connecting to QMC6310.") ;
    while(1);
  } 

  // echo the device status code to serial monitor
  Serial.println("QMC6310 STATUS 0x" + String(mag_sensor.status(), HEX));

  // echo the sensor details to serial terminal
  mag_sensor.printSensorDetails();

}

void loop() {

  // initialize the sensor event
  sensors_event_t event; 

  // populate the event properties with latest magnetometer values
  mag_sensor.getEvent(&event);

  // echo the results to serial terminal (magnetic vector values are in micro-Tesla (uT))
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // wait a second
  delay(1000);
  
}