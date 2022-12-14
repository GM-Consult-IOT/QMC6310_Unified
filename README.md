# QMC6310 (3-Axis Magnetometer) Unified Sensor Driver

A Unified Sensor driver for the QMC6310 magnetometer/compass IC from QST Corporation.

## Contents
- [QMC6310 (3-Axis Magnetometer) Unified Sensor Driver](#qmc6310-3-axis-magnetometer-unified-sensor-driver)
  - [Contents](#contents)
  - [Overview](#overview)
  - [About the QMC6310](#about-the-qmc6310)
  - [Dependencies](#dependencies)
  - [Usage](#usage)
  - [Calibration](#calibration)
  - [Licensing](#licensing)
  - [Issues](#issues)

## Overview

This library is a driver for the [QMC6310 magnetometer/compass IC from QST Corporation](https://www.qstcorp.com/en_comp_prod/QMC6310). It implements the [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) interface.

(*[back to top](#)*)

## About the QMC6310

The QMC6310 is a three-axis magnetic sensor, which integrates magnetic sensors and a signal condition ASIC into one silicon chip.  The QMC6310 enables 1° to 2° compass heading accuracy. The I²C serial bus allows for easy interface.

More information on the QMC6310 can be found in the [datasheet](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/main/assets/QMC6310_Datasheet.pdf).

(*[back to top](#)*)

## Dependencies

Requires the [Adafruit Unified Sensor by Adafruit](https://github.com/adafruit/Adafruit_Sensor), version ^1.1.6.

(*[back to top](#)*)

## Usage

The driver is consistent with the `Adafruit Unified Sensor` abstraction layer. The sensor is initialized in the `setup` method by calling `begin` on a `QMC6310_Unified` driver instance. In the `loop` method, define a `sensors_event_t` event and pass it to the driver's `getEvent` method to populate the event with the most recent magnetic flux vectors.

main.cpp
```C++

#include <Arduino.h>
#include <Wire.h> 
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

```

Refer to the [sensor example](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/examples/QMC6310_sensor/QMC6310_sensor.ino) for calculation of heading.

(*[back to top](#)*)

## Calibration

Calibration is essential to obtain reasonable heading outputs from the measured flux vectors. Without calibration the calculated headings are not only offset but also non-linear with respect to the compass directions. To obtain accurate headings, your implementation should also apply a tilt correction (from a 3-axis accelerometer/gyro).

More information on sensor calibration [here](https://www.digikey.com.au/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b).

(*[back to top](#)*)

## Licensing

This library is open-source under the [BSD 3-Clause license](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE) and redistribution and use in source and binary forms, with or without modification, are permitted, provided that the license conditions are met.

The original HMC5883 driver was written by Kevin Townsend for Adafruit Industries. The Adafruit library is open-source under the [GPL-3.0 license](https://www.gnu.org/licenses/gpl-3.0.en.html). *Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!*

(*[back to top](#)*)

## Issues

If you find a bug please fill an [issue](https://github.com/GM-Consult-IOT/QMC6310_Unified/issues).  

(*[back to top](#)*)