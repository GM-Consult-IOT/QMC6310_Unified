# QMC6310  (3-Axis Magnetometer) Unified Sensor Driver

A driver for the QMC6310 magnetometer/compass IC from QST Corporation.

## Contents
- [QMC6310  (3-Axis Magnetometer) Unified Sensor Driver](#qmc6310--3-axis-magnetometer-unified-sensor-driver)
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

The QMC6310 is a three-axis magnetic sensor, which integrates magnetic sensors and signal condition ASIC into one silicon chip.  The QMC6310 enables 1° to 2° compass heading accuracy. The I²C serial bus allows for easy interface.

More information on the QMC6310 can be found in the [datasheet](https://github.com/GM-Consult-IOT/libraries/blob/main/datasheets/QMC6310_magnetometer_qst.pdf).

(*[back to top](#)*)

## Dependencies

Requires the [Adafruit Unified Sensor by Adafruit](https://github.com/adafruit/Adafruit_Sensor).

(*[back to top](#)*)

## Usage

The driver is consistent with the `Adafruit Unified Sensor` abstraction layer. The sensor is initialized in the `setup` method by calling `begin` on a `QMC6310_Unified` driver instance. In the `loop` method, define a `sensors_event_t` event and pass it to the driver's `getEvent` method to populate the event with the most recent magnetic flux vectors.

main.cpp
```C++
#include <Arduino.h>
#include <Wire.h> 
#include <QMC6310_Unified.h>

// Hydrate a sensor and assign a unique ID 
QMC6310_Unified mag = QMC6310_Unified(12345);

void setup() {
  
  // start the i2c comms
  Wire.begin(); 
  
  // start serial port
  Serial.begin(115200); 

  // Initialise the sensor and handle status errors
  if (!mag.begin()){
    Serial.println("Error connecting to QMC6310. Status code is " + String(mag.status()));
    while(1);
  }
}

void loop() {

    // Get a new sensor event
  sensors_event_t event; 
  mag.getEvent(&event);

  // Calculate heading (in radians) from the event.magnetic vectors.
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  
}

```

Refer to the [sensor example](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/examples/QMC6310_sensor/QMC6310_sensor.ino) for calculation of heading.

(*[back to top](#)*)

## Calibration

The driver allows for the setting of a calibration array using the `setCalibration` method. The [setCalibration] method requires six integer values `X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN and Z_MAX`. These are the maximum and minimum magnetic flux vector values obtained when the sensor is rotated fully around all three axes. The calibration values are used to scale and offset the raw readings.

The [calibration example](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/examples/QMC6310_calibration/QMC6310_calibration.ino) contains a sketch that shows how the calibration values may be obtained.

(*[back to top](#)*)

## Licensing

This library is open-source under the [BSD 3-Clause license](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE) and redistribution and use in source and binary forms, with or without modification, are permitted provided that the license conditions are met.

The original HMC5883 driver was written by Kevin Townsend for Adafruit Industries with some heading example from [Love Electronics](loveelectronics.co.uk). The Adafruit library is open-source under the [GPL-3.0 license](https://www.gnu.org/licenses/gpl-3.0.en.html). *Adafruit invests time and resources providing this open source code, please support Adafruit and open-source hardware by purchasing products from Adafruit!*

The calibration methods are from the [QMC5883L Compass Arduino Library by MPrograms](https://github.com/mprograms/QMC5883LCompass/), based on the work of [Claus Näveke - TheNitek](https://github.com/TheNitek) and is open-source under the [GPL-3.0 license](https://github.com/mprograms/QMC5883LCompass/blob/master/LICENSE).

(*[back to top](#)*)

## Issues

If you find a bug please fill an [issue](https://github.com/GM-Consult-IOT/QMC6310_Unified/issues).  

(*[back to top](#)*)