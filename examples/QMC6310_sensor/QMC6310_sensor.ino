/***************************************************************************
*  A Unified Sensor (1) driver for the QMC6310 magnetometer/compass IC (2) 
*  from QST Corporation. 
* 
* VERSION 4 - BREAKING CHANGES
*
* Version 4 of the QMC6310_Unified driver library is a complete re-write. 
* This means many of the internals and non-interface elements have changed. 
* The Adafruit Unified Sensor interface is still strictly implemented, 
* however, so the breaking changes will only affect code that used 
* device-specific fields, enums and structs.
* 
* We have also added a host of methods that allow setting of device 
* parameters and reading the device registers. This allows implementers to 
* customize the device and also obtain diagnostic data during operation.
*    
*  The QMC6310 is a three-axis magnetic sensor with I²C serial interface that 
*  integrates magnetic sensors and a signal condition ASIC into one silicon 
*  chip.
*  
*  The library requires the `adafruit/Adafruit Unified Sensor@^1.1.6`.
*  
*  This library is open-source under the BSD 3-Clause license (3) and 
*  redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the license conditions are met.
*
*  References:
*  1. https://github.com/adafruit/Adafruit_Sensor
*  2. https://www.qstcorp.com/en_comp_prod/QMC6310
*  3. https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE
****************************************************************************/

// include the driver (it already includes <arduino.h> and <wire.h>)
#include <QMC6310_Unified.h>

// hydrate the sensor driver and assign a unique ID
// set the `verbose` flag to true if you want to print debug information to the serial monitor
QMC6310_Unified mag_sensor = QMC6310_Unified(12345, false);

void setup() {
  
  // start i2c comms
  Wire.begin(); 
  
  // start serial port
  Serial.begin(115200); 

  // initialise the sensor and handle status errors
  if (!mag_sensor.begin()){

    // echo the error to serial monitor
    Serial.println("Error connecting to QMC6310.") ;
    while(1);
  } 

  // print the sensor details to serial terminal
  mag_sensor.printSensorDetails();

  // print the sensor settings to serial terminal
  mag_sensor.printSensorSettings();

  /***  EXAMPLE of SETTING NON-DEFAULT DEVICE PARAMETERS  ***/
  // set the flux range to +/- 2 Gauss
  mag_sensor .setFieldRange(QMC6310_FLD_RNG_2);
  // set the output data rate (ODR) to 50Hz 
  mag_sensor.setDataRate(QMC6310_DATA_RATE_50);
  // set the over-sampling rate (OSR) to 8x 
  mag_sensor.setOSR(QMC6310_OSR_8);
  // print the sensor details to serial terminal
  mag_sensor.printSensorDetails();

  // print the sensor settings again to see the changes
  mag_sensor.printSensorSettings();

}

// Calculate a uncalibrated magnetic heading from the sensor output.
float getHeadingMagnetic(sensors_event_t *event){
  
  // calculate heading and correct for declination
  float heading = atan2(event->magnetic.y, event->magnetic.x);
  
  // correct for when signs are reversed
  if(heading < 0) heading += 2*PI;
    
  // check for wrap due to addition of declination
  if(heading > 2*PI) heading -= 2*PI;

  // convert radians to degrees and return
  return heading * 180/M_PI;
}

void loop() {

  // initialize the sensor event
  sensors_event_t event; 

  // populate the event properties with latest magnetometer values
  mag_sensor.getEvent(&event);

  // print the heading to the serial terminal window  
  Serial.println("Heading: " + String(getHeadingMagnetic(&event), 0)+ "°M"); 

  // wait a second
  delay(1000);
  
}