/***************************************************************************
  This library is a Adafruit Unified Sensor (1) driver for the QMC6310 
  magnetometer/compass IC from QST Corporation (2). 
  
  The QMC6310 is a three-axis magnetic sensor, which integrates magnetic 
  sensors and signal condition ASIC into one silicon chip.  The QMC6310 
  enables 1° to 2° compass heading accuracy. The I²C serial bus allows for 
  easy interface.
  
  The library is based on the Adafruit HMC5883L Driver (3) for the Adafruit 
  HMC5883 Breakout (4).
 
  The original HMC5883 driver was written by Kevin Townsend for Adafruit 
  Industries. The Adafruit library is open-source under the 
  [GPL-3.0 license] (5).

  The calibration methods are from the QMC5883L Compass Arduino Library by 
  MPrograms(6), based on the work of [Claus Näveke - TheNitek](7) and is 
  open-source under the GPL-3.0 license(8).
  
  *** You will also need to install the Adafruit_Sensor library! ***

  *Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!*
 
  This library is open-source under the BSD 3-Clause license (9) and 
  redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the license conditions are met.

  References:
  1. https://github.com/adafruit/Adafruit_Sensor
  2. https://www.qstcorp.com/en_comp_prod/QMC6310
  4. https://github.com/adafruit/Adafruit_HMC5883_Unified
  4. http://www.adafruit.com/products/1746
  5. https://www.gnu.org/licenses/gpl-3.0.en.html
  6. https://github.com/mprograms/QMC5883LCompass/
  7. https://github.com/TheNitek
  8. https://github.com/mprograms/QMC5883LCompass/blob/master/LICENSE
  9. https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE

 ***************************************************************************/
#include <Arduino.h>
#include <inttypes.h>
#include <Wire.h> 
#include <QMC6310_Unified.h>

/* Assign a unique ID to this sensor at the same time */
QMC6310_Unified mag = QMC6310_Unified(12345);

// Prints basic information about the QMC6310 sensor to the serial port.
void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);  
  Serial.print  ("Status:       "); Serial.println(mag.status());
  Serial.print  ("Sensitivity:  "); Serial.println(mag.sensitivity());
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(2500);
}

void setup() {

  // start the i2c comms
  Wire.begin(); 

  // start serial port
  Serial.begin(115200); 

  // send something to the serial port
  Serial.println("Starting up..."); 
    
  // Initialise the sensor and handle status errors 
  if (!mag.begin()){
    Serial.println("Error connecting to QMC6310. Status code is " + String(mag.status()));
    while(1);
  }
  
  // Display basic information about the QMC6310 sensor.
  displaySensorDetails();
  
  /* 
  Uncomment the line below to set the calibration values `X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN and Z_MAX` 
  if you have these. You can get an idea of the max/min for each axis by looking at the serial port output,
  or use the sketch in the examples.
  */
  // mag.setCalibration(-43, 78, -53, 71, -65, 61);
}

void loop() {

  // Get a new sensor event
  sensors_event_t event; 
  mag.getEvent(&event);

  // Display the results (magnetic vector values are in micro-Tesla (uT)).  You can use the maximum and minimum values
  // from this terminal listing to populate the paramaters for mag.setCalibration(X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX).
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x and y vectors.
  // Calculate heading when the magnetometer is level, then correct for the signs of the axes.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  /*
  Once you have your heading, you can add the declination angle, at your location. You can find the 
  declination angle by searching the web or looking it up at http://www.magnetic-declination.com/.
  I used ~15.44 degrees (0.27 radians) for Tasmania. 
  */
  float declinationAngle = 0.27;
  heading += declinationAngle;  

  // Correct the heading for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap (more than 360 degrees) due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  // Send the heading to the serial port
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
 
  // wait a second
  delay(1000); 
}