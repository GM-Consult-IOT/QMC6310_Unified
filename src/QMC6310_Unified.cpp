/*!
 * @file QMC6310_Unified.cpp
 *
 * @mainpage QMC6310 Unified Library
 *
 * @section intro_sec Introduction
 *
 * A Unified Sensor (1) driver for the QMC6310 magnetometer/compass IC (2) 
 * from QST Corporation. 
 * 
 * The QMC6310 is a three-axis magnetic sensor with I²C serial interface that 
 * integrates magnetic sensors and a signal condition ASIC into one silicon 
 * chip.
 * 
 * The library requires the `adafruit/Adafruit Unified Sensor@^1.1.6`.
 * 
 * The library is based on the Adafruit HMC5883L Driver (3) for the Adafruit 
 * HMC5883 Breakout (4). The original HMC5883 driver was written by Kevin 
 * Townsend for Adafruit Industries. The Adafruit HMC5883L Driver library is 
 * open-source under the GPL-3.0 license (5). Adafruit invests time and 
 * resources providing this open source code, please support Adafruit and 
 * open-source hardware by purchasing products from Adafruit.
 * 
 * This library is open-source under the BSD 3-Clause license (6) and 
 * redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the license conditions are met.
 * 
 * References:
 * 1. https://github.com/adafruit/Adafruit_Sensor
 * 2. https://www.qstcorp.com/en_comp_prod/QMC6310
 * 4. https://github.com/adafruit/Adafruit_HMC5883_Unified
 * 4. http://www.adafruit.com/products/1746
 * 5. https://www.gnu.org/licenses/gpl-3.0.en.html
 * 6. https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution.
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

#include <limits.h>

#include "QMC6310_Unified.h"

static float _QMC6310_Gauss_LSB = 2500; 

void QMC6310_Unified::write8(byte address, byte reg, byte value) {
  Wire.beginTransmission(address);
#if ARDUINO >= 100
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
#else
  Wire.send(reg);
  Wire.send(value);
#endif
  Wire.endTransmission();
}

byte QMC6310_Unified::read8(byte address, byte reg) {
  byte value;

  Wire.beginTransmission(address);
#if ARDUINO >= 100
  Wire.write((uint8_t)reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)1);
#if ARDUINO >= 100
  value = Wire.read();
#else
  value = Wire.receive();
#endif
  Wire.endTransmission();

  return value;
}

float QMC6310_Unified::sensitivity() {
  return _QMC6310_Gauss_LSB;
}


void QMC6310_Unified::read() {
  // Read the magnetometer
  Wire.beginTransmission((byte)QMC6310_ADDRESS_MAG);
#if ARDUINO >= 100
  Wire.write(QMC6310_REGISTER_MAG_OUT_X_L_M);
#else
  Wire.send(QMC6310_REGISTER_MAG_OUT_X_H_M);
#endif
  Wire.endTransmission(false);
  Wire.requestFrom((byte)QMC6310_ADDRESS_MAG, (byte)6, (uint8_t)1);
#if ARDUINO >= 100
  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();
#else
  uint8_t xhi = Wire.receive();
  uint8_t xlo = Wire.receive();
  uint8_t zhi = Wire.receive();
  uint8_t zlo = Wire.receive();
  uint8_t yhi = Wire.receive();
  uint8_t ylo = Wire.receive();
#endif

// Shift values to create properly formed integer (low byte first)
  _magData.x = (int16_t)(xlo | ((int16_t)xhi << 8));
  _magData.y = (int16_t)(ylo | ((int16_t)yhi << 8));
  _magData.z = (int16_t)(zlo | ((int16_t)zhi << 8));
}

QMC6310_Unified::QMC6310_Unified(int32_t sensorID) {
  _sensorID = sensorID;
}

bool QMC6310_Unified::begin() {

  // Enable I2C
  Wire.begin();

  // Define the sign for X Y and Z axis)
  write8(QMC6310_ADDRESS_MAG, QMC6310_REGISTER_MAG_SGN_REG_M, 0x02);

  // Set the gain to a known level
  setMagGain();

  // Write to control register 1
  //  - set normal mode 
  //  - set output data rate (ODR) 200kHz
  write8(QMC6310_ADDRESS_MAG, QMC6310_REGISTER_MAG_CRA_REG_M, 0xC3);

  // get the STATUS register value and return true if it not 0x00
  return status() > 0x00;
  // return true;
}


byte QMC6310_Unified::status() {  
  // Write to the STATUS register to request status
  Wire.beginTransmission(QMC6310_ADDRESS_MAG);  
  #if ARDUINO >= 100
  Wire.write(0x09);
  Wire.endTransmission(false);
  #else
  Wire.send(QMC6310_REGISTER_MAG_SR_REG_Mg);
  #endif
  // read the value and update _staus
  Wire.requestFrom((byte)QMC6310_ADDRESS_MAG, (byte)1);
  #if ARDUINO >= 100
  _status = Wire.read();
  #else
  _status = Wire.receive();
  #endif
  return _status;
}

void QMC6310_Unified::setMagGain(qmc6310MagGain gain) {
  // write the new sensitivity field range to the device control register 2
  write8(QMC6310_ADDRESS_MAG, QMC6310_REGISTER_MAG_CRB_REG_M, (byte)gain);
  // set the gain LSB/Gaus value
  switch (gain) {
  case QMC6310_MAGGAIN_30:
    _QMC6310_Gauss_LSB = 1000;
    break;
  case QMC6310_MAGGAIN_12:
    _QMC6310_Gauss_LSB = 2500;
    break;
  case QMC6310_MAGGAIN_8:
    _QMC6310_Gauss_LSB = 3750;
    break;
  case QMC6310_MAGGAIN_2:
    _QMC6310_Gauss_LSB = 15000;
    break;

  }
}

bool QMC6310_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = 0;
  event->magnetic.x =
      _magData.x / _QMC6310_Gauss_LSB * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y =
      _magData.y / _QMC6310_Gauss_LSB * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z =
      _magData.z / _QMC6310_Gauss_LSB * SENSORS_GAUSS_TO_MICROTESLA;
  return true;
}

void QMC6310_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, SENSOR_NAME, sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = DRIVER_VERSION;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;    // minimum delay 0
  sensor->max_value = 800;  // 8 gauss == 800 microTesla
  sensor->min_value = -800; // -8 gauss == -800 microTesla
  sensor->resolution = 0.2; // 2 milligauss == 0.2 microTesla
}
