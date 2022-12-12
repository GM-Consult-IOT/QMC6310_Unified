/*!
 * @file QMC6310_U.cpp
 *
 * @mainpage Adafruit HMC5883 Unified Library
 *
 * @section intro_sec Introduction
 *
 * This library is a `Unified Sensor` driver for the QMC6310 magnetometer/compass IC from QST Corporation.
 * 
 * The library is a fork of the [Adafruit driver](https://github.com/adafruit/Adafruit_QMC6310_Unified) 
 * for the [Adafruit HMC5883 Breakout](http://www.adafruit.com/products/1746).
 *
 * Adafruit invests time and resources providing this open source code, please support Adafruit and 
 * open-source hardware by purchasing products from Adafruit!
 *
 * @section author Author
 *
 * Adapted by Gerhard Malan from original work by Kevin Townsend for Adafruit Industries.
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

static float _QMC6310_Gauss_LSB = 2500; // Varies with gain
// static float _QMC6310_Gauss_LSB_Z = QMC6310_MAGGAIN_8;   // Varies with gain
// static byte _status = 0x00; 

// static qmc6310MagGain _fieldRange = QMC6310_MAGGAIN_12;
/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief  Returns the current magnetometer sensitivity.
*/
/**************************************************************************/
float QMC6310_Unified::sensitivity() {
  return _QMC6310_Gauss_LSB;
}


/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void QMC6310_Unified::read() {
  // Read the magnetometer
  Wire.beginTransmission((byte)QMC6310_ADDRESS_MAG);
#if ARDUINO >= 100
  Wire.write(QMC6310_REGISTER_MAG_OUT_X_L_M);
#else
  Wire.send(QMC6310_REGISTER_MAG_OUT_X_H_M);
#endif
  Wire.endTransmission(false);
  uint8_t ret = Wire.requestFrom((byte)QMC6310_ADDRESS_MAG, (byte)6, (uint8_t)1);
// Note high before low (different than accel)
#if ARDUINO >= 100
  // while(Wire.available()<6); //Wait if above blocking then this not needed.
  //   _magData.x   = (int16_t)(Wire.read() | Wire.read() << 8);
  //   _magData.y  = (int16_t)(Wire.read() | Wire.read() << 8);
  //   _magData.z  = (int16_t)(Wire.read() | Wire.read() << 8);
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

  // ToDo: Calculate orientation
  _magData.orientation = 0.0;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new QMC6310_Unified class
*/
/**************************************************************************/
QMC6310_Unified::QMC6310_Unified(int32_t sensorID) {
  _sensorID = sensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool QMC6310_Unified::begin() {

  // Enable I2C
  Wire.begin();

  // Define the sign for X Y and Z axis)
  write8(QMC6310_ADDRESS_MAG, QMC6310_REGISTER_MAG_SGN_REG_M, 0x02);

  // Write to CONTROL2 register 
  //  - Define Set/Reset mode, with Set/Reset On, 
  //  - set Field Range 8 Gauss
  write8(QMC6310_ADDRESS_MAG, QMC6310_REGISTER_MAG_CRB_REG_M, _fieldRange);

  // Write to control register 1
  //  - set normal mode 
  //  - set output data rate (ODR) 200kHz
  write8(QMC6310_ADDRESS_MAG, QMC6310_REGISTER_MAG_CRA_REG_M, 0xC3);

  // Set the gain to a known level
  setMagGain(_fieldRange);

  // get the STATUS register value and return true if it not 0x00
  return status() > 0x00;
  // return true;
}


byte QMC6310_Unified::status() {  
  uint8_t ret;
  // Write to the STATUS register to request status
  Wire.beginTransmission(QMC6310_ADDRESS_MAG);  
  #if ARDUINO >= 100
  Wire.write(0x09);
  ret = Wire.endTransmission(false);
  #else
  Wire.send(QMC6310_REGISTER_MAG_SR_REG_Mg);
  #endif
  // read the value and update _staus
  ret = Wire.requestFrom((byte)QMC6310_ADDRESS_MAG, (byte)1);
  #if ARDUINO >= 100
  _status = Wire.read();
  #else
  _status = Wire.receive();
  #endif
  return _status;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void QMC6310_Unified::setMagGain(qmc6310MagGain gain) {
  write8(QMC6310_ADDRESS_MAG, QMC6310_REGISTER_MAG_CRB_REG_M, (byte)gain);

  _magGain = gain;

  switch (gain) {
  case QMC6310_MAGGAIN_30:
    _QMC6310_Gauss_LSB = 1000;
    // _QMC6310_Gauss_LSB_Z = 1000;
    break;
  case QMC6310_MAGGAIN_12:
    _QMC6310_Gauss_LSB = 2500;
    // _QMC6310_Gauss_LSB_Z = 2500;
    break;
  case QMC6310_MAGGAIN_8:
    _QMC6310_Gauss_LSB = 3750;
    // _QMC6310_Gauss_LSB_Z = 3750;
    break;
  case QMC6310_MAGGAIN_2:
    _QMC6310_Gauss_LSB = 15000;
    // _QMC6310_Gauss_LSB_Z = 15000;
    break;

  }
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
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
  if (_calibrationUse){
    _applyCalibration(event);
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void QMC6310_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "QMC6310", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->max_value = 800;  // 8 gauss == 800 microTesla
  sensor->min_value = -800; // -8 gauss == -800 microTesla
  sensor->resolution = 0.2; // 2 milligauss == 0.2 microTesla
}


/**
    SET CALIBRATION
	Set calibration values for more accurate readings
		
	@author Claus Näveke - TheNitek [https://github.com/TheNitek]
	
	@since v1.1.0
**/
void QMC6310_Unified::setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max){
	_calibrationUse = true;
// _magData.x / _QMC6310_Gauss_LSB * SENSORS_GAUSS_TO_MICROTESLA;
	_vCalibration[0][0] = x_min;
	_vCalibration[0][1] = x_max;
	_vCalibration[1][0] = y_min;
	_vCalibration[1][1] = y_max;
	_vCalibration[2][0] = z_min;
	_vCalibration[2][1] = z_max;
}


/**
  APPLY CALIBRATION
	This function uses the calibration data provided via @see setCalibration() to calculate more
	accurate readings.  Based on the [QMC5883L Compass Arduino Library by MPrograms](https://github.com/mprograms/QMC5883LCompass/).
	
	@author Claus Näveke - TheNitek [https://github.com/TheNitek]
	
	Based on this awesome article:
	https://appelsiini.net/2018/calibrate-magnetometer/
	
	@since v1.1.0
	
**/
void QMC6310_Unified::_applyCalibration(sensors_event_t *event){
  // calculate the offset for each axis
	int x_offset = (_vCalibration[0][0] + _vCalibration[0][1])/2;
	int y_offset = (_vCalibration[1][0] + _vCalibration[1][1])/2;
	int z_offset = (_vCalibration[2][0] + _vCalibration[2][1])/2;
	// calculate the average delta for each axis
  int x_avg_delta = (_vCalibration[0][1] - _vCalibration[0][0])/2;
	int y_avg_delta = (_vCalibration[1][1] - _vCalibration[1][0])/2;
	int z_avg_delta = (_vCalibration[2][1] - _vCalibration[2][0])/2;
  // get the average delta for all three axes
	int avg_delta = (x_avg_delta + y_avg_delta + z_avg_delta) / 3;
  // calculate scale factors
	float x_scale = (float)avg_delta / x_avg_delta;
	float y_scale = (float)avg_delta / y_avg_delta;
	float z_scale = (float)avg_delta / z_avg_delta;
  // adjust event x, y, z for offset and scale
	event->magnetic.x = (event->magnetic.x - x_offset) * x_scale;
	event->magnetic.y = (event->magnetic.y - y_offset) * y_scale;
	event->magnetic.z = (event->magnetic.z - z_offset) * z_scale;
}