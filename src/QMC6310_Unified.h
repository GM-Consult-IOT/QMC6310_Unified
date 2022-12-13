/*!
 * @file QMC6310_Unified.h
 *
 * @mainpage QMC6310 Unified Library
 *
 * @section intro_sec Introduction
 *
 * This library is a `Unified Sensor` driver for the QMC6310 magnetometer/compass IC from QST Corporation.
 * See https://www.qstcorp.com/upload/pdf/202202/%EF%BC%88%E5%B7%B2%E4%BC%A0%EF%BC%8913-52-17%20QMC6310%20Datasheet%20Rev.C(1).pdf
 * for the device datasheet.
 * 
 * The library is based on the [Adafruit driver](https://github.com/adafruit/Adafruit_QMC6310_Unified) 
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

#ifndef __QMC6310_H__
#define __QMC6310_H__

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>

#ifdef __AVR_ATtiny85__
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

// The current driver version.
#define DRIVER_VERSION 0x02

/// @brief The QMC6310 has a fixed I2C address of [0x1c] as per data sheet.
#define QMC6310_ADDRESS_MAG 0x1c 

/// @brief Enumeration of all the addressable registers as per data sheet.
typedef enum {
  QMC6310_REGISTER_MAG_CID_REG_M = 0x00,    // ID register. Value is always 0x80
  QMC6310_REGISTER_MAG_CRA_REG_M = 0x0A,    // Control register 1
  QMC6310_REGISTER_MAG_CRB_REG_M = 0x0B,    // Control register 2
  QMC6310_REGISTER_MAG_OUT_X_L_M = 0x01,    // Magnetic flux X LSB
  QMC6310_REGISTER_MAG_OUT_X_H_M = 0x02,    // Magnetic flux X MSB
  QMC6310_REGISTER_MAG_OUT_Y_L_M = 0x03,    // Magnetic flux Y LSB
  QMC6310_REGISTER_MAG_OUT_Y_H_M = 0x04,    // Magnetic flux Y MSB
  QMC6310_REGISTER_MAG_OUT_Z_L_M = 0x05,    // Magnetic flux Z LSB
  QMC6310_REGISTER_MAG_OUT_Z_H_M = 0x06,    // Magnetic flux Z MSB
  QMC6310_REGISTER_MAG_SR_REG_Mg = 0x09,    // STATUS Register
  QMC6310_REGISTER_MAG_SGN_REG_M = 0x29,    // Axis SIGN register
} qmc6310MagRegisters_t;

/// @brief Magnetometer gain settings as per data sheet. 
typedef enum {
  QMC6310_MAGGAIN_30 = 0b00000000, // +/- 30 Gauss
  QMC6310_MAGGAIN_12 = 0b00000100, // +/- 12 Gauss (default)
  QMC6310_MAGGAIN_8 = 0b00001000, // +/- 8 Gauss
  QMC6310_MAGGAIN_2 = 0b00001100 // +/- 2 Gauss
} qmc6310MagGain;


/// @brief The raw magnetic flux vector values reported by the device.
typedef struct magsensorOutput_s {
  /// @brief Magnetic flux vector output in x-axis.
  float x;
  /// @brief Magnetic flux vector output in y-axis.
  float y;
  /// @brief Magnetic flux vector output in z-axis.
  float z;
} magsensorOutput;

/// @brief Chip ID from QMC6310 datasheet, stored in register 0x00.
#define QMC6310_ID (0b10000000)

/// @brief Unified sensor driver for the QMC6310 magnetometer.
class QMC6310_Unified : public Adafruit_Sensor {
public:

  /// @brief Instantiate a [QMC6310_Unified] driver with the [sensorID].
  /// @param sensorID The unique identifier of the sensor. Defaults to [-1].
  QMC6310_Unified(int32_t sensorID = -1);

  /// @brief Gets the device status from the STATUS register and returns it as byte.  
  /// @return Returns 0x01 if the device is ready, 0x11 if the measurement range was exceeded. See the datasheet
  byte status(void); 
  
  /// @brief Initializes the devices by setting the axes signs and sensitivity.
  /// @return Returns true if the device was initialized and returned a status code greater than 0x00.
  bool begin(void);

  /// @brief Sets the magnetic gain of the sensor.
  /// @param gain The gain value for the device. Default is [QMC6310_MAGGAIN_12].
  void setMagGain(qmc6310MagGain gain = QMC6310_MAGGAIN_12);

  /// @brief Polls the device for new magnetometer output.
  /// @param event A pointer to a [sensors_event_t].
  /// @return Returns true if the [event] was updated with new sensor output.
  bool getEvent(sensors_event_t *event); 

  /// @brief Get the current QMC6310-specific gain value for the device in LSB/Gauss.
  /// @return Returns the LSB/Gauss gain value currently applied.
  float sensitivity(); 

  /// @brief Gets the data for the sensor.
  /// @param sensor A pointer to the [sensor_t] for the device driver instance.
  void getSensor(sensor_t *sensor);


private:  
  /// @brief Last read magnetometer data.
  magsensorOutput _magData; 


  /// @brief  The  sensor id value as reported in the @see [sensor_t] properties.
  int32_t _sensorID;  

  /// @brief The last known status code of the device.
  byte _status = 0x00;
  
  /// @brief Utility function to abstract away platform differences in Arduino wire library.
  /// @param address The device address to write to.
  /// @param reg The register to write to.
  /// @param value The value to write to the register at @see [reg].
  void write8(byte address, byte reg, byte value);

  /// @brief Utility function to abstract away platform differences in Arduino wire library.
  /// @param address The device address to read from.
  /// @param reg The register to read from.
  /// @return A byte value from the register at @see [reg].
  byte read8(byte address, byte reg);

  /// @brief Requests new magnetometer data from the device and writes it to @see [_magData]
  void read(void);

};

#endif