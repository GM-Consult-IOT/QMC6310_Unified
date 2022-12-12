/*!
 * @file QMC6310_Unified.h
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

/*!
 * @brief I2C address/bits
 */
#define QMC6310_ADDRESS_MAG 0x1c // 0011110x

/*!
 @brief Registers
 */
typedef enum {
  QMC6310_REGISTER_MAG_CRA_REG_M = 0x0A,    // changed from 0x00 to 0x0a
  QMC6310_REGISTER_MAG_CRB_REG_M = 0x0B,    // changed from 0x01 to 0x0b
  QMC6310_REGISTER_MAG_OUT_X_L_M = 0x01,    // changed from 0x04 to 0x01
  QMC6310_REGISTER_MAG_OUT_X_H_M = 0x02,    // changed from 0x03 to 0x02
  QMC6310_REGISTER_MAG_OUT_Y_L_M = 0x03,    // changed from 0x08 to 0x03
  QMC6310_REGISTER_MAG_OUT_Y_H_M = 0x04,    // changed from 0x07 to 0x04
  QMC6310_REGISTER_MAG_OUT_Z_L_M = 0x05,    // changed from 0x06 to 0x05
  QMC6310_REGISTER_MAG_OUT_Z_H_M = 0x06,    // changed from 0x05 to 0x06
  QMC6310_REGISTER_MAG_SR_REG_Mg = 0x09,    // STATUS Register, remains the same
  QMC6310_REGISTER_MAG_SGN_REG_M = 0x29,    // The SIGN register of the QMC6310 
} qmc6310MagRegisters_t;

/*!
 * @brief Magnetometer gain settings
 */
typedef enum {
  QMC6310_MAGGAIN_30 = 0b00000000, // +/- 30 Gauss
  QMC6310_MAGGAIN_12 = 0b00000100, // +/- 12 Gauss (default)
  QMC6310_MAGGAIN_8 = 0b00001000, // +/- 8 Gauss
  QMC6310_MAGGAIN_2 = 0b00001100 // +/- 2 Gauss
} qmc6310MagGain;

/*!
 * @brief Internal magnetometer data type
 */
typedef struct qmc6310MagData_s {
  float x;           //!< Magnetometer x value
  float y;           //!< Magnetometer y value
  float z;           //!< Magnetometer z value
  float orientation; //!< Magnetometer orientation
} qmc6310MagData;

/*!
 * @brief Chip ID
 */
#define QMC6310_ID (0b11010100)

//! Unified sensor driver for the QMC6310 magnetometer. ///
class QMC6310_Unified : public Adafruit_Sensor {
public:
  /*!
   * @param sensorID sensor ID, -1 by default
   */
  QMC6310_Unified(int32_t sensorID = -1);
  byte status(void); //!< @return Returns 0x01 if the device is ready, 0x11 if the measurement range was exceeded.
  bool begin(void); //!< @return Returns whether connection was successful.
  void setMagGain(qmc6310MagGain gain); //!< @param gain Desired magnetic gain.
  void setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
  bool getEvent(sensors_event_t *); //!< @return Returns the most recent sensor event
  float sensitivity(); //!< @return Returns the magnetometer sensitivity in LSB/Gauss.
  void getSensor(sensor_t *);

private:
  qmc6310MagGain _magGain;
  qmc6310MagData _magData; // Last read magnetometer data will be available here
  qmc6310MagGain _fieldRange = QMC6310_MAGGAIN_12;
  int32_t _sensorID;  
	bool _calibrationUse = false;
	int _vCalibration[3][2];
	void _applyCalibration(sensors_event_t *event);
  byte _status = 0x00;
  
  void write8(byte address, byte reg, byte value);
  byte read8(byte address, byte reg);
  void read(void);
};

#endif
