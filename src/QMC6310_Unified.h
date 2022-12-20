/*!
 * @file QMC6310_Unified.h
 *
 * @mainpage QMC6310 Unified Library
 *
 * @section intro_sec Introduction
 *
 * A Unified Sensor (1) driver for the QMC6310 magnetometer/compass IC from 
 * QST Corporation.
 * 
 * **BREAKING CHANGES**
 * 
 * Version 4 of the QMC6310_Unified driver library is a complete re-write. 
 * This means many of the internals and non-interface elements have changed. 
 * The Adafruit Unified Sensor interface is still strictly implemented, 
 * however, so the breaking changes will only affect code that used 
 * device-specific fields, enums and structs.
 *
 * We have also added a host of methods that allow setting of device parameters 
 * and reading the device registers. This allows implementers to customize the 
 * device and also obtain diagnostic data during operation. 
 * 
 * The QMC6310 is a three-axis magnetic sensor with I²C serial interface that 
 * integrates magnetic sensors and a signal condition ASIC into one silicon 
 * chip.
 * 
 * The library requires the `adafruit/Adafruit Unified Sensor@^1.1.6`.
 * 
 * ## References *
 * - [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)
 * - [How to Calibrate a Magnetometer](https://www.digikey.com.au/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b) 
 * - [License](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/LICENSE)
 * - [QMC6310 datasheet](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/main/assets/QMC6310_Datasheet.pdf)
 * - [QMC6310 magnetometer/compass IC from QST Corporation](https://www.qstcorp.com/en_comp_prod/QMC6310)
 * - [QMC6310_Unified example code](https://github.com/GM-Consult-IOT/QMC6310_Unified/blob/master/examples/QMC6310_sensor/QMC6310_sensor.ino)
 *
 * @section author Author
 *
 * Gerhard Malan for GM Consult Pty Ltd.
 *
 * @section license License
 * 
 * This library is open-source under the BSD 3-Clause license (6) and 
 * redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the license conditions are met.
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

/// @brief The name of the sensor.
#define DEVICE "QMC6310"

/// @brief Chip ID from QMC6310 datasheet, stored in register 0x00.
#define QMC6310_ID 0b10000000

/// @brief The current driver (MAJOR) version.
#define DRIVER_VERSION 0x04

/// @brief The QMC6310 has a fixed I2C address [0x1c]as per data sheet.
#define QMC6310_I2C_ADDRESS 0x1c 

/// @brief The default field-range (sensitivity) for the device
#define FIELD_RANGE QMC6310_FLD_RNG_2

/// @brief The default orientation of the magenetometer axes with X, Y and 
/// Z axes pointing North, West and Up respectively.
#define AXIS_ORIENTATION QMC6310_AXIS_NORMAL

/* 
@brief The default value for Control Register 1 is 0b11001101:
  - MODE => NORMAL
  - Output Data Rate (ODR) => 50kHz
  - Over Sampling Rate (OSR1) => 4x 
  - Down Sampling Rate (OSR2) => 1
*/
#define CTRL_REG_1 0b00010101

/* 
@brief The default value for Control Register 2 is 0b00001100:
  - SOFT_RST => off.
  - SELF_TEST => off.
  - RNG => 2 Gauss.
  - SET/RESET MODE.=> Set and reset on
*/
#define CTRL_REG_2 0b00001100

/* @brief Enumeration of all the addressable QMC6310 registers as per data sheet. 

Registers 01H to 06H store the measurement data from each axis magnetic 
sensor in each working mode. In normal mode, the output data is refreshed 
periodically based on the data update rate ODR setup in control registers 0AH.

The data stays the same, regardless of reading status through I2C, until 
new data replaces them. Each axis has 16-bit data width in 2’s complement, 
i.e., MSB of 02H/04H/06H indicates the sign of each axis. The output data 
of each channel saturates at -32,768 and 32,767.
*/
typedef enum QMC6310_REGISTER {

  /// @brief ID register. Value is always 0x80
  QMC6310_REGISTER_CID = 0x00,    
  
  /// @brief Control register 1 at 0AH sets the operational 
  /// modes (MODE) and over sampling rate (OSR).
  QMC6310_REGISTER_CTRL1 = 0x0A,
  
  /// @brief Control register 2 at 0BH controls soft reset, self-test and 
  /// set/reset mode.
  QMC6310_REGISTER_CTRL2 = 0x0B,
  
  /// @brief Magnetic flux X LSB
  QMC6310_REGISTER_OUTPUT_X_LSB = 0x01,
  
  /// @brief Magnetic flux X MSB
  QMC6310_REGISTER_OUTPUT_X_MSB = 0x02,
  
  /// @brief Magnetic flux Y LSB
  QMC6310_REGISTER_OUTPUT_Y_LSB = 0x03,
  
  /// @brief Magnetic flux Y MSB
  QMC6310_REGISTER_OUTPUT_Y_MSB = 0x04,
  
  /// @brief Magnetic flux Z LSB
  QMC6310_REGISTER_OUTPUT_Z_LSB = 0x05,
  
  /// @brief Magnetic flux Z MSB 
  QMC6310_REGISTER_OUTPUT_Z_MSB = 0x06,
  
  /* 
  @brief STATUS Register at 09H. 
  Register 09H has two bits indicating for status flags, the rest are reserved f
  or factory use. 
  
  The status registers are read only bits. Bit 0, the "Data Ready" bit (DRDY), is 
  set when all three-axis data is ready and loaded to the output data registers in 
  each mode. It is reset to “0” by reading the status register through I2C commands. 
  Bit 1, the "Overflow" bit (OVFL), set high when either axis code output exceeds 
  the range of [-30000,30000] LSB and reset to “0” after this bit is read. 
  */
  QMC6310_REGISTER_STATUS = 0x09,
  
  /* 
  @brief Axis SIGN register at 29H is undocumented in the datasheet. 
  
  The axis register is set to 06H in normal mode when the X, Y and Z axes point 
  North, West and Up respectively. See [QMC6310_AXIS enum]( (@ref QMC6310_AXIS) for 
  other axis settings.
  */
  QMC6310_REGISTER_AXIS = 0x29,
} qmc6310_register_t;

/*
 @brief Magnetometer gain/sensitivity settings as per data sheet ("field-range"). 

 Field ranges of the magnetic sensor can be selected through bits 2-3 of Control 
 Register 2 (0x0B). The full-scale range is determined by the application 
 environments. The lowest field range has the highest sensitivity, therefore, 
 higher resolution.
*/
typedef enum QMC6310_FLD_RNG {

  /// Field range +/- 30 Gauss / 3,000 uT
  QMC6310_FLD_RNG_30 = 0b00000000, 

  /// Field range +/- 12 Gauss / 1,200 uT (default)
  QMC6310_FLD_RNG_12 = 0b00000100,

  /// Field range +/- 8 Gauss / 800 uT
  QMC6310_FLD_RNG_8 = 0b00001000, 

  /// Field range +/- 2 Gauss / 200 uT
  QMC6310_FLD_RNG_2 = 0b00001100 

} qmc6310_fld_rng_t;

/* 
 @brief Control register 1 at 0AH sets the operational modes (MODE) 
 and over sampling rate (OSR).

 The device has four different modes, controlled by register (0x0A), bits <1:0>. 
 The main purpose of these modes is for power management. The modes 
 can be transited from one to another by changing mode bits via I2C. 

 The first two bits set the modes to SUSPEND, NORMAL, SINGLE or CONTINUOUS. The 
 default mode after Power-On-Reset (POR) is SUSPEND mode. SUSPEND mode should 
 be added in the middle of mode shifting between any of CONTINUOUS, SINGLE
 or NORMAL modes.
*/
typedef enum QMC6310_MODE {
  
  /*
   @brief SUSPEND mode (no new readings are taken).

   SUSPEND mode is the default magnetometer state after power-on and soft reset. 
   Only a few function blocks are activated in this mode which keeps power 
   consumption as low as possible. 
   
   In SUSPEND mode: 
   - no magnetometer readings are taken;
   - all register values are maintained by a lower power LDO;
   - the I2C interface is active; and
   - and all register read/writes are allowed.   
  */
  QMC6310_MODE_SUSP = 0b00000000,
  
  /* 
   @brief NORMAL mode 

   During NORMAL mode the device continuously takes flux readings measures and places 
   measured data in data output registers. 
   
   The frequency with which data points are acquired (sampling rate, or SR) is the 
   output data rate (ODR) multiplied by the over-sampling rate (OSR).
  */
  QMC6310_MODE_NRML = 0b00000001,
  
  /*
   @brief SINGLE mode.

   During SINGLE mode the device takes a single flux readin and then enters SUSPEND mode.
   */
  QMC6310_MODE_SNGL = 0b00000010,
  
  /*
   @brief CONTINUOUS mode

   During CONTINUOUS mode, the whole chip runs all the time without sleep time, 
   so the maximum output data rate (ODR) can be achieved in this mode. 
   
   The self-test function can only be enabled in CONTINUOUS mode and the device 
   enters SUSPEND modeafter the (self-test) data is updated.
  */
  QMC6310_MODE_CONT = 0b00000011

} qmc6310_mode_t;

/// @brief The Output data rate (ODR) is controlled by ODR registers. 
///
/// Four ODR frequencies can be selected: 10Hz, 50Hz, 100Hz or 200Hz. 
///
/// The default ODR is 200Hz.
typedef enum QMC6310_DATA_RATE {

  /// @brief Output Data Rate 10Hz.
  QMC6310_DATA_RATE_10 = 0b00000000,

  /// @brief Output Data Rate 50Hz.
  QMC6310_DATA_RATE_50 = 0b00000100,
  
  /// @brief Output Data Rate 100Hz.
  QMC6310_DATA_RATE_100 = 0b00001000,
  
  /// @brief Output Data Rate 200Hz (default).
  QMC6310_DATA_RATE_200 = 0b00001100
} qmc6310_data_rate_t;

/*
@brief Over Sampling Rate (OSR1) register is used to control bandwidth of an 
internal digital filter. 

Larger OSR value leads to smaller filter bandwidth, less in-band noise and 
higher power consumption. It could be used to reach a good balance between 
noise and power. Four over sampling rates are available: 8,4,2 or 1.
*/
typedef enum QMC6310_OSR {

  /// @brief Over Sampling Rate (OSR1) 8.
  QMC6310_OSR_8 = 0b00000000,

  /// @brief Over Sampling Rate (OSR1) 4.
  QMC6310_OSR_4 = 0b00010000,

  /// @brief Over Sampling Rate (OSR1) 2.
  QMC6310_OSR_2 = 0b00100000,

  /// @brief Over Sampling Rate (OSR1) 1.
  QMC6310_OSR_1 = 0b00110000
} qmc6310O_osr1_t;

/*

@brief The Down Sampling Rate (OSR2) register is used to reduce the number
of readings processed, reducing sampling frequency and power consumption.

The Down Sampling Rate depth can be adjusted through OSR2 register.
*/
typedef enum QMC6310_DSR {

  /// @brief Down Sampling Rate (OSR2) 1.
  QMC6310_DSR_1 = 0b00000000,

  /// @brief Down Sampling Rate (OSR2) 2.
  QMC6310_DSR_2 = 0b01000000,

  /// @brief Down Sampling Rate (OSR2) 4.
  QMC6310_DSR_4 = 0b10000000,

  /// @brief Down Sampling Rate (OSR2) 8.
  QMC6310_DSR_8 = 0b11000000
} qmc6310_osr2_t;

/*
 @brief The device SET/RESET mode is controlled by bits 0 and 1 of Control 
 Register 2.
 
 There are three modes for SET/RESET:
  - SET and RESET ON;
  - SET ONLY ON; and 
  - SET AND RESET OFF. 
 In SET ONLY ON or SET AND RESET OFF mode, the offset is not renewed during measuring.
*/
typedef enum QMC6310_SET_RESET {

  /// @brief SET and RESET on.
  QMC6310_SET_RESET_ON = 0b00000000,

  /// @brief SET only on, RESET off.
  QMC6310_SET_ONLY_ON = 0b00000001,

  /// @brief SET and RESET off.
  QMC6310_SET_RESET_OFF = 0b00000011,
} qmc631_set_reset_t;

/*
  @brief Enumeration of axis orientation values for the device.

  Axis orientation is set via register 0x29. For normal compass orientation, 
  set it to QMC6310_AXIS_NORMAL, (x-axis NORTH, y-axis WEST, z-axis UP (0x06H)).
*/
typedef enum QMC6310_AXIS {

   /// @brief Normal orientation: x-axis NORTH, y-axis WEST, z-axis UP (0x06H).
  QMC6310_AXIS_NORMAL = 0b00000110, 

  /// @brief All three axes reversed: x-axis SOUTH, y-axis EAST, z-axis DOWN.
  QMC6310_AXIS_REVERSED = 0b00000001,

  /// @brief X axis reversed: x-axis SOUTH, y-axis WEST, z-axis DOWN.
  QMC6310_AXIS_XR_YN_ZN = 0b00000111,

  /// @brief Y axis reversed: x-axis NORTH, y-axis EAST, z-axis UP.
  QMC6310_AXIS_XN_YE_ZN = 0b00000100,

  /// @brief Z axis reversed: x-axis NORTH, y-axis WEST, z-axis DOWN.
  QMC6310_AXIS_XN_YN_ZR = 0b00000010,

  /// @brief X and Z axis reversed: x-axis SOUTH, y-axis WEST, z-axis DOWN.
  QMC6310_AXIS_XR_YN_ZR = 0b00000011,

  /// @brief X and Y axis reversed: x-axis SOUTH, y-axis EAST, z-axis UP.
  QMC6310_AXIS_XR_YE_ZN = 0b00000101,

  /// @brief Y and Z axes reversed: x-axis NORTH, y-axis EAST, z-axis DOWN.
  QMC6310_AXIS_XN_YE_ZR = 0b00001000,

} qmc6310_axis_t;

/// @brief The magnetic flux vector values reported by the device.
typedef struct fluxVector_s {

  /// @brief Magnetic flux vector output in x-axis.
  float x;

  /// @brief Magnetic flux vector output in y-axis.
  float y;

  /// @brief Magnetic flux vector output in z-axis.
  float z;
} fluxVector;


/// @brief Unified sensor driver for the QMC6310 magnetometer.
class QMC6310_Unified : public Adafruit_Sensor {
public:

  /// @brief Instantiate a [QMC6310_Unified] driver with the [sensorID].
  /// @param sensorID The unique identifier of the sensor. Defaults to [-1].
  /// @param verbose A flag that controls whether the device sends debug data to the 
  /// serial port.
  QMC6310_Unified(int32_t sensorID, bool verbose = false);

  /// @brief Reads the value of the STATUS register.
  /// @return Returns the currently setvalue from the STATUS register as byte.
  byte readStatusReg(void);

  /// @brief Reads the value of CONTROL_1 register.
  /// @return Returns the currently setvalue from CONTROL_1 register as byte.
  byte readCtrl1Reg(void);

  /// @brief Reads the value of CONTROL_2 register.
  /// @return Returns the currently setvalue from CONTROL_2 register as byte.
  byte readCtrl2Reg(void);

  /// @brief Reads the value of the AXIS register.
  /// @return Returns the currently setvalue from the AXIS register as byte.
  byte readAxisReg(void); 
  
  /// @brief Initializes the devices by setting the axes signs and sensitivity.
  /// @return Returns true if the device was initialized and returned a status code greater than 0x00.
  bool begin(void);

  /// @brief Polls the device for new magnetometer output.
  /// @param event A pointer to a [sensors_event_t].
  /// @return Returns true if the [event] was updated with new sensor output.
  bool getEvent(sensors_event_t *event); 

  /// @brief Get the current QMC6310-specific gain value for the device in LSB/Gauss.
  /// @return Returns the LSB/Gauss gain value currently applied.
  float getSensitivity(); 

  /// @brief Gets the data for the sensor.
  /// @param sensor A pointer to the [sensor_t] for the device driver instance.
  void getSensor(sensor_t *sensor);

  /*
    @brief Sets the axis orientation of the device.

    Axis orientation is set via register 0x29. For normal compass orientation, 
    set it to QMC6310_AXIS_NORMAL, x-axis NORTH, y-axis WEST, z-axis UP (0x06H).

    @param The desired axis orientation as QMC6310_AXIS.  
  */
  void setAxisOrientation(QMC6310_AXIS axisOrientation);
  
  /* 
    @brief Set the device mode to Suspend, Normal, Single or Continuous. 
    
    The `setMode(QMC6310_MODE mode)` method always transits through suspend mode 
    before setting the new mode, so only a call passing the final desired mode is 
    necessary.

    @param mode The new mode setting for the device.
  */
  void setMode(QMC6310_MODE mode);

  /*
    @brief Sets the device over sampling rate (OSR).

    A higher OSR value results in less in-band noise and higher power consumption.

    @param The desired over-sampling rate (OSR).  
  */
  void setOSR(QMC6310_OSR osrRate);

  /*
    @brief Sets the device down sampling rate (DSR).

    A higher DSR value results in lower power consumption.
    @param The desired down-sampling rate (DSR). */
  void setDSR(QMC6310_DSR dsrRate);

  /*
    @brief Sets the output data rate (ODR) of the device. 

    The ODR is the frequency with which new readings are written to the 
    data output registers 0x01-0x06. 
    
    ODR is not affected by the over-sampling and down-sampling settings. The 
    frequency with which the device samples readings (SR) is, however affected 
    by the ODR and over-sampling rate (OSR)(`SR = ODR x OSR`). Device power
    consumption is increased at higher sampling rates.

    @param The desired output data rate (ODR).
  */
  void setDataRate(QMC6310_DATA_RATE odrRate);

  /*
    @brief Sets the device SET/RESET mode.

     There are three modes for SET/RESET:
      - SET and RESET ON;
      - SET ONLY ON; and 
      - SET AND RESET OFF. 

    @param The desired SET_RESET mode.
  */
  void setResetMode(QMC6310_SET_RESET srMode);

  /*
    @brief Sets the field measurement range (sensitivity) of the device to fldRange.

    The appropriate field range setting is determined through trial and error. Too
    low a sensitivity (high field range) results in loss of resolution while too high
    sensititivity results in overflow and erroneous output.

    Verify the that the magnetometer output range falls in the upper end of the 
    field-range setting. It is also advisable to check for overflow occasionally using
    the `overflow()` method.

    @param fldRange The desired field range setting.
  */
  void setFieldRange(QMC6310_FLD_RNG fldRange);

  /*
    @brief Enable the SELF_TEST function. 
    
    The SELF_TEST funstion allows for verification of the signal chain.

    When enabled, a built-in magnetic flux is generated and an additional 
    signal is added to the sensor offsetting the 3 axis value. Record the value 
    before and after the self-test and compare with threshold value.
  */
  void setSelfTest(bool enable);

  /* 
    @brief Invokes a soft-rest of the device.
    
    A soft reset clears all device registers and then calls `begin()` to set
    the device to its start-up state.
  */
  void softReset();

  /*
   @brief Call `dataReady()` to check if data is available to read.
  
   @return Returns true if new data is ready. Returns false if the most recent 
   data has been read via I2C or no data has been been read by the device (e.g. 
   in suspend mode or single mode).
  */
  bool dataReady();

  /*
    @brief Call `overflow()` to check if the last reading exceed the device 
    field range.

    It is advisable to check for overflow, especially in new installations, as 
    overflow does not cause an error, but results in incorrect output as the flux 
    values are clipped at the maximum.

    @return Returns true if the last reading exceed the device field range.
  */
  bool overflow();

  /// @brief Gets the current output data rate (ODR) from Control Register 1.
  /// @return Returns the ODR frequency in Hertz as int32_t.
  int32_t getODR();

  /// @brief Gets the current over-sampling rate (OSR) from Control Register 1.
  /// @return Returns the OSR as byte.
  byte getOSR();

  /// @brief Calculates the current sampling rate from the output data rate  and 
  /// over-sampling rate (SR = ODR * OSR).
  /// @return Returns the sampling rate in Hertz as int32_t.
  int32_t getSamplingRate();

  /// @brief Estimates the current draw of the device at current sampling rate (SR).
  /// Uses a non-linear curve estimated from datasheet current.  
  /// ``` C++
  ///   i = 5.6562 * pow(SR, 0.6989)` 
  /// ```
  /// The current estimate is a good fit up to a sampling rate of 800. At a sampling rate
  /// of 1,600 the current draw is manually set at 1,180uA.
  /// @return Returns the estimated current draw of the device in micro-Ampere (uA).
  float getCurrentDraw();

   /// @brief Gets the current down-sampling rate (DSR) from Control Register 1.
  /// @return Returns the DSR as byte.
  byte getDSR();

  /// @brief Gets the current axis orientation from the axis register.
  /// @return Returns the name of the QMC6310_AXIS enumeration value currently set.
  String getAxisOrientation();

  /// @brief Calculate the absolute value of the field range from the current setting in 
  /// Control Register 2.
  /// @return Returns the maximum value of the flux that can be measured with the current 
  /// field range setting in micro-Tesla (uT) as float.
  float getFieldRange();

  /// @brief Calculate the output data refresh period from the output data rate (ODR) set in 
  /// Control Register 1.
  /// @return Returns data refresh period in in microseconds.
  int32_t getOutputDataPeriod();
  
  /// @brief Prints the device settings and control register values to the serial port.
   void printSensorSettings();

  /// @brief Prints event to the serial monitor.
  /// @param event The sensors_event_t event to print.
  void printSensorEvent(sensors_event_t *event);

  /// @brief Requests new magnetometer data from the device and writes it to _magData
  void read(void);

private:  

  /// @brief Private variable that holds the last read magnetometer data.
  fluxVector _magData; 

  /// @brief Private variable that holds the sensor id value as reported in 
  /// the sensor_t properties.
  int32_t _sensorID;  

  /// @brief Private flag that controls whether the device sends debug data to the 
  /// serial port.
  bool _verbose; 

  /// @brief Private variable that holds the current sensitivity in LSB/Gauss.
  int32_t _sensitivity;
  
  /// @brief Sets the values of bits in oldValue where the mask bits are set.
  /// @param oldValue The byte to modify.
  /// @param newValue The new value of the set bits in mask.
  /// @param mask The mask controlling which bits will be set.
  /// @return Returns a clone of oldValue with the set bits in mask changed to the 
  /// values in newValue
  byte setBitValues(byte oldValue, byte newValue, byte mask);

  /// @brief Writes a byte to a register at the device I2C address. Platform agnostic.
  /// @param reg The register to write to.
  /// @param value The value to write to the register at reg.
  void regWriteByte(byte reg, byte value);

  /// @brief Reads a byte from a register at the device I2C address. Platform agnostic.
  /// @param reg The register to read from.
  /// @return A byte value from the register at reg.
  byte regReadByte(byte reg);

  /// @brief Returns a formatted binary string from a byte.
  /// @param value the byte value to format.
  String getHexString(byte value);

  /// @brief Returns a formatted hex string from the value.
  /// @param value the byte value to format.
  String getBinaryString(byte value);

};

#endif
