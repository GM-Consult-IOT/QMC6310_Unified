<!-- QMC6310_Unified -->

## 4.0.0

**BREAKING CHANGES**

Version 4 of the QMC6310_Unified driver library is a complete re-write. This means many of the internals and non-interface elements have changed. The Adafruit Unified Sensor interface is still strictly implemented, however, so the breaking changes will only affect code that used device-specific fields, enums and structs.

We have also added a host of methods that allow setting of device parameters and reading the device registers. This allows implementers to customize the device and also obtain diagnostic data during operation.

* *BREAKING* renamed most enums and structs.
* Added public method `byte readStatusReg(void)`.
* Added public method `byte readCtrl2Reg(void)`.
* Added public method `byte readCtrl1Reg(void)`.
* Added public method `byte readAxisReg(void)`.
* Added public method `float getSensitivity()`.
* Added public method `void setAxisOrientation(QMC6310_AXIS axisOrientation)`.
* Added public method `void setOSR(QMC6310_OSR osrRate)`.
* Added public method `void setDSR(QMC6310_DSR dsrRate)`.
* Added public method `void setDataRate(QMC6310_DATA_RATE odrRate)`.
* Added public method `void setResetMode(QMC6310_SET_RESET srMode)`.
* Added public method `void setFieldRange(QMC6310_FLD_RNG fldRange)`.
* Added public method `void setSelfTest(bool enable)`.
* Added public method `void softReset()`.
* Added public method `bool dataReady()`.
* Added public method `bool overflow()`.
* Added public method `void printSensorSettings()`.
* Added public method `void printSensorEvent(sensors_event_t *event)`.
* Added public method `int32_t getODR()`.
* Added public method `byte getOSR()`.
* Added public method `int32_t getSamplingRate()`.
* Added public method `float getCurrentDraw()`.
* Added public method `byte getDSR()`.
* Added public method `String getAxisOrientation()`.
* Added public method `float getFieldRange()`.
* Added public method `int32_t getOutputDataPeriod()`.
* Added private method `byte setBitValues(byte oldValue, byte newValue, byte mask)`.
* Added private field `int32_t _sensitivity`.
* Added private method `String getHexString(byte address)`.
* Added private method `String getBinaryString(byte address)`.

## 3.2.0

* Added `QMC6310_MODE` enum.
* Added `QMC6310_AXIS` enum.
* Added `setMode` method.
* Added `setAxis` method

## 3.1.0

* BREAKING: Changed axis orientation so z-axis is positive up.
* `library.json` updated.
* Added defines for `AXIS_ORIENTATION` and `MODE`
* Added enum names.

## 3.0.1+2

* `library.json` updated.

## 3.0.1

* Documentation updated.
* Example updated.
* Replaced `library.properties` with `library.json`.

## 3.0.0

* BREAKING CHANGE: Calibration functions removed. Calibration will be included in a new [lodestone library](https://github.com/GM-Consult-IOT/lodestone) that processes magnetometer and accelerometer data to produce stable, calibrated compass output.

## 2.0.0

* Chip id `(QMC6310_ID)` value changed to 0x80.
* Added `QMC6310_REGISTER_CID = 0x00` to enum `qmc6310MagRegisters_t`.
* Minor doc changes.

## 1.0.3+1

* Minor doc changes.

## 1.0.3

* Documentation edits on examples.

## 1.0.2

* Changed example names.
* Documentation edits.

## 1.0.1

* Minor code cleanup, removed debug variable definitions.

## 1.0.0+1

* Stable.

## 1.0.0

* Stable.

## 0.0.1

* Initial version.