/*!
 * @file QMC6310_Unified.cpp
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



void QMC6310_Unified::regWriteByte(byte reg, byte value) {
  if (_verbose){
    Serial.print("Setting register ");
    Serial.print(getHexString(reg));
    Serial.print(" to ");
    Serial.println(getBinaryString(value));
  }
  Wire.beginTransmission(QMC6310_I2C_ADDRESS);
#if ARDUINO >= 100
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
#else
  Wire.send(reg);
  Wire.send(value);
#endif
  Wire.endTransmission();
}

byte QMC6310_Unified::regReadByte(byte reg) {
  byte value;

  Wire.beginTransmission(QMC6310_I2C_ADDRESS);
#if ARDUINO >= 100
  Wire.write((uint8_t)reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();
  Wire.requestFrom(QMC6310_I2C_ADDRESS, 1);
#if ARDUINO >= 100
  value = Wire.read();
#else
  value = Wire.receive();
#endif
  Wire.endTransmission();

  return value;
}

float QMC6310_Unified::getSensitivity() {
  return _sensitivity;
}

void QMC6310_Unified::read() {
  // Read the magnetometer
  Wire.beginTransmission((byte)QMC6310_I2C_ADDRESS);
#if ARDUINO >= 100
  Wire.write(QMC6310_REGISTER_OUTPUT_X_LSB);
#else
  Wire.send(QMC6310_REGISTER_OUTPUT_X_MSB);
#endif
  Wire.endTransmission(false);
  Wire.requestFrom((byte)QMC6310_I2C_ADDRESS, (byte)6, (uint8_t)1);
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

QMC6310_Unified::QMC6310_Unified(int32_t sensorID, bool verbose) {
  _verbose = verbose;
  _sensorID = sensorID;
}

bool QMC6310_Unified::begin() {
  
  // ensure mode is set to suspend in case begin is called during operation
  setMode(QMC6310_MODE_SUSP);

  // set the default orientation for the axes
  setAxisOrientation(AXIS_ORIENTATION);

  // write defaults to control register 2
  regWriteByte(QMC6310_REGISTER_CTRL2, CTRL_REG_2);

  // set the field range to the default
  setFieldRange(FIELD_RANGE);

  // write defaults to control register 1
  regWriteByte(QMC6310_REGISTER_CTRL1, CTRL_REG_1);

  // print the device details and settings to serial monitor if _verbose flag is set
  if (_verbose){

   // print the sensor details to serial terminal
  printSensorDetails();

  // print the sensor settings to serial terminal
  printSensorSettings();
  }

  // get the STATUS register value and return true if it not 0x00
  return readStatusReg() > 0x00;

}

void QMC6310_Unified::setAxisOrientation(QMC6310_AXIS axisOrientation){
  if (_verbose){
    Serial.println("   SETTING AXIS ORIENTATION");
    Serial.println(F("------------------------------------"));
  }
  regWriteByte(QMC6310_REGISTER_AXIS, axisOrientation);  
}

void QMC6310_Unified::setMode(QMC6310_MODE mode){  
  if (_verbose){
    Serial.println("         SETTING MODE");
    Serial.println(F("------------------------------------"));
  }
  if (mode!=QMC6310_MODE_SUSP){
  regWriteByte(
          QMC6310_REGISTER_CTRL1, 
          setBitValues(readCtrl1Reg(),(byte)QMC6310_MODE_SUSP, 0x03));
  }
  regWriteByte(
          QMC6310_REGISTER_CTRL1, 
          setBitValues(readCtrl1Reg(),(byte)mode, 0x03));
}

void QMC6310_Unified::setOSR(QMC6310_OSR osrRate){
  if (_verbose){
    Serial.println("         SETTING OSR");
    Serial.println(F("------------------------------------"));
  }
  regWriteByte(
          QMC6310_REGISTER_CTRL1, 
          setBitValues(readCtrl1Reg(),(byte)osrRate, 0x30));
}

void QMC6310_Unified::setDSR(QMC6310_DSR dsrRate){
  Serial.println("          SETTING DSR");
  Serial.println(F("------------------------------------"));
  regWriteByte(
          QMC6310_REGISTER_CTRL1, 
          setBitValues(readCtrl1Reg(),(byte)dsrRate, 0xC0));
}

void QMC6310_Unified::setDataRate(QMC6310_DATA_RATE odrRate){
  if (_verbose){
    Serial.println("      SETTING DATA RATE");
    Serial.println(F("------------------------------------"));
  }
  regWriteByte(
          QMC6310_REGISTER_CTRL1, 
          setBitValues(readCtrl1Reg(),(byte)odrRate, 0x0C));
}

void QMC6310_Unified::setResetMode(QMC6310_SET_RESET srMode){
  if (_verbose){
    Serial.println("      SETTING SET_RESET MODE");
    Serial.println(F("------------------------------------"));
  }
  regWriteByte(
          QMC6310_REGISTER_CTRL2, 
          setBitValues(readCtrl2Reg(),(byte)srMode, 0x03));
}

void QMC6310_Unified::setSelfTest(bool enable){
  if (_verbose){
    Serial.println("SETTING SELF_TEST MODE");
    Serial.println(F("------------------------------------"));
  }
  byte newValue = 0x00;
  if (enable){
    newValue = 0b01000000;
  }
  regWriteByte(
          QMC6310_REGISTER_CTRL2, 
          setBitValues(readCtrl2Reg(),newValue, 0b01000000));
}

void QMC6310_Unified::softReset(){
  if (_verbose){
    Serial.println("           SOFT RESET");
    Serial.println(F("------------------------------------"));
  }
  regWriteByte(QMC6310_REGISTER_CTRL2, 0b10000000);
  delay(250);
  setMode(QMC6310_MODE_SUSP);
  begin();
}

bool QMC6310_Unified::dataReady(){
  return readStatusReg() & 0b00000001;
}

bool QMC6310_Unified::overflow(){
  return readStatusReg() & 0b00000010;
}

byte QMC6310_Unified::readStatusReg() {  
  return regReadByte(QMC6310_REGISTER_STATUS);
}

byte QMC6310_Unified::readCtrl1Reg() {  
  return regReadByte(QMC6310_REGISTER_CTRL1);
}

byte QMC6310_Unified::readCtrl2Reg() {  
  return regReadByte(QMC6310_REGISTER_CTRL2);
}

byte QMC6310_Unified::readAxisReg() {  
  return regReadByte(QMC6310_REGISTER_AXIS);
}

void QMC6310_Unified::setFieldRange(qmc6310_fld_rng_t range) {
  if (_verbose){
    Serial.println("       SETTING FIELD_RANGE");
    Serial.println(F("------------------------------------"));
  }
  // write the new sensitivity field range to the device control register 2
  regWriteByte(
          QMC6310_REGISTER_CTRL2, 
          setBitValues(regReadByte(QMC6310_REGISTER_CTRL2), 
                        (byte)range, 
                        0b0001100));
  // set the gain LSB/Gauss value
  switch (range) {
  case QMC6310_FLD_RNG_30:
    _sensitivity = 1000;
    break;
  case QMC6310_FLD_RNG_12:
    _sensitivity = 2500;
    break;
  case QMC6310_FLD_RNG_8:
    _sensitivity = 3750;
    break;
  case QMC6310_FLD_RNG_2:
    _sensitivity = 15000;
    break;
  }
}

byte QMC6310_Unified::setBitValues(byte oldValue, byte newValue, byte mask){ 
  return (oldValue & ~mask) | (newValue & mask);
}

bool QMC6310_Unified::getEvent(sensors_event_t *event) {  
  memset(event, 0, sizeof(sensors_event_t));
  read();
  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = millis();
  event->magnetic.x =
      _magData.x / _sensitivity * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y =
      _magData.y / _sensitivity * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z =
      _magData.z / _sensitivity * SENSORS_GAUSS_TO_MICROTESLA;
  if (_verbose){
    Serial.println();
    Serial.println("    NEW SENSOR EVENT");
    Serial.println(F("------------------------------------"));
    printSensorEvent(event);
    Serial.println();
  }    
  return true;
}

void QMC6310_Unified::getSensor(sensor_t *sensor) {
  // clear the sensor
  memset(sensor, 0, sizeof(sensor_t));
  // insert the sensor name in the fixed length char array 
  strncpy(sensor->name, DEVICE, sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = DRIVER_VERSION;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = getOutputDataPeriod() ;    // minimum delay 0
  sensor->max_value = getFieldRange();  
  sensor->min_value = - sensor->max_value; 
  sensor->resolution = 0.2; // report the x/y-axis resolution
}

int32_t QMC6310_Unified::getOutputDataPeriod() {
  byte odr = readCtrl1Reg() & 0b00001100;  
switch (odr) {
  case QMC6310_DATA_RATE_10:
    return (int32_t)(100000);
    break;
  case QMC6310_DATA_RATE_50:
    return (int32_t)(20000);
    break;
  case QMC6310_DATA_RATE_100:
    return (int32_t)(10000);
    break;
  case QMC6310_DATA_RATE_200:
    return (int32_t)(5000);
    break;
  }
  return (int32_t)0;
}

String QMC6310_Unified::getAxisOrientation(){
  byte axis = readAxisReg();
  switch (axis) {
  case QMC6310_AXIS_NORMAL:
    return "QMC6310_AXIS_NORMAL";
    break;
  case QMC6310_AXIS_REVERSED:
    return "QMC6310_AXIS_REVERSED";
    break;
  case QMC6310_AXIS_XR_YN_ZN:
    return "QMC6310_AXIS_XR_YN_ZN";
    break;
  case QMC6310_AXIS_XN_YE_ZN:
    return "QMC6310_AXIS_XN_YE_ZN";
    break;
  case QMC6310_AXIS_XN_YN_ZR:
    return "QMC6310_AXIS_XN_YN_ZR";
    break;
  case QMC6310_AXIS_XR_YN_ZR:
    return "QMC6310_AXIS_XR_YN_ZR";
    break;
  case QMC6310_AXIS_XR_YE_ZN:
    return "QMC6310_AXIS_XR_YE_ZN";
    break;
  case QMC6310_AXIS_XN_YE_ZR:
    return "QMC6310_AXIS_XN_YE_ZR";
    break;
  }
  return "QMC6310_AXIS";
}

int32_t QMC6310_Unified::getODR() {
  byte odr = readCtrl1Reg() & 0b00001100;  
switch (odr) {
  case QMC6310_DATA_RATE_10:
    return (int32_t)(10);
    break;
  case QMC6310_DATA_RATE_50:
    return (int32_t)(50);
    break;
  case QMC6310_DATA_RATE_100:
    return (int32_t)(100);
    break;
  case QMC6310_DATA_RATE_200:
    return (int32_t)(200);
    break;
  }
  return (int32_t)0;
}

byte QMC6310_Unified::getOSR() {
  byte odr = readCtrl1Reg() & 0b00110000;  
switch (odr) {
  case QMC6310_OSR_8:
    return (byte)8;
    break;
  case QMC6310_OSR_4:
    return (byte)4;
    break;
  case QMC6310_OSR_2:
    return (byte)2;
    break;
  case QMC6310_OSR_1:
    return (byte)1;
    break;
  }
  return (byte)0;
}

byte QMC6310_Unified::getDSR() {
  byte odr = readCtrl1Reg() & 0b11000000;  
switch (odr) {
  case QMC6310_DSR_8:
    return (byte)8;
    break;
  case QMC6310_DSR_4:
    return (byte)4;
    break;
  case QMC6310_DSR_2:
    return (byte)2;
    break;
  case QMC6310_DSR_1:
    return (byte)1;
    break;
  }
  return (byte)0;
}

int32_t QMC6310_Unified::getSamplingRate(){
  int32_t odr = getODR();
  int32_t osr = (int32_t)getOSR();
  return odr * osr;
}

float QMC6310_Unified::getCurrentDraw(){
  int32_t sr = getSamplingRate();
  if (sr==1600) return 1180;
  return (5.6562 * pow(sr, 0.6989) ) ;
}

float QMC6310_Unified::getFieldRange() {
  byte range = readCtrl2Reg() & 0b00001100;  
  switch (range) {
  case QMC6310_FLD_RNG_30:
    return 3000;
    break;
  case QMC6310_FLD_RNG_12:
    return 1200;
    break;
  case QMC6310_FLD_RNG_8:
    return 800;
    break;
  case QMC6310_FLD_RNG_2:
    return 200;
    break;
  }
  return 200;
}

String QMC6310_Unified::getHexString(byte value){
  String str = "0x";
      if (value<16) {
        str = str + "0";
      }
      return str+String(value, HEX);
}

String QMC6310_Unified::getBinaryString(byte value){
  String str = String(value, BIN);
  int zeros = 8 - str.length();
      for (int i = 0; i<zeros; i++){
        str = "0" + str;
      }
      return "0b" + str;
}

void QMC6310_Unified::printSensorEvent(sensors_event_t *event){
  Serial.print("Sensor ID:     ");Serial.println(event->sensor_id);
  Serial.print("Sensor Type:   ");Serial.println(event->type);
  Serial.print("Timestamp:     ");Serial.println(event->timestamp);
  Serial.print("Magnetic X:    ");Serial.println(event->magnetic.x);
  Serial.print("Magnetic Y:    ");Serial.println(event->magnetic.y);
  Serial.print("Magnetic Z:    ");Serial.println(event->magnetic.z);
}

void QMC6310_Unified::printSensorSettings(){
  Serial.println(F("------------------------------------"));
  byte axis = regReadByte(QMC6310_REGISTER_AXIS);
  byte ctrl1 = regReadByte(QMC6310_REGISTER_CTRL1);
  byte ctrl2 = regReadByte(QMC6310_REGISTER_CTRL2);
  byte statusReg = readStatusReg();
  Serial.print("STATUS Register: ");Serial.println(getBinaryString(readStatusReg()));
  Serial.print("CTRL_1 Register: ");Serial.println(getBinaryString(ctrl1));
  Serial.print("CTRL_2 Register: ");Serial.println(getBinaryString(ctrl2));
  Serial.print("AXIS Register:   ");Serial.println(getBinaryString(axis));
  Serial.print("Orientation:     ");Serial.println(getAxisOrientation());
  Serial.print("Data rate:       ");Serial.print(getODR());Serial.println(" Hz");
  Serial.print("Over-sampling:   ");Serial.println(getOSR());
  Serial.print("Down-sampling:   ");Serial.println(getDSR());
  Serial.print("Sampling rate:   ");Serial.print(getSamplingRate());Serial.println(" Hz");
  Serial.print("Current Draw:    ");Serial.print(getCurrentDraw(),1);Serial.println(" uA");
  Serial.print("Flux range:      ");Serial.println("+/-" + String(getFieldRange(), 0) + " uT");
  Serial.print("Gain:            ");Serial.print(getSensitivity(), 0);Serial.println(" LSB/Gauss");
  Serial.print("Data ready:      ");
  if (dataReady()) {
    Serial.println("true");
  } else {
    Serial.println("false");
  }
  Serial.print("Overflow:        ");
  if (overflow()) {
    Serial.println("true");
  } else {
    Serial.println("false");
  }
  Serial.println(F("------------------------------------"));
  Serial.println();
}