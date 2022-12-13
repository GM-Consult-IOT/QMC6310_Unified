#include <Arduino.h>
#include <inttypes.h>
#include <Wire.h> 
#include <oled_display.h>
#include <QMC6310_Unified.h>
#include <i2c_diagnostics.h>
#include <magsensor_calibrate.h>


#define DISPLAYLABEL "HEADING" // A label for the sensorData data for the splash screen

#define SPLASHDATA "QMC6310" // The part number of the sensorData

#define SPLASHLABEL "Sensor Test" // A label for the splash screen.

// The variable that holds the output range of the magnetometer.
MagsensorOutputRange sensorRange;

// Assign a unique ID to this sensorData at the same time 
QMC6310_Unified magSensor = QMC6310_Unified(12345);

// 
MagSensorCalibrate calibrator = MagSensorCalibrate(true);

void displaySensorDetails(void)
{
  sensor_t sensorData;
  magSensor.getSensor(&sensorData);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensorData.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensorData.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensorData.sensor_id);  
  Serial.print  ("Status:       "); Serial.println(magSensor.status());
  Serial.print  ("Sensitivity:  "); Serial.println(magSensor.sensitivity());
  Serial.print  ("Max Value:    "); Serial.print(sensorData.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensorData.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensorData.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(5000);
}

// int calibrationData[3][2];
bool changed = false;
bool done = false;
int t = 0;
int c = 0;

void setup() {

  byte devices[9];
  
  Wire.begin(); // start the i2c comms
  
  Serial.begin(115200); // start serial port

  Serial.println("Starting up..."); // send something to the serial port

  TwoWireDiagnostics diags = TwoWireDiagnostics(Wire);
  
  oledInit(); // initialize the oled display
  
  oledSplash(SPLASHLABEL, SPLASHDATA); // show the splash screen
  
  diags.devices(devices, true);

  // loop through the devices
  for (byte i = 0; i<9; i++){
    byte device = devices[i];
    
    if (device!=0x00){
      Serial.println(device, HEX);
      // display the address for each I2C device
      oledWrite("I2C Device at", diags.getAddressString(device));
      // show the address for 2 seconds
      delay(2000);
    }
  }

    
  /* Initialise the sensorData and handle status errors */
  if (!magSensor.begin()){
    Serial.println("Error connecting to QMC6310. Status code is " + String(magSensor.status()));
    while(1);
  }

  // change the sensitivity if required.
  // magSensor.setMagGain(QMC6310_MAGGAIN_30);
  
  /* Display some basic information on this sensorData */
  displaySensorDetails();

//  Just put in very small numbers.
// calibrator.setCalibration(-1, 1, -1, 1, -1, 1);
  // calibrator.setCalibration(-43, 78, -53, 71, -65, 61);
  calibrator.setCalibration(-41, 78, -49, 67, -59, 61);
}



void loop() {

  // Get a new sensorData event
  sensors_event_t event; 
  magSensor.getEvent(&event);

  // calibrate(event);
  
  magsensorData data;
  data.x = event.magnetic.x;
  data.y = event.magnetic.y;
  data.z = event.magnetic.z;

  calibrator.calibrate(&data);
  // Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print(" -> ");Serial.print(data.x); Serial.print(" ");
  // Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print(" -> ");Serial.print(data.y); Serial.print(" ");
  // Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print(" -> ");Serial.print(data.z); Serial.println(" ");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(data.y, data.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is ~15.44 Degrees, or 0.27 radians
  // If you cannot find your Declination, comment out these two lines, your magSensor will be slightly off.
  float declinationAngle = 0.27;
  heading += declinationAngle;  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  // Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
 
  oledWrite(DISPLAYLABEL, String(headingDegrees, 0)+ "°");  // write the sensorData data here



  delay(100); // wait a bit
}