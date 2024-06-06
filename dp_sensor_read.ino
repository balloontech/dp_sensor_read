// Reference Honeywell Sensor I2C communication doc
// https://prod-edam.honeywell.com/content/dam/honeywell-edam/sps/siot/fr-ca/products/sensors/pressure-sensors/board-mount-pressure-sensors/common/documents/sps-siot-i2c-comms-digital-output-pressure-sensors-tn-008201-3-en-ciid-45841.pdf

#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <SparkFun_MS5803_I2C.h>

#define DP_SENSOR_ADDRESS 0x28
const float pMax = 40.;
const float pMin = -40.;
const float outputMax = 14745.;
const float outputMin = 1638.;

OpenLog myLog; //Create instance
MS5803 sensor(ADDRESS_HIGH); // Instantiate the sensor using ADDRESS_HIGH

void writeToSD(float *dP, float *T1, float *P, float *T2){ 
    // Print the values to SD card
    myLog.print(String(*dP, 2) + "; ");           
    myLog.print(String(*T1, 2) + "; ");
    myLog.print(String(*P, 2) + "; ");
    myLog.println(String(*T2, 2) + "; ");
    myLog.syncFile();
}

void printUSBSerial(float *dP, float *T1, float *P, float *T2){ 
  // Print the values to USB serial
  SerialUSB.print("dP: " + String(*dP) + " mBar; ");
  SerialUSB.print("T1: " + String(*T1) + " C; ");
  SerialUSB.print("P: " + String(*P) + " mBar; ");
  SerialUSB.println("T2: " + String(*T2) + " C");
}

void readDpSensor (float *dP, float *T1){
  Wire.beginTransmission(DP_SENSOR_ADDRESS);
  Wire.write(0x01);  // Send request for data
  Wire.endTransmission();

  delay(10);  // Wait for the sensor to process the request

  Wire.requestFrom(DP_SENSOR_ADDRESS, 4);  // Request 4 bytes from the sensor

  float pressureValue = 0., temperatureValue = 0.;
  if (Wire.available() == 4) {
    uint8_t pressure1 = Wire.read();       // Read the MSB of pressure
    uint8_t pressure2 = Wire.read();       // Read the LSB of pressure
    uint8_t msbTemperature = Wire.read();  // Read the MSB of temperature
    uint8_t lsbTemperature = Wire.read();  // Read the LSB of temperature

    // Remove 2 MSB status bit
    pressure1 &= 0x3F;

    // Remove 5 ignore bits
    lsbTemperature &= 0xF8;

    // Combine the pressure bytes to get the actual values
    uint16_t pressure = (uint16_t)((pressure1 << 8) | pressure2);
    uint16_t temperature = (uint16_t)(((msbTemperature << 8) | lsbTemperature) >> 5);

    // Convert raw values to actual units
    *dP = ((pressure - outputMin) * (pMax - pMin)) / (outputMax - outputMin) + pMin;
    *T1 = (float) (((temperature / 2047.0) * 200.0) - 50.0);
  }
}

void readApSensor (float *P, float *T2){
  *P = (float) sensor.getPressure(ADC_4096);
  *T2 = (float) sensor.getTemperature(CELSIUS, ADC_4096);
}

void setup() {
  Wire.begin();        // Initialize I2C communication
  myLog.begin();        // Open connection to OpenLog
  SerialUSB.begin(9600);  // Start serial communication at 9600 baud rate
  sensor.reset();
  sensor.begin(); // Begin the sensor using Wire
}

void loop() {
  unsigned long startTime = micros();
  float dP = 0., P = 0., T1 = 0., T2 = 0.;
  readDpSensor(&dP, &T1);
  readApSensor(&P, &T2);
  writeToSD(&dP, &T1, &P, &T2);
  printUSBSerial(&dP, &T1, &P, &T2);
  delay(5000. - ((micros()-startTime)/1000.));  // Wait before next reading
}