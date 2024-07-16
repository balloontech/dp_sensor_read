#include <ArduinoBLE.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SparkFun_MS5803_I2C.h>

#define DP_SENSOR_ADDRESS 0x28
const float pMax = 40.;
const float pMin = -40.;
const float outputMax = 14745.;
const float outputMin = 1638.;
const long interval = 5000000;  //Logging time interval
unsigned long startTime = 0;

// Pin definitions
#define SCK_PIN 4
#define MISO_PIN 5
#define MOSI_PIN 6
#define SS_PIN 7

#define CONFIG_SDA 8
#define CONFIG_SCL 9


//dp service
BLEService dpSensordp("0001");
BLEService dpSensorTemp("0002");

//dp value characteristic
BLECharCharacteristic dpValue("74e19ea8-e895-4272-a659-e8388b47b3be",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);
//temperature value characteristic
BLECharCharacteristic tempValue("ab85e8ef-d448-4e42-b520-e122dedeae19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);

//dp value descriptor
BLEDescriptor dpValueDesc("2901", "dp Value [mbar]");  // CUD UUID
BLEDescriptor tempValueDesc("2901", "Temperature Value [C]");  // CUD UUID

MS5803 sensor(ADDRESS_HIGH); // Instantiate the sensor using ADDRESS_HIGH

void writeToSD(unsigned long time, int8_t dP, int8_t T1){ 
  File file = SD.open("/dp_log_z005_apex.txt", FILE_APPEND);
  file.print(time);
  file.print("; ");
  file.print(dP);
  file.print("; ");
  file.println(T1);
  file.close();
}

void printUSBSerial(float *dP, float *T1, float *P, float *T2){ 
  // Print the values to USB serial
  Serial.print("dP: " + String(*dP) + " mBar; ");
  Serial.print("T1: " + String(*T1) + " C; ");
  Serial.print("P: " + String(*P) + " mBar; ");
  Serial.println("T2: " + String(*T2) + " C");
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

void setup() {
  Serial.begin(115200);    // initialize serial communication
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  Wire.begin(CONFIG_SDA, CONFIG_SCL);        // Initialize I²C with custom SCL and SDA
  sensor.reset();
  sensor.begin(); // Begin the sensor using Wire
  
  // Initialize SD card
  if (!SD.begin(SS_PIN)) {
    Serial.println("Card Mount Failed");
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
  }

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed. Retrying...");
    while (1);
  }

  BLE.setLocalName("dP_Sensor_Z005");
  BLE.setAdvertisedService(dpSensordp); // service UUID
  BLE.setAdvertisedService(dpSensorTemp); // service UUID
  dpSensordp.addCharacteristic(dpValue); // characteristic
  dpSensorTemp.addCharacteristic(tempValue); // add the battery level characteristic  
  dpValue.addDescriptor(dpValueDesc);
  tempValue.addDescriptor(tempValueDesc);
  BLE.addService(dpSensordp); // Add the service
  BLE.addService(dpSensorTemp); // Add the service

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  unsigned long nowTime = 0;
  float dp = 0., temp = 0.;
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      nowTime = micros();
      if (nowTime - startTime >= interval) {
        startTime = nowTime;
        readDpSensor(&dp, &temp);
        writeToSD(millis()/1000, dp, temp);
        dpValue.writeValue((uint8_t) dp); 
        tempValue.writeValue((uint8_t) temp); 
        Serial.print("dP [mbar]:");
        Serial.print((uint8_t) dp);
        Serial.print("\t Temp [C]:");
        Serial.println((uint8_t) temp);
      }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  nowTime = micros();
  if (nowTime - startTime >= interval) {
    startTime = nowTime;
    readDpSensor(&dp, &temp);
    writeToSD(millis()/1000, dp, temp);
  }
}
