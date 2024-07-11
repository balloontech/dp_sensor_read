#include <ArduinoBLE.h>

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

long previousMillis = 0;

void setup() {
  Serial.begin(115200);    // initialize serial communication

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
  int8_t dp = 99, temp = -30;
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
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 2000) {
        previousMillis = currentMillis;
        dpValue.writeValue((uint8_t) dp); 
        tempValue.writeValue((uint8_t) temp); 
        Serial.println("Updating value...");
      }
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
