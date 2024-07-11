#include <ArduinoBLE.h>

BLECharCharacteristic dpValue("74e19ea8-e895-4272-a659-e8388b47b3be",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);
//temperature value characteristic
BLECharCharacteristic tempValue("ab85e8ef-d448-4e42-b520-e122dedeae19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);

long previousMillis = 0;
const char* dpServiceId = "0001";
const char* tempServiceId = "0002";

void printData(const unsigned char data[], int length) {
  for (int i = 0; i < length; i++) {
    unsigned char b = data[i];
    if (b < 16) {
      Serial.print("0");
    }
    Serial.print(b, HEX);
  }
}

bool connectPeripheral(BLEDevice peripheral) {
    Serial.println("Connecting...");
    if (peripheral.connect()) {
        Serial.println("Connected");
    } 
    else {
        Serial.println("Failed to connect");
        return false;  // Return false if connection fails
    }
    // Discover peripheral attributes
    Serial.println("Discovering attributes...");
    if (peripheral.discoverAttributes()) {
        Serial.println("Attributes discovered");
        return true;  // Return true if both connection and attribute discovery succeed
    } else {
        Serial.println("Attribute discovery failed");
        peripheral.disconnect();
        return false;  // Return false if attribute discovery fails
    }
}

int8_t readValue(BLEDevice peripheral, int serviceId, BLECharacteristic targetCharacteristic) {
  BLEService service = peripheral.service(serviceId);
  BLECharacteristic characteristic = service.characteristic(targetCharacteristic);

  for (int i = 0; i < peripheral.serviceCount(); i++) {
    BLEService service = peripheral.service(i);
    int serviceUUID = atoi(service.uuid());
    if (serviceUUID == serviceId) {
      Serial.print("Service ");
      Serial.println(service.uuid());

      for (int i = 0; i < service.characteristicCount(); i++) {
        BLECharacteristic characteristic = service.characteristic(i);
        Serial.print("\tCharacteristic ");
        Serial.print(characteristic.uuid());
        Serial.print(", properties 0x");
        Serial.print(characteristic.properties(), HEX);
        // check if the characteristic is readable
        if (characteristic.canRead()) {
          // read the characteristic value
          characteristic.read();
          if (characteristic.valueLength() > 0) {
            // print out the value of the characteristic
            Serial.print(", value 0x");
            printData(characteristic.value(), characteristic.valueLength());
            uint8_t output = 0;
            characteristic.readValue(output);
            Serial.println();
            return output;
          }
        }
        Serial.println();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  Serial.println("Bluetooth® Low Energy Starting ");

  // start scanning for peripherals
  BLE.scan();
  Serial.println("Bluetooth® device active. Scanning for dP_Sensor_Z005...");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();
  int8_t dp = 0, temp = 0;
  if (peripheral.localName() == "dP_Sensor_Z005") {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.println("Found dP_Sensor_Z005");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    // stop scanning
    BLE.stopScan();
    if (!peripheral.connected()){
      connectPeripheral(peripheral);
        } 
    while (peripheral.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 5000) {
        previousMillis = currentMillis;
        Serial.println("Reading dp value");
        dp = (int8_t) readValue(peripheral, atoi(dpServiceId), dpValue);
        Serial.println("Reading temp value");
        temp = (int8_t) readValue(peripheral, atoi(tempServiceId), tempValue);
        Serial.println(dp);
        Serial.println(temp);
      }
    }
    Serial.println("Disconnected");
    Serial.println("Scanning for dP_Sensor_Z005...");
    BLE.disconnect();
    BLE.scan();
  }
}
