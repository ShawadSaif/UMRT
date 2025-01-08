#include <AltSoftSerial.h>
//for AltSoft use pin 8 and 9 for RO and DI Pin
// RS-485 configuration pins
#define RE 6
#define DE 7

// Modbus commands (requests for data)
const byte temp[] = {0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA}; // Temperature
const byte mois[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}; // Moisture
const byte econ[] = {0x01, 0x03, 0x00, 0x02, 0x00, 0x01, 0x25, 0xCA}; // Conductivity
const byte ph[] = {0x01, 0x03, 0x00, 0x03, 0x00, 0x01, 0x74, 0x0A};   // pH
const byte nitro[] = {0x01, 0x03, 0x00, 0x04, 0x00, 0x01, 0xC5, 0xCB}; // Nitrogen
const byte phos[] = {0x01, 0x03, 0x00, 0x05, 0x00, 0x01, 0x94, 0x0B};  // Phosphorus
const byte pota[] = {0x01, 0x03, 0x00, 0x06, 0x00, 0x01, 0x64, 0x0B};  // Potassium

byte values[7]; // Buffer for response
AltSoftSerial mod;

float envHumidity = 0.0, envTemperature = 0.0, soilPH = 0.0, soilMoisture = 0.0, soilTemp = 0.0;
int nitrogenVal = 0, phosphorusVal = 0, potassiumVal = 0;

// Forward declaration
float readSensor(const byte *command, byte responseSize, int scaleFactor = 1);

void setup() {
  Serial.begin(9600);
  mod.begin(4800);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  // Set RS-485 to receive mode initially
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  delay(3000);
}

void loop() {
  soilMoisture = readSensor(mois, 7, 10); // Scaling: 0.1%
  delay(1000);
  soilTemp = readSensor(temp, 7, 10);     // Scaling: 0.1°C
  delay(1000);
  float conductivity = readSensor(econ, 7); // Scaling: None
  delay(1000);
  soilPH = readSensor(ph, 7, 10);          // Scaling: 0.1
  delay(1000);
  nitrogenVal = readSensor(nitro, 7);      // Scaling: None
  delay(1000);
  phosphorusVal = readSensor(phos, 7);     // Scaling: None
  delay(1000);
  potassiumVal = readSensor(pota, 7);      // Scaling: None
  delay(1000);

  // Print values
  Serial.print("Moisture: ");
  Serial.print(soilMoisture);
  Serial.println(" %");
  delay(1000);

  Serial.print("Temperature: ");
  Serial.print(soilTemp);
  Serial.println(" °C");
  delay(1000);

  Serial.print("EC: ");
  Serial.print(conductivity);
  Serial.println(" us/cm");
  delay(1000);

  Serial.print("pH: ");
  Serial.print(soilPH);
  Serial.println();
  delay(1000);

  Serial.print("Nitrogen: ");
  Serial.print(nitrogenVal);
  Serial.println(" mg/kg");
  delay(1000);

  Serial.print("Phosphorus: ");
  Serial.print(phosphorusVal);
  Serial.println(" mg/kg");
  delay(1000);

  Serial.print("Potassium: ");
  Serial.print(potassiumVal);
  Serial.println(" mg/kg");
  delay(3000);
}

// Function to read data from the sensor
float readSensor(const byte *command, byte responseSize, int scaleFactor) {
  mod.flushInput();

  // Switch RS-485 to transmit mode
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(1);

  // Send the command
  for (uint8_t i = 0; i < 8; i++) { // Assuming all commands are 8 bytes
    mod.write(command[i]);
  }

  mod.flush();

  // Switch RS-485 to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  delay(200); // Wait for the response

  // Read the response
  for (byte i = 0; i < responseSize; i++) {
    values[i] = mod.read();
  }

  // Combine high and low bytes
  int rawValue = (values[3] << 8) | values[4];
  
  // Debug: Print the raw data to Check if the sensor is responding properly. no respone hex: FFFFFFFF
  Serial.print("Raw Data: ");
  for (byte i = 0; i < responseSize; i++) {
    Serial.print(values[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  return rawValue / (float)scaleFactor; // Apply scaling factor
}
