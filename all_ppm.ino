#include <Wire.h>
#include <DFRobot_SCD4X.h>
#include "DFRobot_MultiGasSensor.h"

// I2C Addresses
#define GAS_SENSOR_1_ADDR 0x74
#define GAS_SENSOR_2_ADDR 0x75
#define GAS_SENSOR_3_ADDR 0x77
#define SCD4X_ADDR        0x62

// Analog MQ-4 Configuration
const int mq4Pin = A0;
const float Vcc = 5.0;
const float RL = 10.0;
const float Ro = 4.4;

// Sensor Objects
DFRobot_SCD4X scd4x(&Wire, SCD4X_ADDR);
DFRobot_GAS_I2C gasSensor1(&Wire, GAS_SENSOR_1_ADDR);
DFRobot_GAS_I2C gasSensor2(&Wire, GAS_SENSOR_2_ADDR);
DFRobot_GAS_I2C gasSensor3(&Wire, GAS_SENSOR_3_ADDR);

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();

  // Initialize SCD4X
  unsigned long start = millis();
  while (!scd4x.begin()) {
    delay(1000);
    if (millis() - start > 5000) break;
  }

  scd4x.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
  delay(500);
  scd4x.setTempComp(4.0);
  scd4x.setSensorAltitude(540);
  scd4x.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);

  // Initialize Multi-Gas Sensors
  DFRobot_GAS_I2C* gasSensors[] = { &gasSensor1, &gasSensor2, &gasSensor3 };
  for (int i = 0; i < 3; i++) {
    start = millis();
    while (!gasSensors[i]->begin()) {
      delay(1000);
      if (millis() - start > 5000) break;
    }
    gasSensors[i]->changeAcquireMode(gasSensors[i]->PASSIVITY);
    gasSensors[i]->setTempCompensation(gasSensors[i]->ON);
  }
}

void loop() {
  delay(4000);

  // --- SCD4X (CO2, Temp, Humidity) ---
  if (scd4x.getDataReadyStatus()) {
    DFRobot_SCD4X::sSensorMeasurement_t data;
    scd4x.readMeasurement(&data);
    Serial.print("CO2: ");
    Serial.print(data.CO2ppm);
    Serial.println(" ppm");
    Serial.print("Temp: ");
    Serial.print(data.temp);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(data.humidity);
    Serial.println(" %RH");
  }

  // --- Multi-Gas Sensors (O2 in %, others in ppm) ---
  DFRobot_GAS_I2C* gasSensors[] = { &gasSensor1, &gasSensor2, &gasSensor3 };

  for (int i = 0; i < 3; i++) {
    String gasType = gasSensors[i]->queryGasType();
    float value = gasSensors[i]->readGasConcentrationPPM();

    if (gasType != "O2") {
      Serial.print(gasType);
      Serial.print(": ");
      Serial.print(value, 2);
      Serial.println(" ppm");
    } else {
      Serial.print("O2: ");
      Serial.print(value, 2);
      Serial.println(" %");
    }
  }

  // --- MQ-4 Methane Sensor (Analog) ---
  int adcValue = analogRead(mq4Pin);
  float voltage = adcValue * (Vcc / 1023.0);
  float Rs = ((Vcc - voltage) / voltage) * RL;
  float ratio = Rs / Ro;
  float ppm_log = (log10(ratio) - 0.518) / -0.318;
  float ppm = pow(10, ppm_log);

  Serial.print("CH4: ");
  Serial.print(ppm, 2);
  Serial.println(" ppm");

  Serial.println();
  delay(2000);
}
