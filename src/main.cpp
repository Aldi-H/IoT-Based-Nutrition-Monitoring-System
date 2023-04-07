#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <ThingerESP32.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "arduino_secrets.h"  // Wifi and Thinger.io credentials

#define TdsSensorPin 34
#define VREF 3.3  // analog reference voltage (V) ADC
#define SCOUNT 20 // sum of sample point
#define oneWireBus 33   // OneWire bus pin

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 0;

// Change this using temperature water sensor
// float temperature = 25; // current temperature for compensation

// Try to connect to wifi
void StartWifi() {
  WiFi.begin(SSID, SSID_PASSWORD);
  // thing.add_wifi(SSID, SSID_PASSWORD);

  Serial.printf("Connecting to %s\n ", SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("Connected to %s\r\n", SSID);
  Serial.printf("IP address: %s\r\n", WiFi.localIP().toString().c_str());

  thing.add_wifi(SSID, SSID_PASSWORD);
}

// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];

  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;

  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

void readTDS() {
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);

  static unsigned long analogSampleTimepoint = millis();

  if (millis() - analogSampleTimepoint > 50U) { // every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer

    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();

  if (millis() - printTimepoint > 1000U) {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }

    // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;

    // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    // temperature compensation
    float compensationVoltage = averageVoltage / compensationCoefficient;

    // convert voltage value to tds value
    float KValue = 0.76;
    // float KValue = 1;
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * KValue;
    float raw_adc = analogRead(TdsSensorPin);
    Serial.printf("Temperature: %.2fC   ", temperature);
    Serial.printf("ADC %.2f   ", raw_adc);
    Serial.printf("Voltage: %.2fV", averageVoltage);
    Serial.printf("  TDS Value: %.2f \n", tdsValue);
  }
}

void setup() {
  Serial.begin(115200);

  StartWifi();
  delay(1000);

  readTDS();

  thing["SensorRead"] >> [](pson& out) {
    out["TDS"] = tdsValue;
    out["Temperature"] = temperature;
  };

  // pinMode(22, OUTPUT);

  // thing["GPIO_16"] << digitalPin(22);
}

void loop() {
  thing.handle();
  readTDS();
}
