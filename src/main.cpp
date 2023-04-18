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
#define LED_BUILTIN 22  // Built-in LED pin number
#define SSR_Pin 26    // Solid State Relay pin number

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

unsigned long prevTime_T1 = millis();

long timePointLED = 1000;   // blink LED every 1s

int LED_BUILTIN_State = LOW;

// Change this using temperature water sensor
// float temperature = 25; // current temperature for compensation

// Try to connect to wifi
void StartWifi() {
  WiFi.begin(SSID, SSID_PASSWORD);

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

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SSR_Pin, OUTPUT);

  StartWifi();
  delay(1000);

  thing["SensorRead"] >> [](pson &out)
  {
    readTDS();
    out["TDS"] = tdsValue;
    out["Temperature"] = temperature;
  };
}

void loop() {
  thing.handle();
  readTDS();

  unsigned long currentTime = millis();

  // Blink Internal LED every 1s
  if (currentTime - prevTime_T1 > timePointLED) {
    LED_BUILTIN_State = !LED_BUILTIN_State;
    digitalWrite(LED_BUILTIN, LED_BUILTIN_State);

    prevTime_T1 = currentTime;
  }
  

}
