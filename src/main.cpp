#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include "GravityTDS.h"

GravityTDS gravityTds;

#define TdsSensorPin 34

float temperature = 25.0, tdsValue = 0.0;

// potentiometer connected to GPIO34 (ADC1_CH6)
const int potPin = 34;

// variable to store the value read from the potentiometer
int potValue = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.0);        // reference voltage on ADC
  gravityTds.setAdcRange(4095.0); // 1024 for 10bit ADC; 4095 for 12bit ADC
  gravityTds.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  gravityTds.setTemperature(temperature);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  potValue = analogRead(potPin);
  Serial.print("ADC Value: ");
  Serial.print(potValue);
  Serial.printf("    ");
  Serial.print("TDS Value: ");
  Serial.print(tdsValue);
  Serial.print("ppm");
  delay(4000);
}