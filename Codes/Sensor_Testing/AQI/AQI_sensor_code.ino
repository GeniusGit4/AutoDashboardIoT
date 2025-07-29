#include <Arduino.h>

#define PM_SENSOR_PIN 36  // Connect your PM2.5 sensor output to GPIO34

// ADC and Voltage parameters (ESP32 specifics)
const float ADC_RESOLUTION = 4095.0;  // 12-bit ADC: 0-4095
const float VOLTAGE_REF = 3.3;        // Reference voltage in volts

// Calibration parameters (example values, adjust them with actual calibration)
const float BASELINE_VOLTAGE = 0.6;   // Voltage when there is negligible dust (in volts)
const float SCALING_FACTOR = 150.0;   // Conversion factor from (V - Baseline) to ug/m³

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("PM2.5 Sensor Test");
}

void loop() {
  // Read the raw ADC value
  int rawValue = analogRead(PM_SENSOR_PIN);
  
  // Convert the raw ADC reading to voltage
  float voltage = (rawValue / ADC_RESOLUTION) * VOLTAGE_REF;
  
  // Calculate the difference from the baseline voltage
  float voltageDifference = voltage - BASELINE_VOLTAGE;
  
  // Calculate PM2.5 concentration (ug/m³) using a linear conversion
  // Clamp to zero if the voltage is below baseline
  float pm25 = (voltageDifference > 0) ? (voltageDifference * SCALING_FACTOR) : 0;
  
  // Print the readings to the serial monitor
  Serial.print("Raw ADC Value: ");
  Serial.print(rawValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 3);
  Serial.print(" V | PM2.5: ");
  Serial.print(pm25, 2);
  Serial.println(" ug/m³");
  
  // Repeat every second
  delay(1000);
}
