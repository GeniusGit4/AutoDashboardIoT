#define BLYNK_TEMPLATE_ID "TMPL35oK2jpYk"
#define BLYNK_TEMPLATE_NAME "Car Dashboard"
#define BLYNK_AUTH_TOKEN "o_51-nsNhArkMZN-qTPbBoBF_PIh5UMQ"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Replace with your WiFi credentials
char ssid[] = "ESP_HOTSPOT";
char pass[] = "123456789";

BlynkTimer timer;

void sendSensorData() {
  float temp = random(25, 35);  // Dummy temperature
  float hum = random(40, 70);   // Dummy humidity

  Blynk.virtualWrite(V0, temp);
  Blynk.virtualWrite(V1, hum);

  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print(" °C | Humidity: ");
  Serial.print(hum);
  Serial.println(" %");
}

void setup() {
  Serial.begin(115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Send data every 5 seconds
  timer.setInterval(5000L, sendSensorData);
}

void loop() {
  Blynk.run();
  timer.run();
}
