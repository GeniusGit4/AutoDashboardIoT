#include <WiFi.h>
#include <HTTPClient.h>

#define MQ135_PIN 36  // GPIO36 (ADC1_CH0)
#define WIFI_SSID "ESP_HOTSPOT"
#define WIFI_PASS "12345678"
#define THINGSPEAK_API_KEY "LNNNPSZXV5GW787H"

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
}

int getStableADC() {
  int total = 0;
  for (int i = 0; i < 10; i++) {
    total += analogRead(MQ135_PIN);
    delay(10);
  }
  return total / 10;
}

void loop() {
  int rawADC = getStableADC();
  float voltage = rawADC * (3.3 / 4095.0);

  Serial.print("ADC: "); Serial.print(rawADC);
  Serial.print(" | Voltage: "); Serial.print(voltage); Serial.println("V");

  // Basic air quality level estimation (not calibrated PPM)
  String quality;
  if (voltage < 0.4) quality = "Clean";
  else if (voltage < 1.0) quality = "Moderate";
  else quality = "Poor";

  // Upload to ThingSpeak
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY)
                 + "&field1=" + String(voltage)
                 + "&field2=" + quality;
    http.begin(url);
    int code = http.GET();
    http.end();
    Serial.println("ThingSpeak Response: " + String(code));
  }

  delay(10000);  // Upload every 10 seconds
}
