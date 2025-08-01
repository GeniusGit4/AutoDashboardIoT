#include <WiFi.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "DHT.h"

// ---------------- WiFi Credentials ----------------
const char* ssid = "ESP_HOTSPOT";
const char* password = "12345678";

// ---------------- ThingSpeak Info ----------------
const char* host = "api.thingspeak.com";
const char* apiKey = "LNNNPSZXV5GW787H";  // Write API Key
unsigned long channelID = 3017335;

// ---------------- DHT11 Setup ----------------
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ---------------- Air Quality Sensor (MQ-135) ----------------
#define MQ135_PIN 36  // GPIO36 = ADC1_CH0
const float ADC_RESOLUTION = 4095.0;
const float VOLTAGE_REF = 3.3;
const float BASELINE_VOLTAGE = 0.6;     // Adjust if needed
const float SCALING_FACTOR = 150.0;

// ---------------- GPS Setup ----------------
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2); // RX = GPIO16, TX = GPIO17

WiFiClient client;

unsigned long lastUpload = 0;
const unsigned long uploadInterval = 15000;

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17);
  dht.begin();

  Serial.println("Connecting to WiFi...");
  IPAddress myDNS(8, 8, 8, 8);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, myDNS);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n WiFi connection failed.");
  } else {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  Serial.println("System initialized. Waiting for GPS fix...");
}

void loop() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  if (gps.location.isUpdated() && millis() - lastUpload > uploadInterval) {
    // Read sensors
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    int rawValue = analogRead(MQ135_PIN);
    float voltage = (rawValue / ADC_RESOLUTION) * VOLTAGE_REF;
    float voltageDifference = voltage - BASELINE_VOLTAGE;
    float pm25 = (voltageDifference > 0) ? (voltageDifference * SCALING_FACTOR) : 0;

    // Debug output
    Serial.println(" Uploading to ThingSpeak...");
    Serial.printf("Lat: %.6f | Lng: %.6f\n", latitude, longitude);
    Serial.printf("Temp: %.2f °C | Humidity: %.2f %%\n", temperature, humidity);
    Serial.printf("PM2.5: %.2f ug/m³ | ADC: %d\n", pm25, rawValue);

    if (WiFi.status() == WL_CONNECTED) {
      if (client.connect(host, 80)) {
        String postData = "api_key=" + String(apiKey) +
                          "&field1=" + String(latitude, 6) +
                          "&field2=" + String(longitude, 6) +
                          "&field3=" + String(temperature, 2) +
                          "&field4=" + String(humidity, 2) +
                          "&field5=" + String(pm25, 2);

        client.println("POST /update HTTP/1.1");
        client.println("Host: api.thingspeak.com");
        client.println("Connection: close");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.print("Content-Length: ");
        client.println(postData.length());
        client.println();
        client.print(postData);

        Serial.println(" Data sent. Waiting for response...");

        while (client.connected()) {
          if (client.available()) {
            String line = client.readStringUntil('\n');
            Serial.println(line);
          }
        }

        client.stop();
        Serial.println("✅ Upload complete.\n");
      } else {
        Serial.println(" Connection to ThingSpeak failed.");
      }
    } else {
      Serial.println(" WiFi disconnected. Skipping update.");
    }

    lastUpload = millis();
  }

  if (!gps.location.isValid()) {
    Serial.println(" No GPS fix yet. Waiting...");
    delay(2000);
  }
}
