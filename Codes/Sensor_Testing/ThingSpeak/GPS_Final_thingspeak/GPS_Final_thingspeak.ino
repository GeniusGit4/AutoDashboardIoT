#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFi.h>

// Your hotspot credentials
const char* ssid = "ESP_HOTSPOT";          // Your phone hotspot name
const char* password = "12345678";         // Your phone hotspot password

// ThingSpeak info
const char* host = "api.thingspeak.com";
const char* apiKey = "LNNNPSZXV5GW787H";    // Your ThingSpeak Write API Key
unsigned long channelID = 3017335;

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);  // RX = GPIO16, TX = GPIO17

WiFiClient client;
unsigned long lastUpload = 0;
const unsigned long uploadInterval = 15000;  // 15 seconds

void setup() {
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // GPS Baudrate

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  IPAddress myDNS(8, 8, 8, 8);  // Set DNS
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, myDNS);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n❌ WiFi connection failed.");
  } else {
    Serial.println("\n✅ WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }

  Serial.println("Waiting for GPS signal...");
}

void loop() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  // Wait until GPS gives a new fix & enough time passed
  if (gps.location.isUpdated() && millis() - lastUpload > uploadInterval) {
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    Serial.println("Sending GPS to ThingSpeak:");
    Serial.print("Latitude: "); Serial.println(latitude, 6);
    Serial.print("Longitude: "); Serial.println(longitude, 6);

    if (WiFi.status() == WL_CONNECTED) {
      if (client.connect(host, 80)) {
        // Form the HTTP POST body
        String postData = "api_key=" + String(apiKey) +
                          "&field1=" + String(latitude, 6) +
                          "&field2=" + String(longitude, 6);

        // Create HTTP request
        client.println("POST /update HTTP/1.1");
        client.println("Host: api.thingspeak.com");
        client.println("Connection: close");
        client.println("Content-Type: application/x-www-form-urlencoded");
        client.print("Content-Length: ");
        client.println(postData.length());
        client.println();
        client.print(postData);

        Serial.println("✅ Data sent. Waiting for response...");

        // Wait for response
        while (client.connected()) {
          if (client.available()) {
            String line = client.readStringUntil('\n');
            Serial.println(line);  // Print server response
          }
        }

        client.stop();
        Serial.println("✅ Upload complete.\n");
      } else {
        Serial.println("❌ Connection to ThingSpeak failed.");
      }
    } else {
      Serial.println("❌ WiFi disconnected. Cannot send data.");
    }

    lastUpload = millis();
  }

  // Show GPS status
  if (!gps.location.isValid()) {
    Serial.println("⚠️ No GPS fix yet. Waiting...");
    delay(2000);
  }
}
