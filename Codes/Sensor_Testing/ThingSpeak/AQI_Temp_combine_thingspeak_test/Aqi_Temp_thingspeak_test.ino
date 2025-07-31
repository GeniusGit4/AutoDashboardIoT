#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

//----------------------------
// WiFi Configuration
//----------------------------
#define WIFI_SSID "ESP_HOTSPOT"
#define WIFI_PASSWORD "12345678"

//----------------------------
// ThingSpeak MQTT Configuration
//----------------------------
#define MQTT_SERVER "mqtt3.thingspeak.com"
#define MQTT_PORT 1883
#define CHANNEL_ID "3017335"                      // Replace with your ThingSpeak Channel ID
#define MQTT_CLIENT_ID "GgQ7JBQ3PAUzMjkTMCMNFxc"           // Unique name
#define MQTT_USERNAME "GgQ7JBQ3PAUzMjkTMCMNFxc"                   // Same as your Channel ID
#define MQTT_PASSWORD "+QEUuF5/fsSnwV+PTusAwd6W"   // From ThingSpeak > API Keys tab

//----------------------------
// Sensor Pins and Type
//----------------------------
#define DHTPIN 18
#define DHTTYPE DHT22
#define MQ135PIN 34  // Analog pin for Air Quality sensor (MQ-135)

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastPublish = 0;
const long publishInterval = 16000; // ThingSpeak allows 15 sec interval per channel

void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("✅ Connected to MQTT!");
    } else {
      Serial.print("❌ Failed, rc=");
      Serial.print(client.state());
      Serial.println(" trying again in 5 sec");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastPublish >= publishInterval) {
    lastPublish = now;

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    int airQuality = analogRead(MQ135PIN);

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("❌ Failed to read from DHT sensor!");
      return;
    }

    // Format data
    char payload[100];
    snprintf(payload, sizeof(payload), 
      "field1=%.2f&field2=%.2f&field3=%d", 
      temperature, humidity, airQuality);

    // Publish to ThingSpeak topic
    char topic[100];
    snprintf(topic, sizeof(topic), "channels/%s/publish", CHANNEL_ID);

    if (client.publish(topic, payload)) {
      Serial.println("✅ Data published to ThingSpeak:");
      Serial.println(payload);
    } else {
      Serial.println("❌ Failed to publish data");
    }
  }
}
