#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPS_Serial(2);  // UART2 (GPIO16=RX, GPIO17=TX)

void setup() {
  Serial.begin(115200);            // For debug output
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // Baud, mode, RX, TX

  Serial.println("Waiting for GPS signal...");
}

void loop() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());

    if (gps.location.isUpdated()) {
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);
      Serial.print("Speed (km/h): ");
      Serial.println(gps.speed.kmph());
      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
      Serial.println("-------------------");
    }
  }
}
