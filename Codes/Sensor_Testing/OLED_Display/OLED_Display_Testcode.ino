#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Sample variables (replace with live data)
float speed = 60.0;
float humidity = 45.0;
float aqi = 182.0;
float engineTemp = 75.3;
float carTemp = 31.5;

void setup() {
  Serial.begin(115200);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F(" SSD1306 not found"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
}

void loop() {
  display.clearDisplay();
  
  display.setCursor(0, 0);
  display.printf("Speed:     %.1f km/h\n", speed);
  display.printf("Humidity:  %.1f %%\n", humidity);
  display.printf("AQI:       %.1f ug/m3\n", aqi);
  display.printf("Engine T:  %.1f C\n", engineTemp);
  display.printf("Car Temp:  %.1f C\n", carTemp);
  
  display.display();
  delay(1000);
}
