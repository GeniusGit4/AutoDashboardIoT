#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT11 setup
#define DHTPIN 4       // GPIO4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// MQ-135 (Air Quality Sensor)
#define MQ135_PIN 36   // ADC1_CH0

// Dummy CAN data from STM32
float dummySpeed = 55.6;         // km/h
float dummyEngineTemp = 70.0;    // °C

// Sensor reading holders
float lastValidTemp = 0.0;
float lastValidHum = 0.0;
float lastValidAQI = 0.0;

void setup() {
  Serial.begin(115200);
  dht.begin();

  // OLED setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F(" OLED not found"));
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  Serial.println(" System Initialized");
}

void loop() {
  display.clearDisplay();
  display.setCursor(0, 0);

  // Read DHT11
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h) && h >= 20 && h <= 90) lastValidHum = h;
  else Serial.println(" Invalid Humidity");

  if (!isnan(t) && t >= 0 && t <= 50) lastValidTemp = t;
  else Serial.println(" Invalid Temp");

  // Read MQ-135
  int rawADC = analogRead(MQ135_PIN);
  float voltage = rawADC * (3.3 / 4095.0);
  float baseline = 0.6;
  float AQI = (voltage - baseline) > 0 ? (voltage - baseline) * 150.0 : 0;

  if (!isnan(AQI) && AQI <= 500) lastValidAQI = AQI;
  else Serial.println(" Invalid AQI");

  // OLED Display
  display.printf("Speed:     %.1f km/h\n", dummySpeed);
  display.printf("Car Temp:  %.1f C\n", lastValidTemp);
  display.printf("Humidity:  %.1f %%\n", lastValidHum);
  display.printf("Air Qual.: %.1f ug/m3\n", lastValidAQI);
  display.printf("Eng Temp:  %.1f C\n", dummyEngineTemp);
  
  // Print same data to Serial Monitor
  Serial.println("------ SENSOR VALUES ------");
  Serial.printf("Speed:     %.1f km/h\n", dummySpeed);
  Serial.printf("Car Temp:  %.1f C\n", lastValidTemp);
  Serial.printf("Humidity:  %.1f %%\n", lastValidHum);
  Serial.printf("Air Qual.: %.1f ug/m3\n", lastValidAQI);
  Serial.printf("Eng Temp:  %.1f C\n", dummyEngineTemp);
  Serial.println("----------------------------\n");
  
  display.display();
  delay(2000);
}
