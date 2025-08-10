#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// PIN Definitions
#define CAN_CS_PIN    5
#define CAN_INT_PIN   4
#define LED_PIN       2
#define DHTPIN        15
#define DHTTYPE       DHT11
#define PM_SENSOR_PIN 36  // ADC1_CH0

// CAN IDs
#define STM32_CAN_ID_TEMPERATURE  0x100
#define STM32_CAN_ID_DISTANCE     0x101
#define STM32_CAN_ID_MPU          0x102
#define STM32_CAN_ID_SPEED        0x103

// PM2.5 Calibration
const float ADC_RESOLUTION = 4095.0;   
const float VOLTAGE_REF = 3.3;         
const float BASELINE_VOLTAGE = 0.6;    
const float SCALING_FACTOR = 150.0;    

// OLED display config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MCP_CAN CAN(CAN_CS_PIN); 
DHT dht(DHTPIN, DHTTYPE);

// GPS Setup
TinyGPSPlus gps;
HardwareSerial GPS_Serial(2); // UART2 (RX=16, TX=17)

// Shared sensor data (volatile for safe update/read)
volatile float canTemperature = 0.0;
volatile uint32_t canDistance = 0;
volatile uint32_t canSpeed = 0;
volatile float dhtTemperature = 0.0;
volatile float dhtHumidity = 0.0;
volatile float pm25Value = 0.0;

volatile double gpsLatitude = 0.0;
volatile double gpsLongitude = 0.0;
volatile double gpsSpeedKmph = 0.0;
volatile int gpsSatellites = 0;

// --- Tasks ---

void Task_CANReceiver(void *pvParameters) {
  Serial.println("CAN Task started");
  pinMode(CAN_INT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  while(true) {
    if (!digitalRead(CAN_INT_PIN)) {
      unsigned long rxId;
      unsigned char len = 0;
      unsigned char rxBuf[8];

      if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
        Serial.print("CAN >> ID: 0x"); Serial.print(rxId, HEX);
        Serial.print(" | Data: ");
        for (int i=0; i<len; i++) {
          Serial.print(rxBuf[i], HEX); Serial.print(" ");
        }

        if (rxId == STM32_CAN_ID_TEMPERATURE && len == 4) {
          int32_t temp = (int32_t)(rxBuf[0] | ((int32_t)rxBuf[1]<<8) | ((int32_t)rxBuf[2]<<16) | ((int32_t)rxBuf[3]<<24));
          canTemperature = float(temp);
          Serial.print(" | Temp: "); Serial.print(temp); Serial.print(" C");
        }
        if (rxId == STM32_CAN_ID_DISTANCE && len == 4) {
          canDistance = (uint32_t)(rxBuf[0] | ((uint32_t)rxBuf[1]<<8) | ((uint32_t)rxBuf[2]<<16) | ((uint32_t)rxBuf[3]<<24));
          Serial.print(" | Dist: "); Serial.print(canDistance); Serial.print(" cm");
        }
        if (rxId == STM32_CAN_ID_SPEED && len == 4) {
          canSpeed = (uint32_t)(rxBuf[0] | ((uint32_t)rxBuf[1]<<8) | ((uint32_t)rxBuf[2]<<16) | ((uint32_t)rxBuf[3]<<24));
          Serial.print(" | Speed: "); Serial.print(canSpeed); Serial.print(" KMH");
        }

        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(LED_PIN, LOW);
        Serial.println();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Task_ReadDHT(void *pvParameters) {
  dht.begin();
  Serial.println("DHT Task started");

  while(true) {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (!(isnan(humidity) || isnan(temperature))) {
      dhtTemperature = temperature;
      dhtHumidity = humidity;
      Serial.print("DHT11 >> Temp: "); Serial.print(temperature);
      Serial.print(" °C | Humidity: "); Serial.print(humidity);
      Serial.println(" %");
    } else {
      Serial.println("DHT11 >> Sensor read failed!");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void Task_ReadPM(void *pvParameters) {
  Serial.println("PM2.5 Task started");
  analogReadResolution(12);

  while(true) {
    int rawVal = analogRead(PM_SENSOR_PIN);
    float voltage = float(rawVal) / ADC_RESOLUTION * VOLTAGE_REF;
    float vDiff = voltage - BASELINE_VOLTAGE;
    float pm25 = (vDiff > 0) ? vDiff * SCALING_FACTOR : 0;
    pm25Value = pm25;

    Serial.print("PM2.5 >> Raw ADC: "); Serial.print(rawVal);
    Serial.print(" | Voltage: "); Serial.print(voltage, 3);
    Serial.print(" V | PM2.5: "); Serial.print(pm25, 2); Serial.println(" ug/m³");

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void Task_ReadGPS(void *pvParameters) {
  GPS_Serial.begin(9600, SERIAL_8N1, 16, 17); // RX=GPIO16, TX=GPIO17

  Serial.println("GPS Task started");

  while(true) {
    while (GPS_Serial.available() > 0) {
      char c = GPS_Serial.read();
      gps.encode(c);
    }

    if (gps.location.isUpdated()) {
      gpsLatitude = gps.location.lat();
      gpsLongitude = gps.location.lng();
      gpsSpeedKmph = gps.speed.kmph();
      gpsSatellites = gps.satellites.value();

      Serial.print("GPS >> Lat: "); Serial.print(gpsLatitude, 6);
      Serial.print(", Lon: "); Serial.print(gpsLongitude, 6);
      Serial.print(", Speed: "); Serial.print(gpsSpeedKmph);
      Serial.print(" km/h, Sats: "); Serial.println(gpsSatellites);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void Task_Display(void *pvParameters) {
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed!");
    vTaskDelete(NULL);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  while(true) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.printf("CAN Temp: %.1f C\n", canTemperature);
    display.printf("CAN Dist: %u cm\n", canDistance);
    display.printf("CAN Speed: %u KMH\n", canSpeed);
    display.printf("DHT11 Temp: %.1f C\n", dhtTemperature);
    display.printf("Humidity: %.1f %%\n", dhtHumidity);
    display.printf("PM2.5 AQ: %.1f ug/m3\n", pm25Value);
    display.printf("GPS Sat: %d\n", gpsSatellites);
    display.printf("Lat: %.5f\n", gpsLatitude);
    display.printf("Lon: %.5f\n", gpsLongitude);
    display.printf("Speed: %.1f km/h\n", gpsSpeedKmph);
    display.display();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing MCP2515 CAN...");
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN init failed, retrying...");
    delay(1000);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN Initialized Successfully!");

  // Create all the FreeRTOS tasks pinned to cores
  xTaskCreatePinnedToCore(Task_CANReceiver, "CANReceiver", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Task_ReadDHT,    "ReadDHT",    2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_ReadPM,     "ReadPM",     2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_ReadGPS,    "ReadGPS",    4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Display,    "Display",    4096, NULL, 1, NULL, 1);
}

void loop() {
  // FreeRTOS task scheduling handles everything
}
