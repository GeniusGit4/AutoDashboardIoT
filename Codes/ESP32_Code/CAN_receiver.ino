#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN 5     // Chip Select pin
#define CAN_INT_PIN 4    // Interrupt pin
#define LED_PIN 2        // Onboard LED (change to your actual LED pin)

MCP_CAN CAN(CAN_CS_PIN); // Set CS pin

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CAN_INT_PIN, INPUT); // Use this pin to check for CAN message reception

  Serial.println("Initializing MCP2515 CAN Receiver...");

  // Initialize MCP2515 for 125 kbps:
  // Make sure the baud rate matches your STM32's baud rate (125kbps in your STM32 code)
  while (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("CAN Init Failed, retrying...");
    delay(1000);
  }
  Serial.println("MCP2515 Initialized!");

  CAN.setMode(MCP_NORMAL); // Set to normal mode (not loopback)

  // Configure CAN filters if necessary (highly recommended for busy CAN buses)
  // For simplicity in this example, we are not setting specific filters,
  // meaning it will receive all messages. If you want to filter for ID 0x123:
  // CAN.init_Mask(0, 0, 0xFFF); // Mask for standard ID (all bits must match)
  // CAN.init_Filt(0, 0, 0x123); // Filter for ID 0x123
}

void loop() {
  // Check if CAN message is available (interrupt pin goes low)
  if (!digitalRead(CAN_INT_PIN)) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8]; // Buffer to store received data

    // Read the message buffer
    if (CAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
      Serial.print("Received - ID: 0x");
      Serial.print(rxId, HEX);
      Serial.print(" | DLC: "); // Print Data Length Code
      Serial.print(len);

      Serial.print(" | Raw Data: ");
      for (int i = 0; i < len; i++) {
        if (rxBuf[i] < 0x10) Serial.print("0"); // Pad with a leading zero if byte is less than 0x10
        Serial.print(rxBuf[i], HEX);
        Serial.print(" ");
      }

      // --- NEW: Special handling for our STM32's 4-byte signed integer message ---
      if (rxId == 0x123 && len == 4) { // Expect ID 0x123 and 4 bytes of data
        int received_sensor_data;

        // Reconstruct the 32-bit signed integer from the 4 bytes.
        // Assuming little-endian (LSB first) from your STM32 code:
        // TxData[0] = LSB, TxData[1], TxData[2], TxData[3] = MSB
        received_sensor_data = (int32_t)(rxBuf[0] |((int32_t)rxBuf[1] << 8) |((int32_t)rxBuf[2] << 16) | ((int32_t)rxBuf[3] << 24)); // Cast MSB to int32_t before shifting for proper sign extension

        Serial.print(" | Sensor Value: ");
        Serial.print(received_sensor_data); // Print the reconstructed signed integer
      }
      // --- END NEW ---

      Serial.println(); // Newline after each received message

      // Blink LED on received message
      digitalWrite(LED_PIN, HIGH);
      delay(50); // Short delay for visual indication
      digitalWrite(LED_PIN, LOW);
    }
  }
}
