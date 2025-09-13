/*
 * Land Rover Td5 ECU Interface for ESP32
 * K-Line Communication at 10400 baud
 * Extracts live data: Vehicle Speed, Brake Pedal Switch, Engine Parameters
 * 
 * Hardware Requirements:
 * - ESP32 development board
 * - L9637D K-Line transceiver IC
 * - 1kΩ resistor (ESP32 TX to L9637D)
 * - 510Ω pull-up resistor (K-Line to +12V)
 * - Protection diodes and filtering capacitors
 * 
 * Connections:
 * - GPIO 23 (ESP32 TX) -> 1kΩ -> L9637D Pin 5 (TX)
 * - GPIO 22 (ESP32 RX) <- L9637D Pin 1 (RX)
 * - L9637D Pin 6 (K-Line) -> 510Ω -> +12V
 * - L9637D Pin 6 -> Vehicle K-Line (ECU Pin B18 or OBD Pin 7)
 */

#include <SoftwareSerial.h>

// Pin definitions
#define KLINE_TX_PIN 23
#define KLINE_RX_PIN 22
#define DEBUG_LED_PIN 2

// Protocol constants
#define TD5_BAUD_RATE 10400
#define ECU_ADDRESS 0x19
#define TESTER_ADDRESS 0xF7
#define RESPONSE_TIMEOUT 2000
#define KEEPALIVE_INTERVAL 1500

// Communication state
enum Td5State {
  TD5_DISCONNECTED,
  TD5_INITIALIZING,
  TD5_AUTHENTICATING,
  TD5_CONNECTED,
  TD5_ERROR
};

// Global variables
SoftwareSerial kLineSerial(KLINE_RX_PIN, KLINE_TX_PIN);
Td5State currentState = TD5_DISCONNECTED;
unsigned long lastKeepAlive = 0;
unsigned long lastDataRequest = 0;
bool ecuAuthenticated = false;

// Message buffers
uint8_t txBuffer[256];
uint8_t rxBuffer[256];
uint8_t rxIndex = 0;

// Live data variables
uint16_t vehicleSpeed = 0;          // km/h
bool brakePedalPressed = false;
bool cruiseBrakePressed = false;
uint16_t engineRPM = 0;
int16_t coolantTemp = 0;            // °C
uint16_t batteryVoltage = 0;        // mV

void setup() {
  Serial.begin(115200);
  Serial.println("=== Land Rover Td5 ECU Interface ===");
  Serial.println("Initializing...");
  
  // Configure LED for status indication
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, LOW);
  
  // Configure K-Line pins - CRITICAL: Set pin states BEFORE UART init
  pinMode(KLINE_TX_PIN, OUTPUT);
  pinMode(KLINE_RX_PIN, INPUT);
  digitalWrite(KLINE_TX_PIN, HIGH);
  
  delay(1000);
  
  // Initialize K-Line communication
  initializeKLine();
}

void loop() {
  static unsigned long lastConnectionAttempt = 0;
  
  switch (currentState) {
    case TD5_DISCONNECTED:
      if (millis() - lastConnectionAttempt > 5000) {
        Serial.println("Attempting ECU connection...");
        if (performFastInit()) {
          currentState = TD5_AUTHENTICATING;
          Serial.println("Fast init successful, authenticating...");
        } else {
          Serial.println("Fast init failed, retrying in 5s");
          lastConnectionAttempt = millis();
        }
      }
      break;
      
    case TD5_AUTHENTICATING:
      if (authenticateECU()) {
        currentState = TD5_CONNECTED;
        ecuAuthenticated = true;
        lastKeepAlive = millis();
        digitalWrite(DEBUG_LED_PIN, HIGH);
        Serial.println("ECU authenticated and connected!");
      } else {
        Serial.println("Authentication failed, disconnecting");
        currentState = TD5_DISCONNECTED;
        lastConnectionAttempt = millis();
      }
      break;
      
    case TD5_CONNECTED:
      // Send keep-alive message
      if (millis() - lastKeepAlive > KEEPALIVE_INTERVAL) {
        sendKeepAlive();
        lastKeepAlive = millis();
      }
      
      // Request live data every 500ms
      if (millis() - lastDataRequest > 500) {
        requestLiveData();
        lastDataRequest = millis();
      }
      
      // Process any incoming data
      processIncomingData();
      break;
      
    case TD5_ERROR:
      digitalWrite(DEBUG_LED_PIN, LOW);
      delay(2000);
      currentState = TD5_DISCONNECTED;
      break;
  }
  
  delay(50);
}

void initializeKLine() {
  // Initialize software serial for K-Line communication
  kLineSerial.begin(TD5_BAUD_RATE);
  Serial.println("K-Line interface initialized at 10400 baud");
}

bool performFastInit() {
  Serial.println("Performing fast initialization sequence...");
  
  // Fast init sequence: 25ms LOW, 25ms HIGH
  digitalWrite(KLINE_TX_PIN, LOW);
  delay(25);
  digitalWrite(KLINE_TX_PIN, HIGH);
  delay(25);
  
  // Re-initialize serial after timing sequence
  kLineSerial.begin(TD5_BAUD_RATE);
  
  // Send initialization frame: 0x81 0x13 0xF7 0x81 0x0C
  uint8_t initFrame[] = {0x81, 0x13, 0xF7, 0x81, 0x0C};
  sendMessage(initFrame, 5);
  
  // Wait for response: 0x03 0xC1 0x57 0x8F 0xAA
  uint8_t expectedResponse[] = {0x03, 0xC1, 0x57, 0x8F, 0xAA};
  
  if (waitForResponse(expectedResponse, 5, 1000)) {
    Serial.println("ECU responded to initialization");
    
    // Send start diagnostics command: 0x02 0x10 0xA0 0xB2
    uint8_t startDiag[] = {0x02, 0x10, 0xA0, 0xB2};
    sendMessage(startDiag, 4);
    
    // Wait for positive response
    delay(100);
    if (kLineSerial.available()) {
      Serial.println("Start diagnostics acknowledged");
      return true;
    }
  }
  
  Serial.println("Fast init failed - no valid response");
  return false;
}

bool authenticateECU() {
  Serial.println("Requesting security seed...");
  
  // Request seed: 0x02 0x27 0x01 [checksum]
  uint8_t seedRequest[] = {0x02, 0x27, 0x01, 0x00};
  seedRequest[3] = calculateChecksum(seedRequest, 3);
  
  sendMessage(seedRequest, 4);
  
  // Read seed response
  delay(200);
  if (kLineSerial.available() >= 4) {
    uint8_t response[10];
    int bytesRead = readResponse(response, 10);
    
    if (bytesRead >= 4 && response[1] == 0x67) {
      // Extract 16-bit seed (big-endian)
      uint16_t seed = (response[2] << 8) | response[3];
      Serial.print("Received seed: 0x");
      Serial.println(seed, HEX);
      
      // Calculate key using Td5 algorithm
      uint16_t key = calculateTd5Key(seed);
      Serial.print("Calculated key: 0x");
      Serial.println(key, HEX);
      
      // Send key: 0x04 0x27 0x02 [key_high] [key_low] [checksum]
      uint8_t keyResponse[] = {0x04, 0x27, 0x02, (uint8_t)(key >> 8), (uint8_t)(key & 0xFF), 0x00};
      keyResponse[5] = calculateChecksum(keyResponse, 5);
      
      sendMessage(keyResponse, 6);
      
      // Check for positive response
      delay(200);
      if (kLineSerial.available()) {
        bytesRead = readResponse(response, 10);
        if (bytesRead >= 2 && response[1] == 0x67) {
          Serial.println("Authentication successful!");
          return true;
        }
      }
    }
  }
  
  Serial.println("Authentication failed");
  return false;
}

uint16_t calculateTd5Key(uint16_t seed) {
  // Td5 seed-key algorithm (reverse engineered)
  uint16_t key = seed;
  
  // Rotate bits and apply transformations
  key = ((key & 0xFF) << 8) | ((key & 0xFF00) >> 8);  // Swap bytes
  key ^= 0x2E71;                                       // XOR with constant
  key += 0xCF;                                         // Add constant
  key = ((key & 1) << 15) | (key >> 1);              // Rotate right
  
  return key;
}

void requestLiveData() {
  static uint8_t requestType = 0;
  
  switch (requestType) {
    case 0:
      // Request fuelling data (includes speed): 0x02 0x21 0x20 [checksum]
      {
        uint8_t fuellingRequest[] = {0x02, 0x21, 0x20, 0x00};
        fuellingRequest[3] = calculateChecksum(fuellingRequest, 3);
        sendMessage(fuellingRequest, 4);
      }
      break;
      
    case 1:
      // Request input status (brake switches): 0x02 0x21 0x21 [checksum]
      {
        uint8_t inputRequest[] = {0x02, 0x21, 0x21, 0x00};
        inputRequest[3] = calculateChecksum(inputRequest, 3);
        sendMessage(inputRequest, 4);
      }
      break;
  }
  
  requestType = (requestType + 1) % 2;
}

void processIncomingData() {
  if (kLineSerial.available()) {
    while (kLineSerial.available() && rxIndex < sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = kLineSerial.read();
    }
    
    // Process complete messages
    if (rxIndex >= 3) {
      uint8_t messageLength = rxBuffer[0];
      if (rxIndex >= messageLength + 1) {
        processECUResponse(rxBuffer, messageLength + 1);
        rxIndex = 0;  // Reset for next message
      }
    }
  }
}

void processECUResponse(uint8_t* data, int length) {
  if (length < 3) return;
  
  uint8_t service = data[1];
  uint8_t subfunction = data[2];
  
  switch (service) {
    case 0x61:  // Positive response to data request
      if (subfunction == 0x20) {
        // Fuelling data response (includes speed)
        if (length >= 8) {
          vehicleSpeed = (data[6] << 8) | data[7];  // Speed in km/h
          engineRPM = (data[4] << 8) | data[5];     // RPM
          
          Serial.print("Speed: ");
          Serial.print(vehicleSpeed);
          Serial.print(" km/h (");
          Serial.print(vehicleSpeed * 0.621371, 1);
          Serial.print(" mph), RPM: ");
          Serial.println(engineRPM);
        }
      } else if (subfunction == 0x21) {
        // Input status response (brake switches)
        if (length >= 4) {
          uint8_t inputByte = data[3];
          brakePedalPressed = (inputByte & 0x01) != 0;     // Bit 0
          cruiseBrakePressed = (inputByte & 0x02) == 0;    // Bit 1 (inverted)
          
          Serial.print("Brake Pedal: ");
          Serial.print(brakePedalPressed ? "PRESSED" : "Released");
          Serial.print(", Cruise Brake: ");
          Serial.println(cruiseBrakePressed ? "PRESSED" : "Released");
        }
      }
      break;
      
    case 0x7F:  // Negative response
      Serial.print("ECU Error - Service: 0x");
      Serial.print(subfunction, HEX);
      Serial.print(", Error: 0x");
      Serial.println(data[3], HEX);
      break;
  }
}

void sendKeepAlive() {
  // Tester present: 0x01 0x3E 0x3F
  uint8_t keepAlive[] = {0x01, 0x3E, 0x3F};
  sendMessage(keepAlive, 3);
}

void sendMessage(uint8_t* message, int length) {
  // Echo cancellation - read and discard echoed bytes
  for (int i = 0; i < length; i++) {
    kLineSerial.write(message[i]);
    delay(5);  // Inter-byte delay for reliability
    
    // Read and discard echo
    unsigned long echoTimeout = millis() + 50;
    while (millis() < echoTimeout && !kLineSerial.available()) {
      delayMicroseconds(100);
    }
    if (kLineSerial.available()) {
      kLineSerial.read();  // Discard echoed byte
    }
  }
}

bool waitForResponse(uint8_t* expected, int expectedLength, unsigned long timeout) {
  unsigned long startTime = millis();
  int bytesReceived = 0;
  
  while (millis() - startTime < timeout && bytesReceived < expectedLength) {
    if (kLineSerial.available()) {
      uint8_t receivedByte = kLineSerial.read();
      if (receivedByte == expected[bytesReceived]) {
        bytesReceived++;
      } else {
        bytesReceived = 0;  // Reset if sequence broken
      }
    }
  }
  
  return bytesReceived == expectedLength;
}

int readResponse(uint8_t* buffer, int maxLength) {
  int bytesRead = 0;
  unsigned long timeout = millis() + RESPONSE_TIMEOUT;
  
  while (millis() < timeout && bytesRead < maxLength) {
    if (kLineSerial.available()) {
      buffer[bytesRead++] = kLineSerial.read();
      timeout = millis() + 100;  // Reset timeout on new data
    }
  }
  
  return bytesRead;
}

uint8_t calculateChecksum(uint8_t* data, int length) {
  uint8_t checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum += data[i];
  }
  return checksum;
}

void printHexBuffer(uint8_t* buffer, int length) {
  for (int i = 0; i < length; i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
