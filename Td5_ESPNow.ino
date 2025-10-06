/*
 * Land Rover Td5 ECU Interface with ESP-NOW Broadcasting
 * Extended version with wireless data transmission capabilities
 *
 * ESP-NOW Data Broadcasting Features:
 * - Broadcast mode for multiple recipients (no MAC address pairing required)
 * - Structured message schema with 5 data packet types
 * - Configurable transmission intervals
 * - Message sequencing and error detection
 * - Maintains all original ECU interface functionality
 *
 * Message Schema:
 * - FUELLING (0x01): Speed, RPM, injection, MAF, throttle
 * - INPUTS (0x02): Brake switches, clutch, handbrake, gear position
 * - TEMPERATURES (0x03): Coolant, fuel, inlet, ambient temps, battery
 * - PRESSURES (0x04): MAP, AAP, boost, reference voltage
 * - ACTUATORS (0x05): EGR, wastegate positions
 *
 * Packet Structure (max 250 bytes ESP-NOW limit):
 * [Header: 8 bytes] [Data Payload: variable] [Checksum: 1 byte]
 *
 * Recipients can filter by message type and implement their own
 * processing logic for adaptive cruise control, data logging, etc.
 */

#include <SoftwareSerial.h>
#include <esp_now.h>
#include <WiFi.h>

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

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 1
#define ESPNOW_BROADCAST_INTERVAL 250  // 250ms between broadcasts (4Hz)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Broadcast to all

// ESP-NOW Message Types
enum Td5MessageType {
  TD5_MSG_FUELLING = 0x01,
  TD5_MSG_INPUTS = 0x02,
  TD5_MSG_TEMPERATURES = 0x03,
  TD5_MSG_PRESSURES = 0x04,
  TD5_MSG_ACTUATORS = 0x05,
  TD5_MSG_STATUS = 0x06
};

// ESP-NOW Packet Header Structure (8 bytes)
struct __attribute__((packed)) Td5PacketHeader {
  uint32_t timestamp;      // 4 bytes: millis() when packet created
  uint16_t sequence;       // 2 bytes: incrementing sequence number
  uint8_t messageType;     // 1 byte: Td5MessageType enum
  uint8_t dataLength;      // 1 byte: length of data payload
};

// Data Payload Structures
struct __attribute__((packed)) Td5FuellingData {
  uint16_t vehicleSpeed;      // km/h
  uint16_t engineRPM;         // RPM
  uint16_t injectionQuantity; // mg/stroke * 100
  uint16_t manifoldAirFlow;   // kg/h * 10
  uint16_t driverDemand;      // % * 100
};

struct __attribute__((packed)) Td5InputsData {
  uint8_t switchStates;       // Bit field: brake, cruise brake, clutch, handbrake, A/C, cruise, neutral
  uint8_t gearPosition;       // 0=Park, 1=Reverse, 2=Neutral, 3=Drive, etc.
  uint8_t reserved[2];        // For future expansion
};

struct __attribute__((packed)) Td5TemperaturesData {
  int16_t coolantTemp;        // °C
  int16_t fuelTemp;           // °C
  int16_t inletAirTemp;       // °C
  int16_t ambientAirTemp;     // °C
  uint16_t batteryVoltage;    // mV
};

struct __attribute__((packed)) Td5PressuresData {
  uint16_t manifoldPressure;  // kPa
  uint16_t ambientPressure;   // kPa
  uint16_t boostPressure;     // kPa (calculated)
  uint16_t referenceVoltage;  // mV
};

struct __attribute__((packed)) Td5ActuatorsData {
  uint16_t egrPosition;       // % * 100
  uint16_t wastegatePosition; // % * 100
  uint8_t reserved[4];        // For future actuators
};

struct __attribute__((packed)) Td5StatusData {
  uint8_t connectionState;    // TD5_DISCONNECTED, TD5_CONNECTED, etc.
  uint8_t lastErrorCode;      // Last ECU error code received
  uint16_t connectionUptime;  // Seconds since successful connection
  uint32_t totalPacketsSent;  // ESP-NOW transmission statistics
};

// Complete ESP-NOW packet structure
struct __attribute__((packed)) Td5EspNowPacket {
  Td5PacketHeader header;
  union {
    Td5FuellingData fuelling;
    Td5InputsData inputs;
    Td5TemperaturesData temperatures;
    Td5PressuresData pressures;
    Td5ActuatorsData actuators;
    Td5StatusData status;
    uint8_t rawData[240];     // Maximum payload size
  } payload;
  uint8_t checksum;
};

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
unsigned long lastESPNowBroadcast = 0;
bool ecuAuthenticated = false;

// ESP-NOW statistics
uint16_t espNowSequence = 0;
uint32_t totalPacketsSent = 0;
uint32_t totalPacketErrors = 0;
unsigned long connectionStartTime = 0;

// Message buffers
uint8_t txBuffer[256];
uint8_t rxBuffer[256];
uint8_t rxIndex = 0;

// Live data variables (same as original)
uint16_t vehicleSpeed = 0;
bool brakePedalPressed = false;
bool cruiseBrakePressed = false;
bool clutchPedalPressed = false;
bool handbrakeEngaged = false;
bool airConRequest = false;
bool cruiseControlOn = false;
bool neutralSelected = false;
uint8_t gearPosition = 0;
uint16_t engineRPM = 0;
int16_t coolantTemp = 0;
int16_t fuelTemp = 0;
int16_t inletAirTemp = 0;
int16_t ambientAirTemp = 0;
uint16_t batteryVoltage = 0;
uint16_t referenceVoltage = 0;
uint16_t manifoldPressure = 0;
uint16_t ambientPressure = 0;
uint16_t boostPressure = 0;
uint16_t manifoldAirFlow = 0;
uint16_t injectionQuantity = 0;
uint16_t driverDemand = 0;
uint16_t egrPosition = 0;
uint16_t wastegatePosition = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Land Rover Td5 ECU Interface with ESP-NOW ===");
  Serial.println("Initializing...");

  // Configure LED for status indication
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, LOW);

  // Configure K-Line pins
  pinMode(KLINE_TX_PIN, OUTPUT);
  pinMode(KLINE_RX_PIN, INPUT);
  digitalWrite(KLINE_TX_PIN, HIGH);

  // Initialize ESP-NOW
  initializeESPNow();

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
        connectionStartTime = millis();
        digitalWrite(DEBUG_LED_PIN, HIGH);
        Serial.println("ECU authenticated and connected!");
        Serial.println("ESP-NOW broadcasting enabled");
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

      // Broadcast ESP-NOW data
      if (millis() - lastESPNowBroadcast > ESPNOW_BROADCAST_INTERVAL) {
        broadcastCurrentData();
        lastESPNowBroadcast = millis();
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

void initializeESPNow() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register for Send CB to get the status of transmitted packet
  esp_now_register_send_cb(onDataSent);

  // Register broadcast peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add broadcast peer");
    return;
  }

  Serial.println("ESP-NOW initialized successfully");
  Serial.println("Broadcasting on channel " + String(ESPNOW_CHANNEL));
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    totalPacketsSent++;
  } else {
    totalPacketErrors++;
    Serial.println("ESP-NOW transmission failed");
  }
}

void broadcastCurrentData() {
  static uint8_t currentMessageType = TD5_MSG_FUELLING;

  // Cycle through all message types
  switch (currentMessageType) {
    case TD5_MSG_FUELLING:
      broadcastFuellingData();
      break;
    case TD5_MSG_INPUTS:
      broadcastInputsData();
      break;
    case TD5_MSG_TEMPERATURES:
      broadcastTemperaturesData();
      break;
    case TD5_MSG_PRESSURES:
      broadcastPressuresData();
      break;
    case TD5_MSG_ACTUATORS:
      broadcastActuatorsData();
      break;
    case TD5_MSG_STATUS:
      broadcastStatusData();
      break;
  }

  // Move to next message type
  currentMessageType++;
  if (currentMessageType > TD5_MSG_STATUS) {
    currentMessageType = TD5_MSG_FUELLING;
  }
}

void broadcastFuellingData() {
  Td5EspNowPacket packet;

  // Fill header
  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_FUELLING;
  packet.header.dataLength = sizeof(Td5FuellingData);

  // Fill payload
  packet.payload.fuelling.vehicleSpeed = vehicleSpeed;
  packet.payload.fuelling.engineRPM = engineRPM;
  packet.payload.fuelling.injectionQuantity = injectionQuantity;
  packet.payload.fuelling.manifoldAirFlow = manifoldAirFlow;
  packet.payload.fuelling.driverDemand = driverDemand;

  // Calculate checksum
  packet.checksum = calculatePacketChecksum((uint8_t*)&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);

  // Send packet
  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength + 1);
}

void broadcastInputsData() {
  Td5EspNowPacket packet;

  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_INPUTS;
  packet.header.dataLength = sizeof(Td5InputsData);

  // Pack switch states into bit field
  uint8_t switches = 0;
  if (brakePedalPressed) switches |= 0x01;
  if (cruiseBrakePressed) switches |= 0x02;
  if (clutchPedalPressed) switches |= 0x04;
  if (handbrakeEngaged) switches |= 0x08;
  if (airConRequest) switches |= 0x10;
  if (cruiseControlOn) switches |= 0x20;
  if (neutralSelected) switches |= 0x40;

  packet.payload.inputs.switchStates = switches;
  packet.payload.inputs.gearPosition = gearPosition;
  packet.payload.inputs.reserved[0] = 0;
  packet.payload.inputs.reserved[1] = 0;

  packet.checksum = calculatePacketChecksum((uint8_t*)&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength + 1);
}

void broadcastTemperaturesData() {
  Td5EspNowPacket packet;

  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_TEMPERATURES;
  packet.header.dataLength = sizeof(Td5TemperaturesData);

  packet.payload.temperatures.coolantTemp = coolantTemp;
  packet.payload.temperatures.fuelTemp = fuelTemp;
  packet.payload.temperatures.inletAirTemp = inletAirTemp;
  packet.payload.temperatures.ambientAirTemp = ambientAirTemp;
  packet.payload.temperatures.batteryVoltage = batteryVoltage;

  packet.checksum = calculatePacketChecksum((uint8_t*)&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength + 1);
}

void broadcastPressuresData() {
  Td5EspNowPacket packet;

  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_PRESSURES;
  packet.header.dataLength = sizeof(Td5PressuresData);

  packet.payload.pressures.manifoldPressure = manifoldPressure;
  packet.payload.pressures.ambientPressure = ambientPressure;
  packet.payload.pressures.boostPressure = boostPressure;
  packet.payload.pressures.referenceVoltage = referenceVoltage;

  packet.checksum = calculatePacketChecksum((uint8_t*)&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength + 1);
}

void broadcastActuatorsData() {
  Td5EspNowPacket packet;

  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_ACTUATORS;
  packet.header.dataLength = sizeof(Td5ActuatorsData);

  packet.payload.actuators.egrPosition = egrPosition;
  packet.payload.actuators.wastegatePosition = wastegatePosition;
  packet.payload.actuators.reserved[0] = 0;
  packet.payload.actuators.reserved[1] = 0;
  packet.payload.actuators.reserved[2] = 0;
  packet.payload.actuators.reserved[3] = 0;

  packet.checksum = calculatePacketChecksum((uint8_t*)&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength + 1);
}

void broadcastStatusData() {
  Td5EspNowPacket packet;

  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_STATUS;
  packet.header.dataLength = sizeof(Td5StatusData);

  packet.payload.status.connectionState = currentState;
  packet.payload.status.lastErrorCode = 0; // Would need to track this from ECU responses
  packet.payload.status.connectionUptime = (millis() - connectionStartTime) / 1000;
  packet.payload.status.totalPacketsSent = totalPacketsSent;

  packet.checksum = calculatePacketChecksum((uint8_t*)&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength + 1);
}

void sendESPNowPacket(Td5EspNowPacket* packet, size_t length) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)packet, length);

  if (result != ESP_OK) {
    Serial.println("Error sending ESP-NOW packet: " + String(result));
    totalPacketErrors++;
  }
}

uint8_t calculatePacketChecksum(uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];  // XOR checksum for packet validation
  }
  return checksum;
}

// Original Td5 ECU communication functions (unchanged from original code)
void initializeKLine() {
  kLineSerial.begin(TD5_BAUD_RATE);
  Serial.println("K-Line interface initialized at 10400 baud");
}

bool performFastInit() {
  Serial.println("Performing fast initialization sequence...");

  digitalWrite(KLINE_TX_PIN, LOW);
  delay(25);
  digitalWrite(KLINE_TX_PIN, HIGH);
  delay(25);

  kLineSerial.begin(TD5_BAUD_RATE);

  uint8_t initFrame[] = {0x81, 0x13, 0xF7, 0x81, 0x0C};
  sendMessage(initFrame, 5);

  uint8_t expectedResponse[] = {0x03, 0xC1, 0x57, 0x8F, 0xAA};

  if (waitForResponse(expectedResponse, 5, 1000)) {
    Serial.println("ECU responded to initialization");

    uint8_t startDiag[] = {0x02, 0x10, 0xA0, 0xB2};
    sendMessage(startDiag, 4);

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

  uint8_t seedRequest[] = {0x02, 0x27, 0x01, 0x00};
  seedRequest[3] = calculateChecksum(seedRequest, 3);

  sendMessage(seedRequest, 4);

  delay(200);
  if (kLineSerial.available() >= 4) {
    uint8_t response[10];
    int bytesRead = readResponse(response, 10);

    if (bytesRead >= 4 && response[1] == 0x67) {
      uint16_t seed = (response[2] << 8) | response[3];
      Serial.print("Received seed: 0x");
      Serial.println(seed, HEX);

      uint16_t key = calculateTd5Key(seed);
      Serial.print("Calculated key: 0x");
      Serial.println(key, HEX);

      uint8_t keyResponse[] = {0x04, 0x27, 0x02, (uint8_t)(key >> 8), (uint8_t)(key & 0xFF), 0x00};
      keyResponse[5] = calculateChecksum(keyResponse, 5);

      sendMessage(keyResponse, 6);

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
  uint16_t key = seed;
  key = ((key & 0xFF) << 8) | ((key & 0xFF00) >> 8);
  key ^= 0x2E71;
  key += 0xCF;
  key = ((key & 1) << 15) | (key >> 1);
  return key;
}

void requestLiveData() {
  static uint8_t requestType = 0;

  switch (requestType) {
    case 0:
      {
        uint8_t fuellingRequest[] = {0x02, 0x21, 0x20, 0x00};
        fuellingRequest[3] = calculateChecksum(fuellingRequest, 3);
        sendMessage(fuellingRequest, 4);
      }
      break;
    case 1:
      {
        uint8_t inputRequest[] = {0x02, 0x21, 0x21, 0x00};
        inputRequest[3] = calculateChecksum(inputRequest, 3);
        sendMessage(inputRequest, 4);
      }
      break;
    case 2:
      {
        uint8_t tempRequest[] = {0x02, 0x21, 0x22, 0x00};
        tempRequest[3] = calculateChecksum(tempRequest, 3);
        sendMessage(tempRequest, 4);
      }
      break;
    case 3:
      {
        uint8_t pressureRequest[] = {0x02, 0x21, 0x23, 0x00};
        pressureRequest[3] = calculateChecksum(pressureRequest, 3);
        sendMessage(pressureRequest, 4);
      }
      break;
    case 4:
      {
        uint8_t actuatorRequest[] = {0x02, 0x21, 0x24, 0x00};
        actuatorRequest[3] = calculateChecksum(actuatorRequest, 3);
        sendMessage(actuatorRequest, 4);
      }
      break;
  }

  requestType = (requestType + 1) % 5;
}

void processIncomingData() {
  if (kLineSerial.available()) {
    while (kLineSerial.available() && rxIndex < sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = kLineSerial.read();
    }

    if (rxIndex >= 3) {
      uint8_t messageLength = rxBuffer[0];
      if (rxIndex >= messageLength + 1) {
        processECUResponse(rxBuffer, messageLength + 1);
        rxIndex = 0;
      }
    }
  }
}

void processECUResponse(uint8_t* data, int length) {
  if (length < 3) return;

  uint8_t service = data[1];
  uint8_t subfunction = data[2];

  switch (service) {
    case 0x61:
      if (subfunction == 0x20) {
        if (length >= 20) {
          vehicleSpeed = (data[6] << 8) | data[7];
          engineRPM = (data[4] << 8) | data[5];
          injectionQuantity = (data[8] << 8) | data[9];
          manifoldAirFlow = (data[10] << 8) | data[11];
          driverDemand = (data[12] << 8) | data[13];

          Serial.print("FUELLING - Speed: ");
          Serial.print(vehicleSpeed);
          Serial.print(" km/h, RPM: ");
          Serial.print(engineRPM);
          Serial.print(", IQ: ");
          Serial.print(injectionQuantity / 100.0, 2);
          Serial.print(" mg/stroke, MAF: ");
          Serial.print(manifoldAirFlow / 10.0, 1);
          Serial.print(" kg/h, Throttle: ");
          Serial.print(driverDemand / 100.0, 1);
          Serial.println("%");
        }
      } else if (subfunction == 0x21) {
        if (length >= 6) {
          uint8_t inputByte1 = data[3];
          uint8_t inputByte3 = data[5];

          brakePedalPressed = (inputByte1 & 0x01) != 0;
          cruiseBrakePressed = (inputByte1 & 0x02) == 0;
          clutchPedalPressed = (inputByte1 & 0x04) == 0;
          handbrakeEngaged = (inputByte1 & 0x08) != 0;
          airConRequest = (inputByte1 & 0x10) != 0;
          cruiseControlOn = (inputByte1 & 0x20) != 0;
          neutralSelected = (inputByte1 & 0x40) != 0;
          gearPosition = inputByte3 & 0x0F;

          Serial.print("INPUTS - Brake: ");
          Serial.print(brakePedalPressed ? "PRESSED" : "Released");
          Serial.print(", Clutch: ");
          Serial.print(clutchPedalPressed ? "PRESSED" : "Released");
          Serial.print(", Handbrake: ");
          Serial.print(handbrakeEngaged ? "ON" : "Off");
          Serial.print(", CC: ");
          Serial.print(cruiseControlOn ? "ON" : "Off");
          Serial.println();
        }
      } else if (subfunction == 0x22) {
        if (length >= 12) {
          coolantTemp = ((data[3] << 8) | data[4]);
          coolantTemp = (coolantTemp - 2732) / 10;
          fuelTemp = ((data[5] << 8) | data[6]);
          fuelTemp = (fuelTemp - 2732) / 10;
          inletAirTemp = ((data[7] << 8) | data[8]);
          inletAirTemp = (inletAirTemp - 2732) / 10;
          ambientAirTemp = ((data[9] << 8) | data[10]);
          ambientAirTemp = (ambientAirTemp - 2732) / 10;
          batteryVoltage = (data[11] << 8) | data[12];

          Serial.print("TEMPS - Coolant: ");
          Serial.print(coolantTemp);
          Serial.print("°C, Battery: ");
          Serial.print(batteryVoltage / 1000.0, 2);
          Serial.println("V");
        }
      } else if (subfunction == 0x23) {
        if (length >= 10) {
          manifoldPressure = (data[3] << 8) | data[4];
          ambientPressure = (data[5] << 8) | data[6];
          boostPressure = manifoldPressure - ambientPressure;
          referenceVoltage = (data[7] << 8) | data[8];

          Serial.print("PRESSURE - Boost: ");
          Serial.print(boostPressure);
          Serial.print(" kPa (");
          Serial.print(boostPressure * 0.145038, 1);
          Serial.println(" psi)");
        }
      } else if (subfunction == 0x24) {
        if (length >= 8) {
          egrPosition = (data[3] << 8) | data[4];
          wastegatePosition = (data[5] << 8) | data[6];

          Serial.print("ACTUATORS - EGR: ");
          Serial.print(egrPosition / 100.0, 1);
          Serial.print("%, Wastegate: ");
          Serial.print(wastegatePosition / 100.0, 1);
          Serial.println("%");
        }
      }
      break;

    case 0x7F:
      Serial.print("ECU Error - Service: 0x");
      Serial.print(subfunction, HEX);
      Serial.print(", Error: 0x");
      Serial.println(data[3], HEX);
      break;
  }
}

void sendKeepAlive() {
  uint8_t keepAlive[] = {0x01, 0x3E, 0x3F};
  sendMessage(keepAlive, 3);
}

void sendMessage(uint8_t* message, int length) {
  for (int i = 0; i < length; i++) {
    kLineSerial.write(message[i]);
    delay(5);

    unsigned long echoTimeout = millis() + 50;
    while (millis() < echoTimeout && !kLineSerial.available()) {
      delayMicroseconds(100);
    }
    if (kLineSerial.available()) {
      kLineSerial.read();
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
        bytesReceived = 0;
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
      timeout = millis() + 100;
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