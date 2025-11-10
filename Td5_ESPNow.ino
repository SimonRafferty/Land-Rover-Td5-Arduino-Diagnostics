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
#include "td5_parameters.h"  // Structured PID definitions with scaling

// Pin definitions
#define KLINE_TX_PIN GPIO_NUM_2  // D1 on XIAO ESP32S3
#define KLINE_RX_PIN GPIO_NUM_1  // D0 on XIAO ESP32S3
#define DEBUG_LED_PIN LED_BUILTIN

// Protocol constants
#define TD5_BAUD_RATE 10400
#define ECU_ADDRESS 0x19
#define TESTER_ADDRESS 0xF7
#define RESPONSE_TIMEOUT 500         // Wait up to 500ms for composite PIDs
#define KEEPALIVE_INTERVAL 1500
#define DATA_TIMEOUT 5000            // Connection lost if no ECU data for 5 seconds
#define P3_MIN_DELAY 25              // Minimum 25ms between ECU response and next request (TD5 default)

// ESP-NOW Configuration
#define ESPNOW_CHANNEL 0  // Channel 0 per Seeed Studio XIAO ESP32S3 official example
// Event-driven broadcasting: ESP-NOW now broadcasts immediately when data changes (no timer)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Broadcast to all

// ESP-NOW Message Types
enum Td5MessageType {
  TD5_MSG_NONE = 0x00,          // No data needed
  TD5_MSG_FUELLING = 0x01,
  TD5_MSG_INPUTS = 0x02,
  TD5_MSG_TEMPERATURES = 0x03,
  TD5_MSG_PRESSURES = 0x04,
  TD5_MSG_ACTUATORS = 0x05,
  TD5_MSG_STATUS = 0x06
};

// Display Priority Message (received from gauge)
struct __attribute__((packed)) DisplayPriorityMessage {
  uint8_t messageType;        // 0x10 = Display Priority Update
  uint8_t currentMsgType;     // Current screen (0x00-0x06)
  uint8_t prevMsgType;        // Previous screen (swipe right)
  uint8_t nextMsgType;        // Next screen (swipe left)
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
  int8_t coolantTemp;         // °C (must match gauge: int8_t)
  int8_t fuelTemp;            // °C (must match gauge: int8_t)
  int8_t inletAirTemp;        // °C (must match gauge: int8_t)
  int8_t ambientTemp;         // °C (must match gauge: int8_t)
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
  uint8_t errorCode;          // Last ECU error code received (must match gauge field name)
  uint32_t uptime;            // Seconds since successful connection (must match gauge: uint32_t)
};

// Complete ESP-NOW packet structure (NO CHECKSUM - gauge doesn't expect it)
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
};

// Communication state
enum Td5State {
  TD5_DISCONNECTED,
  TD5_INITIALIZING,
  TD5_AUTHENTICATING,
  TD5_CONNECTED,
  TD5_CONNECTION_LOST,  // NEW: Timeout detected, clearing stale data
  TD5_ERROR
};

// Request/Response state machine for non-blocking I/O
enum RequestState {
  REQ_IDLE,              // No request pending
  REQ_SENT,              // Request sent, waiting for response
  REQ_WAITING_RESPONSE   // Response started arriving
};

// State tracking with timeout management
struct StateInfo {
  Td5State state;
  unsigned long stateEnterTime;
  unsigned long lastECUResponse;
  uint8_t retryCount;
};

// Request handler for non-blocking operation
struct RequestHandler {
  RequestState state;
  uint8_t currentPID;
  unsigned long sentTime;
  unsigned long responseStartTime;
};

// Global variables
SoftwareSerial kLineSerial(KLINE_RX_PIN, KLINE_TX_PIN);
Td5State currentState = TD5_DISCONNECTED;
unsigned long lastKeepAlive = 0;
unsigned long lastDataRequest = 0;
// unsigned long lastESPNowBroadcast = 0;  // No longer used - event-driven broadcasting
bool ecuAuthenticated = false;

// State and request tracking
StateInfo stateInfo = {TD5_DISCONNECTED, 0, 0, 0};
RequestHandler reqHandler = {REQ_IDLE, 0, 0, 0};

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
int16_t ambientTemp = 0;
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

// Previous values for change detection
uint16_t prev_vehicleSpeed = 0;
uint16_t prev_engineRPM = 0;
uint16_t prev_injectionQuantity = 0;
uint16_t prev_manifoldAirFlow = 0;
uint16_t prev_driverDemand = 0;
uint8_t prev_switchStates = 0;
int16_t prev_coolantTemp = 0;
int16_t prev_fuelTemp = 0;
int16_t prev_inletAirTemp = 0;
int16_t prev_ambientTemp = 0;
uint16_t prev_batteryVoltage = 0;
uint16_t prev_manifoldPressure = 0;
uint16_t prev_ambientPressure = 0;
uint16_t prev_boostPressure = 0;
uint16_t prev_referenceVoltage = 0;
uint16_t prev_egrPosition = 0;
uint16_t prev_wastegatePosition = 0;

// Data validity flags - only broadcast when we have received valid data from ECU
bool validFuellingData = false;
bool validInputsData = false;
bool validTemperaturesData = false;
bool validPressuresData = false;
bool validActuatorsData = false;

// Clear all validity flags (called on connection loss/timeout)
void clearAllValidityFlags() {
  validFuellingData = false;
  validInputsData = false;
  validTemperaturesData = false;
  validPressuresData = false;
  validActuatorsData = false;
  Serial.println("All validity flags cleared - stale data will not be broadcast");
}

// ECU data tracking
unsigned long lastECUDataReceived = 0;  // Track last successful ECU response (for info only)

// Dynamic PID priority adjustment - unreliable PIDs polled less frequently
int8_t pidReliabilityScore[256] = {0};     // -100 to +100, affects polling frequency
unsigned long pidLastAttempt[256] = {0};   // Last time each PID was requested
#define PID_SUCCESS_INCREMENT 2             // Increase score on success
#define PID_FAILURE_DECREMENT -5            // Decrease score on failure/timeout
#define PID_UNRELIABLE_THRESHOLD -30        // Below this = unreliable (poll every 10s)
#define PID_UNRELIABLE_RETRY_INTERVAL 10000 // Retry unreliable PIDs every 10 seconds

// Display-driven priority system
uint8_t currentDisplayMsgType = TD5_MSG_NONE;  // Current screen on display
uint8_t prevDisplayMsgType = TD5_MSG_NONE;     // Previous screen (swipe right)
uint8_t nextDisplayMsgType = TD5_MSG_NONE;     // Next screen (swipe left)
unsigned long lastDisplayPriorityUpdate = 0;   // Last time we received priority update
#define DISPLAY_PRIORITY_TIMEOUT 5000          // Revert to default after 5s no updates

void runESPNowGaugeSweepTest() {
  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║   ESP-NOW GAUGE SWEEP TEST                        ║");
  Serial.println("║   Fast sweep: 0 → 5000 → 0 (~2 seconds)           ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");

  // TEMPORARILY enable broadcasting for test - will be cleared after
  validFuellingData = true;

  const int sweepSteps = 40;        // Number of steps (smoother sweep)
  const int stepDelayMs = 20;       // Fast: 20ms between steps

  Serial.print("Sweeping UP");

  // Sweep UP: 0 → 5000 RPM
  for (int step = 0; step <= sweepSteps; step++) {
    float progress = (float)step / sweepSteps;
    engineRPM = progress * 5000;  // 0 → 5000 RPM

    broadcastFuellingData();  // Only send fuelling data (contains RPM)
    delay(stepDelayMs);

    if (step % 8 == 0) {
      Serial.print(".");
    }
  }
  Serial.println(" [5000 RPM]");

  delay(100);  // Brief pause at max RPM

  Serial.print("Sweeping DOWN");

  // Sweep DOWN: 5000 → 0 RPM
  for (int step = sweepSteps; step >= 0; step--) {
    float progress = (float)step / sweepSteps;
    engineRPM = progress * 5000;

    broadcastFuellingData();
    delay(stepDelayMs);

    if (step % 8 == 0) {
      Serial.print(".");
    }
  }
  Serial.println(" [0 RPM]");

  // Reset RPM to zero and CLEAR validity flags
  engineRPM = 0;
  broadcastFuellingData();

  // CRITICAL FIX: Clear all validity flags after sweep test
  // They will be set again when real ECU data arrives
  clearAllValidityFlags();

  Serial.println("\n✓ Gauge Sweep Complete (~1.8 seconds)");
  Serial.println("Proceeding to ECU connection...\n");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Land Rover Td5 ECU Interface with ESP-NOW ===");
  Serial.println("Initializing...");

  // Configure LED for status indication
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(DEBUG_LED_PIN, LOW);
  delay(1000);
  digitalWrite(DEBUG_LED_PIN, HIGH);
  delay(1000);
  digitalWrite(DEBUG_LED_PIN, LOW);

  // Configure K-Line pins
  pinMode(KLINE_TX_PIN, OUTPUT);
  pinMode(KLINE_RX_PIN, INPUT);
  digitalWrite(KLINE_TX_PIN, HIGH);

  // Initialize ESP-NOW
  initializeESPNow();

  delay(1000);

  // Run ESP-NOW gauge sweep test (indoor testing without ECU)
  runESPNowGaugeSweepTest();

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
      // Try authentication first
      if (authenticateECU()) {
        currentState = TD5_CONNECTED;
        stateInfo.state = TD5_CONNECTED;
        stateInfo.stateEnterTime = millis();
        stateInfo.lastECUResponse = millis();
        stateInfo.retryCount = 0;
        ecuAuthenticated = true;
        lastKeepAlive = millis();
        connectionStartTime = millis();
        lastECUDataReceived = millis();
        digitalWrite(DEBUG_LED_PIN, HIGH);
        Serial.println("ECU authenticated and connected!");
        Serial.println("ESP-NOW broadcasting enabled");
      } else {
        // Authentication failed - try proceeding anyway for read-only access
        Serial.println("Authentication failed - trying unauthenticated read access...");
        currentState = TD5_CONNECTED;
        stateInfo.state = TD5_CONNECTED;
        stateInfo.stateEnterTime = millis();
        stateInfo.lastECUResponse = millis();
        stateInfo.retryCount = 0;
        ecuAuthenticated = false;
        lastKeepAlive = millis();
        connectionStartTime = millis();
        lastECUDataReceived = millis();
        digitalWrite(DEBUG_LED_PIN, HIGH);
        Serial.println("Proceeding with unauthenticated mode (read-only)");
      }
      break;

    case TD5_CONNECTED:
      {
        // CRITICAL FIX: Check for connection timeout
        if (millis() - stateInfo.lastECUResponse > DATA_TIMEOUT) {
          Serial.println("*** ECU CONNECTION TIMEOUT - No data for 5 seconds ***");
          currentState = TD5_CONNECTION_LOST;
          stateInfo.state = TD5_CONNECTION_LOST;
          stateInfo.stateEnterTime = millis();
          digitalWrite(DEBUG_LED_PIN, LOW);
          clearAllValidityFlags();  // CRITICAL: Clear stale data flags
          break;
        }

        // Send keep-alive message
        if (millis() - lastKeepAlive > KEEPALIVE_INTERVAL) {
          sendKeepAlive();
          lastKeepAlive = millis();
        }

        // IMPROVED: NON-BLOCKING REQUEST-RESPONSE CYCLE
        // State machine handles request/response without blocking waits

        switch (reqHandler.state) {
          case REQ_IDLE:
            // Send next request
            requestLiveData();
            reqHandler.state = REQ_SENT;
            reqHandler.sentTime = millis();
            break;

          case REQ_SENT:
            // Check if response has started arriving (either in serial buffer or rxBuffer)
            if (kLineSerial.available() || rxIndex > 0) {
              processIncomingData();
              stateInfo.lastECUResponse = millis();  // Update timeout tracker
              lastECUDataReceived = millis();
              reqHandler.state = REQ_IDLE;  // Ready for next request

              // P3 minimum delay: 25ms between ECU response and next tester request
              delay(P3_MIN_DELAY);
            } else if (millis() - reqHandler.sentTime > RESPONSE_TIMEOUT) {
              // Response timeout - move on without error (dynamic priority handles it)
              reqHandler.state = REQ_IDLE;

              // Still need P3 delay even without response
              delay(P3_MIN_DELAY);
            }
            break;

          case REQ_WAITING_RESPONSE:
            // Not currently used, but available for multi-byte response handling
            reqHandler.state = REQ_IDLE;
            break;
        }

        // Broadcast ESP-NOW data immediately when it changes (event-driven)
        broadcastCurrentData();
      }
      break;

    case TD5_CONNECTION_LOST:
      {
        Serial.println("Connection lost - attempting reconnection...");
        digitalWrite(DEBUG_LED_PIN, LOW);

        // Clear all state
        clearAllValidityFlags();
        reqHandler.state = REQ_IDLE;
        reqHandler.currentPID = 0;

        // Attempt reconnection
        delay(1000);
        currentState = TD5_DISCONNECTED;
        stateInfo.state = TD5_DISCONNECTED;
        stateInfo.stateEnterTime = millis();
        stateInfo.retryCount++;
      }
      break;

    case TD5_ERROR:
      digitalWrite(DEBUG_LED_PIN, LOW);
      delay(2000);
      currentState = TD5_DISCONNECTED;
      break;
  }

  delay(1);  // Ultra-minimal delay - maximize async response processing
}

void initializeESPNow() {
  Serial.println("\n=== Initializing ESP-NOW (ESP32-S3) ===");

  // Set device as a Wi-Fi Station (must be done BEFORE esp_now_init)
  Serial.println("Setting WiFi mode to STA...");
  WiFi.mode(WIFI_STA);
  delay(100);  // Give WiFi time to initialize

  // Disconnect from any WiFi network (call AFTER WiFi.mode)
  Serial.println("Disconnecting from WiFi networks...");
  WiFi.disconnect();
  delay(100);

  // ESP32S3 CRITICAL: Set WiFi channel BEFORE esp_now_init
  Serial.print("Setting WiFi channel to ");
  Serial.print(ESPNOW_CHANNEL);
  Serial.println("...");
  WiFi.setChannel(ESPNOW_CHANNEL);
  delay(100);

  // Disable WiFi sleep to prevent ESP-NOW from being de-initialized
  Serial.println("Disabling WiFi sleep...");
  WiFi.setSleep(false);

  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Check if MAC is valid
  String mac = WiFi.macAddress();
  if (mac == "00:00:00:00:00:00") {
    Serial.println("ERROR: Invalid MAC address - WiFi not initialized!");
    Serial.println("ESP-NOW CANNOT be initialized without valid WiFi");
    return;
  }

  // Initialize ESP-NOW
  Serial.println("Calling esp_now_init()...");
  esp_err_t initResult = esp_now_init();
  if (initResult != ESP_OK) {
    Serial.print("ERROR: esp_now_init() failed with code: ");
    Serial.println(initResult);
    return;
  }
  Serial.println("esp_now_init() SUCCESS");

  // Register for Send CB to get the status of transmitted packet
  Serial.println("Registering send callback...");
  esp_err_t cbResult = esp_now_register_send_cb(onDataSent);
  if (cbResult != ESP_OK) {
    Serial.print("ERROR: register_send_cb failed: ");
    Serial.println(cbResult);
    return;
  }

  // Register for Receive CB to get display priority updates
  Serial.println("Registering receive callback...");
  esp_err_t recvResult = esp_now_register_recv_cb(onDataReceive);
  if (recvResult != ESP_OK) {
    Serial.print("ERROR: register_recv_cb failed: ");
    Serial.println(recvResult);
    return;
  }

  // Register broadcast peer
  Serial.println("Adding broadcast peer...");
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  // Add peer
  esp_err_t peerResult = esp_now_add_peer(&peerInfo);
  if (peerResult != ESP_OK) {
    Serial.print("ERROR: Failed to add broadcast peer, code: ");
    Serial.println(peerResult);
    return;
  }

  Serial.println("*** ESP-NOW INITIALIZED SUCCESSFULLY ***");
  Serial.print("Broadcasting on channel: ");
  Serial.println(ESPNOW_CHANNEL);
  Serial.println("================================\n");
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    totalPacketsSent++;
  } else {
    totalPacketErrors++;
    Serial.println("ESP-NOW transmission failed");
  }
}

void onDataReceive(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
  if (len < 1) return;

  uint8_t messageType = data[0];

  // Check for Display Priority Update (0x10)
  if (messageType == 0x10 && len == sizeof(DisplayPriorityMessage)) {
    DisplayPriorityMessage* msg = (DisplayPriorityMessage*)data;

    // Update transmission priorities
    currentDisplayMsgType = msg->currentMsgType;
    prevDisplayMsgType = msg->prevMsgType;
    nextDisplayMsgType = msg->nextMsgType;
    lastDisplayPriorityUpdate = millis();

    Serial.print("[DISPLAY PRIORITY] Current: 0x");
    Serial.print(msg->currentMsgType, HEX);
    Serial.print(", Prev: 0x");
    Serial.print(msg->prevMsgType, HEX);
    Serial.print(", Next: 0x");
    Serial.println(msg->nextMsgType, HEX);
  }
}

// No timeout function - if data doesn't arrive, we just keep requesting
// Values stay at their last known state, validity flags remain set

bool fuellingDataChanged() {
  return (vehicleSpeed != prev_vehicleSpeed ||
          engineRPM != prev_engineRPM ||
          injectionQuantity != prev_injectionQuantity ||
          manifoldAirFlow != prev_manifoldAirFlow ||
          driverDemand != prev_driverDemand);
}

bool inputsDataChanged() {
  uint8_t switches = 0;
  if (brakePedalPressed) switches |= 0x01;
  if (cruiseBrakePressed) switches |= 0x02;
  if (clutchPedalPressed) switches |= 0x04;
  if (handbrakeEngaged) switches |= 0x08;
  if (airConRequest) switches |= 0x10;
  if (cruiseControlOn) switches |= 0x20;
  if (neutralSelected) switches |= 0x40;
  return (switches != prev_switchStates);
}

bool temperaturesDataChanged() {
  return (coolantTemp != prev_coolantTemp ||
          fuelTemp != prev_fuelTemp ||
          inletAirTemp != prev_inletAirTemp ||
          ambientTemp != prev_ambientTemp ||
          batteryVoltage != prev_batteryVoltage);
}

bool pressuresDataChanged() {
  return (manifoldPressure != prev_manifoldPressure ||
          ambientPressure != prev_ambientPressure ||
          boostPressure != prev_boostPressure ||
          referenceVoltage != prev_referenceVoltage);
}

bool actuatorsDataChanged() {
  return (egrPosition != prev_egrPosition ||
          wastegatePosition != prev_wastegatePosition);
}

void broadcastCurrentData() {
  static uint8_t currentMessageType = TD5_MSG_FUELLING;
  bool dataSent = false;

  // Only broadcast if data is VALID and has changed
  switch (currentMessageType) {
    case TD5_MSG_FUELLING:
      if (validFuellingData && fuellingDataChanged()) {
        broadcastFuellingData();
        prev_vehicleSpeed = vehicleSpeed;
        prev_engineRPM = engineRPM;
        prev_injectionQuantity = injectionQuantity;
        prev_manifoldAirFlow = manifoldAirFlow;
        prev_driverDemand = driverDemand;
        dataSent = true;
      }
      break;
    case TD5_MSG_INPUTS:
      if (validInputsData && inputsDataChanged()) {
        broadcastInputsData();
        uint8_t switches = 0;
        if (brakePedalPressed) switches |= 0x01;
        if (cruiseBrakePressed) switches |= 0x02;
        if (clutchPedalPressed) switches |= 0x04;
        if (handbrakeEngaged) switches |= 0x08;
        if (airConRequest) switches |= 0x10;
        if (cruiseControlOn) switches |= 0x20;
        if (neutralSelected) switches |= 0x40;
        prev_switchStates = switches;
        dataSent = true;
      }
      break;
    case TD5_MSG_TEMPERATURES:
      if (validTemperaturesData && temperaturesDataChanged()) {
        broadcastTemperaturesData();
        prev_coolantTemp = coolantTemp;
        prev_fuelTemp = fuelTemp;
        prev_inletAirTemp = inletAirTemp;
        prev_ambientTemp = ambientTemp;
        prev_batteryVoltage = batteryVoltage;
        dataSent = true;
      }
      break;
    case TD5_MSG_PRESSURES:
      if (validPressuresData && pressuresDataChanged()) {
        broadcastPressuresData();
        prev_manifoldPressure = manifoldPressure;
        prev_ambientPressure = ambientPressure;
        prev_boostPressure = boostPressure;
        prev_referenceVoltage = referenceVoltage;
        dataSent = true;
      }
      break;
    case TD5_MSG_ACTUATORS:
      if (validActuatorsData && actuatorsDataChanged()) {
        broadcastActuatorsData();
        prev_egrPosition = egrPosition;
        prev_wastegatePosition = wastegatePosition;
        dataSent = true;
      }
      break;
    case TD5_MSG_STATUS:
      // Always send status updates (less frequent, important for connection state)
      broadcastStatusData();
      dataSent = true;
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

  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_FUELLING;
  packet.header.dataLength = sizeof(Td5FuellingData);

  packet.payload.fuelling.vehicleSpeed = vehicleSpeed;
  packet.payload.fuelling.engineRPM = engineRPM;
  packet.payload.fuelling.injectionQuantity = injectionQuantity;
  packet.payload.fuelling.manifoldAirFlow = manifoldAirFlow;
  packet.payload.fuelling.driverDemand = driverDemand;

  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
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

  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
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
  packet.payload.temperatures.ambientTemp = ambientTemp;
  packet.payload.temperatures.batteryVoltage = batteryVoltage;

  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
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

  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
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

  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
}

void broadcastStatusData() {
  Td5EspNowPacket packet;

  packet.header.timestamp = millis();
  packet.header.sequence = espNowSequence++;
  packet.header.messageType = TD5_MSG_STATUS;
  packet.header.dataLength = sizeof(Td5StatusData);

  packet.payload.status.connectionState = currentState;
  packet.payload.status.errorCode = 0; // Would need to track this from ECU responses
  packet.payload.status.uptime = (millis() - connectionStartTime) / 1000;

  sendESPNowPacket(&packet, sizeof(Td5PacketHeader) + packet.header.dataLength);
}

void sendESPNowPacket(Td5EspNowPacket* packet, size_t length) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)packet, length);

  if (result != ESP_OK) {
    totalPacketErrors++;
    // Print ESP-NOW errors occasionally to avoid cluttering output
    static uint32_t lastErrorPrint = 0;
    if (millis() - lastErrorPrint > 5000) {  // Print max once per 5 seconds
      Serial.print("ESP-NOW ERROR: ");
      Serial.print(totalPacketErrors);
      Serial.print(" failures (code: ");
      Serial.print(result);
      Serial.print(" = ");

      // Decode common ESP-NOW error codes
      switch (result) {
        case 12396: Serial.print("ESP_ERR_ESPNOW_NOT_INIT"); break;
        case 12397: Serial.print("ESP_ERR_ESPNOW_ARG"); break;
        case 12398: Serial.print("ESP_ERR_ESPNOW_NO_MEM"); break;
        case 12399: Serial.print("ESP_ERR_ESPNOW_FULL"); break;
        case 12400: Serial.print("ESP_ERR_ESPNOW_NOT_FOUND"); break;
        case 12401: Serial.print("ESP_ERR_ESPNOW_INTERNAL"); break;
        case 12402: Serial.print("ESP_ERR_ESPNOW_EXIST"); break;
        case 12403: Serial.print("ESP_ERR_ESPNOW_IF"); break;
        default: Serial.print("UNKNOWN");
      }
      Serial.println(")");

      if (result == 12396) {
        Serial.println("  -> ESP-NOW not initialized! Check WiFi mode and initialization sequence");
      }

      lastErrorPrint = millis();
    }
  }
}

uint8_t calculatePacketChecksum(uint8_t* data, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; i++) {
    checksum ^= data[i];  // XOR checksum for packet validation
  }
  return checksum;
}

// K-Line Hardware Diagnostic Functions

void testKLineLoopback() {
  Serial.println("\n=== K-Line Hardware Loopback Test ===");
  Serial.println("This test requires TX and RX shorted together (bypass L9637D)");
  Serial.println("Sending test pattern...");

  kLineSerial.begin(TD5_BAUD_RATE);
  delay(100);

  // Clear buffer
  while (kLineSerial.available()) {
    kLineSerial.read();
  }

  uint8_t testPattern[] = {0xAA, 0x55, 0x01, 0x02, 0x03};

  for (int i = 0; i < 5; i++) {
    kLineSerial.write(testPattern[i]);
    delay(10);
  }

  delay(100);

  int bytesReceived = 0;
  uint8_t receivedBytes[10];

  while (kLineSerial.available() && bytesReceived < 10) {
    receivedBytes[bytesReceived++] = kLineSerial.read();
  }

  if (bytesReceived > 0) {
    Serial.print("Received ");
    Serial.print(bytesReceived);
    Serial.print(" bytes: ");
    for (int i = 0; i < bytesReceived; i++) {
      Serial.print("0x");
      if (receivedBytes[i] < 0x10) Serial.print("0");
      Serial.print(receivedBytes[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println("Loopback test: PASSED - Hardware is working");
  } else {
    Serial.println("No bytes received in loopback test");
    Serial.println("Loopback test: FAILED - Check wiring or SoftwareSerial");
  }
  Serial.println("=== End Loopback Test ===\n");
}

void monitorKLineActivity(int durationMs) {
  Serial.println("\n=== K-Line Activity Monitor ===");
  Serial.print("Monitoring for ");
  Serial.print(durationMs);
  Serial.println("ms...");

  unsigned long startTime = millis();
  int bytesReceived = 0;

  while (millis() - startTime < durationMs) {
    if (kLineSerial.available()) {
      uint8_t b = kLineSerial.read();
      Serial.print("0x");
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
      bytesReceived++;

      if (bytesReceived % 16 == 0) {
        Serial.println();
      }
    }
  }

  if (bytesReceived > 0) {
    Serial.println();
    Serial.print("Total bytes received: ");
    Serial.println(bytesReceived);
  } else {
    Serial.println("No activity detected on K-Line");
  }
  Serial.println("=== End Monitor ===\n");
}

// Original Td5 ECU communication functions (unchanged from original code)
void initializeKLine() {
  kLineSerial.begin(TD5_BAUD_RATE);
  Serial.println("K-Line interface initialized at 10400 baud");

  // Optional: Enable these for hardware diagnostics
  // Uncomment the line below to test K-Line hardware loopback (requires TX/RX shorted)
  // testKLineLoopback();

  // Uncomment to monitor any existing K-Line traffic for 5 seconds
  // monitorKLineActivity(5000);
}

bool performFastInit() {
  Serial.println("Performing fast initialization sequence...");

  // Clear any stale data
  while (kLineSerial.available()) {
    kLineSerial.read();
  }

  // Stop serial to control pin directly for fast init
  kLineSerial.end();
  pinMode(KLINE_TX_PIN, OUTPUT);

  // CRITICAL: ISO 14230 requires minimum 300ms idle time after ECU power-up
  // Ensure K-Line is HIGH (idle) for sufficient time
  digitalWrite(KLINE_TX_PIN, HIGH);
  Serial.println("Waiting 300ms idle time (ISO 14230 requirement)...");
  delay(300);

  // Fast init pulse - timing must be precise: 25ms ±1ms
  Serial.println("Sending fast init pulse...");
  digitalWrite(KLINE_TX_PIN, LOW);
  delay(25);
  digitalWrite(KLINE_TX_PIN, HIGH);
  delay(25);

  // Restart serial communication
  kLineSerial.begin(TD5_BAUD_RATE);
  delay(30);  // Wait for serial to stabilize

  uint8_t initFrame[] = {0x81, 0x13, 0xF7, 0x81, 0x0C};

  Serial.print("Sending init frame: ");
  for (int i = 0; i < 5; i++) {
    Serial.print("0x");
    if (initFrame[i] < 0x10) Serial.print("0");
    Serial.print(initFrame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  sendMessage(initFrame, 5);

  uint8_t expectedResponse[] = {0x03, 0xC1, 0x57, 0x8F, 0xAA};

  // Enhanced response monitoring
  Serial.println("Waiting for ECU response (1000ms timeout)...");
  unsigned long startWait = millis();
  int bytesReceived = 0;
  uint8_t receivedBytes[20];

  while (millis() - startWait < 1000) {
    if (kLineSerial.available()) {
      if (bytesReceived < 20) {
        receivedBytes[bytesReceived++] = kLineSerial.read();
      }
    }
  }

  if (bytesReceived > 0) {
    Serial.print("Received ");
    Serial.print(bytesReceived);
    Serial.print(" bytes: ");
    for (int i = 0; i < bytesReceived; i++) {
      Serial.print("0x");
      if (receivedBytes[i] < 0x10) Serial.print("0");
      Serial.print(receivedBytes[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Check for negative response indicating ECU still in active session
    // Response: 0x03 0x7F 0x81 0x10 (length=3, negativeResponse, requestedService=0x81, error=generalReject)
    if (bytesReceived >= 4 &&
        receivedBytes[0] == 0x03 &&
        receivedBytes[1] == 0x7F &&
        receivedBytes[2] == 0x81 &&
        receivedBytes[3] == 0x10) {
      Serial.println("*** ECU REJECTED: Still in active diagnostic session ***");
      Serial.println("*** Waiting 10 seconds for ECU to timeout old session ***");
      delay(10000);
      Serial.println("Backoff complete, ready to retry");
      return false;
    }

    // Check if it matches expected response
    bool matchesExpected = (bytesReceived >= 5);
    for (int i = 0; i < 5 && matchesExpected; i++) {
      if (receivedBytes[i] != expectedResponse[i]) {
        matchesExpected = false;
      }
    }

    if (matchesExpected) {
      Serial.println("ECU responded to initialization");

      uint8_t startDiag[] = {0x02, 0x10, 0xA0, 0xB2};
      Serial.print("Sending Start Diagnostics: ");
      for (int i = 0; i < 4; i++) {
        Serial.print("0x");
        if (startDiag[i] < 0x10) Serial.print("0");
        Serial.print(startDiag[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      sendMessage(startDiag, 4);

      delay(200);

      // Read and display the Start Diagnostics response
      uint8_t diagResponse[10];
      int diagBytesRead = 0;
      unsigned long diagTimeout = millis() + 500;

      while (millis() < diagTimeout && diagBytesRead < 10) {
        if (kLineSerial.available()) {
          diagResponse[diagBytesRead++] = kLineSerial.read();
        }
      }

      Serial.print("Start Diagnostics response - received ");
      Serial.print(diagBytesRead);
      Serial.print(" bytes: ");
      for (int i = 0; i < diagBytesRead; i++) {
        Serial.print("0x");
        if (diagResponse[i] < 0x10) Serial.print("0");
        Serial.print(diagResponse[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      if (diagBytesRead > 0) {
        Serial.println("Start diagnostics acknowledged");
        return true;
      } else {
        Serial.println("No response to Start Diagnostics command");
        return false;
      }
    } else {
      Serial.println("Response doesn't match expected pattern");
    }
  } else {
    Serial.println("No bytes received from ECU - check wiring/power");
  }

  Serial.println("Fast init failed - no valid response");
  return false;
}

bool authenticateECU() {
  // Security access level 1 (standard diagnostic level)
  uint8_t seedRequest[] = {0x02, 0x27, 0x01, 0x00};
  seedRequest[3] = calculateChecksum(seedRequest, 3);
  sendMessage(seedRequest, 4);
  delay(200);

  uint8_t response[10];
  int bytesRead = readResponse(response, 10);

  // Find 0x67 (positive seed response) in the buffer, accounting for echo
  int responseStart = -1;
  for (int i = 1; i < bytesRead; i++) {
    if (response[i] == 0x67 && i >= 1 && response[i-1] <= 0x10) {
      responseStart = i - 1;
      break;
    }
  }

  if (responseStart >= 0 && (bytesRead - responseStart) >= 6) {
    // Extract seed in BIG-ENDIAN format: [Seed_HIGH] [Seed_LOW]
    uint16_t seed = (response[responseStart + 3] << 8) | response[responseStart + 4];
    uint16_t key = calculateTd5Key(seed);

    // Send key in BIG-ENDIAN format: [Key_HIGH] [Key_LOW]
    uint8_t keyResponse[] = {0x04, 0x27, 0x02, (uint8_t)(key >> 8), (uint8_t)(key & 0xFF), 0x00};
    keyResponse[5] = calculateChecksum(keyResponse, 5);
    sendMessage(keyResponse, 6);

    delay(200);
    bytesRead = readResponse(response, 10);

    // Search for 0x67 (success) or 0x7F (failure) in response
    bool foundSuccess = false;
    bool foundError = false;
    int errorCode = 0;

    for (int i = 0; i < bytesRead; i++) {
      if (response[i] == 0x67 && i > 0 && response[i-1] <= 0x10) {
        foundSuccess = true;
        break;
      }
      if (response[i] == 0x7F && i + 2 < bytesRead) {
        foundError = true;
        errorCode = response[i + 2];
        break;
      }
    }

    if (foundSuccess) {
      Serial.println("Authentication successful!");
      return true;
    } else if (foundError) {
      Serial.print("Authentication rejected by ECU - Error code: 0x");
      Serial.println(errorCode, HEX);
    }
  } else {
    // Check if seed request was rejected
    bool foundError = false;
    int errorCode = 0;

    for (int i = 0; i < bytesRead; i++) {
      if (response[i] == 0x7F && i + 2 < bytesRead) {
        foundError = true;
        errorCode = response[i + 2];
        break;
      }
    }

    if (foundError) {
      Serial.print("Seed request rejected - Error code: 0x");
      Serial.println(errorCode, HEX);
    } else {
      Serial.println("Unexpected response format to seed request");
    }
  }

  Serial.println("Authentication failed");
  return false;
}

// Helper function to get a specific bit
inline bool getBit(uint16_t value, uint8_t bit) {
  return (value >> bit) & 1;
}

uint16_t calculateTd5Key(uint16_t seed) {
  // pajacobson/td5keygen algorithm - CONFIRMED WORKING on MSB and NNN ECUs
  uint16_t tmp = 0;
  uint8_t count = 0;
  uint8_t idx;
  uint8_t tap = 0;

  // Calculate iteration count from seed bits
  count = ((seed >> 0xC & 0x8) | (seed >> 0x5 & 0x4) | (seed >> 0x3 & 0x2) | (seed & 0x1)) + 1;

  for (idx = 0; idx < count; idx++) {
    // Calculate tap from bits 1, 2, 8, 9
    tap = ((seed >> 1) ^ (seed >> 2) ^ (seed >> 8) ^ (seed >> 9)) & 1;

    // Right shift and place tap at bit 15
    tmp = ((seed >> 1) | (tap << 0xF));

    // Set/clear bit 0 based on bits 3 and 13
    if ((seed >> 0x3 & 1) && (seed >> 0xD & 1)) {
      seed = tmp & ~1;  // Clear bit 0
    } else {
      seed = tmp | 1;   // Set bit 0
    }
  }

  return seed;
}

// getPIDsForMessageType() is now defined in td5_parameters.h

void requestLiveData() {
  // Dynamic request scheduler based on display priorities
  // ONLY sends ONE request per call to avoid overwhelming the ECU
  static uint8_t requestIndex = 0;
  static unsigned long lastRequestDebug = 0;
  uint8_t request[] = {0x02, 0x21, 0x00, 0x00};  // Template: [Length][Service][PID][Checksum]

  // Check if display priority is active (received update in last 5 seconds)
  bool displayPriorityActive = (millis() - lastDisplayPriorityUpdate < DISPLAY_PRIORITY_TIMEOUT);

  uint8_t pidSequence[20];  // Max possible PIDs in sequence
  uint8_t sequenceLength = 0;

  if (displayPriorityActive) {
    // BUILD DYNAMIC SEQUENCE BASED ON DISPLAY PRIORITIES
    // Current screen PIDs (highest priority - appear 4× in sequence)
    uint8_t currentPids[5], prevPids[5], nextPids[5], otherPids[10];
    uint8_t currentCount = 0, prevCount = 0, nextCount = 0, otherCount = 0;

    getPIDsForMessageType(currentDisplayMsgType, currentPids, currentCount);
    getPIDsForMessageType(prevDisplayMsgType, prevPids, prevCount);
    getPIDsForMessageType(nextDisplayMsgType, nextPids, nextCount);

    // Add current screen PIDs 4× (highest priority)
    for (uint8_t i = 0; i < 4 && sequenceLength < 20; i++) {
      for (uint8_t j = 0; j < currentCount && sequenceLength < 20; j++) {
        pidSequence[sequenceLength++] = currentPids[j];
      }
    }

    // Add prev/next screen PIDs 2× each (medium priority)
    for (uint8_t i = 0; i < 2 && sequenceLength < 20; i++) {
      for (uint8_t j = 0; j < prevCount && sequenceLength < 20; j++) {
        pidSequence[sequenceLength++] = prevPids[j];
      }
      for (uint8_t j = 0; j < nextCount && sequenceLength < 20; j++) {
        pidSequence[sequenceLength++] = nextPids[j];
      }
    }

    // Add all other PIDs once (low priority)
    uint8_t allMsgTypes[] = {TD5_MSG_FUELLING, TD5_MSG_INPUTS, TD5_MSG_TEMPERATURES,
                              TD5_MSG_PRESSURES, TD5_MSG_ACTUATORS};
    for (uint8_t m = 0; m < 5 && sequenceLength < 20; m++) {
      if (allMsgTypes[m] != currentDisplayMsgType &&
          allMsgTypes[m] != prevDisplayMsgType &&
          allMsgTypes[m] != nextDisplayMsgType) {
        uint8_t pids[5];
        uint8_t count = 0;
        getPIDsForMessageType(allMsgTypes[m], pids, count);
        for (uint8_t j = 0; j < count && sequenceLength < 20; j++) {
          pidSequence[sequenceLength++] = pids[j];
        }
      }
    }
  } else {
    // DEFAULT SEQUENCE (no display priority updates received recently)
    // Optimized for general-purpose monitoring - ONLY Nanocom-validated PIDs
    uint8_t defaultSequence[] = {
      // Fast cycle PIDs (high priority)
      0x09,  // RPM (critical)
      0x0D,  // Speed (critical)
      0x1A,  // Coolant temp, MAP, Boost pressure (composite)
      0x21,  // Digital Inputs - brake/clutch (high)
      0x09,  // RPM (critical repeat)
      0x1C,  // Inlet Air + Fuel Temp (composite)
      0x0D,  // Speed (critical repeat)
      0x10,  // Battery Voltage (composite)
      0x09,  // RPM (critical repeat)
      0x1B,  // Airflow + Ambient Pressure (composite)
      0x21,  // Digital Inputs (high repeat)
      0x09,  // RPM (critical repeat)
      0x0D,  // Speed (critical repeat)
      0x38,  // Wastegate Position
      0x37,  // EGR Position
      0x1E,  // Cruise Control & Brake Switches
      0x23,  // Accelerator position tracks (little-endian, composite)
      0x40,  // Cylinder fuel trim (composite)
    };
    sequenceLength = sizeof(defaultSequence);
    memcpy(pidSequence, defaultSequence, sequenceLength);
  }

  // Ensure we have at least one PID
  if (sequenceLength == 0) {
    pidSequence[0] = 0x09;  // Default to RPM
    sequenceLength = 1;
  }

  // Find next PID to request, respecting reliability-based polling intervals
  uint8_t attempts = 0;
  uint8_t selectedPID;
  bool pidAcceptable = false;

  do {
    selectedPID = pidSequence[requestIndex % sequenceLength];
    requestIndex = (requestIndex + 1) % sequenceLength;
    attempts++;

    // Check if this PID is acceptable to request right now
    if (pidReliabilityScore[selectedPID] >= PID_UNRELIABLE_THRESHOLD) {
      // Reliable PID - always request
      pidAcceptable = true;
    } else {
      // Unreliable PID - only request if 10 seconds have passed since last attempt
      if (millis() - pidLastAttempt[selectedPID] >= PID_UNRELIABLE_RETRY_INTERVAL) {
        pidAcceptable = true;
      }
    }

    // If we've checked all PIDs and none are acceptable, just take the current one
    if (attempts >= sequenceLength) {
      pidAcceptable = true;
    }

  } while (!pidAcceptable);

  // Select PID from sequence
  request[2] = selectedPID;

  // Track when we attempted this PID
  pidLastAttempt[selectedPID] = millis();

  // Calculate checksum and send
  request[3] = calculateChecksum(request, 3);

  // Debug: Print request every 2 seconds (avoid spamming)
  if (millis() - lastRequestDebug > 2000) {
    Serial.print("[ECU REQUEST] PID 0x");
    if (selectedPID < 0x10) Serial.print("0");
    Serial.print(selectedPID, HEX);
    Serial.print(" (seq: ");
    Serial.print(requestIndex);
    Serial.print("/");
    Serial.print(sequenceLength);
    Serial.println(")");
    lastRequestDebug = millis();
  }

  sendMessage(request, 4);
}

void processIncomingData() {
  // Read all available bytes from serial into buffer
  int bytesReadThisCall = 0;
  while (kLineSerial.available() && rxIndex < sizeof(rxBuffer) - 1) {
    rxBuffer[rxIndex++] = kLineSerial.read();
    bytesReadThisCall++;
  }

  // Debug: Log incoming data
  static unsigned long lastIncomingDebug = 0;
  if (bytesReadThisCall > 0 && millis() - lastIncomingDebug > 2000) {
    Serial.print("[ECU RESPONSE] Received ");
    Serial.print(bytesReadThisCall);
    Serial.print(" bytes, buffer: ");
    Serial.println(rxIndex);
    lastIncomingDebug = millis();
  }

  // CRITICAL FIX: Reset buffer if it grows too large (indicates processing failure)
  if (rxIndex > 50) {
    Serial.println("[BUFFER OVERFLOW] Clearing stuck data");
    rxIndex = 0;
    return;
  }

  // Process ALL complete messages in buffer (may be multiple pipelined responses)
  int messagesProcessed = 0;
  while (rxIndex >= 3 && messagesProcessed < 10) {  // Limit iterations to prevent infinite loop
    uint8_t messageLength = rxBuffer[0];

    // Validate message length is reasonable (ECU messages are typically 3-10 bytes)
    if (messageLength < 2 || messageLength > 20) {
      // Invalid length - search for valid message start (service byte)
      static unsigned long lastSyncDebug = 0;
      bool foundValidStart = false;

      // Debug: Show buffer contents when sync fails
      if (millis() - lastSyncDebug > 2000) {
        Serial.print("[SYNC] Invalid length 0x");
        Serial.print(messageLength, HEX);
        Serial.print(" buffer[");
        Serial.print(rxIndex);
        Serial.print("]: ");
        int displayCount = (rxIndex < 10) ? rxIndex : 10;
        for (int j = 0; j < displayCount; j++) {
          if (rxBuffer[j] < 0x10) Serial.print("0");
          Serial.print(rxBuffer[j], HEX);
          Serial.print(" ");
        }
        Serial.println();
        lastSyncDebug = millis();
      }

      for (int i = 1; i < rxIndex; i++) {
        // Look for valid ECU service bytes
        if (rxBuffer[i] == 0x61 || rxBuffer[i] == 0x7F ||
            rxBuffer[i] == 0x67 || rxBuffer[i] == 0xC1 ||
            rxBuffer[i] == 0x50) {  // 0x50 = TesterPresent response
          // Found potential message start - this should be preceded by length byte
          if (i > 0) {
            // Shift buffer to align with potential message
            Serial.print("[SYNC] Found 0x");
            Serial.print(rxBuffer[i], HEX);
            Serial.print(" at position ");
            Serial.print(i);
            Serial.println(" - realigning");
            memmove(rxBuffer, rxBuffer + i - 1, rxIndex - (i - 1));
            rxIndex -= (i - 1);
            foundValidStart = true;
            break;
          }
        }
      }

      if (!foundValidStart) {
        // No valid message found - clear buffer
        Serial.println("[SYNC] No valid service byte found - clearing buffer");
        rxIndex = 0;
        break;
      }
      continue;  // Try again with aligned buffer
    }

    // Check if we have a complete message
    if (rxIndex >= messageLength + 1) {
      // Verify checksum before processing
      uint8_t expectedChecksum = calculateChecksum(rxBuffer, messageLength);
      uint8_t receivedChecksum = rxBuffer[messageLength];

      if (expectedChecksum == receivedChecksum) {
        // Valid message - process it
        processECUResponse(rxBuffer, messageLength + 1);
        messagesProcessed++;

        // Shift remaining data to beginning of buffer
        int remainingBytes = rxIndex - (messageLength + 1);
        if (remainingBytes > 0) {
          memmove(rxBuffer, rxBuffer + messageLength + 1, remainingBytes);
        }
        rxIndex = remainingBytes;
      } else {
        // Checksum mismatch - discard first byte and resync
        memmove(rxBuffer, rxBuffer + 1, rxIndex - 1);
        rxIndex--;
      }
    } else {
      // Incomplete message, wait for more data
      break;
    }
  }
}

void processECUResponse(uint8_t* data, int length) {
  if (length < 3) return;

  uint8_t service = data[1];
  uint8_t subfunction = data[2];

  switch (service) {
    case 0x61:  // Positive response to Service 0x21
      // Increase reliability score for this PID - it's working!
      if (pidReliabilityScore[subfunction] < 100) {
        pidReliabilityScore[subfunction] += PID_SUCCESS_INCREMENT;
      }

      // Handle COMPOSITE PIDs first (multi-byte responses)
      switch (subfunction) {
        case 0x1A:  // Composite Multi-Parameter (18 bytes expected)
          if (length >= 5) {
            // Bytes 3-4: Coolant temp (Kelvin*10)
            uint16_t coolant = (data[3] << 8) | data[4];
            coolantTemp = ((int16_t)coolant - 2732) / 10;
            validTemperaturesData = true;
            lastECUDataReceived = millis();
          }
          if (length >= 7) {
            // Bytes 5-6: MAP (Manifold Absolute Pressure) in Pa/100 = kPa
            uint16_t mapRaw = (data[5] << 8) | data[6];
            manifoldPressure = mapRaw / 100;

            // Calculate boost (MAP - ambient ~100kPa) in kPa
            boostPressure = manifoldPressure - 100;
            validPressuresData = true;
          }
          return;  // Composite handled, exit

        case 0x1B:  // Composite Fuel/Air (12 bytes expected)
          if (length >= 7) {
            // Bytes 5-6: Airflow (g/s)
            uint16_t airflowRaw = (data[5] << 8) | data[6];
            manifoldAirFlow = airflowRaw / 100;  // Convert g/s to kg/h*10 for storage
            validFuellingData = true;
            lastECUDataReceived = millis();
          }
          if (length >= 9) {
            // Bytes 7-8: Ambient Pressure (raw / 46.94 = kPa)
            uint16_t ambientRaw = (data[7] << 8) | data[8];
            ambientPressure = ambientRaw / 47;  // Approximate: /46.94 ≈ /47
            validPressuresData = true;
          }
          return;  // Composite handled, exit

        case 0x1C:  // Composite Temperatures (11 bytes expected)
          if (length >= 5) {
            // Bytes 3-4: Inlet Air Temperature (Kelvin*10)
            uint16_t inletRaw = (data[3] << 8) | data[4];
            inletAirTemp = ((int16_t)inletRaw - 2732) / 10;
            validTemperaturesData = true;
            lastECUDataReceived = millis();
          }
          if (length >= 7) {
            // Bytes 5-6: Fuel Temperature (Kelvin*10)
            uint16_t fuelRaw = (data[5] << 8) | data[6];
            fuelTemp = ((int16_t)fuelRaw - 2732) / 10;
          }
          return;  // Composite handled, exit

        case 0x10:  // Battery Voltage (6 bytes expected)
          if (length >= 5) {
            // Bytes 3-4: Battery voltage in millivolts
            batteryVoltage = (data[3] << 8) | data[4];
            validTemperaturesData = true;
            lastECUDataReceived = millis();
          }
          if (length >= 7) {
            // Bytes 5-6: Reference voltage in millivolts
            referenceVoltage = (data[5] << 8) | data[6];
            validPressuresData = true;
          }
          return;  // Composite handled, exit

        case 0x23:  // Accelerator Tracks (LITTLE-ENDIAN, 10 bytes expected)
          if (length >= 5) {
            // Bytes 3-4: Track 1 (LITTLE-ENDIAN!)
            uint16_t track1 = (data[4] << 8) | data[3];
            // Track value stored but not used in ESP-NOW (could add to fuelling data)
            lastECUDataReceived = millis();
          }
          if (length >= 7) {
            // Bytes 5-6: Track 2 (LITTLE-ENDIAN!)
            uint16_t track2 = (data[6] << 8) | data[5];
            // Track value stored but not used in ESP-NOW
          }
          return;  // Composite handled, exit

        case 0x40:  // Cylinder Fuel Trim (13 bytes expected)
          if (length >= 13) {
            // 5 cylinders × 2 bytes each (big-endian)
            // Values are trim adjustments, not directly used in ESP-NOW
            // Could add to diagnostics/status message if needed
            lastECUDataReceived = millis();
          }
          return;  // Composite handled, exit

        case 0x37:  // EGR Position
          if (length >= 5) {
            uint16_t rawValue = (data[3] << 8) | data[4];
            egrPosition = rawValue;  // Already scaled by 100
            validActuatorsData = true;
            lastECUDataReceived = millis();
          }
          return;  // Composite handled, exit

        case 0x38:  // Wastegate Position
          if (length >= 5) {
            uint16_t rawValue = (data[3] << 8) | data[4];
            wastegatePosition = rawValue;  // Already scaled by 100
            validActuatorsData = true;
            lastECUDataReceived = millis();
          }
          return;  // Composite handled, exit

        case 0x1E:  // Cruise Control & Brake Switches (composite)
          if (length >= 5) {
            uint8_t inputByte1 = data[3];
            uint8_t inputByte2 = data[4];

            // Decode switches (similar to 0x21)
            brakePedalPressed = (inputByte1 & 0x01) == 0;    // INVERTED
            cruiseBrakePressed = (inputByte1 & 0x02) == 0;   // INVERTED
            cruiseControlOn = (inputByte1 & 0x20) != 0;
            validInputsData = true;
            lastECUDataReceived = millis();
          }
          return;  // Composite handled, exit

        case 0x36:  // Unknown status (returns constant 0x0005)
          if (length >= 5) {
            // Just acknowledge we received it
            lastECUDataReceived = millis();
          }
          return;  // Composite handled, exit
      }

      // Handle SIMPLE 2-byte PIDs (only Nanocom-validated PIDs)
      if (length >= 5) {  // Minimum: [Length][0x61][PID][Data_High][Data_Low][Checksum]
        uint16_t rawValue = (data[3] << 8) | data[4];

        switch (subfunction) {
          case 0x09:  // RPM
            engineRPM = rawValue;
            validFuellingData = true;
            lastECUDataReceived = millis();
            break;

          case 0x0D:  // Vehicle Speed
            vehicleSpeed = data[3];  // Only 1 byte for speed
            validFuellingData = true;
            lastECUDataReceived = millis();
            break;

          case 0x21:  // Digital Inputs (switches)
            if (length >= 5) {
              uint8_t inputByte1 = data[3];
              uint8_t inputByte2 = data[4];

              brakePedalPressed = (inputByte1 & 0x01) == 0;    // INVERTED
              cruiseBrakePressed = (inputByte1 & 0x02) == 0;   // INVERTED
              clutchPedalPressed = (inputByte1 & 0x04) == 0;   // INVERTED
              handbrakeEngaged = (inputByte1 & 0x08) != 0;
              airConRequest = (inputByte1 & 0x10) != 0;
              cruiseControlOn = (inputByte1 & 0x20) != 0;
              neutralSelected = (inputByte1 & 0x40) != 0;
              gearPosition = inputByte2 & 0x0F;

              validInputsData = true;
              lastECUDataReceived = millis();
            }
            break;
        }
      }
      break;

    case 0x7F:
      {
        // ECU is communicating, just rejecting this specific request
        lastECUDataReceived = millis();

        // Decrease reliability score for this PID
        uint8_t rejectedPID = subfunction;
        if (pidReliabilityScore[rejectedPID] > -100) {
          pidReliabilityScore[rejectedPID] += PID_FAILURE_DECREMENT;
        }

        // Only print occasional failures (not every one)
        static uint8_t lastRejectedPID = 0xFF;
        static unsigned long lastRejectionPrint = 0;

        if (rejectedPID != lastRejectedPID || millis() - lastRejectionPrint > 5000) {
          Serial.print("PID 0x");
          Serial.print(rejectedPID, HEX);
          Serial.print(" rejected (score: ");
          Serial.print(pidReliabilityScore[rejectedPID]);
          Serial.print(") - Error: 0x");
          Serial.print(data[3], HEX);

          if (pidReliabilityScore[rejectedPID] <= PID_UNRELIABLE_THRESHOLD) {
            Serial.print(" [UNRELIABLE - retry every 10s]");
          }
          Serial.println();

          lastRejectedPID = rejectedPID;
          lastRejectionPrint = millis();
        }
      }
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