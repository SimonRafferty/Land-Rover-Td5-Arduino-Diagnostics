# Td5 ESP-NOW Data Protocol

This document describes the ESP-NOW wireless data protocol used to broadcast Land Rover Td5 ECU data for real-time monitoring, data logging, and diagnostic applications.

## Overview

The Td5 ESP-NOW protocol broadcasts real-time vehicle data using ESP32's ESP-NOW wireless technology. No network configuration or device pairing is required - receivers simply listen for broadcast packets on a specified Wi-Fi channel.

### Key Features
- **Broadcast Mode**: No MAC address pairing required
- **Multiple Recipients**: Unlimited number of listening devices
- **Real-time Updates**: 24Hz total update rate across all data types
- **Structured Format**: Six distinct message types for organized data
- **Error Detection**: XOR checksum validation
- **Range**: 100-200m line of sight, 20-50m through obstacles

## Packet Structure

All packets follow a standardized format with an 8-byte header, variable-length payload, and 1-byte checksum:

```
[Header: 8 bytes] [Payload: variable] [Checksum: 1 byte]
```

### Packet Header (8 bytes)

```c
struct __attribute__((packed)) Td5PacketHeader {
    uint32_t timestamp;      // Milliseconds since transmitter boot
    uint16_t sequence;       // Incrementing sequence number (0-65535)
    uint8_t messageType;     // Message type identifier (0x01-0x06)
    uint8_t dataLength;      // Payload size in bytes
};
```

### Message Types

| Type | ID   | Name         | Payload Size | Update Rate | Description |
|------|------|--------------|--------------|-------------|-------------|
| 1    | 0x01 | FUELLING     | 10 bytes     | 4Hz         | Speed, RPM, injection, airflow, throttle |
| 2    | 0x02 | INPUTS       | 4 bytes      | 4Hz         | Brake, clutch, handbrake, gear position |
| 3    | 0x03 | TEMPERATURES | 10 bytes     | 4Hz         | Engine temps, battery voltage |
| 4    | 0x04 | PRESSURES    | 8 bytes      | 4Hz         | Manifold, boost, reference voltage |
| 5    | 0x05 | ACTUATORS    | 8 bytes      | 4Hz         | EGR, wastegate positions |
| 6    | 0x06 | STATUS       | 8 bytes      | 4Hz         | Connection state, error codes |

## Message Payload Definitions

### 1. FUELLING Data (0x01)

Engine performance and fuelling parameters.

```c
struct __attribute__((packed)) Td5FuellingData {
    uint16_t vehicleSpeed;      // Vehicle speed in km/h
    uint16_t engineRPM;         // Engine RPM
    uint16_t injectionQuantity; // Injection quantity: mg/stroke * 100
    uint16_t manifoldAirFlow;   // Manifold airflow: kg/h * 10
    uint16_t driverDemand;      // Throttle position: % * 100
};
```

**Example Values:**
- `vehicleSpeed = 65` → 65 km/h
- `engineRPM = 2500` → 2500 RPM
- `injectionQuantity = 1250` → 12.50 mg/stroke
- `manifoldAirFlow = 425` → 42.5 kg/h
- `driverDemand = 3500` → 35.00%

### 2. INPUTS Data (0x02)

Driver input switches and gear position.

```c
struct __attribute__((packed)) Td5InputsData {
    uint8_t switchStates;       // Bit field of switch states
    uint8_t gearPosition;       // Gear position (0-7)
    uint8_t reserved[2];        // Reserved for future use
};
```

**Switch States Bit Field:**
```
Bit 7: Reserved
Bit 6: Neutral switch (1=neutral selected)
Bit 5: Cruise control master (1=on)
Bit 4: A/C compressor request (1=on)
Bit 3: Handbrake (1=engaged)
Bit 2: Clutch pedal (1=pressed)
Bit 1: Cruise brake switch (1=pressed)
Bit 0: Brake pedal (1=pressed)
```

**Gear Position Values:**
- `0` = Park
- `1` = Reverse
- `2` = Neutral
- `3` = Drive
- `4` = 3rd gear
- `5` = 2nd gear
- `6` = 1st gear
- `7` = Unknown/Invalid

**Example:** `switchStates = 0x09` means brake pedal pressed (bit 0) and handbrake engaged (bit 3).

### 3. TEMPERATURES Data (0x03)

Engine and ambient temperature sensors plus battery voltage.

```c
struct __attribute__((packed)) Td5TemperaturesData {
    int16_t coolantTemp;        // Coolant temperature in °C
    int16_t fuelTemp;           // Fuel temperature in °C
    int16_t inletAirTemp;       // Inlet air temperature in °C
    int16_t ambientAirTemp;     // Ambient air temperature in °C
    uint16_t batteryVoltage;    // Battery voltage in mV
};
```

**Example Values:**
- `coolantTemp = 89` → 89°C
- `fuelTemp = 45` → 45°C
- `inletAirTemp = 28` → 28°C
- `ambientAirTemp = 22` → 22°C
- `batteryVoltage = 14200` → 14.2V

### 4. PRESSURES Data (0x04)

Pressure sensors and calculated boost pressure.

```c
struct __attribute__((packed)) Td5PressuresData {
    uint16_t manifoldPressure;  // Manifold absolute pressure in kPa
    uint16_t ambientPressure;   // Ambient absolute pressure in kPa
    uint16_t boostPressure;     // Calculated boost pressure in kPa (MAP - AAP)
    uint16_t referenceVoltage;  // ECU 5V reference in mV
};
```

**Example Values:**
- `manifoldPressure = 250` → 250 kPa
- `ambientPressure = 101` → 101 kPa (sea level)
- `boostPressure = 149` → 149 kPa (21.6 PSI boost)
- `referenceVoltage = 5020` → 5.02V

### 5. ACTUATORS Data (0x05)

Actuator positions for emissions and turbo control.

```c
struct __attribute__((packed)) Td5ActuatorsData {
    uint16_t egrPosition;       // EGR throttle position: % * 100
    uint16_t wastegatePosition; // Wastegate position: % * 100
    uint8_t reserved[4];        // Reserved for future actuators
};
```

**Example Values:**
- `egrPosition = 1520` → 15.20% open
- `wastegatePosition = 4580` → 45.80% open

### 6. STATUS Data (0x06)

System status and diagnostic information.

```c
struct __attribute__((packed)) Td5StatusData {
    uint8_t connectionState;    // ECU connection state (0-4)
    uint8_t lastErrorCode;      // Last ECU error code received
    uint16_t connectionUptime;  // Seconds since successful connection
    uint32_t totalPacketsSent;  // Total ESP-NOW packets transmitted
};
```

**Connection States:**
- `0` = TD5_DISCONNECTED
- `1` = TD5_INITIALIZING
- `2` = TD5_AUTHENTICATING
- `3` = TD5_CONNECTED
- `4` = TD5_ERROR

## Checksum Validation

Each packet includes an XOR checksum calculated across the header and payload:

```c
uint8_t calculateChecksum(uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
```

## Receiver Implementation

### Basic Setup

```c
#include <esp_now.h>
#include <WiFi.h>

// Define packet structures (copy from above)
struct __attribute__((packed)) Td5PacketHeader { /* ... */ };
struct __attribute__((packed)) Td5FuellingData { /* ... */ };
// ... other structures

void setup() {
    Serial.begin(115200);

    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return;
    }

    // Register callback for received data
    esp_now_register_recv_cb(onDataReceived);

    Serial.println("ESP-NOW receiver initialized");
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    // Main application logic here
    delay(100);
}
```

### Data Reception Handler

```c
void onDataReceived(const uint8_t *senderMac, const uint8_t *data, int len) {
    // Minimum packet size check
    if (len < sizeof(Td5PacketHeader) + 1) {
        Serial.println("Packet too short");
        return;
    }

    // Parse header
    Td5PacketHeader* header = (Td5PacketHeader*)data;
    uint8_t* payload = (uint8_t*)data + sizeof(Td5PacketHeader);
    uint8_t receivedChecksum = data[len - 1];

    // Validate checksum
    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < len - 1; i++) {
        calculatedChecksum ^= data[i];
    }

    if (calculatedChecksum != receivedChecksum) {
        Serial.println("Checksum validation failed");
        return;
    }

    // Validate payload length
    if (header->dataLength != len - sizeof(Td5PacketHeader) - 1) {
        Serial.println("Invalid payload length");
        return;
    }

    // Process by message type
    processMessage(header, payload);
}
```

### Message Processing

```c
void processMessage(Td5PacketHeader* header, uint8_t* payload) {
    switch (header->messageType) {
        case 0x01: // FUELLING
            processFuellingData((Td5FuellingData*)payload);
            break;

        case 0x02: // INPUTS
            processInputsData((Td5InputsData*)payload);
            break;

        case 0x03: // TEMPERATURES
            processTemperaturesData((Td5TemperaturesData*)payload);
            break;

        case 0x04: // PRESSURES
            processPressuresData((Td5PressuresData*)payload);
            break;

        case 0x05: // ACTUATORS
            processActuatorsData((Td5ActuatorsData*)payload);
            break;

        case 0x06: // STATUS
            processStatusData((Td5StatusData*)payload);
            break;

        default:
            Serial.print("Unknown message type: 0x");
            Serial.println(header->messageType, HEX);
    }
}

void processFuellingData(Td5FuellingData* data) {
    Serial.print("Speed: ");
    Serial.print(data->vehicleSpeed);
    Serial.print(" km/h, RPM: ");
    Serial.print(data->engineRPM);
    Serial.print(", Throttle: ");
    Serial.print(data->driverDemand / 100.0, 1);
    Serial.println("%");
}

void processInputsData(Td5InputsData* data) {
    bool brakePressed = (data->switchStates & 0x01) != 0;
    bool clutchPressed = (data->switchStates & 0x04) != 0;
    bool handbrakeOn = (data->switchStates & 0x08) != 0;

    Serial.print("Brake: ");
    Serial.print(brakePressed ? "PRESSED" : "Released");
    Serial.print(", Clutch: ");
    Serial.print(clutchPressed ? "PRESSED" : "Released");
    Serial.print(", Handbrake: ");
    Serial.println(handbrakeOn ? "ON" : "Off");
}
```

## Application Examples

### Real-Time Data Monitor

```c
void processFuellingData(Td5FuellingData* data) {
    Serial.print("=== FUELLING DATA ===\n");
    Serial.print("Speed: ");
    Serial.print(data->vehicleSpeed);
    Serial.print(" km/h (");
    Serial.print(data->vehicleSpeed * 0.621371, 1);
    Serial.print(" mph)\n");

    Serial.print("RPM: ");
    Serial.println(data->engineRPM);

    Serial.print("Injection: ");
    Serial.print(data->injectionQuantity / 100.0, 2);
    Serial.println(" mg/stroke");

    Serial.print("Airflow: ");
    Serial.print(data->manifoldAirFlow / 10.0, 1);
    Serial.println(" kg/h");

    Serial.print("Throttle: ");
    Serial.print(data->driverDemand / 100.0, 1);
    Serial.println("%\n");
}

void processInputsData(Td5InputsData* data) {
    Serial.print("=== INPUT SWITCHES ===\n");
    Serial.print("Brake Pedal: ");
    Serial.println((data->switchStates & 0x01) ? "PRESSED" : "Released");

    Serial.print("Cruise Brake: ");
    Serial.println((data->switchStates & 0x02) ? "PRESSED" : "Released");

    Serial.print("Clutch Pedal: ");
    Serial.println((data->switchStates & 0x04) ? "PRESSED" : "Released");

    Serial.print("Handbrake: ");
    Serial.println((data->switchStates & 0x08) ? "ENGAGED" : "Released");

    Serial.print("A/C Request: ");
    Serial.println((data->switchStates & 0x10) ? "ON" : "OFF");

    Serial.print("Cruise Control: ");
    Serial.println((data->switchStates & 0x20) ? "ON" : "OFF");

    Serial.print("Neutral Switch: ");
    Serial.println((data->switchStates & 0x40) ? "SELECTED" : "In Gear");

    Serial.print("Gear Position: ");
    const char* gears[] = {"Park", "Reverse", "Neutral", "Drive", "3rd", "2nd", "1st", "Unknown"};
    Serial.println(gears[data->gearPosition > 7 ? 7 : data->gearPosition]);
    Serial.println();
}

void processTemperaturesData(Td5TemperaturesData* data) {
    Serial.print("=== TEMPERATURES ===\n");
    Serial.print("Coolant: ");
    Serial.print(data->coolantTemp);
    Serial.println("°C");

    Serial.print("Fuel: ");
    Serial.print(data->fuelTemp);
    Serial.println("°C");

    Serial.print("Inlet Air: ");
    Serial.print(data->inletAirTemp);
    Serial.println("°C");

    Serial.print("Ambient: ");
    Serial.print(data->ambientAirTemp);
    Serial.println("°C");

    Serial.print("Battery: ");
    Serial.print(data->batteryVoltage / 1000.0, 2);
    Serial.println("V\n");
}

void processPressuresData(Td5PressuresData* data) {
    Serial.print("=== PRESSURES ===\n");
    Serial.print("Manifold: ");
    Serial.print(data->manifoldPressure);
    Serial.println(" kPa");

    Serial.print("Ambient: ");
    Serial.print(data->ambientPressure);
    Serial.println(" kPa");

    Serial.print("Boost: ");
    Serial.print(data->boostPressure);
    Serial.print(" kPa (");
    Serial.print(data->boostPressure * 0.145038, 1);
    Serial.println(" PSI)");

    Serial.print("Reference: ");
    Serial.print(data->referenceVoltage / 1000.0, 2);
    Serial.println("V\n");
}

void processActuatorsData(Td5ActuatorsData* data) {
    Serial.print("=== ACTUATORS ===\n");
    Serial.print("EGR Position: ");
    Serial.print(data->egrPosition / 100.0, 1);
    Serial.println("%");

    Serial.print("Wastegate: ");
    Serial.print(data->wastegatePosition / 100.0, 1);
    Serial.println("%\n");
}

void processStatusData(Td5StatusData* data) {
    Serial.print("=== SYSTEM STATUS ===\n");
    const char* states[] = {"DISCONNECTED", "INITIALIZING", "AUTHENTICATING", "CONNECTED", "ERROR"};
    Serial.print("ECU State: ");
    Serial.println(states[data->connectionState > 4 ? 4 : data->connectionState]);

    Serial.print("Uptime: ");
    Serial.print(data->connectionUptime);
    Serial.println(" seconds");

    Serial.print("Packets Sent: ");
    Serial.println(data->totalPacketsSent);
    Serial.println();
}
```

### CSV Data Logger

```c
void logAllData(Td5PacketHeader* header, uint8_t* payload) {
    // Create CSV log entry with timestamp
    String logEntry = String(millis()) + ",";
    logEntry += String(header->messageType) + ",";
    logEntry += String(header->sequence) + ",";

    switch (header->messageType) {
        case 0x01: {
            Td5FuellingData* data = (Td5FuellingData*)payload;
            logEntry += String(data->vehicleSpeed) + ",";
            logEntry += String(data->engineRPM) + ",";
            logEntry += String(data->injectionQuantity / 100.0, 2) + ",";
            logEntry += String(data->manifoldAirFlow / 10.0, 1) + ",";
            logEntry += String(data->driverDemand / 100.0, 1);
            break;
        }
        case 0x02: {
            Td5InputsData* data = (Td5InputsData*)payload;
            logEntry += String(data->switchStates, BIN) + ",";
            logEntry += String(data->gearPosition);
            break;
        }
        case 0x03: {
            Td5TemperaturesData* data = (Td5TemperaturesData*)payload;
            logEntry += String(data->coolantTemp) + ",";
            logEntry += String(data->fuelTemp) + ",";
            logEntry += String(data->inletAirTemp) + ",";
            logEntry += String(data->ambientAirTemp) + ",";
            logEntry += String(data->batteryVoltage / 1000.0, 2);
            break;
        }
        // Handle other message types...
    }

    // Output CSV data to serial (can be redirected to SD card)
    Serial.println(logEntry);
}
```

### Simple Alert Monitor

```c
void processTemperaturesData(Td5TemperaturesData* data) {
    // Check for overheating
    if (data->coolantTemp > 105) {
        Serial.println("ALERT: Engine overheating! Coolant: " + String(data->coolantTemp) + "°C");
    }

    // Check battery voltage
    if (data->batteryVoltage < 11000) {  // Below 11V
        Serial.println("ALERT: Low battery voltage: " + String(data->batteryVoltage / 1000.0, 2) + "V");
    }
}

void processPressuresData(Td5PressuresData* data) {
    // Check for excessive boost
    if (data->boostPressure > 200) {  // Above 200 kPa
        Serial.println("ALERT: High boost pressure: " + String(data->boostPressure) + " kPa");
    }
}

void processInputsData(Td5InputsData* data) {
    // Monitor critical inputs
    static bool lastBrakeState = false;
    bool currentBrakeState = (data->switchStates & 0x01) != 0;

    if (currentBrakeState != lastBrakeState) {
        Serial.println("INPUT CHANGE: Brake pedal " + String(currentBrakeState ? "PRESSED" : "RELEASED"));
        lastBrakeState = currentBrakeState;
    }
}
```

## Troubleshooting

### Common Issues

**No Data Received:**
- Check ESP-NOW initialization
- Verify WiFi mode is set to WIFI_STA
- Ensure transmitter and receiver are on same channel
- Check power supply stability

**Checksum Errors:**
- Check for RF interference
- Verify packet length calculations
- Check for memory corruption

**Missing Packets:**
- Monitor sequence numbers for gaps
- Check receiver processing speed
- Verify sufficient power supply

**Range Issues:**
- Use ESP32 variants with better RF performance
- Add external antenna if possible
- Check for physical obstructions
- Monitor RSSI if available

### Debug Output

Enable debug output to monitor reception:

```c
void onDataReceived(const uint8_t *senderMac, const uint8_t *data, int len) {
    Serial.print("Received ");
    Serial.print(len);
    Serial.print(" bytes from ");

    // Print sender MAC
    for (int i = 0; i < 6; i++) {
        if (senderMac[i] < 0x10) Serial.print("0");
        Serial.print(senderMac[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    // Continue with normal processing...
}
```

## Integration Notes

- **Thread Safety**: ESP-NOW callbacks run in interrupt context - keep processing minimal
- **Memory Management**: Use static buffers or copy data immediately
- **Power Consumption**: ESP-NOW is designed for low power operation
- **Network Coexistence**: ESP-NOW can operate alongside WiFi but may affect performance
- **Channel Selection**: Use channels 1, 6, or 11 to avoid interference with WiFi

## Packet Timing

The transmitter cycles through all message types continuously:
- Each message type broadcasts every 1.25 seconds (4Hz)
- Total update rate across all types: 24Hz
- Sequence numbers increment independently for each message type
- No guaranteed delivery order between different message types