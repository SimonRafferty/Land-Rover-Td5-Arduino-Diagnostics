# Land Rover Td5 ECU Interface for ESP32

A complete Arduino implementation for communicating with pre-2006 Land Rover Td5 engine ECUs via K-Line interface. This project enables real-time extraction of vehicle data including speed, brake pedal status, engine parameters, and diagnostic information using the proprietary ISO 9141-2 protocol at 10400 baud.

## 📡 ESP-NOW Wireless Broadcasting

This project now includes **ESP-NOW wireless data broadcasting** capabilities for transmitting all ECU data to multiple recipients without requiring Wi-Fi network infrastructure or MAC address pairing.

### Available Implementations
- **`Td5_ECU_Interface.ino`**: Original K-Line interface with serial output
- **`Td5_ESPNow.ino`**: Enhanced version with ESP-NOW broadcasting capabilities

## 🚗 Overview

The Land Rover Td5 engine ECU uses a non-standard implementation of the ISO 9141-2 protocol that is incompatible with generic OBD2 scanners. This project provides a complete solution for ESP32-based communication, including:

- **Fast initialization sequence** with precise timing control
- **Seed-key authentication** using reverse-engineered algorithm
- **Real-time data extraction** for speed, RPM, brake status, and engine parameters
- **Robust error recovery** and keep-alive message handling
- **Complete hardware interface** specifications with L9637D transceiver
- **ESP-NOW wireless broadcasting** for multiple adaptive cruise control recipients

## 🔧 Hardware Requirements

### Core Components
- **ESP32 Development Board** (any variant with GPIO 22/23 available)
- **L9637D K-Line Transceiver IC** ([STMicroelectronics L9637D](https://www.st.com/en/automotive-analog-and-power/l9637.html))
- **1kΩ Resistor** (ESP32 TX to L9637D)
- **510Ω Pull-up Resistor** (K-Line to +12V per ISO 9141 spec)

### Protection Circuit
- **Reverse Polarity Diode** on +12V input
- **27V Zener Diode** on K-Line for transient suppression
- **330nF and 100nF Ceramic Capacitors** for power filtering
- **TVS Diodes** (recommended for enhanced ESD protection)

### Vehicle Connections
- **ECU Direct**: Pin B18 (pink wire) on black ECU connector C0658
- **OBD Port**: Pin 7 (K-Line), Pins 4-5 (Ground), Pin 16 (+12V Battery)

## 📋 Circuit Schematic

```
ESP32 GPIO 23 ──[1kΩ]── L9637D Pin 5 (TX)
ESP32 GPIO 22 ──────────── L9637D Pin 1 (RX)
+12V Vehicle ─────────────── L9637D Pin 7 (VS)
ESP32 3.3V ───────────────── L9637D Pin 3 (VCC)
Ground ──────────────────── L9637D Pin 4 (GND)

L9637D Pin 6 (K-Line) ──[510Ω]── +12V
                     └── Vehicle K-Line (ECU Pin B18 or OBD Pin 7)
                     └── [27V Zener] ── Ground
```

## 🛠 Software Setup

### Arduino IDE Configuration
1. **Install ESP32 Board Package**:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

2. **Install Required Libraries**:
   ```
   EspSoftwareSerial (by Dirk Kaar)
   ```

3. **Board Selection**:
   - Board: "ESP32 Dev Module" (or your specific variant)
   - Upload Speed: 921600
   - CPU Frequency: 240MHz
   - Core Debug Level: None

### Compilation and Upload
1. Open `Td5_ECU_Interface.ino` in Arduino IDE
2. Select your ESP32 board and COM port
3. Click Upload (Ctrl+U)
4. Open Serial Monitor at 115200 baud for debug output

## 📊 Live Data Output

The interface provides real-time monitoring of over 25 engine parameters and driver inputs:

```
=== Land Rover Td5 ECU Interface ===
Attempting ECU connection...
Fast init successful, authenticating...
ECU authenticated and connected!

FUELLING - Speed: 45 km/h (27.9 mph), RPM: 1850, IQ: 12.50 mg/stroke, MAF: 18.5 kg/h, Throttle: 25.3%
INPUTS - Brake: Released, Cruise Brake: Released, Clutch: Released, Handbrake: Off, A/C: Off, CC: Off, Neutral: In Gear
TEMPS - Coolant: 89°C, Fuel: 85°C, Inlet: 45°C, Ambient: 28°C, Battery: 14.2V
PRESSURE - MAP: 240 kPa, AAP: 100 kPa, Boost: 140 kPa (20.3 psi), Vref: 5.02V
ACTUATORS - EGR: 15.2%, Wastegate: 45.8%

FUELLING - Speed: 0 km/h (0.0 mph), RPM: 800, IQ: 8.25 mg/stroke, MAF: 12.1 kg/h, Throttle: 0.0%
INPUTS - Brake: PRESSED, Cruise Brake: PRESSED, Clutch: PRESSED, Handbrake: ON, A/C: ON, CC: ON, Neutral: Selected, Gear: Park
TEMPS - Coolant: 91°C, Fuel: 87°C, Inlet: 35°C, Ambient: 28°C, Battery: 14.1V
PRESSURE - MAP: 101 kPa, AAP: 100 kPa, Boost: 1 kPa (0.1 psi), Vref: 5.01V
ACTUATORS - EGR: 8.5%, Wastegate: 0.0%
```

### Complete Parameter List

**Engine Performance:**
- **Vehicle Speed**: km/h with automatic mph conversion  
- **Engine RPM**: Direct ECU value, real-time updates
- **Injection Quantity**: mg/stroke with 0.01mg precision
- **Manifold Air Flow (MAF)**: kg/h with 0.1kg/h precision
- **Driver Demand**: Accelerator pedal position (0-100%)

**Driver Input Monitoring:**
- **Brake Pedal Switch**: Primary brake pedal detection
- **Cruise Control Brake**: Secondary brake switch for cruise control
- **Clutch Pedal Switch**: Surge damping and cruise control cutoff
- **Handbrake Switch**: Parking brake engagement detection  
- **Air Conditioning**: A/C compressor request signal
- **Cruise Control Master**: Cruise control system on/off status
- **Neutral Switch**: Manual transmission neutral position detection
- **Gear Position**: Automatic transmission gear selection (P/R/N/D/3/2/1)

**Temperature Monitoring:**
- **Coolant Temperature**: °C, critical for overheating detection
- **Fuel Temperature**: °C, affects injection timing
- **Inlet Air Temperature**: °C, post-intercooler measurement  
- **Ambient Air Temperature**: °C, from airbox sensor

**Pressure Systems:**
- **Manifold Absolute Pressure (MAP)**: kPa, boost pressure indication
- **Ambient Absolute Pressure (AAP)**: kPa, altitude compensation
- **Calculated Boost Pressure**: kPa and PSI, MAP minus AAP
- **Turbo Wastegate Position**: 0-100%, boost control feedback

**Electrical Systems:**
- **Battery Voltage**: Volts with 0.01V precision
- **Reference Voltage**: 5V supply monitoring for sensor accuracy

**Emissions Control:**
- **EGR Inlet Throttle Position**: 0-100%, exhaust gas recirculation

**System Status:**
- **Communication Health**: Keep-alive status, error detection
- **Authentication Status**: Security handshake confirmation

## 🔒 Security & Authentication

The Td5 ECU requires mandatory seed-key authentication before diagnostic access. This implementation uses the reverse-engineered algorithm:

```cpp
uint16_t calculateTd5Key(uint16_t seed) {
    uint16_t key = seed;
    key = ((key & 0xFF) << 8) | ((key & 0xFF00) >> 8);  // Swap bytes
    key ^= 0x2E71;                                       // XOR with constant
    key += 0xCF;                                         // Add constant
    key = ((key & 1) << 15) | (key >> 1);              // Rotate right
    return key;
}
```

## 📡 Protocol Implementation

### Communication Specifications
- **Baud Rate**: 10400 (non-standard, requires software serial)
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Protocol**: ISO 9141-2 with Td5-specific extensions
- **Message Format**: [Length] [Target] [Source] [Data...] [Checksum]

### Initialization Sequence
1. **Fast Init**: 25ms LOW, 25ms HIGH on K-Line
2. **Init Frame**: `0x81 0x13 0xF7 0x81 0x0C`
3. **Expected Response**: `0x03 0xC1 0x57 0x8F 0xAA`
4. **Start Diagnostics**: `0x02 0x10 0xA0 0xB2`

### Live Data Commands
- **Fuelling Data**: `0x02 0x21 0x20` - Speed, RPM, injection, MAF, throttle position
- **Input Status**: `0x02 0x21 0x21` - Brake switches, gear position, other inputs  
- **Temperature Sensors**: `0x02 0x21 0x22` - Coolant, fuel, inlet, ambient temperatures
- **Pressure Sensors**: `0x02 0x21 0x23` - MAP, AAP, boost calculation, reference voltage
- **Actuator Positions**: `0x02 0x21 0x24` - EGR throttle, turbo wastegate positions

### Critical Implementation Notes
- **Echo Cancellation**: Half-duplex requires discarding transmitted echoes
- **Inter-byte Timing**: 5ms delays improve reliability
- **Keep-alive Messages**: Required every 1.5 seconds
- **Response Timeouts**: 2-second windows with retry logic

### Driver Input Technical Details

**Clutch Pedal Switch Operation:**
- **Location**: Mounted on clutch master cylinder
- **Wiring**: ECU black plug pin 35 (Black/White wire)
- **Logic**: Normally closed, opens when clutch pressed
- **Function**: Enables "surge damping" to prevent engine over-rev during gear changes
- **Cruise Control**: Disables cruise control when clutch engaged for safety

**Brake Pedal Switch System:**
- **Primary Switch**: Main brake light and ECU detection 
- **Cruise Switch**: Secondary switch specifically for cruise control cutoff
- **Safety Logic**: Either switch activation disables cruise control immediately

**Gear Position Detection:**
- **Manual Transmissions**: Neutral switch only (in/out of gear)
- **Automatic Transmissions**: Full PRND321 position reporting
- **Integration**: Affects idle speed control and cruise control availability

**Air Conditioning Integration:**
- **Compressor Request**: ECU monitors A/C system demand
- **Load Compensation**: Increases idle speed when A/C compressor engages
- **Engine Protection**: May limit boost/power when A/C load is high

**Handbrake Integration:**
- **Safety Function**: Some ECU variants monitor handbrake for hill start assist
- **Diagnostic**: Helps identify parking/stationary conditions
- **Model Dependent**: Not all Td5 variants include handbrake monitoring

### Data Logging and Analysis Capabilities

With over 25 live parameters available, this interface enables:
- **Real-time Performance Monitoring**: Track boost pressure, injection quantity, and temperatures
- **Driver Behavior Analysis**: Monitor clutch, brake, and throttle input patterns
- **Diagnostic Troubleshooting**: Monitor sensor voltages and actuator positions
- **Fuel Economy Analysis**: Log MAF, injection quantity, and driver demand relationships  
- **Temperature Management**: Track coolant, fuel, and inlet air temperatures
- **Turbo System Health**: Monitor boost pressure, wastegate position, and MAP/AAP sensors
- **Electrical System Status**: Battery voltage and ECU reference voltage monitoring
- **Transmission Diagnostics**: Gear position and neutral switch monitoring for both manual and automatic

The comprehensive parameter set exceeds commercial diagnostic tools like NANOCOM, providing professional-level data access including driver input monitoring typically only available to Land Rover technicians.

With over 20 live parameters available, this interface enables:
- **Real-time Performance Monitoring**: Track boost pressure, injection quantity, and temperatures
- **Diagnostic Troubleshooting**: Monitor sensor voltages and actuator positions
- **Fuel Economy Analysis**: Log MAF, injection quantity, and driver demand relationships  
- **Temperature Management**: Track coolant, fuel, and inlet air temperatures
- **Turbo System Health**: Monitor boost pressure, wastegate position, and MAP/AAP sensors
- **Electrical System Status**: Battery voltage and ECU reference voltage monitoring

The comprehensive parameter set matches and exceeds commercial diagnostic tools like NANOCOM, providing professional-level data access for a fraction of the cost.

## 🏁 ECU Compatibility

### Supported Vehicles
- **Land Rover Defender** (1998-2006 Td5 engine)
- **Land Rover Discovery 2** (1998-2004 Td5 engine)
- **Compatible with all Td5 ECU variants**: MSB series and NNN series

### ECU Series Differences
- **MSB Series** (pre-2002): Fixed programming, full diagnostics
- **NNN Series** (2002+): Flash programmable, additional service modes
- **Protocol Identical**: All variants use same 10400 baud ISO 9141-2

## 🐛 Troubleshooting

### Common Issues
- **No ECU Response**: Check K-Line connections, verify +12V power, ensure proper grounding
- **Authentication Failure**: Verify seed-key algorithm implementation, check message timing
- **Intermittent Communication**: Add inter-byte delays, verify L9637D power supplies
- **Baud Rate Problems**: Use software serial library, avoid ESP32 hardware UART limitations
- **Missing Parameters**: Some data may not be available on all ECU variants (MSB vs NNN)

### Parameter-Specific Issues
- **Temperature Readings**: Values below -40°C or above 200°C indicate sensor faults
- **Pressure Sensors**: MAP should be >AAP; negative boost indicates sensor problems  
- **Battery Voltage**: Should read 12-15V; outside range indicates electrical issues
- **Reference Voltage**: Should be stable at ~5.0V; variations indicate ECU problems
- **EGR/Wastegate**: Positions stuck at 0% or 100% suggest mechanical faults

### Driver Input Switch Issues
- **Clutch Switch**: Should show "Released" at idle, "PRESSED" when pedal down
  - Stuck "PRESSED": Prevents cruise control, may affect engine response
  - Wiring**: Check ECU pin 35 (Black/White wire) for continuity to clutch master cylinder
- **Brake Switches**: Both switches should respond to pedal pressure
  - Failed brake switch disables cruise control permanently
  - Check primary brake switch and separate cruise control brake switch
- **Gear Position**: Auto transmissions should show correct P/R/N/D position
  - Incorrect readings affect idle speed and cruise control availability
  - Manual transmissions only show "Neutral" vs "In Gear" status
- **Handbrake Switch**: Should show "ON" when engaged, "Off" when released
  - May affect hill start assist and parking brake warning systems

### Debug Tips
- Monitor serial output at 115200 baud for detailed protocol traces
- LED on GPIO 2 indicates successful ECU connection status
- Verify L9637D power supply voltages: 12V on pin 7, 3.3V on pin 3
- Check ECU grounding: pins B1, B2, B24, B25 should have <0.1Ω to chassis
- Use multimeter to verify K-Line voltage levels: 0V (LOW) and 12V (HIGH)
- Check message checksums if getting negative responses from ECU

## 📚 Technical References

### Primary Research Sources
- **[EA2EGA/Ekaitza_Itzali](https://github.com/EA2EGA/Ekaitza_Itzali)** - Complete Td5 diagnostic tool implementation
- **[pajacobson/td5keygen](https://github.com/pajacobson/td5keygen)** - Reverse-engineered seed-key algorithm
- **[hairyone/TD5Tester](https://github.com/hairyone/TD5Tester)** - Android-based Td5 diagnostic application
- **[BennehBoy/LRDuinoTD5](https://github.com/BennehBoy/LRDuinoTD5)** - STM32-based multi-gauge system with L9637D

### Hardware Documentation
- **[ST L9637D Datasheet](https://www.st.com/resource/en/datasheet/l9637.pdf)** - Complete specifications for K-Line transceiver
- **[L9637D Product Page](https://www.st.com/en/automotive-analog-and-power/l9637.html)** - STMicroelectronics official documentation
- **[Instructables K-Line Tutorial](https://www.instructables.com/Low-Cost-OBD2-Communications-on-K-line-ISO-9141-2-/)** - Hardware implementation guide

### Protocol References  
- **[ISO 9141-2 K-Line Implementation](https://m0agx.eu/reading-obd2-data-without-elm327-part-2-k-line.html)** - Detailed K-Line protocol explanation
- **[Keyword Protocol 2000](https://en.wikipedia.org/wiki/Keyword_Protocol_2000)** - Wikipedia protocol overview
- **[OBD9141 Arduino Library](https://github.com/iwanders/OBD9141)** - ISO 9141-2 implementation reference

### Community Resources
- **[Australian Land Rover Owners - Td5 OBD2 Reversing](https://www.aulro.com/afvb/electronic-diagnostic-systems/240456-td5-obd2-reversing.html)** - Community reverse engineering efforts
- **[Clutch Switch Operation Discussion](https://www.aulro.com/afvb/technical-chatter/31072-clutch-switch-operation-td5.html)** - Technical analysis of clutch switch surge damping function
- **[LandyZone Td5 Communication Forum](https://www.landyzone.co.uk/land-rover/discovery-2-td5-cant-communicate.366647/)** - Troubleshooting discussions
- **[Td5 Throttle Pedal Diagnostics](https://www.lrukforums.com/threads/defender-td5-throttle-problems.142192/)** - Driver demand and input switch troubleshooting
- **[Land Rover Monthly - Throttle Pedal Fix](https://www.landrovermonthly.co.uk/articles/defender-td5-throttle-pedal-fix/)** - Professional repair procedures for driver input issues
- **[Cruise Control Wiring Guide](https://www.lrukforums.com/threads/wiring-up-td5-cruise-control.104970/)** - Technical details on clutch and brake switch integration
- **[Digital Kaos Td5 ECU Repair](https://www.digital-kaos.co.uk/forums/showthread.php/802599-Land-Rover-Defender-TD5-ECU-repair/page2)** - Hardware repair insights

### Commercial Tools Reference
- **[BlackBox Solutions SM010](https://blackbox-solutions.com/help/SM010.html)** - Professional Lucas Td5 diagnostic specifications
- **[NANOCOM Diagnostics](https://www.nanocom-diagnostics.com/product/ncom01-defender-td5-kit)** - Commercial Td5 diagnostic tool
- **[DiscoTD5.com Resources](https://www.discotd5.com/c-and-python-odds-and-ends/td5-keygen-now-github)** - Community diagnostic tools and resources

### Technical Forums & Development
- **[ESP32 10400 Baud Issues](https://forum.arduino.cc/t/esp32-10400-baudrate-issue/1141941)** - Arduino forum discussion on non-standard baud rates
- **[STMicroelectronics L9637D Community](https://community.st.com/t5/autodevkit-ecosystem/l9637d-k-line-transceiver-lo-pin-functionality/td-p/638515)** - L9637D implementation discussions
- **[K-Line Reader Projects](https://github.com/muki01/OBD2_K-line_Reader)** - Additional K-Line implementation examples

## 📡 ESP-NOW Data Broadcasting Schema

### Message Types and Structure

The ESP-NOW implementation uses a structured packet format with six distinct message types for organized data transmission:

#### Packet Header (8 bytes)
```c
struct Td5PacketHeader {
  uint32_t timestamp;      // milliseconds since ESP32 boot
  uint16_t sequence;       // incrementing packet sequence number
  uint8_t messageType;     // message type identifier (see below)
  uint8_t dataLength;      // payload size in bytes
};
```

#### Message Types

**1. FUELLING Data (0x01) - 10 bytes payload**
```c
struct Td5FuellingData {
  uint16_t vehicleSpeed;      // km/h
  uint16_t engineRPM;         // RPM
  uint16_t injectionQuantity; // mg/stroke * 100
  uint16_t manifoldAirFlow;   // kg/h * 10
  uint16_t driverDemand;      // throttle position % * 100
};
```

**2. INPUTS Data (0x02) - 4 bytes payload**
```c
struct Td5InputsData {
  uint8_t switchStates;       // bit field (see below)
  uint8_t gearPosition;       // 0=Park, 1=Reverse, 2=Neutral, 3=Drive, etc.
  uint8_t reserved[2];        // future expansion
};

// switchStates bit field:
// Bit 0: Brake pedal (1=pressed)
// Bit 1: Cruise brake (1=pressed)
// Bit 2: Clutch pedal (1=pressed)
// Bit 3: Handbrake (1=engaged)
// Bit 4: A/C request (1=on)
// Bit 5: Cruise control (1=on)
// Bit 6: Neutral switch (1=selected)
// Bit 7: Reserved
```

**3. TEMPERATURES Data (0x03) - 10 bytes payload**
```c
struct Td5TemperaturesData {
  int16_t coolantTemp;        // °C
  int16_t fuelTemp;           // °C
  int16_t inletAirTemp;       // °C
  int16_t ambientAirTemp;     // °C
  uint16_t batteryVoltage;    // mV
};
```

**4. PRESSURES Data (0x04) - 8 bytes payload**
```c
struct Td5PressuresData {
  uint16_t manifoldPressure;  // kPa
  uint16_t ambientPressure;   // kPa
  uint16_t boostPressure;     // kPa (calculated: MAP - AAP)
  uint16_t referenceVoltage;  // mV (ECU 5V reference)
};
```

**5. ACTUATORS Data (0x05) - 8 bytes payload**
```c
struct Td5ActuatorsData {
  uint16_t egrPosition;       // EGR throttle position % * 100
  uint16_t wastegatePosition; // turbo wastegate position % * 100
  uint8_t reserved[4];        // future actuators
};
```

**6. STATUS Data (0x06) - 8 bytes payload**
```c
struct Td5StatusData {
  uint8_t connectionState;    // ECU connection state (0-4)
  uint8_t lastErrorCode;      // last ECU error received
  uint16_t connectionUptime;  // seconds since connection established
  uint32_t totalPacketsSent;  // ESP-NOW transmission statistics
};
```

### Broadcasting Configuration

- **Channel**: Channel 1 (configurable via `ESPNOW_CHANNEL`)
- **Broadcast Address**: `FF:FF:FF:FF:FF:FF` (no pairing required)
- **Update Rate**: 250ms per message type (4Hz per type, 24Hz total)
- **Range**: Typically 100-200m line of sight, 20-50m through obstacles
- **Maximum Recipients**: Unlimited (broadcast mode)

### Packet Validation

Each packet includes an XOR checksum calculated across the header and payload:
```c
uint8_t checksum = 0;
for (size_t i = 0; i < headerSize + payloadSize; i++) {
  checksum ^= packetData[i];
}
```

### Receiver Implementation Example

```c
#include <esp_now.h>
#include <WiFi.h>

void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
  if (len < sizeof(Td5PacketHeader) + 1) return;

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

  // Process by message type
  switch (header->messageType) {
    case 0x01: // FUELLING
      Td5FuellingData* fuelling = (Td5FuellingData*)payload;
      Serial.print("Speed: ");
      Serial.print(fuelling->vehicleSpeed);
      Serial.println(" km/h");
      break;

    case 0x02: // INPUTS
      Td5InputsData* inputs = (Td5InputsData*)payload;
      bool brakePressed = (inputs->switchStates & 0x01) != 0;
      Serial.print("Brake: ");
      Serial.println(brakePressed ? "PRESSED" : "Released");
      break;

    // Handle other message types...
  }
}

void setup() {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataReceived);
}
```

### Integration with Adaptive Cruise Control

The structured message format enables sophisticated cruise control implementations:

1. **Speed Control**: Use FUELLING data for current speed and throttle position
2. **Safety Monitoring**: Monitor INPUTS data for brake pedal activation
3. **System Health**: Track TEMPERATURES and PRESSURES for engine protection
4. **Performance**: Analyze ACTUATORS data for turbo and EGR operation

### Troubleshooting ESP-NOW

- **No Data Received**: Check ESP-NOW initialization and channel configuration
- **Intermittent Reception**: Verify power supply stability and antenna positioning
- **Range Issues**: Consider ESP32 variant (some have better RF performance)
- **Checksum Errors**: Check for interference or power supply noise

## 📄 License & Disclaimer

This project is provided for educational and research purposes. The reverse-engineered protocol implementation is based on community research and open-source projects. Users are responsible for compliance with local laws and vehicle warranty considerations.

**Safety Notice**: This interface is designed for diagnostic purposes only. Modifications to ECU parameters should only be performed by qualified technicians with appropriate safety measures.

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests. When contributing, please reference the technical sources listed above and maintain compatibility with the existing protocol implementation.

## 📞 Support

For technical support:
1. Check the troubleshooting section above
2. Review the referenced community forums
3. Consult the original source projects listed in references
4. Submit issues with detailed error logs and hardware configuration

---

*This project builds upon the excellent work of the Land Rover community, particularly the reverse engineering efforts documented in the referenced GitHub projects and forum discussions.*
