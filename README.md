# Land Rover Td5 ECU Interface for ESP32

A complete Arduino implementation for communicating with pre-2006 Land Rover Td5 engine ECUs via K-Line interface. This project enables real-time extraction of vehicle data including speed, brake pedal status, engine parameters, and diagnostic information using the proprietary ISO 9141-2 protocol at 10400 baud.

## 🚗 Overview

The Land Rover Td5 engine ECU uses a non-standard implementation of the ISO 9141-2 protocol that is incompatible with generic OBD2 scanners. This project provides a complete solution for ESP32-based communication, including:

- **Fast initialization sequence** with precise timing control
- **Seed-key authentication** using reverse-engineered algorithm
- **Real-time data extraction** for speed, RPM, brake status, and engine parameters
- **Robust error recovery** and keep-alive message handling
- **Complete hardware interface** specifications with L9637D transceiver

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

The interface provides real-time monitoring of over 20 engine parameters:

```
=== Land Rover Td5 ECU Interface ===
Attempting ECU connection...
Fast init successful, authenticating...
ECU authenticated and connected!

FUELLING - Speed: 45 km/h (27.9 mph), RPM: 1850, IQ: 12.50 mg/stroke, MAF: 18.5 kg/h, Throttle: 25.3%
INPUTS - Brake Pedal: Released, Cruise Brake: Released
TEMPS - Coolant: 89°C, Fuel: 85°C, Inlet: 45°C, Ambient: 28°C, Battery: 14.2V
PRESSURE - MAP: 240 kPa, AAP: 100 kPa, Boost: 140 kPa (20.3 psi), Vref: 5.02V
ACTUATORS - EGR: 15.2%, Wastegate: 45.8%

FUELLING - Speed: 50 km/h (31.1 mph), RPM: 2100, IQ: 15.75 mg/stroke, MAF: 22.3 kg/h, Throttle: 35.8%
INPUTS - Brake Pedal: PRESSED, Cruise Brake: PRESSED
TEMPS - Coolant: 91°C, Fuel: 87°C, Inlet: 48°C, Ambient: 30°C, Battery: 14.1V
PRESSURE - MAP: 195 kPa, AAP: 100 kPa, Boost: 95 kPa (13.8 psi), Vref: 5.01V
ACTUATORS - EGR: 8.5%, Wastegate: 25.2%
```

### Complete Parameter List

**Engine Performance:**
- **Vehicle Speed**: km/h with automatic mph conversion  
- **Engine RPM**: Direct ECU value, real-time updates
- **Injection Quantity**: mg/stroke with 0.01mg precision
- **Manifold Air Flow (MAF)**: kg/h with 0.1kg/h precision
- **Driver Demand**: Accelerator pedal position (0-100%)

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
- **Brake Pedal Switches**: Primary and cruise control detection

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

### Data Logging and Analysis Capabilities

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
- **[LandyZone Td5 Communication Forum](https://www.landyzone.co.uk/land-rover/discovery-2-td5-cant-communicate.366647/)** - Troubleshooting discussions
- **[Digital Kaos Td5 ECU Repair](https://www.digital-kaos.co.uk/forums/showthread.php/802599-Land-Rover-Defender-TD5-ECU-repair/page2)** - Hardware repair insights

### Commercial Tools Reference
- **[BlackBox Solutions SM010](https://blackbox-solutions.com/help/SM010.html)** - Professional Lucas Td5 diagnostic specifications
- **[NANOCOM Diagnostics](https://www.nanocom-diagnostics.com/product/ncom01-defender-td5-kit)** - Commercial Td5 diagnostic tool
- **[DiscoTD5.com Resources](https://www.discotd5.com/c-and-python-odds-and-ends/td5-keygen-now-github)** - Community diagnostic tools and resources

### Technical Forums & Development
- **[ESP32 10400 Baud Issues](https://forum.arduino.cc/t/esp32-10400-baudrate-issue/1141941)** - Arduino forum discussion on non-standard baud rates
- **[STMicroelectronics L9637D Community](https://community.st.com/t5/autodevkit-ecosystem/l9637d-k-line-transceiver-lo-pin-functionality/td-p/638515)** - L9637D implementation discussions
- **[K-Line Reader Projects](https://github.com/muki01/OBD2_K-line_Reader)** - Additional K-Line implementation examples

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
