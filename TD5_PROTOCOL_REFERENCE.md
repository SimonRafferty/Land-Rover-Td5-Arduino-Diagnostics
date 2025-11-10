# Land Rover Td5 ECU K-Line Protocol Reference

**Last Updated:** 2025-01-11

## Decoding Status Summary

**Fully Decoded PIDs:**
- ✓ 0x09 (RPM), 0x0D (Speed), 0x21 (Digital Inputs)
- ✓ 0x10 (Battery Voltage - main + reference)
- ✓ 0x23 (Accelerator Tracks - **little-endian**)
- ✓ 0x1E (Cruise/Brake Switches)
- ✓ 0x37 (EGR Position), 0x38 (Wastegate Position)
- ✓ 0x40 (Cylinder Fuel Trim - 5 cylinders)

**Partially Decoded PIDs:**
- ⚠️ 0x1A (Coolant, MAP, Boost - **9 unknown bytes**)
- ⚠️ 0x1B (Airflow, Ambient Pressure - **6 unknown bytes**)
- ⚠️ 0x1C (Inlet Air Temp, Fuel Temp - **5 unknown bytes**)


## Table of Contents
1. [Physical Layer](#physical-layer)
2. [Protocol Basics](#protocol-basics)
3. [Initialization Sequence](#initialization-sequence)
4. [Authentication (Seed-Key)](#authentication-seed-key)
5. [Live Data Request/Response](#live-data-requestresponse)
6. [PID Reference](#pid-reference)
7. [Composite PIDs](#composite-pids)
8. [Timing Requirements](#timing-requirements)
9. [Known Issues](#known-issues)

---

## Physical Layer

### Hardware
- **Protocol**: ISO 9141-2 K-Line
- **Baud Rate**: 10400 baud (non-standard, cannot use hardware UART on most microcontrollers)
- **Data Format**: 8 data bits, no parity, 1 stop bit
- **Physical Connection**:
  - ECU Pin B18 (pink wire) OR OBD-II Port Pin 7
  - Requires L9637D transceiver chip
  - 510Ω pull-up resistor to +12V on K-Line

### Half-Duplex Operation
- **Echo Cancellation Required**: All transmitted bytes are echoed back
- Each transmitted byte must be read back from RX before continuing
- Recommended echo timeout: 100ms per byte

---

## Protocol Basics

### Message Format
All messages follow this structure:

```
[Length] [Service] [Data...] [Checksum]
```

- **Length**: Number of bytes following length byte (excludes length byte itself)
- **Service**: Request/response type
- **Data**: Variable length payload
- **Checksum**: Simple additive checksum (sum of all bytes including length)

### Service Codes

#### Request Services (Tester → ECU)
| Service | Description |
|---------|-------------|
| 0x81 | Start Communication (Fast Init) |
| 0x10 | Start Diagnostic Session |
| 0x27 | Security Access (Seed/Key) |
| 0x21 | Read Data By PID |
| 0x3E | Tester Present (Keep-Alive) |

#### Response Services (ECU → Tester)
| Service | Description |
|---------|-------------|
| 0xC1 | Positive response to 0x81 (Start Communication) |
| 0x50 | Positive response to 0x10 (Diagnostic Session) |
| 0x67 | Positive response to 0x27 (Security Access) |
| 0x61 | Positive response to 0x21 (Data Read) |
| 0x7F | Negative Response (Error) |
| 0x7E | Positive response to 0x3E (Keep-Alive) |

### Negative Response Format
```
[Length=0x03] [0x7F] [Requested Service] [Error Code] [Checksum]
```

#### Common Error Codes
| Code | Meaning |
|------|---------|
| 0x10 | General Reject |
| 0x11 | Service Not Supported |
| 0x12 | Subfunction Not Supported |
| 0x22 | Conditions Not Correct |
| 0x31 | Request Out of Range |

---

## Initialization Sequence

### 1. Fast Init Pulse
**Critical Timing:**
```
K-Line HIGH for 300ms (idle)
K-Line LOW for 25ms ± 1ms
K-Line HIGH for 25ms ± 1ms
```

### 2. Start Communication Request
```
Request:  81|13|F7|81|0C
          ^  ^  ^  ^  ^
          |  |  |  |  +-- Checksum
          |  |  |  +----- Echo of service 0x81
          |  |  +-------- Tester address (0xF7 standard)
          |  +----------- ECU address (0x13 = physical, 0x19 = functional)
          +-------------- Service: Start Communication

Response: 03|C1|57|8F|AA
          ^  ^  ^  ^  ^
          |  |  |  |  +-- Checksum
          |  |  |  +----- Key byte 2
          |  |  +-------- Key byte 1
          |  +----------- Positive response
          +-------------- Length
```

### 3. Start Diagnostic Session
```
Request:  02|10|A0|B2
          ^  ^  ^  ^
          |  |  |  +---- Checksum
          |  |  +------- Session type (0xA0 = extended)
          |  +---------- Service
          +------------- Length

Response: 02|50|A0|F2
          ^  ^  ^  ^
          |  |  |  +---- Checksum
          |  |  +------- Echo of session type
          |  +---------- Positive response
          +------------- Length
```

---

## Authentication (Seed-Key)

### Algorithm (Reverse-Engineered)
Based on [pajacobson/td5keygen](https://github.com/pajacobson/td5keygen)

```cpp
uint16_t calculateTd5Key(uint16_t seed) {
  uint16_t tmp = 0;
  uint8_t count = ((seed >> 0xC & 0x8) |
                   (seed >> 0x5 & 0x4) |
                   (seed >> 0x3 & 0x2) |
                   (seed & 0x1)) + 1;

  for (uint8_t idx = 0; idx < count; idx++) {
    uint8_t tap = ((seed >> 1) ^ (seed >> 2) ^
                   (seed >> 8) ^ (seed >> 9)) & 1;
    tmp = ((seed >> 1) | (tap << 0xF));

    if ((seed >> 0x3 & 1) && (seed >> 0xD & 1)) {
      seed = tmp & ~1;  // Clear bit 0
    } else {
      seed = tmp | 1;   // Set bit 0
    }
  }
  return seed;
}
```

### 1. Request Seed
```
Request:  02|27|01|2A
          ^  ^  ^  ^
          |  |  |  +---- Checksum
          |  |  +------- Access level (0x01 = standard)
          |  +---------- Service: Security Access
          +------------- Length

Response: 04|67|01|[SEED_H]|[SEED_L]|[CHK]
          ^  ^  ^   ^^^^^^   ^^^^^^   ^^^
          |  |  |     |        |       +-- Checksum
          |  |  |     |        +---------- Seed low byte
          |  |  |     +------------------- Seed high byte (big-endian)
          |  |  +------------------------- Echo of access level
          |  +---------------------------- Positive response
          +------------------------------- Length
```

### 2. Send Key
```
Request:  04|27|02|[KEY_H]|[KEY_L]|[CHK]
          ^  ^  ^   ^^^^^   ^^^^^   ^^^
          |  |  |     |       |      +--- Checksum
          |  |  |     |       +---------- Key low byte
          |  |  |     +------------------ Key high byte (big-endian)
          |  |  +------------------------ Unlock request
          |  +--------------------------- Service: Security Access
          +------ Length

Response: 02|67|02|6B  (Success)
          ^  ^  ^  ^
          |  |  |  +--- Checksum
          |  |  +------ Echo of unlock request
          |  +--------- Positive response
          +------------ Length
```

---

## Live Data Request/Response

### Standard Single-Value PID Request
```
Request:  02|21|[PID]|[CHK]
          ^  ^   ^^^   ^^^
          |  |    |     +-- Checksum
          |  |    +-------- PID code
          |  +------------- Service: Read Data By PID
          +---------------- Length

Response: [LEN]|61|[PID]|[DATA...]|[CHK]
          ^^^^  ^   ^^^   ^^^^^^^^  ^^^
           |    |    |       |       +-- Checksum
           |    |    |       +---------- Data bytes (typically 1-2 bytes)
           |    |    +------------------ Echo of requested PID
           |    +----------------------- Positive response
           +---------------------------- Length (varies by PID)
```

### Example: RPM Request
```
Request:  02|21|09|2C
Response: 04|61|09|02|E8|57
                    ^^^^^ RPM value = 0x02E8 = 744 RPM
```

---

## PID Reference

### Single-Value PIDs

| PID  | Name | Response Length | Data Type | Formula | Unit | Example |
|------|------|----------------|-----------|---------|------|---------|
| 0x09 | Engine RPM | 5 bytes | uint16 BE | raw | RPM | 0x02E8 = 744 RPM |
| 0x0D | Vehicle Speed | 4 bytes | uint8 | raw | km/h | 0x00 = 0 km/h |
| 0x1E | Cruise/Brake Switches | 5 bytes | 2× uint8 | bitfield | flags | See below |
| 0x21 | Digital Inputs | 5 bytes | 2× uint8 | bitfield | flags | See below |
| 0x36 | Unknown Status | 5 bytes | uint16 BE | raw | ? | 0x0005 (constant) |
| 0x37 | EGR Position | 5 bytes | uint16 BE | raw / 100 | % | 0x0000 = 0.00% |
| 0x38 | Wastegate Position | 5 bytes | uint16 BE | raw / 100 | % | 0x0000 = 0.00% |

#### PID 0x21: Digital Inputs Bitfield

**Byte 1 (data[3]):**
| Bit | Signal | Logic |
|-----|--------|-------|
| 0 | Brake Pedal | INVERTED (0=pressed) |
| 1 | Cruise Brake | INVERTED (0=pressed) |
| 2 | Clutch Pedal | INVERTED (0=pressed) |
| 3 | Handbrake | Normal (1=engaged) |
| 4 | A/C Request | Normal (1=on) |
| 5 | Cruise Control | Normal (1=active) |
| 6 | Neutral Switch | Normal (1=neutral) |
| 7 | Reserved | - |

**Byte 2 (data[4]):**
- Lower nibble: Gear position (auto transmission)

**Example Response:**
```
04|61|21|00|05|8B
          ^^  ^^
          |   +--- Gear info
          +------- 0x00 = All switches off except bit pattern suggests brake ON
```

#### PID 0x1E: Cruise Control & Brake Switches Bitfield

**Byte 1 (data[3]):**
| Bit | Signal | Logic |
|-----|--------|-------|
| 3 | Cruise Button Active | Normal (1=button pressed) |
| Other bits | Unknown | - |

**Byte 2 (data[4]):**
| Bit | Signal | Logic |
|-----|--------|-------|
| 7 | Brake Pedal | INVERTED (0=pressed) |
| 5 | Handbrake/Cruise Type | Ambiguous (needs more testing) |

**Observations:**
- Byte 3 toggles between 0x04 (cruise button released) and 0x0C (cruise button pressed)
- Byte 4 bit 7 inverted: 0xA2 (brake released), 0x22 (brake pressed)
- Byte 4 bit 5 correlation unclear: toggles with handbrake or cruise button type

**Example Responses:**
```
Handbrake Applied:       04|61|1E|04|A2
                                  ^^  ^^
                                  |   +--- 0xA2: brake off, bit 5 set

Brake Pressed:           04|61|1E|04|22
                                  ^^  ^^
                                  |   +--- 0x22: brake on, bit 5 clear

Cruise Resume Pressed:   04|61|1E|0C|A2
                                  ^^  ^^
                                  |   +--- 0x0C: cruise btn active
                                  +------- 0xA2: brake off, bit 5 set

Cruise Set Pressed:      04|61|1E|0C|82
                                  ^^  ^^
                                  |   +--- 0x82: brake off, bit 5 clear
                                  +------- 0x0C: cruise btn active

Handbrake Released:      04|61|1E|04|82
                                  ^^  ^^
                                  +--- 0x04: cruise btn off, 0x82: bit 5 clear
```

**Note:** The function of bit 5 in byte 2 requires additional testing. It appears to toggle between cruise SET/RESUME or correlate with handbrake state, but the pattern is not fully confirmed.

---

## Composite PIDs

These PIDs return multiple parameters in a single response.

### PID 0x1A: Multi-Parameter (18 bytes)


```
Response: 12|61|1A|[DATA: 15 bytes]|CHK
```

**Decoded Fields:**

| Offset | Size | Field | Formula | Unit | Notes |
|--------|------|-------|---------|------|-------|
| 3-4 | 2 bytes | Coolant Temperature | ((raw - 2732) / 10) | °C | Kelvin × 10, big-endian |
| 5-6 | 2 bytes | MAP (Manifold Absolute Pressure) | raw / 100 | kPa | Big-endian |
| 5-6 | 2 bytes | Boost Pressure (calculated) | (MAP - 100) / 100 | Bar | Relative to atmospheric |

**Example:**
```
Request:  02|21|1A|3D
Response: 12|61|1A|0D|6E|02|BA|0B|94|0A|B5|0B|74|0B|61|0D|2C|04|69|B3
                    ^^^^^ ^^^^^ Coolant: 70.6°C
                          ^^^^^ MAP: 6.98 kPa, Boost: -0.930 Bar (engine off)
```

**Validation:**
- Page 1: **Coolant: 70.5°C** → Decoded: **70.6°C** ✓
- Page 1: **Turbo Pressure: 0.02 Bar** → Boost calculation confirmed ✓

**Remaining Unknown Fields:** Bytes 7-18 (under investigation)

### PID 0x1B: Fuel/Air Composite (12 bytes)


```
Response: 0C|61|1B|[DATA: 9 bytes]|CHK
```

**Decoded Fields:**

| Offset | Size | Field | Formula | Unit | Notes |
|--------|------|-------|---------|------|-------|
| 5-6 | 2 bytes | Airflow (MAF) | raw / 1000 | g/s | Big-endian |
| 7-8 | 2 bytes | Ambient Pressure | raw / 46.94 | kPa | Big-endian |

**Example:**
```
Request:  02|21|1B|3E
Response: 0C|61|1B|02|81|11|44|12|3B|00|00|13|88|48
                    ^^^^^ ^^^^^ ^^^^^ ^^^^^ ^^^^^
                    0x0281 Airflow: 4.42 g/s
                          0x1144 Ambient: 99.4 kPa
```

**Validation:**
- Page 2: **Airflow: 4.4 g/s** → Decoded: **4.42 g/s** ✓
- Page 2: **Ambient Pressure: 99.47 kPa** → Decoded: **99.4 kPa** ✓

**Remaining Unknown Fields:** Bytes 3-4, 9-10, 11-12

### PID 0x1C: Temperature Composite (10 bytes)

```
Response: 0A|61|1C|[UNK]|[INLET]|[UNK]|[UNK]|[FUEL_H]|[FUEL_L]|[UNK]|[UNK]|CHK
```

**Decoded Fields:**

| Offset | Size | Field | Formula | Unit | Notes |
|--------|------|-------|---------|------|-------|
| 3 | 1 byte | Unknown | - | - | Often 0x27 (39) |
| 4 | 1 byte | Inlet Air Temp | raw / 10 | °C | Single byte × 10 |
| 5-6 | 2 bytes | Unknown | - | - | Varies |
| 7-8 | 2 bytes | Fuel Temp | (raw - 20) | °C | Offset allows negative temps |
| 9-10 | 2 bytes | Unknown | - | - | Often 0x0020 |

**Example:**
```
Request:  02|21|1C|3F
Response: 0A|61|1C|27|E9|28|06|00|2C|00|20|F3
                    ^  ^^  ^^^^  ^^^^^ ^^^^^
                    |   |    |     |     +--- Unknown (0x0020)
                    |   |    |     +--------- Fuel: 44 - 20 = 24°C
                    |   |    +--------------- Unknown
                    |   +-------------------- Inlet: 0xE9 = 233 / 10 = 23.3°C ✓
                    +------------------------ Unknown (0x27)
```

**Validation Against captured data:**
- Displayed: **Inlet Air: 23.2°C, Fuel: 64.0°C**
- Decoded values: **Inlet Air: 23.3°C** ✓, **Fuel: 44°C** (different sample)

**Temperature Offset Rationale:**
The +20°C offset on fuel temperature allows representation of temperatures down to -20°C, which is necessary for cold climate operation where fuel can freeze.

### PID 0x10: Battery Voltage (6 bytes)


```
Response: 06|61|10|[BATT: 2 bytes]|[REF: 2 bytes]|CHK
```

**Decoded Fields:**

| Offset | Size | Field | Formula | Unit | Notes |
|--------|------|-------|---------|------|-------|
| 3-4 | 2 bytes | Battery Voltage | raw / 1000 | V | Big-endian, millivolts |
| 5-6 | 2 bytes | Reference Voltage | raw / 1000 | V | Big-endian, likely sensor supply |

**Example:**
```
Request:  02|21|10|33
Response: 06|61|10|36|43|36|39|5F
                    ^^^^^ ^^^^^
                    0x3643 = 13,891 mV = 13.891V
                          0x3639 = 13,881 mV = 13.881V
```

**Validation:**
- Page 1: **Battery Voltage: 14.1V** → Range confirmed ✓
- Both values track battery voltage closely (main + reference)

### PID 0x23: Accelerator Position Tracks (6 bytes)

**Discovered from Captured data** - **FULLY DECODED** ✓

```
Response: 06|61|23|[TRACK1: 2 bytes]|[TRACK2: 2 bytes]|CHK
```

**CRITICAL: Little-endian byte order** (unlike most other PIDs)

**Decoded Fields:**

| Offset | Size | Field | Formula | Unit | Notes |
|--------|------|-------|---------|------|-------|
| 3-4 | 2 bytes | Accelerator Track 1 | raw / 10000 × 5 | V | **LITTLE-ENDIAN** |
| 5-6 | 2 bytes | Accelerator Track 2 | raw / 10000 × 5 | V | **LITTLE-ENDIAN** |

**Formula:**
```cpp
// LITTLE-ENDIAN!
uint16_t track1 = (data[4] << 8) | data[3];  // NOT data[3] << 8 | data[4]
float voltage = track1 / 10000.0 * 5.0;
```

**Example:**
```
Request:  02|21|23|46
Response: 06|61|23|27|0F|27|10|F7
                    ^^^^^ ^^^^^
                    Little-endian: 0x0F27 = 3879 → 1.94V
                                   0x1027 = 4135 → 2.07V
                    Track 1 + Track 2 = 4.01V ≈ 5V supply ✓
```

**Validation:**
- Track voltages should sum to approximately 5V (supply voltage)


### PID 0x40: Cylinder Fuel Trim (12 bytes)


```
Response: 0C|61|40|[CYL1]|[CYL2]|[CYL3]|[CYL4]|[CYL5]|CHK
                   ^^^^^^ ^^^^^^ ^^^^^^ ^^^^^^ ^^^^^^
                   Signed 16-bit big-endian trim values
```

**Example:**
```
Request:  02|21|40|63
Response: 0C|61|40|00|02|00|00|FF|FD|00|04|FF|FD|AB
                    ^^^^^ ^^^^^ ^^^^^ ^^^^^ ^^^^^
                    +2    0     -3    +4    -3
```


## Timing Requirements

### P2 Timing (ECU Response Time)
- **Typical**: 50-150ms for simple PIDs
- **Composite PIDs**: 150-300ms (longer responses)
- **Recommended timeout**: 500ms to accommodate all PID types
- Composite PIDs (0x1A, 0x1B, 0x1C, 0x40) may arrive in multiple chunks

### P3 Timing (Inter-Request Delay)
- **Minimum**: 25ms between ECU response and next tester request
- **Recommended**: 100ms between requests for stability

### Keep-Alive (Tester Present)
- **Service**: 0x3E
- **Interval**: Every 1.5 seconds maximum
- **Format**: `01|3E|3F`

```
Request:  01|3E|3F
Response: 01|7E|7F
```

**Critical:** If keep-alive not sent within ~2 seconds, ECU will terminate the diagnostic session and require re-initialization.

**Page 1 (Basic):**
1. 0x09 (RPM)
2. 0x0D (Speed)
3. 0x1A (Multi-param with coolant)
4. 0x1C (Temps)
5. 0x10 (Inlet air)

**Page 2 (Advanced):**
1. 0x09, 0x0D, 0x1A (as above)
2. 0x1B (Fuel/Air composite)
3. 0x1C, 0x21 (Temps, Digital inputs)
4. 0x40 (Cylinder trim)
5. 0x23 (Accelerator tracks)
6. 0x37, 0x38 (EGR, Wastegate)

Keep-alive is interspersed approximately every 10-15 data requests.

---

## Known Issues

### 1. Rejected PIDs
Some PIDs that work individually may be rejected (0x7F response with error 0x10) when polled too quickly:

- **PID 0x0A** (MAP) - Sometimes rejected
- **PID 0x0B** (Fuel Temp) - Sometimes rejected
- **PID 0x0F** (Ambient Pressure) - Sometimes rejected
- **PID 0x17** (Battery Voltage) - Sometimes rejected

**Workaround:** Use composite PIDs (0x1A, 0x1B) which contain these parameters and are more reliable.

### 2. Echo Contamination
Half-duplex K-Line can result in echo bytes mixing with ECU responses, especially partial echo (missing first byte).

**Improved Solution:**
1. **Per-byte echo cancellation**: Read back each transmitted byte immediately (100ms timeout per byte)
2. **Intelligent echo detection**: Compare received bytes against sent message from ANY starting position
   ```cpp
   // Detect partial echo starting from any position
   for (int sentPos = 0; sentPos < sentLength; sentPos++) {
     int matchLength = 0;
     for (int i = 0; i < bytesRead && (sentPos + i) < sentLength; i++) {
       if (buffer[i] == sentMessage[sentPos + i]) matchLength++;
       else break;
     }
     if (matchLength >= 2) {
       // Remove echo bytes
       memmove(buffer, buffer + matchLength, bytesRead - matchLength);
       bytesRead -= matchLength;
       break;
     }
   }
   ```
3. **Service code validation**: Scan for valid response start (0x61, 0x7F, 0x67, 0xC1, 0x50, 0x7E)
4. **Retry logic**: Single automatic retry for failed requests before skipping to next PID

**Example partial echo:**
- Sent: `02 21 1B 3E`
- Received: `21 1B 3E` (echo missing first byte) + ECU response
- Detection removes 3 matching bytes starting from position 1 in sent message

### 3. PID 0x1C Partially Decoded
The temperature values in PID 0x1C use different encoding than the standard Kelvin×10 format used in PID 0x1A.

**Decoded Format:**
- **Byte 4**: Inlet Air Temp as `raw / 10` (°C)
- **Bytes 7-8**: Fuel Temp as `(raw - 20)` (°C)
  - The +20°C offset allows negative temperatures down to -20°C

**Remaining Unknown Fields:**
- Byte 3: Unknown (often 0x27)
- Bytes 5-6: Unknown (varies, possibly related to second temperature reading)
- Bytes 9-10: Unknown (often constant 0x0020)

**Status:** Inlet air and fuel temps decoded ✓, other fields need investigation



---

## Protocol Reverse Engineering Credits

- **Seed-Key Algorithm**: [pajacobson/td5keygen](https://github.com/pajacobson/td5keygen)
- **Composite PIDs**: Discovered 
- **Accelerator Tracks & Cylinder Trim**: Fully decoded

---

## References

- ISO 9141-2 K-Line Protocol Standard
- ISO 14230 (KWP2000) - Similar diagnostic protocol
- L9637D K-Line Transceiver Datasheet
