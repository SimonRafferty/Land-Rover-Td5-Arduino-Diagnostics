/*
 * TD5 ECU Parameter Definitions
 *
 * Structured approach to ECU diagnostic parameters with:
 * - PID definitions
 * - Response parsing
 * - Scaling functions
 * - Message type mapping
 */

#ifndef TD5_PARAMETERS_H
#define TD5_PARAMETERS_H

#include <Arduino.h>

// Forward declarations of global variables (defined in main .ino)
extern uint16_t vehicleSpeed;
extern uint16_t engineRPM;
extern uint16_t injectionQuantity;
extern uint16_t driverDemand;
extern int16_t fuelTemp;           // int16_t to match main file
extern uint16_t batteryVoltage;
extern uint16_t manifoldPressure;
extern uint16_t ambientPressure;
extern uint16_t boostPressure;     // uint16_t to match main file
extern uint16_t egrPosition;
extern uint16_t wastegatePosition;
extern bool brakePedalPressed;
extern bool clutchPedalPressed;
extern bool handbrakeEngaged;

// ============================================================================
// SCALING FUNCTIONS
// ============================================================================

// Temperature: Kelvin*10 → Celsius
inline float scaleTempKelvin(uint16_t raw) {
  return ((int16_t)raw - 2732) / 10.0f;
}

// Pressure: Pascals → kPa
inline float scalePressurePa(uint16_t raw) {
  return raw / 100.0f;
}

// Fuel quantity: mg*100 → mg/stroke
inline float scaleFuelQuantity(uint16_t raw) {
  return raw / 100.0f;
}

// Voltage: mV → V
inline float scaleVoltage(uint16_t raw) {
  return raw / 1000.0f;
}

// Actuator position: %*100 → %
inline float scaleActuatorPercent(uint16_t raw) {
  return raw / 100.0f;
}

// No scaling (direct value)
inline float scaleNone(uint16_t raw) {
  return (float)raw;
}

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================

inline void updateRPM(float value) { engineRPM = (uint16_t)value; }
inline void updateSpeed(float value) { vehicleSpeed = (uint8_t)value; }
inline void updateInjection(float value) { injectionQuantity = (uint16_t)(value * 100); }
inline void updateFuelTemp(float value) { fuelTemp = (int8_t)value; }
inline void updateBatteryVoltage(float value) { batteryVoltage = (uint16_t)(value * 1000); }
inline void updateMAP(float value) {
  manifoldPressure = (uint16_t)value;
  // Recalculate boost when either MAP or ambient changes
  boostPressure = (int16_t)manifoldPressure - (int16_t)ambientPressure;
}
inline void updateAmbientPressure(float value) {
  ambientPressure = (uint16_t)value;
  // Recalculate boost when either MAP or ambient changes
  boostPressure = (int16_t)manifoldPressure - (int16_t)ambientPressure;
}
inline void updateEGR(float value) { egrPosition = (uint16_t)(value * 100); }
inline void updateWastegate(float value) { wastegatePosition = (uint16_t)(value * 100); }

// Digital inputs - special handler (takes float to match updateFunc signature)
inline void updateDigitalInputs(float value) {
  uint16_t raw = (uint16_t)value;  // Cast from float to uint16_t
  uint8_t inputByte1 = (raw >> 8) & 0xFF;  // High byte
  // uint8_t inputByte2 = raw & 0xFF;      // Low byte (gear position - not used)

  brakePedalPressed = (inputByte1 & 0x01) == 0;    // INVERTED
  clutchPedalPressed = (inputByte1 & 0x04) == 0;   // INVERTED
  handbrakeEngaged = (inputByte1 & 0x08) != 0;     // Normal
}

// ============================================================================
// PARAMETER DEFINITION STRUCTURE
// ============================================================================

struct TD5Parameter {
  uint8_t pid;                               // ECU PID code
  const char* name;                          // Human-readable name
  uint8_t responseLength;                    // Expected response bytes (including header)
  bool isTwoByteValue;                       // true = 16-bit value, false = 8-bit
  float (*scalingFunc)(uint16_t);            // Scaling function
  void (*updateFunc)(float);                 // Update global variable
  uint8_t espnowMessageType;                 // Which ESP-NOW message type (0x01-0x06)
};

// ============================================================================
// TD5 PARAMETER TABLE
// ============================================================================

const TD5Parameter TD5_PARAMS[] = {
  // PID   Name              RespLen  2Byte  Scaling              Update             MsgType
  {0x09, "RPM",                 5,   true,  scaleNone,           updateRPM,         0x01}, // FUELLING
  {0x0D, "Speed",               4,   false, scaleNone,           updateSpeed,       0x01}, // FUELLING
  {0x20, "Injection",           5,   true,  scaleFuelQuantity,   updateInjection,   0x01}, // FUELLING
  {0x21, "Digital Inputs",      5,   true,  scaleNone,           updateDigitalInputs, 0x02}, // INPUTS
  {0x0B, "Fuel Temperature",    5,   true,  scaleTempKelvin,     updateFuelTemp,    0x03}, // TEMPERATURES
  {0x17, "Battery Voltage",     5,   true,  scaleVoltage,        updateBatteryVoltage, 0x03}, // TEMPERATURES
  {0x0A, "MAP",                 5,   true,  scalePressurePa,     updateMAP,         0x04}, // PRESSURES
  {0x0F, "Ambient Pressure",    5,   true,  scalePressurePa,     updateAmbientPressure, 0x04}, // PRESSURES
  {0x2B, "EGR Position",        5,   true,  scaleActuatorPercent, updateEGR,        0x05}, // ACTUATORS
  {0x2C, "Wastegate Position",  5,   true,  scaleActuatorPercent, updateWastegate,  0x05}, // ACTUATORS
};

const uint8_t TD5_PARAM_COUNT = sizeof(TD5_PARAMS) / sizeof(TD5_PARAMS[0]);

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Find parameter by PID
inline const TD5Parameter* findParamByPID(uint8_t pid) {
  for (uint8_t i = 0; i < TD5_PARAM_COUNT; i++) {
    if (TD5_PARAMS[i].pid == pid) {
      return &TD5_PARAMS[i];
    }
  }
  return nullptr;
}

// Get PIDs for a specific message type
inline void getPIDsForMessageType(uint8_t msgType, uint8_t* pids, uint8_t& count) {
  count = 0;
  for (uint8_t i = 0; i < TD5_PARAM_COUNT && count < 10; i++) {
    if (TD5_PARAMS[i].espnowMessageType == msgType) {
      pids[count++] = TD5_PARAMS[i].pid;
    }
  }
}

#endif // TD5_PARAMETERS_H
