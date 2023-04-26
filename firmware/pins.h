// PORTD
const uint8_t brakeInPin = 2;
const uint8_t leftInPin = 4;
const uint8_t rightInPin = 5;

// MCP_GPIOA
const uint8_t hornOutPin = B00000001;
const uint8_t clutchInPin = B00000010;
const uint8_t kickInPin = B00000100;
const uint8_t neutralInPin = B00001000;
const uint8_t hornInPin = B00010000;
const uint8_t hlhighInPin = B00100000;
const uint8_t killInPin = B01000000;
const uint8_t BTStatePin = B10000000;

// MCP_GPIOB
const uint8_t leftOutPin = B00000001;
const uint8_t rightOutPin = B00000010;
const uint8_t rearOutPin = B00000100;
const uint8_t killOutPin = B00000100; // Ignition (coils, ignition module)
const uint8_t brakeOutPin = B00001000;
const uint8_t startEnableOutPin = B00010000;
const uint8_t startOutPin = B00010000; // starter
const uint8_t hlhighOutPin = B00100000;
const uint8_t hllowOutPin = B01000000;
const uint8_t runningOutPin = B10000000;

// PORTC
const uint8_t voltageInPin = A6;

// 3.3V pins labeled BTTX and BTRX
const uint8_t BTRX = 7;
const uint8_t BTTX = 8;

// mcp2515 interrupt
#define CAN_INT A1
const int CAN_CS_PIN = 10;

// Bluetooth "EN"
#define BT_EN 9
