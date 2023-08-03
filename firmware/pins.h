// PORTD
const uint8_t brakeInPin = 2;
const uint8_t leftInPin = 4;
const uint8_t rightInPin = 5;

// 3.3V pins labeled BTTX and BTRX
const uint8_t BTRX = 7;
const uint8_t BTTX = 8;

// Bluetooth "EN"
#define BT_EN 9

// mcp2515 interrupt
#define CAN_INT A1
const int CAN_CS_PIN = 10;

// SPI Pins
// SPI_MOSI = 11;
// SPI_MISO = 12;
// SPI_SCK = 13; // turns on LED dim when active

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


/*
 * Constants for Cmd flags
const uint8_t ENGINE_RUNNING = B10000000;       // serialCmdA
const uint8_t BRAKE_ON = B00000100;             //receivedCmdD
const uint8_t HORN_ON = B00001000;              //receivedCmdD
const uint8_t LEFT_ON = B00010000;              //receivedCmdD
const uint8_t RIGHT_ON = B00100000;             //receivedCmdD
const uint8_t HIGH_BEAMS_ON = B01000000;        //receivedCmdD
const uint8_t KILL_ON = B10000000;              //receivedCmdD

const uint8_t CLUTCH_DISENGAGED = B00000100;    //receivedCmdC
const uint8_t IN_NEUTRAL  = B00010000;          //receivedCmdC
const uint8_t KICKSTAND_UP = B00001000;         //receivedCmdC
 */
