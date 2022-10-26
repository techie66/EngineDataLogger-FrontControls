#include <EnableInterrupt.h>

uint8_t ADCSRA_save = 0;

volatile boolean brakeStart = false;
boolean engineStarted = false;
float systemVoltage = 0;
volatile uint8_t mcpA = 0; // Output buffer for GPIOA
volatile uint8_t mcpB = 0; // Output buffer for GPIOB


const uint8_t RLY_ON = HIGH;
const uint8_t RLY_OFF = LOW;
const unsigned long blinkDelay = 350;
const unsigned long SLEEP_DELAY = 120000;
const unsigned long BRAKE_FLASH_INTERVAL = 50;

//Constants for voltage divider
const int resistor1 = 991; //997 (992 meas)
const int resistor2 = 223; //236 (218 meas)
const float denominator = (float)resistor2 / (resistor1 + resistor2);

// Vars for inputs
bool BRAKE_ON = false;
bool LEFT_ON = false;
bool RIGHT_ON = false;
bool HORN_ON = false;
bool HIGH_BEAMS_ON = false;
bool KILL_ON = false;
bool CLUTCH_DISENGAGED = false;
bool IN_NEUTRAL = false;
bool KICKSTAND_UP = false;

// CAN Constants
#define SPI_CS_PIN 10
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

// Function Prototypes
void sleepNow ();
void readSensors();
ISR(TIMER1_COMPA_vect);
void brakeLight();
void hornSound();
void Wakeup_Routine();
void allRelaysOff ();
void sleepNow ();
void myDeviceISR();


void myDeviceISR()
{
  NeoSWSerial::rxISR( *portInputRegister( digitalPinToPort( BTRX ) ) );
  // if you know the exact PIN register, you could do this:
  //    NeoSWSerial::rxISR( PIND );
}

void Wakeup_Routine()
{
  sleep_disable();
  //detachInterrupt(0);
  //detachInterrupt(1);
  disableInterrupt(0);
  disableInterrupt(1);
  disableInterrupt(CAN_INT);
  power_all_enable ();                                  // power everything back on
  ADCSRA = ADCSRA_save;
  CAN.wake();
  CAN.mcpDigitalWrite(MCP_RX0BF,LOW);
  watchdogSetup();
}

void allRelaysOff ()
{
  //Turns all relays off
 
  Wire.beginTransmission(0x20);
  Wire.write(0x12); // GPIOA
  Wire.write(0x00); // All Off
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x13); // GPIOB
  Wire.write(0x00); // All Off
  Wire.endTransmission();

}

void sleepNow ()
{
  wdt_disable();
  allRelaysOff();
  CAN.mcpDigitalWrite(MCP_RX0BF,HIGH);
  CAN.sleep();
  cli();                                                //disable interrupts
  sleep_enable ();                                      // enables the sleep bit in the mcucr register
  //attachInterrupt (0, Wakeup_Routine, RISING);          // wake up on RISING level on D2
  //attachInterrupt (1, Wakeup_Routine, RISING);          // wake up on RISING level on D2
  enableInterrupt (0, Wakeup_Routine, RISING);          // wake up on RISING level on D2
  enableInterrupt (1, Wakeup_Routine, RISING);          // wake up on RISING level on D2
  enableInterrupt(CAN_INT, Wakeup_Routine, CHANGE);

  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  ADCSRA_save = ADCSRA;
  ADCSRA = 0;                                           //disable the ADC
  sleep_bod_disable();                                  //save power
  power_all_disable ();                                 //power off ADC, Timer 0 and 1, serial
  sleep_enable();
  sei();                                                //enable interrupts
  sleep_cpu ();                                         // here the device is put to sleep
}

void doCmd()
{

  Wire.beginTransmission(0x20);
  Wire.write(0x12); // GPIOA register
  Wire.write(mcpA); // write outputs
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x13); // GPIOB register
  Wire.write(mcpB); // write outputs
  Wire.endTransmission();

}

