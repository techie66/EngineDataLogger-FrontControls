//#include <EnableInterrupt.h>

uint8_t ADCSRA_save = 0;

bool BTConnected = false;
volatile boolean brakeStart = false;
boolean engineStarted = false;
float systemVoltage = 0;
volatile uint8_t mcpA = 0; // Output buffer for GPIOA
volatile uint8_t mcpB = 0; // Output buffer for GPIOB
volatile bool leftOverridden = false, rightOverridden = false;


const unsigned long BT_INTERVAL = 1000;
const uint8_t RLY_ON = HIGH;
const uint8_t RLY_OFF = LOW;
const unsigned long blinkDelay = 350;
const unsigned long SLEEP_DELAY = 120000;
const unsigned long BRAKE_FLASH_INTERVAL = 40;

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
  EIMSK &= B11111100;       // Disable INT0 and INT1
  PCICR &= B11111101;       // Disable Interrupt of PC Port
  power_all_enable ();      // power everything back on
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
  watchdogDisable();
  allRelaysOff();
  CAN.mcpDigitalWrite(MCP_RX0BF,HIGH);
  CAN.sleep();
  delay(50);
  cli();                                //disable interrupts
  sleep_enable ();                      // enables the sleep bit in the mcucr register
  EIMSK  |= B00000011; // Enable INT0 and INT1
  EICRA  |= B00001011; // Set RISING on INT0 and FALLING on INT1
  PCICR  |= B00000010; // We activate the interrupts of the PC port
  PCMSK1 |= B00000010; // We activate the interrupts on pin A1
  PCIFR  |= B00000111; // Clear all Pin Change Interrupt Flags
  EIFR   |= (1<<INTF0); // Clear Interrupt Flag for INT0
  EIFR   |= (1<<INTF1); // Clear Interrupt Flag for INT1

  ADCSRA_save = ADCSRA;
  ADCSRA = 0;                           //disable the ADC
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  power_all_disable ();                 //power off ADC, Timer 0 and 1, serial
  sleep_bod_disable();                  //save power
  sei();                                //enable interrupts
  sleep_cpu ();                         // here the device is put to sleep
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

void readSensors()
{
  byte inputs=0;
  systemVoltage = ( analogRead(voltageInPin));
  systemVoltage = (systemVoltage / 1024) * 5.0;
  systemVoltage = systemVoltage / denominator;

  // Convert inputs to standard commands

  Wire.beginTransmission(0x20);
  Wire.write(0x12); // GPIOA
  Wire.endTransmission();
  Wire.requestFrom(0x20, 1);
  inputs=Wire.read();

  CLUTCH_DISENGAGED = (inputs & clutchInPin);
  KICKSTAND_UP = ( inputs & kickInPin );
  IN_NEUTRAL = ( inputs & neutralInPin );
  HORN_ON = ( inputs & hornInPin );
  HIGH_BEAMS_ON = ( inputs & hlhighInPin );
  KILL_ON = ( inputs & killInPin );
  if (inputs & B10000000 ) {
    //in btstate
  }
  BRAKE_ON = digitalRead(brakeInPin);
  LEFT_ON = digitalRead(leftInPin);
  RIGHT_ON = digitalRead(rightInPin);

  static unsigned long btLastSeen = 0;
  if (inputs & BTStatePin ) {
    BTConnected = true;
    btLastSeen = millis();
  }
  else if ( millis() - btLastSeen > BT_INTERVAL ) {
    BTConnected = false;
  }
}



ISR(TIMER1_COMPA_vect){
  // Interrupt called when time to toggle flashers
  
  // Hazards
  static bool initiate = true;
  if (RIGHT_ON && LEFT_ON) {
	  if (initiate) {
		  mcpB |= leftOutPin;
		  mcpB |= rightOutPin;
		  initiate = false;
	  }
  }
  else {
	  initiate = true;
  }


  //LEFT
  // Determine if blinking or not, bitmask
  if (LEFT_ON && !leftOverridden) { // Pin 4 of PortD
    //Toggle
	mcpB ^= leftOutPin;
  }
  else {
    // - Not, turn light on and exit
	mcpB &= (leftOutPin ^ 0xFF);
  }

  //RIGHT
  // Determine if blinking or not, bitmask
  if (RIGHT_ON && !rightOverridden) { // Pin 5 of PortD
    //Toggle
	mcpB ^= rightOutPin;
  }
  else {
    // - Not, turn light on and exit
	mcpB &= (rightOutPin ^ 0xFF);
  }

  if (LEFT_ON || RIGHT_ON) { // Either left, right or both, Pin 4 or 5 of PortD
    // Enable rear lights
#ifndef BTPOWER
	mcpB |= rearOutPin;
#endif
	mcpB &= (runningOutPin ^ 0xFF);
  }
  else { // Neither
    // Disable rear lights
#ifndef BTPOWER
	mcpB &= (rearOutPin ^ 0xFF);
#endif
	mcpB |= runningOutPin;
  }
}

void brakeLight() {
  /*
   * Initial turn on flashes before going steady until brake is released
   */
  static int i = 0;
  static bool brakeEngaged = false;
  static unsigned long brakeDelayStart = millis();
  unsigned long currentMillis = millis();
  
  if (brakeEngaged) {
    brakeStart=false;
  }
  
  if (BRAKE_ON) {
    if (brakeStart) {
      /*
       * if--else (i<14) construct prevents from tying up processor
       * while waiting for light to flash
       */
      
      if (i < 14) { // while loop would block for almost a second
        if (currentMillis > ( brakeDelayStart + BRAKE_FLASH_INTERVAL )) {
		  mcpB ^= brakeOutPin;
          brakeDelayStart = currentMillis;
          i++;
        }
      }
      else {
		mcpB |= brakeOutPin;
        brakeStart = false;
      }
      
    }
    else {
      mcpB |= brakeOutPin;
      brakeEngaged = true;
    }
  }
  else {
	mcpB &= (brakeOutPin ^ 0xFF);
    i = 0;
    brakeEngaged = false;
    brakeStart=true;
  }
}

void hornSound() {
  if (HORN_ON) {
	mcpA |= hornOutPin;
  }
  else {
	mcpA &= (hornOutPin ^ 0xFF);
  }
}

void common_setup()
{
  watchdogDisable();
  // CAN Setup
  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_12MHz)) {             // init can bus : baudrate = 500k
    delay(100);
  }
  CAN.mcpPinMode(MCP_RX0BF,MCP_PIN_OUT);
  CAN.mcpDigitalWrite(MCP_RX0BF,LOW);
  Wire.begin(); // wake up I2C bus
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #ifdef __AVR__
  Wire.setWireTimeout(3000, true); //timeout value in uSec
  #endif

  Wire.beginTransmission(0x20);
  Wire.write(0x00); // IODIRA register
  Wire.write(0xFE); // set pin 0 of port A to output, 1-7 to input
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x01); // IODIRB register
  Wire.write(0x00); // set all of port B to outputs
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x04); // GPINTENA
  Wire.write(0xFE); // 1-7 enable interrupt
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x02); // IPOLA
  Wire.write(0x0E); // Pins 1-3 Inverted Polarity
  Wire.endTransmission();

  Wire.beginTransmission(0x20);
  Wire.write(0x0C); // GPPUA
  Wire.write(0x0E); // Pins 1-3 Pullup Enabled
  Wire.endTransmission();

  //set timer1 interrupt at 3Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 3hz increments
  OCR1A = 5208;// = (16*10^6) / (3*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  pinMode(brakeInPin, INPUT);
  pinMode(leftInPin, INPUT);
  pinMode(rightInPin, INPUT);
  pinMode(voltageInPin, INPUT); // Analog for Voltage
  
  watchdogSetup();
}
