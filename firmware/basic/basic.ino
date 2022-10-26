/*

    EngineDataLogger-Front Controls -- component that runs on MCU for front controls
    Copyright (C) 2018-2020  Jacob Geigle jacob@geigle.me

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/


/*    Basic Idea
 *  loop reads all input pins and checks for serial commands
 *  create command word and sensor data stream
 *  send sensor data (limit rate by time, etc)
 *  call each action function
 *  each function performs small snippet based on command word (smallest action at a time)
 *    use static vars to maintain state (eg blinking = on first time, off second time)
 *    millis() to maintain timing and synchronization
 *  brake light on interrupt
 *    flash initially, steady until low
 *  horn sounds on interrupt
 *  
 *  Pins read full register at a time and bitmasking used heavily to execute commands
 *  
 *  brake()
 *  blinkRight()
 *  blinkLeft()
 *  hornSound()
 *  enableStart()
 *  hlMode()
 *  
 */

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <mcp2515_can.h>
#include <NeoSWSerial.h>
#include <CANSerial.h>
#include "../pins.h"
#include "../wdt.h"

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
const int SPI_CS_PIN = 10;
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

CANSerial CS(0x6E0,CAN);
NeoSWSerial BTserial(BTRX,BTTX); // NAME(RX,TX)
#define MON Serial
//#define MON BTserial
//#define MON CS

void setup() {
  wdt_disable();
  Serial.begin(115200);
  // CAN Setup
  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_12MHz)) {             // init can bus : baudrate = 500k
    Serial.println("CAN ERROR");
    delay(100);
  }
  CAN.mcpPinMode(MCP_RX0BF,MCP_PIN_OUT);
  CAN.mcpDigitalWrite(MCP_RX0BF,LOW);
  Serial.println("CAN Done");
	static unsigned char UUID[6] = {0xCE, 0xF2, 0x82, 0x47, 0xEB, 0xF3};
  CS.begin(UUID,6);
  MON.println("Startup initiated!");
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

  MON.println("I2c Done");
  //allRelaysOff();

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
  


  MON.println("Startup Complete!");
  watchdogSetup();

}

void loop() {
  wdt_reset();
/*
  static unsigned long sleepWaitStart = 0;
  static boolean sleepCountdown = false;
  readSensors();
  recvCmd();
  doCmd();
  sendData();

  if (systemVoltage < 7) {
    engineStarted = false;
    if (sleepCountdown) {
      if (millis() > sleepWaitStart + SLEEP_DELAY) {
        sleepCountdown = false;
        sleepNow();
      }
    }
    else {
      sleepWaitStart = millis();
      sleepCountdown = true;
    }
  }
  else {
    sleepCountdown = false;
  }
*/
  while(Serial.available()) {
    unsigned char myByte = Serial.read();
    if (myByte == 'a') { 
      // Send CAN Packets
      static unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      stmp[3] = systemVoltage;
      CAN.sendMsgBuf(0x01, 0, 8, stmp);
    }
    else if (myByte == 'b') { 
      // Send CAN Packets
      static unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      stmp[3] = systemVoltage;
      CAN.sendMsgBuf(0x02, 0, 8, stmp);
    }
  }
  
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
  if (LEFT_ON) { // Pin 4 of PortD
    //Toggle
	mcpB ^= leftOutPin;
  }
  else {
    // - Not, turn light on and exit
	mcpB &= (leftOutPin ^ 0xFF);
  }

  //RIGHT
  // Determine if blinking or not, bitmask
  if (RIGHT_ON) { // Pin 5 of PortD
    //Toggle
	mcpB ^= rightOutPin;
  }
  else {
    // - Not, turn light on and exit
	mcpB &= (rightOutPin ^ 0xFF);
  }

  if (LEFT_ON || RIGHT_ON) { // Either left, right or both, Pin 4 or 5 of PortD
    // Enable rear lights
	mcpB |= rearOutPin;
	mcpB &= (runningOutPin ^ 0xFF);
  }
  else { // Neither
    // Disable rear lights
	mcpB &= (rearOutPin ^ 0xFF);
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


void Wakeup_Routine()
{
  sleep_disable();
  detachInterrupt(0);
  power_all_enable ();                                  // power everything back on
  ADCSRA = ADCSRA_save;
  CAN.wake();
  CAN.mcpDigitalWrite(MCP_RX0BF,LOW);
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
  allRelaysOff();
  CAN.mcpDigitalWrite(MCP_RX0BF,HIGH);
  CAN.sleep();
  cli();                                                //disable interrupts
  sleep_enable ();                                      // enables the sleep bit in the mcucr register
  attachInterrupt (0, Wakeup_Routine, RISING);          // wake up on RISING level on D2
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  ADCSRA_save = ADCSRA;
  ADCSRA = 0;                                           //disable the ADC
  sleep_bod_disable();                                  //save power                                              
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

