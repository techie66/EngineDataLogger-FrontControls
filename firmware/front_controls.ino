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
#include "wdt.h"
#include "pins.h"
#include "functions.h"


uint8_t inputCmdD; // pins 0-7
uint8_t inputCmdB; // pins 8-13 (two high bits unusable) (all outputs)
uint8_t inputCmdC; // analog pins A0-A5 (two high bits unuseable ATMega328p)
uint8_t serialCmdD = 0;
uint8_t serialCmdB = 0; // Extra Input, used for overrides D
uint8_t serialCmdC = 0;
uint8_t serialCmdA = 0; // Extra Input, used for overrides C
uint8_t receivedCmdD;
uint8_t receivedCmdC;
bool powerOn = false;
NeoSWSerial BTSerial(BTTX, BTRX); // RX, TX (TX->RX)
uint16_t yaw;
const uint8_t mainOutPin = 6;
const uint8_t auxOutPin = A3;

#define BTPOWER 1
#define FC_CMD_ID 0x226
#define IMU_POS_ID 0x1CECFF80

/********OUTPUTS***************
Left
Right
Horn
hlHigh
hlLow
Brake
startenable
rear
inverted rear (running lights?)
******************************/
/********INPUTS****************
Brake
Horn
Left
Right
hlHigh
Kill
clutch
neutral
voltage
******************************/

const uint8_t CMD_SIZE = 4;  // WIP see doCmd()
const uint16_t SERIAL_OUT_RATE = 50;
const unsigned long SERIAL_EXPIRE = 4000;
const unsigned long POWER_DOWN_DELAY = 2000;

// Constants for Cmd flags
const uint8_t ENGINE_RUNNING = B10000000;       // serialCmdA

//ADD MORE HERE

void recvCmd() {
  // read CMD_SIZE command from serial(USB)
  // TODO start and stop chars, length?
  static uint8_t receivedByte[(CMD_SIZE+1)];
  static uint8_t numBytesRecv = (CMD_SIZE+1);
  unsigned long currentMillis = millis();
  static unsigned long lastMillis = 0;
  boolean goodRead = false;

  if (currentMillis > lastMillis + SERIAL_EXPIRE) {
    serialCmdD = 0;
    serialCmdB = 0;
    serialCmdC = 0;
    serialCmdA = 0;
    lastMillis = currentMillis;
  }

  while (Serial.available() > 0) {
    if (numBytesRecv < CMD_SIZE) {
      receivedByte[numBytesRecv] = Serial.read();
      //Serial.print("ReadByte ");
      //Serial.println(numBytesRecv);
      numBytesRecv++;
    }
    else if (numBytesRecv == CMD_SIZE) {
      if ((char)Serial.read() == 'Z') {
        // End Character. Presumably good data
        goodRead = true;
        numBytesRecv++;
        //Serial.println("GoodRead");
        break;
      }
      else {
        //Serial.println("Not a Z");
        //numBytesRecv++;
      }
    }
    else if (numBytesRecv > CMD_SIZE) {
      if ((char)Serial.read() == 'A') {
        // Start Character
        //Serial.println("Start");
        numBytesRecv = 0;
      }
    }
  }

  if (goodRead) {
    // Written for CMD_SIZE = 4
    
    serialCmdD = receivedByte[0]; // Equivalent to PIND
    serialCmdB = receivedByte[1]; // Overrides for PIND
    serialCmdC = receivedByte[2]; // Equivalent to PINC
    serialCmdA = receivedByte[3]; // Overrides for PINC
    // zero out local vars
    //numBytesRecv = 0;
    goodRead = false;
    for (uint8_t i = 0; i < (CMD_SIZE+1); i++ ) {
      receivedByte[i] = 0;
    }
    // Set Time of last received CMD
    lastMillis = currentMillis;
  }
  
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    static unsigned long CanId=0;
    byte CanLen;
    byte CanBuf[8];
    CAN.readMsgBuf(&CanLen,(byte*)&CanBuf);
    CanId = CAN.getCanId();
    if ( CanId == FC_CMD_ID ) {
      serialCmdA = CanBuf[3]; // Overrides for PINC
      lastMillis = currentMillis;
    }
    if ( CanId == 0x7DF ) { // OBD2 request
      // check for 3501
    }
    if (CanId == IMU_POS_ID ) {
      yaw = (CanBuf[3] + ((CanBuf[4]&0x001F)<<8))/10;
    }
  }
  // This is an override condition. The logic is that system voltages above 14V
  //  indicate the engine is running, and we'd like to act accordingly
  //  i.e. keep headlights on even if the CAN bus stops working correctly.
  if (systemVoltage > 14) {serialCmdA |= ENGINE_RUNNING;}
  if (serialCmdA & ENGINE_RUNNING) {
    engineStarted = true;
  }
}

void sendData() {
  static uint8_t lastCmdD;
  static uint8_t lastCmdC;
  static float lastVoltage;
  unsigned long currentMillis = millis();
  static unsigned long lastMillis = 0;

  if ((inputCmdD == lastCmdD) && (inputCmdC == lastCmdC) && (systemVoltage == lastVoltage) && (currentMillis < (lastMillis + SERIAL_EXPIRE))) {
    //Nothing, everything is the same, no need to repeat ourselves
  }
  else if (currentMillis < (lastMillis + SERIAL_OUT_RATE)) {
    //Nothing, limit output rate
  }
  else {
	// Send CAN Packets
	static unsigned char stmp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	stmp[0] = inputCmdD;
	stmp[1] = inputCmdC;
	stmp[2] = uint16_t(systemVoltage*100);
	stmp[3] = uint16_t(systemVoltage*100) >> 8;
	CAN.sendMsgBuf(0x260, 0, 4, stmp);
    // Send sensor/state data over serial(USB)
    Serial.write(inputCmdD);
    Serial.write(inputCmdC);
    Serial.write((byte *) &systemVoltage, 4);
    Serial.println("");
    
    // Hack to output ASCII version
/*    uint8_t copyD = receivedCmdD;
    uint8_t copyC = receivedCmdC;
  
    Serial.println(systemVoltage);
    Serial.print("receivedCmdD:");
    for(int i=0;i<8;i++) {
      if(copyD&B10000000){
        Serial.print("1");
      }
      else {
        Serial.print("0");
      }
      copyD = copyD << 1;
    }
    Serial.println("");
    Serial.print("receivedCmdC:");
    for(int i=0;i<8;i++) {
      if(copyC&B10000000){
        Serial.print("1");
      }
      else {
        Serial.print("0");
      }
      copyC = copyC << 1;
    }
    Serial.println("");
    */
    
  
  lastCmdD = inputCmdD;
  lastCmdC = inputCmdC;
  lastVoltage = systemVoltage;
  lastMillis = currentMillis;
  }
}

void enableStart() {
  /*
   * Enables / Disables ignition on
   * when disabled, only off or acc
   * when on able to turn on ignition
   * only able to leave ignition on state when enabled
   * 
   * Input: neutral, clutch, kickstand, (engine running), kill switch
   */
#ifndef BTPOWER
   // If either clutch or neutral switches are grounded (engine off or on)
   if (  ((IN_NEUTRAL || CLUTCH_DISENGAGED) && (KILL_ON) ) {
     // enable
	 mcpB |= startEnableOutPin;
   }
   // if kickstand up (engine off)
   else if ( (KICKSTAND_UP) && !(serialCmdA & ENGINE_RUNNING) && (KILL_ON) ) {
     // enable
	 mcpB |= startEnableOutPin;
   }
   else if ( engineStarted && !(KILL_ON) && (IN_NEUTRAL || CLUTCH_DISENGAGED)) {
   //else if (!(receivedCmdD & KILL_ON)) {
	 mcpB |= startEnableOutPin;
   }
   // Otherwise
   else {
     // disable
	 mcpB &= (startEnableOutPin ^ 0xFF);
   }
#else
   // TODO
   // startenable is now the actual starter
   // killOutPin

	if ( (KILL_ON) && powerOn ) {

		// If either clutch, neutral, or kickstand switches are grounded (engine off or on)
		if ( KICKSTAND_UP ) {
		// enable
			mcpB |= killOutPin;
		}
		else if ( IN_NEUTRAL ) {
		// enable
			mcpB |= killOutPin;
		}
		// Otherwise
		else {
		// disable
			mcpB &= (killOutPin ^ 0xFF);
		}
	}
	else {
	// disable
		mcpB &= (killOutPin ^ 0xFF);
	}
#endif
}

void hlMode() {
  /*
   * Control two relays for headlights
   * one relay is high, other is low beams
   */
   if ( (KILL_ON) && (HIGH_BEAMS_ON) && (serialCmdA & ENGINE_RUNNING) ) {
     // high beam switch on, kill switch on, engine running
	 mcpB |= hlhighOutPin;
	 mcpB &= (hllowOutPin ^ 0xFF);
   }
   else if (serialCmdA & ENGINE_RUNNING) {
	 mcpB |= hllowOutPin;
	 mcpB &= (hlhighOutPin ^ 0xFF);
   }
   else {
	 mcpB &= (hlhighOutPin ^ 0xFF);
	 mcpB &= (hllowOutPin ^ 0xFF);
   }
}

void mainPower() {
  static unsigned long powerOffTimer = 0;
  static boolean powerOffBegin = false;
  if (BTConnected) {
    digitalWrite(mainOutPin, HIGH);
    digitalWrite(auxOutPin, HIGH);
    powerOffBegin = false;
  }
  else if (!BTConnected && !(serialCmdA & ENGINE_RUNNING) ) {
    if (powerOffBegin) {
      if (millis() > (powerOffTimer + POWER_DOWN_DELAY) ) {
        digitalWrite(mainOutPin, LOW);
        digitalWrite(auxOutPin, LOW);
      }
    }
    else {
      powerOffBegin = true;
      powerOffTimer = millis();
    }
  }
}

void doMyCmd() {// FIX
  // execute the command
  //TODO figure out priority of serial vs GPIO commands
  /*  On if either On
   *   Use extra cmd space to override off
   *   two bytes to OR with input
   *   two bytes to XOR mask resulting command set
   *   extra bits are status flags (eg. serialCmdA B11000000 and serialCmdB B00000011)
   */
  // Hack to pretend that engine is running
  //serialCmdA |= B10000000;
  
  receivedCmdD = inputCmdD | serialCmdD;
  //receivedCmdB = inputCmdB | serialCmdB; // overrides, Don't need
  receivedCmdC = inputCmdC | serialCmdC;

  /* Enable Overrides
   *  receivedCmdD ^= serialCmdB;  // not complete
   *  receivedCmdC ^= serialCmdA;  // not complete
   */
#ifdef BTPOWER
  mainPower();
#endif
  hornSound();
  hlMode();
  enableStart();
  brakeLight();

  doCmd(); // Writes to GPIO expander
}
uint16_t diffYaw(uint16_t start, uint16_t end) {
  uint16_t diff = start - end;
  diff = abs((diff + 180)%360) -180;
  return diff;
}

void autoCancelBlinkers() {
  static int startYaw;
  static bool leftBlinkStart = false, rightBlinkStart = false;
  static bool leftOverridden = false, rightOverriden = false;
  if ( LEFT_ON ) {
    if ( leftBlinkStart ) {
      if ( diffYaw(startYaw,yaw) > 50 ) {
        leftOverridden = true;
      }
      if ( leftOverridden ) {
        LEFT_ON = false;
      }
    }
    else {
      leftBlinkStart = true;
      startYaw = yaw;
    }
  }
  else { // LEFT-OFF
    leftOverridden = false;
    leftBlinkStart = false;
  }
}

void setup() {
  Serial.begin(115200);
  common_setup(); // functions.h
  /*
  BTSerial.begin(9600);
  BTSerial.println("AT+SLEEP\r\n");
  */

  //allRelaysOff();

  pinMode(mainOutPin, OUTPUT);
  pinMode(auxOutPin, OUTPUT);

  //Serial.println("Startup Complete!");
}

void loop() {
  wdt_reset();
  static unsigned long sleepWaitStart = 0;
  static boolean sleepCountdown = false;
  readSensors();
  recvCmd();
  autoCancelBlinkers();
  doMyCmd();
  sendData();

  if (systemVoltage < 7) {
    engineStarted = false;
    powerOn = false;
    if (sleepCountdown) {
      if (millis() > sleepWaitStart + SLEEP_DELAY) {
        sleepCountdown = false;
        unsigned char stmp[2] = {0, 0xFF};
        CAN.sendMsgBuf(0xDC, 0, 2, stmp);
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
    powerOn = true;
  }

  /*static bool oneshot=false;
  if (!BTConnected && oneshot) {
    BTSerial.write("AT+SLEEP\r\n");
    oneshot=false;
  }
  if (BTConnected) {
    oneshot=true;
  }
  */
  /*if (BTSerial.available()) {
    String BTString = BTSerial.readStringUntil('\n');
    unsigned char stmp[10] = {0,0,0,0,0,0,0,0,0,0};
    BTString.toCharArray(stmp,9);
    CAN.sendMsgBuf(0xAA,0,8,stmp);
  }*/
}

