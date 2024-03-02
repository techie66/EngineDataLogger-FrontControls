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
#include "ATCommands.h"

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
bool monitor_canbus = false;
bool igUnlocked = false;
NeoSWSerial BTSerial(BTTX, BTRX); // RX, TX (TX->RX)
int yaw;
const uint8_t mainOutPin = 6;
const uint8_t auxOutPin = A3;

#define BTPOWER 1
//#define FR_DEBUG
#define FC_CMD_ID 0x226
#define UNLOCK_ID 0x267
#define IMU_POS_ID 0x1CECFF80
#define OBD_ADDRESS 0x7EA

#define AT_WORKING_BUFFER_SIZE 255

ATCommands AT;

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

/**
 * @brief recvCmd
 * Read incoming commands and respond/act accordingly.
 *   Originally written for commands over serial, this now
 *   handles getting CAN packets and responding to requests.
 *   Does not handle commands from bluetooth serial. (ATCommands)
 *   Other CAN packets (ie periodic) are sent elsewhere
 *   see: at_sh_write(), sendData(), AT.update()
 * @param none
 * @return void
*/
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
    byte canReadRet = CAN.readMsgBuf(&CanLen,(byte*)&CanBuf);
    CanId = CAN.getCanId();
#ifdef FR_DEBUG
    unsigned char stmp[3] = {0x11, 0x22};
    stmp[0] = canReadRet;
    stmp[1] = CanId >> 8;
    stmp[2] = CanId;
    CAN.sendMsgBuf(0x2AA, 0, 3, stmp);
#endif
    if ( CanId == FC_CMD_ID ) {
      serialCmdA = CanBuf[3]; // Overrides for PINC
      lastMillis = currentMillis;
    }
    if ( CanId == 0x7DF ) { // OBD2 request
      uint8_t obd_service = CanBuf[1] & B00111111;
      uint8_t obd_pid = CanBuf[2];
      if ( obd_service == 0x35 ) {
        if ( obd_pid == 1 ) {
          byte _response[8] = {0};
          _response[0] = 4;
          _response[1] = obd_service + 0x40;
          _response[2] = obd_pid;
          _response[3] = 0;
          _response[4] = LEFT_ON | RIGHT_ON << 1 | HIGH_BEAMS_ON << 2;
          // Purposely decided not to forward this packet to UART in monitor mode
          CAN.sendMsgBuf(OBD_ADDRESS, 0, 8, _response);
        }
      }
    }
    if (CanId == IMU_POS_ID ) {
      yaw = ((CanBuf[3] + (((int)CanBuf[4]&0x001F)<<8))/10) - 360;
    }

    if (CanId == UNLOCK_ID ) {
      if (CanBuf[0] == 0x13 && CanBuf[1] == 0xAF && CanBuf[2] == 0xB3 && CanBuf[3] == 0x22
          && CanBuf[4] == 0x87 && CanBuf[5] == 0x8E && CanBuf[6] == 0xBC && CanBuf[7] == 0x1F) {
        igUnlocked = true;
      } else {
        igUnlocked = false;
      }
    }

    if (monitor_canbus) {
      char bt_can_buf[10];
      snprintf(bt_can_buf, sizeof(bt_can_buf), "%.3lx", CanId);
      BTSerial.print(bt_can_buf);
      BTSerial.print(" ");
      snprintf(bt_can_buf, sizeof(bt_can_buf), "%.2x", CanLen);
      BTSerial.print(bt_can_buf);
      BTSerial.print(" ");
      for (int i=0;i<CanLen;i++) {
        BTSerial.print(byteToASCIIHEX(bt_can_buf, sizeof(bt_can_buf), CanBuf[i]));
      }
      BTSerial.println();
    }
  }
  // This is an override condition. The logic is that system voltages above 14V
  //  indicate the engine is running, and we'd like to act accordingly
  //  i.e. keep headlights on even if the CAN bus stops working correctly.
  if (systemVoltage > 14) {
    serialCmdA |= ENGINE_RUNNING;
    lastMillis = currentMillis;
  }
  if (serialCmdA & ENGINE_RUNNING) {
    engineStarted = true;
  }
}

uint8_t convert_to_inputCmdD(){
  uint8_t value = 0;
  value |= BRAKE_ON << 2;
  value |= HORN_ON << 3;
  value |= LEFT_ON << 4;
  value |= RIGHT_ON << 5;
  value |= HIGH_BEAMS_ON << 6;
  value |= KILL_ON << 7;
  return value;
}

uint8_t convert_to_inputCmdC(){
  uint8_t value = 0;
  value |= CLUTCH_DISENGAGED << 2;
  value |= KICKSTAND_UP << 3;
  value |= IN_NEUTRAL << 4;
  return value;
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
  if (monitor_canbus) {
    char bt_can_buf[10];
    BTSerial.print("260 04 ");
    for (int i=0;i<4;i++) {
      BTSerial.print(byteToASCIIHEX(bt_can_buf, sizeof(bt_can_buf), stmp[i]));
    }
    BTSerial.println();
  }

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
   if (  ((IN_NEUTRAL || CLUTCH_DISENGAGED) && (KILL_ON) )) {
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
   // possible use for remote start or similair
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
  if (BTConnected && igUnlocked) {
    digitalWrite(mainOutPin, HIGH);
    digitalWrite(auxOutPin, HIGH);
    powerOffBegin = false;
  }
  else if ((!BTConnected && !(serialCmdA & ENGINE_RUNNING) ) || !igUnlocked) {
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

void doMyCmd() {
  // Hack to pretend that engine is running
  //serialCmdA |= B10000000;
  
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
  int diff = start - end;
  diff = abs((abs(diff) + 180)%360 -180);
  return diff;
}

void autoCancelBlinkers() {
  static int startYaw;
  static bool leftBlinkStart = false, rightBlinkStart = false;

  if ( LEFT_ON && RIGHT_ON ) {
    return;
  }

  // LEFT
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
  // RIGHT
  if ( RIGHT_ON ) {
    if ( rightBlinkStart ) {
      if ( diffYaw(startYaw,yaw) > 50 ) {
        rightOverridden = true;
      }
      if ( rightOverridden ) {
        RIGHT_ON = false;
      }
    }
    else {
      rightBlinkStart = true;
      startYaw = yaw;
    }
  }
  else { // LEFT-OFF
    rightOverridden = false;
    rightBlinkStart = false;
  }
  return;
}

/**
 * @brief at_reset
 * handle ATZ and other commands by returning OK\r>
 * @param sender
 * @return true
 * @return false
*/
bool at_reset(ATCommands *sender)
{
  sender->serial->println("OK");
  sender->serial->println(">");
  return true;
}

/**
 * @brief at_ma
 * Put device into CAN monitor Mode
 *   While in monitor mode, all CAN packets are forwarded from
 *   CAN->UART(recvCmd()). Any input on UART cancels this mode (loop())
 * @param sender
 * @return true
 * @return false
*/
bool at_ma(ATCommands *sender)
{
  monitor_canbus = true;
  return true;
}

/**
 * @brief at_sh_write
 * Forward CAN packet from UART->CAN
 *   <atsh> gives the ID and waits for confirmation before sending data.
 *   Data is variable length
 * @param sender
 * @return true
 * @return false
*/
bool at_sh_write(ATCommands *sender)
{
  String strCanId = sender->next();
  unsigned long canId = 0;
  unsigned int canDLC = 0;
  canId = strtoul(strCanId.c_str(),NULL,16);

  sender->serial->println("OK");
  sender->serial->println(">");

  while (!sender->serial->available()) {}

  String strCanData = sender->serial->readStringUntil('\r');
  unsigned int canDataBegin = 0;
  int canDataEnd = 0;
  byte canData[8];
  for (unsigned int i=0;i<sizeof(canData);i++) {
    if (canDataBegin < strCanData.length()) {
      canDataEnd = strCanData.indexOf(' ',canDataBegin);
    } else break;
    if (canDataEnd < 0 && canDataBegin < strCanData.length() ) {
      canDataEnd = strCanData.length();
    }
    if (canDataEnd > 0 ) {
      unsigned long convTemp = strtoul(strCanData.substring(canDataBegin,canDataEnd).c_str(),NULL,16);
      if ( convTemp < 256 ) {
        canData[i] = convTemp;
        canDLC = i + 1;
      }
      canDataBegin = canDataEnd+1;
    } else break;
  }
  bool canExt = canId > 0x000007FF ? 1:0;
  CAN.sendMsgBuf(canId,canExt,canDLC,canData);
  sender->serial->println("OK");
  sender->serial->println(">");
  return true;
}

// declare the commands in an array to be passed during initialization
/**
****Ignored, but responded to****
atz     Initialization
stp31   Set CAN format to ISO 11898, 11-bit Tx, 500kbps, var DLC
stcmm1  Acknowledge CAN packets in monitor mode
ath1    Enable headers on UART
ate0    Disable echo on UART
****Processed****
stma    Go into monitor mode. Forward all CAN packets on UART
stop    Used to stop monitor mode
atsh    UART->CAN sets header, followed by data bytes
*/
static at_command_t commands[] = {
  {"z", at_reset, NULL, NULL, NULL},
  {"p31", at_reset, NULL, NULL, NULL},
  {"cmm1", at_reset, NULL, NULL, NULL},
  {"h1", at_reset, NULL, NULL, NULL},
  {"e0", at_reset, NULL, NULL, NULL},
  {"ma", at_ma, NULL, NULL, NULL},
  {"op", at_reset, NULL, NULL, NULL},
  {"sh", NULL, NULL, NULL, at_sh_write}, // atsh 'CANID'
};

ISR(INT0_vect) {
  brakeStart = true;
  mcpB |= brakeOutPin;
}

ISR(INT1_vect) {
  // Nothing to do really...
}

void setup() {
  Serial.begin(115200);
  common_setup(); // functions.h

  BTSerial.begin(9600);

  //allRelaysOff();

  pinMode(mainOutPin, OUTPUT);
  pinMode(auxOutPin, OUTPUT);

  AT.begin(&BTSerial, commands, sizeof(commands), AT_WORKING_BUFFER_SIZE,"\r");
  unsigned char stmp[2] = {0x11, 0x11};
  CAN.sendMsgBuf(0xDA, 0, 1, stmp);
  Serial.println("Startup Complete!");
}

void loop() {
  wdt_reset();
  static unsigned long sleepWaitStart = 0;
  static boolean sleepCountdown = false;
  readSensors();
  autoCancelBlinkers();

  if (BTSerial.available()) {
    monitor_canbus = false;
    AT_COMMANDS_ERRORS at_ret = AT.update();
    if (at_ret != AT_COMMANDS_SUCCESS) {
      CAN.sendMsgBuf(0x0BB, 0, 1, (byte*)&at_ret);
    }
  }
  recvCmd();
  inputCmdC = convert_to_inputCmdC();
  inputCmdD = convert_to_inputCmdD();
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
        delay(20); // allow CAN msg to send
        sleepNow();
        Wakeup_Routine();
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
}

