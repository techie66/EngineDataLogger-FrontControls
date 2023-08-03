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
#include "../functions.h"

CANSerial CS(0x6E0,CAN);
NeoSWSerial BTserial(BTRX,BTTX); // NAME(RX,TX)
#define MON Serial
//#define MON BTserial
//#define MON CS

void setup() {
  Serial.begin(115200);
  static unsigned char UUID[6] = {0xCE, 0xF2, 0x82, 0x47, 0xEB, 0xF3};
  CS.begin(UUID,6);
  //allRelaysOff();

  common_setup(); // functions.h

  MON.println("Startup Complete!");
}

void loop() {
  wdt_reset();
  if(Serial.available()) {
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
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    static unsigned long newID=0;
    byte newLen;
    byte newBuf[8] = {0};
    CAN.readMsgBuf(&newLen,(byte*)&newBuf);
    newID = CAN.getCanId();
    char s[80];
    if ( newID > 0xFFF ) {
      snprintf(s,80,"%04X%04X : %02X %02X %02X %02X %02X %02X %02X %02X\n\r",(int)(newID/0x10000),(int)newID,newBuf[0],newBuf[1],newBuf[2],newBuf[3],newBuf[4],newBuf[5],newBuf[6],newBuf[7]);
    }
    else {
      snprintf(s,80,"%03X : %02X %02X %02X %02X %02X %02X %02X %02X\n\r",(int)newID,newBuf[0],newBuf[1],newBuf[2],newBuf[3],newBuf[4],newBuf[5],newBuf[6],newBuf[7]);
    }
    MON.print(s);

  } 
}





