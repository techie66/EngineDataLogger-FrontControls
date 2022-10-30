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

#include <avr/interrupt.h>
//#include <EnableInterrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Wire.h>
#include <mcp2515_can.h>
#include "MPU6050_imu.h"
#include <NeoSWSerial.h>
#include "../wdt.h"
#include "../pins.h"
#include "../functions.h"

#include <TinyGPSPlus.h>
#include "gps.h" // CAN DBC functions

TinyGPSPlus gps;
TinyGPSCustom VDOP(gps,"GPGSA", 17);
TinyGPSCustom PDOP(gps,"GPGSA", 15);
TinyGPSCustom GPSFixType(gps,"GPGSA", 2);
TinyGPSCustom SatView(gps,"GPGSV", 3);
NeoSWSerial GPSrx(BTRX,BTTX); // NAME(RX,TX)

ISR (PCINT0_vect) { // D8 -> D13
}

ISR (PCINT1_vect) { // A0 -> A5
  Wakeup_Routine();
}

ISR (PCINT2_vect) { // D0 -> D7
  NeoSWSerial::rxISR( PIND );
}

ISR (INT0_vect) { // D2
  Wakeup_Routine();
}

ISR (INT1_vect) { // D3
  Wakeup_Routine();
}


void setup() {
  wdt_disable();
  Serial.begin(9600);
  // CAN Setup
  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_12MHz)) {             // init can bus : baudrate = 500k
    delay(100);
  }
  CAN.mcpPinMode(MCP_RX0BF,MCP_PIN_OUT);
  CAN.mcpDigitalWrite(MCP_RX0BF,LOW);
  //CAN.setSleepWakeup(1);
  pinMode(CAN_INT, INPUT_PULLUP);

  GPSrx.begin(9600);

  Wire.begin(); // wake up I2C bus
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #ifdef __AVR__
  Wire.setWireTimeout(3000, true); //timeout value in uSec
  #endif

  MPU_setup();
  pinMode(BT_EN, OUTPUT);
  digitalWrite(BT_EN,HIGH);

  //enableInterrupt(BTTX, myDeviceISR, CHANGE);
  PCICR  |= B00000100; // We activate the interrupts of the PD port
  PCMSK2 |= B10000000; // We activate the interrupts on pin D7

  byte can_data[2] = {0xFF,0x00};
  CAN.sendMsgBuf(0x0DD, 0, 2, can_data);

  watchdogSetup();
}

void loop() {
  wdt_reset();
  static unsigned long sleepWaitStart = 0;
  static boolean sleepCountdown = false;

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    sleepCountdown = false;
    byte newLen;
    byte newBuf[8];
    CAN.readMsgBuf(&newLen,(byte*)&newBuf);
  }
  else {
    if (sleepCountdown) {
      if (millis() > sleepWaitStart + 10000) {
        sleepCountdown = false;
        CAN.setSleepWakeup(1);
	//disableInterrupt(BTTX);
        PCICR  &= B11111011; // Disable Interrupt of PD Port
        digitalWrite(BT_EN,LOW);
        mpu.setSleepEnabled(true);
        sleepNow();
        mpu.setSleepEnabled(false);
        digitalWrite(BT_EN,HIGH);
        //enableInterrupt(BTTX, myDeviceISR, CHANGE);
        PCICR  |= B00000100; // We activate the interrupts of the PD port
      }
    }
    else {
      sleepWaitStart = millis();
      sleepCountdown = true;
    }
  }

  unsigned char can_data[8];
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    uint16_t roll = ( (ypr[2] * 180 / M_PI) + 180 ) * 10;
    uint16_t pitch = ( (ypr[1] * 180 / M_PI) + 180 ) * 10;
    uint16_t yaw = ( (ypr[0] * 180 / M_PI) + 360 ) * 10;
    // Send CAN Packets
    can_data[0] = (uint8_t)roll;
    can_data[1] = (uint8_t)(roll >> 8) & 0x0F;
    can_data[1] |= (uint8_t)((pitch << 4) & 0xF0);
    can_data[2] = (uint8_t)(pitch >> 4);
    can_data[3] = (uint8_t)(yaw);
    can_data[4] = (uint8_t)(yaw >> 8);
    CAN.sendMsgBuf(0x1cecff80, CAN_EXTID, 5, can_data);
    for (int i=0;i<8;i++)
      can_data[i]=0;

    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    uint16_t accX = ( 1000 * 20 ) + ( aaReal.x * (long)1000 * 9.80665 ) / 16384.0;
    uint16_t accY = ( 1000 * 20 ) + ( aaReal.y * (long)1000 * 9.80665 ) / 16384.0;
    uint16_t accZ = ( 1000 * 20 ) + ( aaReal.z * (long)1000 * 9.80665 ) / 16384.0;
    // Send CAN Packets
    can_data[0] = (uint8_t)(accX);
    can_data[1] = (uint8_t)(accX >> 8);
    can_data[2] = (uint8_t)(accY);
    can_data[3] = (uint8_t)(accY >> 8);
    can_data[4] = (uint8_t)(accZ);
    can_data[5] = (uint8_t)(accZ >> 8);
    CAN.sendMsgBuf(0x1cecff81, CAN_EXTID, 6, can_data);
    for (int i=0;i<8;i++)
      can_data[i]=0;

  }
  
  // GPS stuff
  struct gps_gps_time_t st_time;
  struct gps_gps_loc_t st_loc;
  struct gps_gps_nav_t st_nav;
  struct gps_gps_stat_t st_stat;



  while (GPSrx.available()) {
    uint8_t readByte;
    readByte = GPSrx.read();
    gps.encode(readByte);
    Serial.write(readByte);
  }
  
  if ( (gps.date.month() != 0) && gps.time.isUpdated() ){
    // Send Time Frame
    st_time.day = gps.date.day();
    st_time.month = gps.date.month();
    st_time.year = gps.date.year();
    st_time.hour = gps.time.hour();
    st_time.minute = gps.time.minute();
    st_time.second = gps.time.second();
    int status = gps_gps_time_pack(can_data,&st_time,sizeof(can_data));
    if (status > 0) {
      CAN.sendMsgBuf(GPS_GPS_TIME_FRAME_ID, CAN_EXTID, 8, can_data);
      for (int i=0;i<8;i++)
        can_data[i]=0;
    }
  }

  if ( gps.location.isValid() && gps.location.isUpdated() ){
    // Send Loc Frame
    st_loc.lat_decimal_degrees = gps_gps_loc_lat_decimal_degrees_encode(gps.location.lat());
    st_loc.long_decimal_degrees = gps_gps_loc_long_decimal_degrees_encode(gps.location.lng());
    int status = gps_gps_loc_pack(can_data,&st_loc,sizeof(can_data));
    if (status > 0) {
      CAN.sendMsgBuf(GPS_GPS_LOC_FRAME_ID, CAN_EXTID, 8, can_data);
    }
    // Send Nav Frame
    st_nav.speed = gps_gps_nav_speed_encode(gps.speed.kmph());
    st_nav.heading = gps_gps_nav_heading_encode(gps.course.deg());
    st_nav.altitude = gps_gps_nav_altitude_encode(gps.altitude.meters());
    status = gps_gps_nav_pack(can_data,&st_nav,sizeof(can_data));
    if (status > 0) {
      CAN.sendMsgBuf(GPS_GPS_NAV_FRAME_ID, CAN_EXTID, 8, can_data);
      for (int i=0;i<8;i++)
        can_data[i]=0;
    }
  }

  if ( SatView.isUpdated() ) {
    st_stat.active_satellites = 0;
    st_stat.type = 0;
    st_stat.visible_satellites = 0;
    st_stat.hdop = 0;
    st_stat.pdop = 0;
    st_stat.vdop = 0;
    if ( PDOP.isValid() ) {
      String _pdop = PDOP.value();
      st_stat.pdop = gps_gps_stat_pdop_encode(_pdop.toDouble());
    }
    if ( VDOP.isValid() ) {
      String _vdop = VDOP.value();
      st_stat.vdop = gps_gps_stat_vdop_encode(_vdop.toDouble());
    }
    if ( GPSFixType.isValid() ) {
      String _type = GPSFixType.value();
      st_stat.type = _type.toInt();
    }
    if ( SatView.isValid() ) {
      String _inview = SatView.value();
      st_stat.visible_satellites = _inview.toInt();
    }
    if ( gps.satellites.isValid() )
      st_stat.active_satellites = gps.satellites.value();
    if ( gps.hdop.isValid() )
      st_stat.hdop = gps.hdop.value() / 10;
    int status = gps_gps_stat_pack(can_data,&st_stat,sizeof(can_data));
    if (status > 0) {
      CAN.sendMsgBuf(GPS_GPS_STAT_FRAME_ID, CAN_EXTID, 8, can_data);
      for (int i=0;i<8;i++)
        can_data[i]=0;
    }
  }
}

