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
#include "../pins.h"
#include <SoftwareSerial.h>

#include <TinyGPSPlus.h>
#include "gps.h" // CAN DBC functions

uint8_t ADCSRA_save = 0;

// CAN Constants
const int SPI_CS_PIN = 10;
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin

// Function Prototypes
void sleepNow ();
void Wakeup_Routine();

TinyGPSPlus gps;
TinyGPSCustom VDOP(gps,"GPGSA", 17);
TinyGPSCustom PDOP(gps,"GPGSA", 15);
TinyGPSCustom GPSFixType(gps,"GPGSA", 2);
TinyGPSCustom SatView(gps,"GPGSV", 3);
SoftwareSerial GPSrx(BTRX,BTTX); // NAME(RX,TX)
//#define GPSrx Serial
#define MON Serial

void setup() {
  Serial.begin(115200);
  // CAN Setup
  while (CAN_OK != CAN.begin(CAN_500KBPS,MCP_12MHz)) {             // init can bus : baudrate = 500k
	  MON.println("CAN ERROR");
      delay(100);
  }
  CAN.mcpPinMode(MCP_RX0BF,MCP_PIN_OUT);
  MON.println("CAN Done");

  GPSrx.begin(9600);
  MON.println("Startup initiated!");
  Wire.begin(); // wake up I2C bus
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  MON.println("I2c Done");

  MON.println("Startup Complete!");

}

void loop() {
  struct gps_gps_time_t st_time;
  struct gps_gps_loc_t st_loc;
  struct gps_gps_nav_t st_nav;
  struct gps_gps_stat_t st_stat;

  unsigned char can_data[8] = {0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC};

  while (GPSrx.available()) {
    uint8_t readByte;
    readByte = GPSrx.read();
    gps.encode(readByte);
    MON.write(readByte);
  }

  if ( gps.time.isValid() && gps.time.isUpdated() ){
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
    }
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

void sleepNow ()
{
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

