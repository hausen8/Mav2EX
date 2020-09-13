/*
  Mav2DupEx Version 1.0
  2014, by DevFor8.com, info@devfor8.com

  part of code is based on ArduCAM OSD
  Bugfixes and some enhancements added by MTBSTEVE

  2017, by Radek Voltr, voltr@voltr.eu, version 2.0
  memory optimization, new jeti sensor wire up, fastserial removal, Mega32u4 compatibility
  
  2018, minimized version by rOsewhite
*/


#include "disableserial.h"
#include <FastSerial.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "SoftwareSerialO2.h"
#include "JETI_EX_SENSOR.h"
#include <GCS_MAVLink.h>

#define MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN 10

#include "Vars.h"
#include "Func.h"

#define GETCHAR_TIMEOUT_ms 100  // 100ms instead of 20ms to let code time to hit fastserial readings

#ifndef JETI_RX
#define JETI_RX 10
#endif

#define TELEMETRY_SPEED  115200  // How fast our MAVLink telemetry is coming to Serial port, 57600 for sure, 115200 better

const unsigned char Jeti_SensorID3 = 0x02;
const unsigned char Jeti_SensorID4 = 0x02;

JETI_Box_class JB;


void JetiboxISR() {
  if (mavlink_comm_0_port->available() >= MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN ) //read one MAV frame between each JB value
    read_mavlink(1);
}


int delayMAV(int _delay) {
  int read = 0;
  unsigned long wait_till = millis() + _delay;
  while (millis() <  wait_till) {
    if (mavlink_comm_0_port->available() >= MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN) {
      read_mavlink(2);
    }
  }
  return read;
}


#if defined(__AVR_ATmega328P__)
  FastSerialPort(FSerial,0);
#else 
  // Mega32U4
  FastSerialPort(FSerial,1);
#endif


BetterStream  *mavlink_comm_0_port;
BetterStream  *mavlink_comm_1_port;
mavlink_system_t mavlink_system; //modified

    
void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  FSerial.begin(TELEMETRY_SPEED,512,8);
  mavlink_comm_0_port = &FSerial;

  pinMode(JETI_RX, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);

  JB.Init(F("Mav2EX"), JETI_RX, 9700);
  JB.FrameISR = JetiboxISR;

  JB.setValue6(JB.addData(F("Armed"), F("")), &sys_armed, 0);
  JB.setValue14(JB.addData(F("Batt V"), F("V")), &sys_vbat, 1);
  JB.setValueGPS(JB.addData( F("Latitude"), F("")), &gps_lat, false);
  JB.setValueGPS(JB.addData( F("Longitude"), F("")), &gps_lon, true);
  JB.setValue14(JB.addData(F("HDOP"), F("m")), &gps_hdop, 2);
  JB.setValue14(JB.addData(F("VDOP"), F("m")), &gps_vdop, 2);
  JB.setValue6(JB.addData(F("GPS Fix"), F("D")), &gps_fix_type, 0);
  JB.setValue14(JB.addData(F("Rel Alt"), F("m")), &gps_rel_alt, 1);
  JB.setValue14(JB.addData(F("Abs Alt"), F("m")), &gps_abs_alt, 1);
  // JB.setValue14(JB.addData(F("UKF AMSL"), F("m")), &ukf_posd, 1);
  JB.setValue14(JB.addData(F("Vario"), F("m/s")), &ukf_veld, 2);
  JB.setValue14(JB.addData(F("Speed"), F("m/s")), &gps_vel, 2);

  JB.SendFrame();
  delayMAV(GETCHAR_TIMEOUT_ms);

  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
}


void loop() {
  relAlt();   // calculate relative altitude from GPS alt

  JB.txMode();
  JB.SendFrame();

  int read = delayMAV (GETCHAR_TIMEOUT_ms);
}


