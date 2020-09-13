/*
	Mav2DupEx Version 0.1
	2014, by DevFor8.com, info@devfor8.com

	part of code is based on ArduCAM OSD

  2017, by Radek Voltr, voltr@voltr.eu
  removed FastSerial and few ArduCopter code dependencies
*/


#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
// #include "../GCS_MAVLink/include/mavlink/v1.0/autoquad/mavlink.h"


static bool mavlink_active;
static uint8_t crlf_count = 0;


boolean getBit(byte Reg, byte whichBit) {
  boolean State;
  State = Reg & (1 << whichBit);
  return State;
}


#pragma pack(push,1)
typedef struct _Stream_params {
  uint8_t stream;
  uint8_t rate;
}
Stream_params;


void request_mavlink_rates() {
  static const Stream_params PROGMEM MAVStreams[] = {
    {  MAV_DATA_STREAM_EXTENDED_STATUS, 4 },
    {  MAV_DATA_STREAM_EXTRA1,    4 },
    {  MAV_DATA_STREAM_EXTRA2,    4 }
  };

  for (uint8_t i=0; i < sizeof(MAVStreams)/sizeof(Stream_params); i++) {
    uint8_t rate = pgm_read_byte(&(MAVStreams[i].rate));
    mavlink_msg_request_data_stream_send(MAVLINK_COMM_0,
    mavlink_system.sysid , mavlink_system.compid ,  // received by HEARTBEAT 
    pgm_read_byte(&(MAVStreams[i].stream)),
    rate, 1);
  }
}


mavlink_status_t status;
mavlink_message_t* msg = (mavlink_message_t*)malloc(MAVLINK_MAX_PACKET_LEN);


void read_mavlink(int maxframes) {
  int current_frames = 0;
  
  if (enable_mav_request) {
    if(!mav_request_done) {         // we got HEARTBEAT packet and still don't send requests
      for(byte n=3; n>0; n--) {
        request_mavlink_rates();    // 3 times to certify it will be readed
        delay(150);
      }
      mav_request_done=1;
    }
  }

  while(mavlink_comm_0_port->available() > 0) { 
    uint8_t c = mavlink_comm_0_port->read();

    if(mavlink_parse_char(MAVLINK_COMM_0, c, msg, &status)) {
      mavlink_active = 1;
      current_frames++;

      if (mav_request_done == 0) {
        aq_mav_system    = msg->sysid;
        aq_mav_component = msg->compid;

        mavlink_system.sysid = msg->sysid;
        mavlink_system.compid = msg->compid;

        if(waitingMAVBeats == 1) {
          enable_mav_request = 1;
        }
      }

      switch(msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavbeat = 1;
          aq_mav_system    = msg->sysid;
          aq_mav_component = msg->compid;

          mavlink_system.sysid = msg->sysid;
          mavlink_system.compid = msg->compid;

          base_mode = mavlink_msg_heartbeat_get_base_mode(msg);

          if(getBit(base_mode,7)) sys_armed = 1;
          else sys_armed = 0;

          lastMAVBeat = millis();
          if(waitingMAVBeats == 1) {
            enable_mav_request = 1;
          }
        }
        break;
        
        case MAVLINK_MSG_ID_STATUSTEXT: {
          char text[MAVLINK_MSG_ID_STATUSTEXT_LEN] ;
          mavlink_msg_statustext_get_text(msg,&text[0]);
          strncpy((char*)LastMessage,text,LCDMaxPos);
        }
        break;
        
        case MAVLINK_MSG_ID_SYS_STATUS: {
          sys_vbat = (mavlink_msg_sys_status_get_voltage_battery(msg) / 100.0f);  // Battery voltage, in millivolts (1 = 1 millivolt)
        }
        break;
        
        case MAVLINK_MSG_ID_GPS_RAW_INT: {
          gps_lat_org = mavlink_msg_gps_raw_int_get_lat(msg);                     // store the orignal data
          gps_lon_org = mavlink_msg_gps_raw_int_get_lon(msg);
          gps_lat = gps_lat_org / 10000000.0f;
          gps_lon = gps_lon_org / 10000000.0f;
          gps_hdop = mavlink_msg_gps_raw_int_get_eph(msg);                        // in centimeters so multiplication not needed
          gps_vdop = mavlink_msg_gps_raw_int_get_epv(msg);                        // in centimeters so multiplication not needed
          gps_fix_type = mavlink_msg_gps_raw_int_get_fix_type(msg);               // 0 = No GPS, 1 = No Fix, 2 = 2D Fix, 3 = 3D Fix, 4: DGPS, 5: RTK
          gps_abs_alt = mavlink_msg_gps_raw_int_get_alt(msg) / 100.0f;            // Altitude (AMSL), in meters * 1000 (positive for up)
          gps_vel = mavlink_msg_gps_raw_int_get_vel(msg) / 100.0f;                // Speed in m/s
        }
        break;
        
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
          ukf_posd = mavlink_msg_local_position_ned_get_z(msg) *-10;              // UKF Altitude (AMSL), in meters
          ukf_veld = mavlink_msg_local_position_ned_get_vz(msg) *100;             // UKF vertical velocity (Vario)
        }
        break;
        
        default: {
        }
        break;
      }
    }
    if (current_frames>maxframes) {
      break; // we need time for Jeti
    }
  }
}
