/*
  APM2EX Version 1.0 
  based on Mav2DupEx Version 0.1 2014, by DevFor8.com, info@devfor8.com

  schiwo1@gmail.com March 2015
  part of code is based on ArduCAM OSD
*/


#include <Arduino.h>

static short                base_mode=0;
static volatile int8_t      sys_armed = 0;
static volatile int16_t     sys_vbat = 0;                   // Battery voltage in millivolt

static float                gps_lat = 0.00f;                // latidude (i.e. -48.600000f)
static float                gps_lon = 0.00f;                // longitude
static long                 gps_lat_org = 0;                // stores the received coordinate in its original high resolution form (i.e. -486000000)
static long                 gps_lon_org = 0;
static volatile int16_t     gps_hdop = 0;                   // HDOP value 
static volatile int16_t     gps_vdop = 0;                   // VDOP value 
static uint8_t              gps_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
static volatile int16_t     gps_abs_alt = 0;                // absolute altitude (AMSL) in m
static uint16_t             gps_vel = 0;                    // Speed in m/s
static volatile int16_t     gps_rel_alt = 0;                // relative altitude (calculated in func.h)
static volatile int16_t     gps_home_alt = 0;               // home altitude (set when disarmed)

static volatile int16_t     ukf_posd = 0;                   // AMSL from UKF
static volatile int16_t     ukf_veld = 0;                   // vertical speed from UKF

// MAVLink session control
static boolean              mavbeat = 0;
static float                lastMAVBeat = 0;
static boolean              waitingMAVBeats = 1;
static uint8_t              aq_mav_type;
static int8_t               aq_mav_system; 
static uint8_t              aq_mav_component;
static boolean              enable_mav_request = 0;
static boolean              mav_request_done = 0;
static char                 LastMessage[LCDMaxPos];

