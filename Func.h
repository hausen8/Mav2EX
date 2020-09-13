/*
	Mav2Ex Version 2.0
	2014, by DevFor8.com, info@devfor8.com
	Functions exchanged by rOsewhite for personal preferencess
*/


//------------------ Climb Rate Calculation ----------------------------------

unsigned long last_millis = 0;
unsigned long last_update = 0;

void relAlt() {
  if (sys_armed == 0) {
    gps_home_alt = gps_abs_alt;
    gps_rel_alt = 0;
  }
  else {
    gps_rel_alt = gps_abs_alt - gps_home_alt;
  }
}

