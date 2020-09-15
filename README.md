# mav2ex
Display Mavlink telemetry values on your Jeti radio

Minimized version, optimized for AutoQuad flight controllers.

Required Libraries:
FastSerialLeo and GCS_MAVLink from https://github.com/DevFor8/Mav2Duplex

Transmitted values:
- Armed/disarmed
- Battery voltage
- GPS latitude
- GPS longitude
- GPS HDOP in 0.01 meters
- GPS VDOP in 0,01 meters
- GPS fix type
- Relative altitude in 0.1 meters
- Absolute altitude in 0.1 meters
- Vario in meters per second
- GPS speed in meters per second
