# cubecell-ogn-tracker
OGN-Tracker implementation on the HELTEC CubeCell 6502 with GPS

## Functionality
As of now the OGN transmission and relaying are implemented however not really well tested thus approach with caution.
Serial console prints GPS NMEA and setting parameters is possible via $POGNS sentence,
however there is an issue with characters being lost thus this needs to be addressed.

## Compile/upload
This project is compiled with platformio and requires a forked HELTEC library
modified to enable OGN transmission and reception https://github.com/pjalocha/CubeCell-Arduino
This library should be placed under <i>lib</i> sub-folder of the project.

