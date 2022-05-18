# cubecell-ogn-tracker
OGN-Tracker implementation on the HELTEC CubeCell 6502 with GPS

## Functionality
As of now the OGN transmission and relaying are implemented however not really well tested thus approach with caution.
Serial console prints GPS NMEA and setting parameters is possible via $POGNS sentence,
however there is an issue with characters being lost thus this needs to be addressed.

## Compile/upload
This project is compiled with PlatformIO and requires a forked HELTEC library
modified to enable OGN transmission and reception https://github.com/pjalocha/CubeCell-Arduino
This library should be placed under <i>lib</i> sub-folder of the project.

I include the commands which worked for me on Linux, it might be a little different on Windows

Install PlatformIO

```
sudo apt-get install python3-venv
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
```

Clone the repository code
```
git clone https://github.com/pjalocha/cubecell-ogn-tracker.git
cd cubecell-ogn-tracker
```

Clone the HELTEC library hacked for additional RF options
```
mkdir lib
cd lib
git clone https://github.com/pjalocha/CubeCell-Arduino.git
cd ..
```

Compile and upload
```
~/.platformio/penv/bin/pio run --target upload
```
