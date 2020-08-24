# Overview
A simple quadcopter hobby project, build from scratch in terms of both software and hardware.

# Suggested Software Setup
1) Download VS code
2) Install the PlatformIO extension (you can try to beta Arduino Extension, but you will probably have library linking issues)
3) You'll have to install the Servo library using the PlatformIO Extension (use the library panel)
4) It should build!

# Hardware Setup
## MPU-9250
 - VCC ---------------------- 3.3V
 - SDA ----------------------- A4
 - SCL ----------------------- A5
 - GND ---------------------- GND
 - 10 kOhm resistor from 3.3V to SDA
 - 10 kOhm resistor from 3.3V to SCL


Uses code from kriswiner/MPU9250 to read accelerometers, magnetometer, and gyroscopes from MPU9250 and then runs [Magdwick](https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf) (or Mahoney, depending on how I'm feeling) state estimation algorithm. Big thanks to Kris for his work there.
 
 ## Radio (FlySky FS-CT6) Setup
  TODO: Show pic

## Other Components:
- Emax MT1806-2280KV motors: https://www.banggood.com/EMAX-MT1806-KV2280-Brushless-Motor-For-Multirotor-p-933931.html?ID=521309&cur_warehouse=CN
- 5" propellers
- Arduino Uno R3 (third-party brands are fine too)
- Li-Ion Battery
