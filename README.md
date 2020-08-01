These docs need lots of work :)

Long story short - this code is for a 4 rotor quadcopter that uses the MPU-9250 chip for state estimation, and a FlySky FS-CT6B radio for control inputs.

This is my hobby project, it is still very much in work!


# Suggested Software Setup
1) Download VS code
2) Insteall the PlatformIO extension (you can try to beta Arduino Extension, but you will probably have library linking issues)
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


