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
  
## Emax MT1806-2280KV Motors:
- https://www.banggood.com/EMAX-MT1806-KV2280-Brushless-Motor-For-Multirotor-p-933931.html?ID=521309&cur_warehouse=CN
- Max Trust: 460g
-	No. of cell: 2-3S
-	Framework: 12N14P
-	Propeller: 5" ~ 6"
-	Length: 26.7mm
-	Shaft: 2mm
- Motor Casing	Diameter: 23mm
-	Weight: 18g

## Battery: 
- http://www.getfpv.com/lumenier-1500mah-3s-45c-lipo-battery.html 
-	Length	88 mm
-	Width	35 mm
-	Height	22 mm
-	Weight	125g
-	C-Rating	45c - Burst 90c
-	Voltage	3s, 11.1v
-	Connector	XT60

## Other Components:
- 5030 propellers: https://www.amazon.com/gp/product/B00Y4QNSFA/ref=oh_aui_detailpage_o00_s01?ie=UTF8&psc=1 
- Arduino Uno R3 (third-party brands are fine too): https://www.amazon.com/gp/product/B00P2FX9WY/ref=oh_aui_detailpage_o00_s03?ie=UTF8&psc=1 
- Frame: FCMODEL X4 M280L Wheelbase Glass Fiber Across Mini Quadcopter Frame Kit DIY RC Multicopter FPV Drone: https://www.amazon.com/gp/product/B014KNMKF0/ref=oh_aui_detailpage_o04_s00?ie=UTF8&psc=1
- Support Legs: https://www.amazon.com/gp/product/B00QHVVKEG/ref=ox_sc_sfl_title_1?ie=UTF8&psc=1&smid=A3Y26MC43SQCQ 
- MPU9250: https://www.amazon.com/Mpu-9250-Nine-axis-Attitude-Acceleration-Magnetic/dp/B00OPNUO9U/ref=pd_cart_rp_2_3?_encoding=UTF8&psc=1&refRID=BYG3BGN2TGJM5B6HS7HY 
