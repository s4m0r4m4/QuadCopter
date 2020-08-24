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
- Max Trust: 360-460g (depending on propeller)
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

# Control Analysis
## TBD - Calculating Desired Thrust

## Propeller Thrust
Using the Arduino Servo library, the brushless motors can be commanded with values from 0 (not moving) to 180 (full speed ~15kRPM). In reality, the motor typically starts spinning when a value of ~40 is written to it. To model the thrust of the propeller, we assume the propeller provides full thrust (360g/3.5N) at full speed and provides 0 thrust at an input of 40. The thrust provided by a propeller varies quadratically with shaft speed. 

To provide a continuous, non-zero differentiable function (which is needed for the controller), we use a piece-wise function that models the thrust as linear from 0-40 (with a low slope) and then quadratic from 40 to 180. (TODO: add more details)

## Controller
Several different controllers were tried out (PD, LQR, and PID). For the moment, I'm trying out the PID more. 

We first calculate the pitch and roll errors (be careful to not mix radians and degress!):
- `pitch_error = desired_pitch` (calculated from radio receiver)` - actual_pitch` (calculated from MPU9250)
- `roll_error = desired_roll` (calculated from radio receiver)` - actual_roll` (calculated from MPU9250)
- `pitch_rate_error = desired_pitch_rate` (0)` - actual_pitch_rate` (calculated from MPU9250)
- `roll_rate_error = desired_roll_rate` (0)` - actual_roll_rate` (calculated from MPU9250)

And then we keep an estimate of the integral of the pitch and roll errors using a simple rectangular integral estimate:
- `pitch_error_integral = pitch_error_integral + (pitch_error * delta_time)` (where `delta_time` is the time since the last control cycle was ran)

But one issue is integrator windup, where small errors can build up over time to create massive control inputs. To alleviate this problem, I do 2 things - 1) constrain the integrated value to always lie within a certain bound, and 2) add a slow decay term (by multiplying by 0.999 each time):
- `pitch_error_integral = constrain(pitch_error_integral*0.999 + (pitch_error * delta_time), -0.25, 0.25)` (where `delta_time` is the time since the last control cycle was ran)
