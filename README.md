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
## Controller - Calculating Desired Thrust
Several different controllers were tried out (PD, LQR, and PID). For the moment, I'm trying out the PID. Note that I only present equations about the pitch axis, because the roll axis is symmetric.

We first calculate the pitch and roll errors (be careful to not mix radians and degress!):
- `pitch_error = desired_pitch` (calculated from radio receiver)` - actual_pitch` (calculated from MPU9250)\
- `pitch_rate_error = desired_pitch_rate` (0)` - actual_pitch_rate` (calculated from MPU9250)

And then we keep an estimate of the integral of the pitch and roll errors using a simple rectangular integral estimate:
- `pitch_error_integral = pitch_error_integral + (pitch_error * delta_time)` (where `delta_time` is the time since the last control cycle was ran)

But one issue is integrator windup, where small errors can build up over time to create massive control inputs. To alleviate this problem, I do 2 things - 1) [constrain](https://www.arduino.cc/reference/en/language/functions/math/constrain/) the integrated value to always lie within a certain bound, and 2) add a slow decay term (by multiplying by 0.999 each time):
- `pitch_error_integral = constrain(pitch_error_integral*0.999 + (pitch_error * delta_time), -0.25, 0.25)` (where `delta_time` is the time since the last control cycle was ran)

Then, using the PID control formula, we calculate the desired moment around the roll and pitch axes.

`moment_pitch = (pitch_error * k_P) + (pitch_rate_error * k_D) + (pitch_error_integral * k_I)`

## Mixer - Calculating Propeller Commands from Desired Thrust
First, we need to translate the desired moment around the pitch axis into force requirements from the motor:
- `force_pitch = moment_pitch / (dist * n_motors)`, where `dist` is the distance from the center to each rotor (~95mm) and `n_motors` is the number of motors that can act around that axis (which is 4).

The "force" required for stability is not the only force required from the motor - the motor must also supply enough force to keep the quadcopter aloft. The controller here is not responsible for calculating the force needed to stay aloft, that is left to the operator via the left joystick. Thus, we must "mix" the force from the left joystick with the "delta force" required for stability, to get a final control value to send to the motor:

- `val_motor = f(force_loft + force_stability)`.
In practice, we know the value required for loft (`val_loft`), since that is given by the operator via the left throttle, and the force required for stability using the equations above:
- `f_stability = (+/-)force_pitch + (+/-)force_roll`, where the `+/-` depends on which motor we're talking about in relation to the given axis.

Using the Arduino Servo library, the brushless motors can be commanded with values from 0 (not moving) to 180 (full speed ~15kRPM). In reality, the motor typically starts spinning when a value of ~40 is written to it. To model the thrust of the propeller, we assume the propeller provides full thrust (360g or 3.5N) at full speed and provides 0 thrust at an input of 40 and below. A first-approximation of propeller dynamics teaches us that the thrust provided by a propeller varies quadratically with shaft speed. 

To provide a continuous, non-zero differentiable function (which is needed for the controller), we use a piece-wise function that models the thrust as linear from 0-40 (with a low slope) and then quadratic from 40 to 180. (TODO: add more details)

