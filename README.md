# 1DoF-helicopter
This art installation is a 1 DoF helicoper using an Arduino Nano to controll a Brushless DC motor to lift the system to a desired angle based on the data provided by a BMI160 low power inertial measurement unit.

![Image](https://github.com/user-attachments/assets/5c7bec05-c591-4370-b35b-d1a1f6707b7e)

Hardware:
- Arduino Nano microcontroller
- BMI160 low power inertial measurement unit
- RS2205 2300KV Brushless dc motor
- PCEONAMP 3000mAh 7.4V battery
- XIDA XXD 30A ESP
- 10k potentiometer
- 2 way switch
- Buzzer
- Leds
- Breadboard
- Metal hinge
- some Lego, hardwood, screws and wires

The breadboard with the Arduino, IMU and other circuit components are based on one side of the hinge, while the battery and motor is on the other end of the system ensuring balance. Applying thrust for the BLDC motor causes the system with all electronics to rotate and applying less or even no thrust at all removes the torque and lets the system return to its resting position, which is about vertical. The angle range of the system is about 70 degrees.

Libraries used:
- Fast IMU https://github.com/LiquidCGS/FastIMU 
- Wire
- Servo

![Image](https://github.com/user-attachments/assets/739eadff-0c4c-452a-8cca-ac990488ee5b)

With the flip of the switch the system boots up.
Bootup sequence: Esc calibration, BMI160 calibration, ledcalibration

After the ~5 second calibration the system chooses a desired angle and adds thrust to the motor for it to lift up the system to the desired angle
The thrust is a pwm signal with a frequency between 1000ms and 2000ms, this signal is fed into the ESC, which has the battery, motor and the circuit connected. The esc uses the pwm signal and the battery voltage to power the bldc motor, but also provides 5V power for the circuit.
The amount of thurst is calculated realtime from the data of the BMI160 IMU with filtering of the gyro and accelerometer. The data logs the current angle and thust of our system and chooses a base thrust from the thrust table based on linear interpolation to the desired angle. After the base thust is chosen it is fed into a PID system, that adds or removes the correct amount of thrust for the system to stabilize in the desired angle.

With the potentiometer the user can set the amount of time intervals when a new desired angle is choosen randomly from the table of possible desired angles.

The leds light up when the desired angle is reached.

The serial monitor is used to monitor the system with the angle, thrust and PID values.

The project help me understand electronics, kinematic systems, BLDC motor controll using a microcontroller, PID controlling and IMUS






