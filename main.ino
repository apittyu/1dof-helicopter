#include "FastIMU.h"
#include <Wire.h>
#include <Servo.h>

#define IMU_ADDRESS 0x69
#define PERFORM_CALIBRATION
BMI160 IMU;
Servo prop;

calData calib = { 0 };
AccelData accelData;
GyroData gyroData;
MagData magData;


// thrust table
const int ffTable[][2] = {

    {1100, 0},
    {1150, 1.12},
    {1300, 3},
    {1400, 3.4},
    {1450, 6.20},
    {1500, 11.34},
    {1550, 16.58},
    {1600, 22.97},
    {1650, 30.72},
    {1700, 38.31},
    {1750, 49},
    {1800, 60},
    {1900, 65}
};

const int ledPin2 = 6;  // LED on D6
const int ledPin1 = 5;  // LED on D5
const int ledPin3 = 3;
const int ledPin4 = 4;
const int buzzer = 7;
const int potpin = A3;
int pot = 800;

unsigned long timerStart;
unsigned long timerDuration;
bool eventTriggered = false;

// Timing variables
float elapsedTime, time, timePrev;

// PID Variables
float PID, pwmLeft, error, previous_error;
float pid_p = 0, pid_i = 0, pid_d = 0;

// PID Gains (adjusted for stability and authority)
double kp = 4.5;
double ki = 0.01;
double kd = 4.3;
double multiplicator = 13;

// Control Parameters
const float desired_angle = 25;  // Target position (0-0.5 scale)
const int neutralThrottle =  100; // ESC neutral position

// Angle calculation
float currentAngle = 0.3;                // Initial angle estimate
const float rad_to_deg = 180/3.141592654;
const float alpha = 0.98;         // Complementary filter coefficient

float currentDesiredAngle = 25.0; // Initial target
unsigned long lastAngleChangeTime = 0;

int getDynamicNeutral(float targetAngle) {
  for (int i = 0; i < sizeof(ffTable)/sizeof(ffTable[0])-1; i++) {
    if (targetAngle >= ffTable[i][1] && targetAngle <= ffTable[i+1][1]) {
        // Linear interpolation
      float ratio = (targetAngle - ffTable[i][1]) / (ffTable[i+1][1] - ffTable[i][1]);
      return ffTable[i][0] + ratio * (ffTable[i+1][0] - ffTable[i][0]);
    }
  }
  return ffTable[0][0]; // Fallback

}

int dynamicNeutral = getDynamicNeutral(currentDesiredAngle);


float newTargetAngle()
 {
 

  return random(30, 401) / 10.0;
}
void setup() {
  
  pinMode(ledPin1, OUTPUT);  
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin4, OUTPUT);  
  pinMode(ledPin3, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(potpin, INPUT);
  
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  while (!Serial);
  // Initialize IMU
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true);
  }

  #ifdef PERFORM_CALIBRATION
  Serial.println("Calibrating IMU - keep device level");
   
  prop.attach(9, 1000, 2000);
  
  // ESC Calibration Sequence
  
  digitalWrite(ledPin1, HIGH);
  digitalWrite(ledPin2, HIGH);
  prop.write(0);  
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration complete!");
  delay(3000);
  IMU.init(calib, IMU_ADDRESS);
  #endif
  digitalWrite(ledPin3, HIGH);
  digitalWrite(ledPin4, HIGH);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  delay(1000);
  
  delay(1000);
  digitalWrite(ledPin3, LOW);
  digitalWrite(ledPin4, LOW);
  time = millis();
  resetTimer();
}



void loop() {
  digitalWrite(ledPin3, HIGH);
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  // Calculate elapsed time
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000.0;

   // Complementary filter for angle estimation
  float accelAngle = atan2(accelData.accelY, accelData.accelZ) * rad_to_deg;
  float gyroRate = gyroData.gyroY; // Already in deg/s
  currentAngle = alpha * (currentAngle + gyroRate * elapsedTime) + (1-alpha) * accelAngle;
  

  // potmeteres okoskodas
  pot = analogRead(potpin);
  timerDuration = map(pot, 0, 1023, 1000, 150000);
 unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - timerStart;

  if (!eventTriggered) {
    float progress = min((float)elapsed / timerDuration, 1.0);
    analogWrite(ledPin3, progress * 255);  // LED gets brighter over time

    // Timer complete - trigger event
    if (elapsed >= timerDuration) {
      triggerEvent();
    }
  }
  if (eventTriggered && (currentTime - timerStart > timerDuration + 2000)) {
    resetTimer();
  }
  static unsigned long lastAngleChange = 0;
 
  


   error = currentDesiredAngle - currentAngle;
  
  

   // Anti-windup (only integrate near target)
  if (abs(error) < 10.0) {
    pid_i += ki * error * elapsedTime;
    pid_i = constrain(pid_i, -200, 200);
  }

  // PID Terms
  pid_p = kp * error;
  pid_d = kd * (error - previous_error) / elapsedTime;
  PID = pid_p + pid_i + pid_d;
 
  int pwm = dynamicNeutral + PID;
  pwm = constrain(pwm, dynamicNeutral - 200, dynamicNeutral + 200); // Limit PID influence
  pwm = constrain(pwm, 1000, 1900); // Absolute ESC limits

  tone(buzzer, pwm - pot);
  
  prop.writeMicroseconds(pwm);
  // Store previous error
  previous_error = error;

  // Debug output
  Serial.print("Angle:"); Serial.print(currentAngle, 3);
  Serial.print(" Error:"); Serial.print(error, 3);
  Serial.print(" PID:"); Serial.print(PID, 1);
  Serial.print(" PWM:"); Serial.print(pwm);
  Serial.print(" P:"); Serial.print(pid_p, 1);
  Serial.print(" I:"); Serial.print(pid_i, 3);
  Serial.print(" D:"); Serial.print(pid_d, 1);
  Serial.println();
  if  (error > -5 && error < 5){
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, LOW);
    
  }
  else {
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, HIGH);
  }



}

void triggerEvent() {
  eventTriggered = true;
  digitalWrite(ledPin4, HIGH);    // Bright flash
  analogWrite(ledPin3, 255);  // Full brightness
  
  currentDesiredAngle = newTargetAngle(); // Get new angle
    dynamicNeutral = getDynamicNeutral(currentDesiredAngle);
    pid_i = 0; // Reset integral term
    

    Serial.print("New target: ");
    Serial.print(currentDesiredAngle);
    Serial.print("°, Neutral PWM: ");
    Serial.println(dynamicNeutral);
    
}

void resetTimer() {
  timerStart = millis();
  eventTriggered = false;
  digitalWrite(ledPin4, LOW);
  analogWrite(ledPin3, 0);
  Serial.println("Timer reset");
}
