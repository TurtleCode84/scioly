// ===== DECLARATIONS =====

void drive(int stps, char dir = 'f');


// ===== SCRIPT =====

void runScript() {
  //drive(cmToStps(50), 'l');
  drive(degToStps(90), 'p');
  drive(degToStps(90), 's');
}

// ===== CONFIGURATIONS =====

#define motorSpeed 255 // works fine at 200, best at 255
#define wheelRadius 34 // mm
#define encoderPpr 540 // complete turn
//#define driveDelay 50 // pause after each action, ms

// ===== FRONT MOTOR DRIVER =====

// Front right wheel
#define fenA 6 // white
#define fin1 31 // brown
#define fin2 32 // purple
// Front left wheel
#define fin3 33 // orange
#define fin4 30 // gray
#define fenB 7 // yellow

// Back motor driver

// Back left wheel
#define benA 4 // white
#define bin1 42 // brown
#define bin2 43 // purple
// Back right wheel
#define bin3 44 // orange
#define bin4 45 // gray
#define benB 5 // yellow

// ===== ENCODERS =====

// Front right wheel
#define frSA 19 // yellow
#define frSB 38 // white
// Front left wheel
#define flSA 18 // yellow
#define flSB 37 // white
// Back left wheel
#define blSA 2 // yellow
#define blSB 26 // white
// Back right wheel
#define brSA 3 // yellow
#define brSB 25 // white

// ===== GYRO/ACCELEROMETER =====

// component uses SDA and SDL pins 20 and 21

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define mpuInterruptPin 23  // using a regular pin because no interrupt-capable pins available

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has changed
void dmpDataReady() { mpuInterrupt = true; }

// ===== STARTER BUTTON =====

#define starter 46 // brown
// other wire goes to GND

// ===== ENCODERS =====

volatile int frPos = 0;
volatile int flPos = 0;
volatile int blPos = 0;
volatile int brPos = 0;

void pulseFr() { frPos++; }
void pulseFl() { flPos++; }
void pulseBl() { blPos++; }
void pulseBr() { brPos++; }

int cmToStps(float cm) {
  int calc;
  float circ = (wheelRadius * 3.14159265) / 10; // diameter converted to circ cm
  float cmPerStp = circ / encoderPpr;
  float temp = cm / cmPerStp;
  calc = (int) temp; // convert to int (not rounded)
  
  return calc;
}

int degToStps(float deg) {
  int calc;
  float degToCm = deg / 2.857142857; // based on cm of 31.5 for 90 degrees
  calc = cmToStps(degToCm);
  
  return calc;
}

void setMtrSpd(int mtrSpd) {
  Serial.print("setting all motor speeds to ");
  Serial.println(mtrSpd);
  
  analogWrite(fenA, mtrSpd);
  analogWrite(fenB, mtrSpd);
  analogWrite(benA, mtrSpd);
  analogWrite(benB, mtrSpd);
}

void drive(int stps, char dir = 'f') {
  Serial.print(dir);
  Serial.print(" for ");
  Serial.print(stps);
  Serial.println(" encoder steps");
  setMtrSpd(0);
  frPos = 0;
  flPos = 0;
  blPos = 0;
  brPos = 0;
  Serial.println("zeroed");

  if (dir == 'f') {

    digitalWrite(fin1, LOW); // fr
    digitalWrite(fin2, HIGH);
    digitalWrite(fin3, LOW); // fl
    digitalWrite(fin4, HIGH);

    digitalWrite(bin1, HIGH); // bl
    digitalWrite(bin2, LOW);
    digitalWrite(bin3, HIGH); // br
    digitalWrite(bin4, LOW);

  } else if (dir == 'b') {

    digitalWrite(fin1, HIGH); // fr
    digitalWrite(fin2, LOW);
    digitalWrite(fin3, HIGH); // fl
    digitalWrite(fin4, LOW);

    digitalWrite(bin1, LOW); // bl
    digitalWrite(bin2, HIGH);
    digitalWrite(bin3, LOW); // br
    digitalWrite(bin4, HIGH);
    
  } else if (dir == 'r') {

    digitalWrite(fin1, HIGH); // fr
    digitalWrite(fin2, LOW);
    digitalWrite(fin3, LOW); // fl
    digitalWrite(fin4, HIGH);

    digitalWrite(bin1, LOW); // bl
    digitalWrite(bin2, HIGH);
    digitalWrite(bin3, HIGH); // br
    digitalWrite(bin4, LOW);
    
  } else if (dir == 'l') {

    digitalWrite(fin1, LOW); // fr
    digitalWrite(fin2, HIGH);
    digitalWrite(fin3, HIGH); // fl
    digitalWrite(fin4, LOW);

    digitalWrite(bin1, HIGH); // bl
    digitalWrite(bin2, LOW);
    digitalWrite(bin3, LOW); // br
    digitalWrite(bin4, HIGH);
    
  } else if (dir == 's') {

    digitalWrite(fin1, HIGH); // fr
    digitalWrite(fin2, LOW);
    digitalWrite(fin3, LOW); // fl
    digitalWrite(fin4, HIGH);

    digitalWrite(bin1, HIGH); // bl
    digitalWrite(bin2, LOW);
    digitalWrite(bin3, LOW); // br
    digitalWrite(bin4, HIGH);
    
  } else if (dir == 'p') {

    digitalWrite(fin1, LOW); // fr
    digitalWrite(fin2, HIGH);
    digitalWrite(fin3, HIGH); // fl
    digitalWrite(fin4, LOW);

    digitalWrite(bin1, LOW); // bl
    digitalWrite(bin2, HIGH);
    digitalWrite(bin3, HIGH); // br
    digitalWrite(bin4, LOW);
    
  }
  Serial.println("direction set");
  Serial.println("going");

  while (stps > frPos || stps > flPos || stps > blPos || stps > brPos) {
    if (flPos >= frPos || blPos >= frPos || brPos >= frPos) {
      analogWrite(fenA, motorSpeed);
    } else {
      analogWrite(fenA, 0);
    }
    if (frPos >= flPos && blPos >= flPos && brPos >= flPos) {
      analogWrite(fenB, motorSpeed);
    } else {
      analogWrite(fenB, 0);
    }
    if (frPos >= blPos && flPos >= blPos && brPos >= blPos) {
      analogWrite(benA, motorSpeed);
    } else {
      analogWrite(benA, 0);
    }
    if (frPos >= brPos && flPos >= brPos && blPos >= brPos) {
      analogWrite(benB, motorSpeed);
    } else {
      analogWrite(benB, 0);
    }
    Serial.println(frPos);
    Serial.println(flPos);
    Serial.println(blPos);
    Serial.println(brPos);
    Serial.println();
  }
  setMtrSpd(0);
  frPos = 0;
  flPos = 0;
  blPos = 0;
  brPos = 0;
}

void stp() {
  Serial.println("stopping");
  setMtrSpd(0);

  digitalWrite(fin1, LOW); // fr
  digitalWrite(fin2, LOW);
  digitalWrite(fin3, LOW); // fl
  digitalWrite(fin4, LOW);

  digitalWrite(bin1, LOW); // bl
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, LOW); // br
  digitalWrite(bin4, LOW);
}

void testDrive() {
  Serial.println("starting test drive");

  //drive(2454.55);
  drive(cmToStps(50));
  
  /*rotLft(2000);
  rotRgt(2000);
  stp();*/
}

void setup() {
  // Configure pin modes
  
  pinMode(fenA, OUTPUT);
  pinMode(fin1, OUTPUT);
  pinMode(fin2, OUTPUT);
  pinMode(fin3, OUTPUT);
  pinMode(fin4, OUTPUT);
  pinMode(fenB, OUTPUT);
  
  pinMode(benA, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(bin3, OUTPUT);
  pinMode(bin4, OUTPUT);
  pinMode(benB, OUTPUT);

  pinMode(starter, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  // Intialize MPU

  Serial.println("Initializing MPU...");
  mpu.initialize();
  pinMode(mpuInterruptPin, INPUT);

  Serial.println("Testing I2C connection...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println("Initializing MPU DMP...");
  devStatus = mpu.dmpInitialize();

  // gyro offsets, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for test chip
    
  if (devStatus == 0) {
      // calibration
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println("Enabling MPU DMP...");
      mpu.setDMPEnabled(true);

      attachInterrupt(INTERRUPT_PIN, dmpDataReady, CHANGE);
      mpuIntStatus = mpu.getIntStatus();

      Serial.println("MPU DMP ready! Waiting for first interrupt...");
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // error
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      Serial.print("MPU DMP initialization failed (code ");
      Serial.print(devStatus);
      Serial.println(")");
  }

  // Initialize motor states

  digitalWrite(fin1, LOW);
  digitalWrite(fin2, LOW);
  digitalWrite(fin3, LOW);
  digitalWrite(fin4, LOW);

  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, LOW);
  digitalWrite(bin4, LOW);

  // Set initial motor speeds

  setMtrSpd(motorSpeed);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(frSA), pulseFr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(flSA), pulseFl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(blSA), pulseBl, CHANGE);
  attachInterrupt(digitalPinToInterrupt(brSA), pulseBr, CHANGE);

  Serial.begin(38400);

  Serial.println("waiting for button press");

  // Debounce
  while (digitalRead(starter) == HIGH) {}
  while (digitalRead(starter) == LOW) {}

  Serial.println("button pressed");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);
  runScript();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // get the latest packet 
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("yaw:\t");
    Serial.println(ypr[2] * 180/M_PI);
  }
}
