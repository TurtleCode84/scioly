// ===== SCRIPT =====
void runScript() {
  testDrive();
}

// ===== CONFIGURATIONS =====

#define motorSpeed 255 // works fine at 200, best at 255
const float wheelDiameter = 68.0; // mm
const float encoderSlots = 540; // # of slots in encoder wheel

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

// ===== STARTER BUTTON =====

#define starter 46 // brown
// other wire goes to GND

#include <basicMPU6050.h>
#include <Encoder.h>

Encoder frEnc(frSA, frSB);
Encoder flEnc(flSA, flSB);
Encoder blEnc(blSA, blSB);
Encoder brEnc(brSA, brSB);

void setup() {
  // Configure pin modes
  
  // gyro and encoder pin modes are handled by libraries

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

  Serial.begin(38400);

  Serial.println("waiting for button press");

  // Debounce
  while (digitalRead(starter) == HIGH) {}
  while (digitalRead(starter) == LOW) {}

  Serial.println("button pressed");
  digitalWrite(LED_BUILTIN, HIGH);
  runScript();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void testDrive() {
  Serial.println("starting test drive");

  fwd(1000);
  strLft(1000);
  bkwd(1000);
  strRgt(1000);
  rotLft(2000);
  rotRgt(2000);
  stp();
}

void fwd(int seconds) {
  Serial.print("forward for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  setMtrSpd(0);

  digitalWrite(fin1, LOW); // fr
  digitalWrite(fin2, HIGH);
  digitalWrite(fin3, LOW); // fl
  digitalWrite(fin4, HIGH);

  digitalWrite(bin1, HIGH); // bl
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, HIGH); // br
  digitalWrite(bin4, LOW);
  setMtrSpd(motorSpeed);

  delay(seconds);
}

void bkwd(int seconds) {
  Serial.print("backward for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  setMtrSpd(0);

  digitalWrite(fin1, HIGH); // fr
  digitalWrite(fin2, LOW);
  digitalWrite(fin3, HIGH); // fl
  digitalWrite(fin4, LOW);

  digitalWrite(bin1, LOW); // bl
  digitalWrite(bin2, HIGH);
  digitalWrite(bin3, LOW); // br
  digitalWrite(bin4, HIGH);
  setMtrSpd(motorSpeed);

  delay(seconds);
}

void strLft(int seconds) {
  Serial.print("strafe left for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  setMtrSpd(0);
  
  digitalWrite(fin1, LOW); // fr
  digitalWrite(fin2, HIGH);
  digitalWrite(fin3, HIGH); // fl
  digitalWrite(fin4, LOW);

  digitalWrite(bin1, HIGH); // bl
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, LOW); // br
  digitalWrite(bin4, HIGH);
  setMtrSpd(motorSpeed);

  delay(seconds);
}

void strRgt(int seconds) {
  Serial.print("strafe right for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  setMtrSpd(0);
  
  digitalWrite(fin1, HIGH); // fr
  digitalWrite(fin2, LOW);
  digitalWrite(fin3, LOW); // fl
  digitalWrite(fin4, HIGH);

  digitalWrite(bin1, LOW); // bl
  digitalWrite(bin2, HIGH);
  digitalWrite(bin3, HIGH); // br
  digitalWrite(bin4, LOW);
  setMtrSpd(motorSpeed);

  delay(seconds);
}

void rotLft(int seconds) {
  Serial.print("rotate to left for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  setMtrSpd(0);
  
  digitalWrite(fin1, LOW); // fr
  digitalWrite(fin2, HIGH);
  digitalWrite(fin3, HIGH); // fl
  digitalWrite(fin4, LOW);

  digitalWrite(bin1, LOW); // bl
  digitalWrite(bin2, HIGH);
  digitalWrite(bin3, HIGH); // br
  digitalWrite(bin4, LOW);
  setMtrSpd(motorSpeed);

  delay(seconds);
}

void rotRgt(int seconds) {
  Serial.print("rotate to right for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  setMtrSpd(0);
  
  digitalWrite(fin1, HIGH); // fr
  digitalWrite(fin2, LOW);
  digitalWrite(fin3, LOW); // fl
  digitalWrite(fin4, HIGH);

  digitalWrite(bin1, HIGH); // bl
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, LOW); // br
  digitalWrite(bin4, HIGH);
  setMtrSpd(motorSpeed);

  delay(seconds);
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

void setMtrSpd(int mtrSpd) {
  Serial.print("setting motor speed to ");
  Serial.print(mtrSpd);
  
  analogWrite(fenA, mtrSpd);
  analogWrite(fenB, mtrSpd);
  analogWrite(benA, mtrSpd);
  analogWrite(benB, mtrSpd);
}

int cmToStps(float cm) {
  int calc;
  float circ = (wheelDiameter * 3.14) / 10; // diameter converted to circ cm
  float cmPerStp = circ / encoderSlots;
  float temp = cm / cmPerStp;
  calc = (int) temp; // convert to int (not rounded)
  
  return calc;
}
