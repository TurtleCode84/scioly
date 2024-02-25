// ===== DECLARATIONS =====

void drive(int stps, char dir = 'f');


// ===== SCRIPT =====

void runScript() {
  drive(cmToStps(50), 'l');
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

#include <basicMPU6050.h>

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

/*#include <Encoder.h>

Encoder frEnc(frSA, frSB);
Encoder flEnc(flSA, flSB);
Encoder blEnc(blSA, blSB);
Encoder brEnc(brSA, brSB);*/

int cmToStps(float cm) {
  int calc;
  float circ = (wheelRadius * 3.14159265) / 10; // diameter converted to circ cm
  float cmPerStp = circ / encoderPpr;
  float temp = cm / cmPerStp;
  calc = (int) temp; // convert to int (not rounded)
  
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

/*void rotLft(int seconds) {
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
}*/

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
  // put your main code here, to run repeatedly:

}
