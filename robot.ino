// ===== CONFIGURATIONS =====

#define motorSpeed 255 // works fine at 200, best at 255

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

  // Set motor speeds

  analogWrite(fenA, motorSpeed);
  analogWrite(fenB, motorSpeed);
  analogWrite(benA, motorSpeed);
  analogWrite(benB, motorSpeed);

  Serial.begin(38400);

  Serial.println("waiting for button press");

  // Debounce
  while (digitalRead(starter) == HIGH) {}
  while (digitalRead(starter) == LOW) {}

  Serial.println("button pressed");
  digitalWrite(LED_BUILTIN, HIGH);
  testDrive();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void testDrive() {
  Serial.println("driving");

  // forward
  
  digitalWrite(fin1, LOW); // front right
  digitalWrite(fin2, HIGH);
  digitalWrite(fin3, LOW); // front left
  digitalWrite(fin4, HIGH);

  digitalWrite(bin1, HIGH); // back left
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, HIGH); // back right
  digitalWrite(bin4, LOW);

  delay(5000);

  // strafe left

  digitalWrite(fin1, LOW); // front right
  digitalWrite(fin2, HIGH);
  digitalWrite(fin3, HIGH); // front left
  digitalWrite(fin4, LOW);

  digitalWrite(bin1, HIGH); // back left
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, LOW); // back right
  digitalWrite(bin4, HIGH);

  delay(5000);
  
  // forward
  
  digitalWrite(fin1, LOW); // front right
  digitalWrite(fin2, HIGH);
  digitalWrite(fin3, LOW); // front left
  digitalWrite(fin4, HIGH);

  digitalWrite(bin1, HIGH); // back left
  digitalWrite(bin2, LOW);
  digitalWrite(bin3, HIGH); // back right
  digitalWrite(bin4, LOW);

}
