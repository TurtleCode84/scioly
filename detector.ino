#define r 3
#define g 4
#define b 5
#define probe A0

int concentrations[] = {0, 700, 1400, 2100, 2800, 3500, 4200, 5000}; // customize these ranges
// to skip a color combo, place the same number on either side of the gap, but be careful of the beginning and end thresholds
struct lights {
  int light1[1] = {3}; // r
  int light2[2] = {3,4}; // r g
  int light3[2] = {3,5}; // r b
  int light4[1] = {4}; // g
  int light5[2] = {4,5}; // g b
  int light6[1] = {5}; // b
  int light7[3] = {3,4,5}; // rgb
};

volatile int reading;

struct lights data;

void lightLED(int led[], int len) {
    Serial.print("len:\t");
    Serial.println(len);
    if (len == 0) {
        Serial.println("Stopping lights");
        digitalWrite(r, LOW);
        digitalWrite(g, LOW);
        digitalWrite(b, LOW);
        return;
    }
    for (int i=0; i<len; i++) {
        Serial.print(led[i]);
        if (led[i] == r) {
          digitalWrite(r, HIGH);
        } else if (led[i] == g) {
          digitalWrite(g, HIGH);
        } else if (led[i] == b) {
          digitalWrite(b, HIGH);
        }
    }
}

void interpretReading() { // 7 possible light combos
    Serial.print("Reading:\t");
    Serial.println(reading);

    if (reading < concentrations[1]) {
        lightLED(data.light1, sizeof(data.light1)/sizeof(int));
    } else if (reading >= concentrations[1] && reading < concentrations[2]) {
        lightLED(data.light2, sizeof(data.light2)/sizeof(int));
    } else if (reading >= concentrations[2] && reading < concentrations[3]) {
        lightLED(data.light3, sizeof(data.light3)/sizeof(int));
    } else if (reading >= concentrations[3] && reading < concentrations[4]) {
        lightLED(data.light4, sizeof(data.light4)/sizeof(int));
    } else if (reading >= concentrations[4] && reading < concentrations[5]) {
        lightLED(data.light5, sizeof(data.light5)/sizeof(int));
    } else if (reading >= concentrations[5] && reading < concentrations[6]) {
        lightLED(data.light6, sizeof(data.light6)/sizeof(int));
    } else if (reading >= concentrations[6]) {
        lightLED(data.light7, sizeof(data.light7)/sizeof(int));
    }
}

void setup() {
    pinMode(r, OUTPUT);
    pinMode(g, OUTPUT);
    pinMode(b, OUTPUT);
    pinMode(probe, INPUT);

    Serial.begin(38400);

    reading = 3000;
    interpretReading();
}

void loop() {

}
