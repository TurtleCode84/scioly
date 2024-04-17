#define r 3
#define g 4
#define b 5
#define reading A0

int concentrations[] = {0, 700, 1400, 2100, 2800, 3500, 4200, 5000}; // customize these ranges
String lights[] = { "r", "rg", "rb", "g", "gb", "b", "rgb" };

volatile int reading;
char* charArray;

void lightLED(char led[] = "") {
    int len = sizeof(led)/sizeof(led[0]);
    if (len == 0) {
        digitalWrite(r, LOW);
        digitalWrite(g, LOW);
        digitalWrite(b, LOW);
        return;
    }
    for (int i=0; i<len; i++) {
        digitalWrite(led[i], HIGH);
    }
}

void stringToCharArray(String str) {
    int arrayLength = str.length() + 1;
    char buffer[arrayLength];
    str.toCharArray(buffer, arrayLength);
    charArray = buffer;
}

void interpretReading() { // 7 possible light combos
    Serial.print("Reading:\t");
    Serial.println(reading);

    if (reading < concentrations[1]) {
        stringToCharArray(lights[0]);
    } else if (reading >= concentrations[1] && reading < concentrations[2]) {
        stringToCharArray(lights[1]);
    } else if (reading >= concentrations[2] && reading < concentrations[3]) {
        stringToCharArray(lights[2]);
    } else if (reading >= concentrations[3] && reading < concentrations[4]) {
        stringToCharArray(lights[3]);
    } else if (reading >= concentrations[4] && reading < concentrations[5]) {
        stringToCharArray(lights[4]);
    } else if (reading >= concentrations[5] && reading < concentrations[6]) {
        stringToCharArray(lights[5]);
    } else if (reading >= concentrations[6]) {
        stringToCharArray(lights[6]);
    }
    lightLED(charArray);

}

void setup() {
    pinMode(r, OUTPUT);
    pinMode(g, OUTPUT);
    pinMode(b, OUTPUT);

    Serial.begin(38400);
}

void loop() {

}