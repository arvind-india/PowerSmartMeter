#include <Wire.h>
uint8_t buffer[32];

#define INT_PIN D2
bool dataAvailable = false;

void setThresholds(int lowThres, int uppThres) {
    Wire.beginTransmission(42);
    Wire.write(2);
    Wire.write(lowThres >> 8);
    Wire.write(lowThres & 0xFF);
    Wire.write(uppThres >> 8);
    Wire.write(uppThres & 0xFF);
    boolean allesgut = Wire.endTransmission();
}

void setModeFreeRunningMode() {
    Wire.beginTransmission(42);
    Wire.write(0x81);
    boolean allesgut = Wire.endTransmission();
}

void setModeMeasureMode() {
    Wire.beginTransmission(42);
    Wire.write(0x82);
    boolean allesgut = Wire.endTransmission();
}

int requestReflection(){
    Wire.beginTransmission(42);
    Wire.write(1);
    boolean allesgut = Wire.endTransmission();
    Wire.requestFrom(42, 2);
    int index = 0;
    while (Wire.available()) {
        buffer[index++] = Wire.read();   
    }
    int m = (buffer[0] << 8) + buffer[1]; 
    Serial.println(m);
    return m;
}

void handleInterrupt() {
    dataAvailable = true;
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);

    pinMode(INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);
 
    Wire.begin(0, 2);
    delay(500);
    //setThresholds(222, 777);
    //delay(500);
    //setModeFreeRunningMode();
    setModeMeasureMode();
}

void loop() {
    if (dataAvailable) {
        Serial.println("Received interrupt that data is available");
        dataAvailable = false;
        Wire.beginTransmission(42);
        Wire.write(4);
        boolean allesgut = Wire.endTransmission();
        Wire.requestFrom(42, 4);
        int index = 0;
        while (Wire.available()) {
            buffer[index++] = Wire.read();   
        }
        uint32_t m = (buffer[0] << 8 <<8 << 8) + (buffer[1] << 8 << 8) + (buffer[2] << 8) + (buffer[3]); 
        Serial.println("period time: " + String(m));
    }
    else {
        requestReflection();
        delay(100);
    }
}
