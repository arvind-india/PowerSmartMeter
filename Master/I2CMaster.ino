#include <Wire.h>
uint8_t buffer[32];
uint32_t startTime = 0;

#define INT_PIN D2
bool dataAvailable = false;

struct totals
{
   uint32_t totalPeriodTime;
   uint32_t totalRevolutions;
};

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

totals requestTotals() {
    Wire.beginTransmission(42);
    Wire.write(8);
    boolean allesgut = Wire.endTransmission();
    Wire.requestFrom(42, 8);
    int index = 0;
    while (Wire.available()) {
        buffer[index++] = Wire.read();   
    }
    totals totalValues;
    totalValues.totalRevolutions = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
    totalValues.totalPeriodTime = (buffer[4] << 24) + (buffer[5] << 16) + (buffer[6] << 8) + buffer[7];
    return totalValues;
}

uint32_t requestActualPeriodTime() {
    Wire.beginTransmission(42);
    Wire.write(4);
    boolean allesgut = Wire.endTransmission();
    Wire.requestFrom(42, 4);
    int index = 0;
    while (Wire.available()) {
        buffer[index++] = Wire.read();   
    }
    uint32_t m = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
    return m;
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

uint8_t requestActualRunningMode() {
    Wire.beginTransmission(42);
    Wire.write(16);
    boolean allesgut = Wire.endTransmission();
    Wire.requestFrom(42, 1);
    int index = 0;
    while (Wire.available()) {
        buffer[index++] = Wire.read();   
    }
    uint8_t m = buffer[0]; 
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
    //setModeMeasureMode();
    Serial.println("actual runningMode = " + String(requestActualRunningMode()));
    startTime = millis();
}

void loop() {
    if (dataAvailable) {
        Serial.println("Received interrupt that data is available");
        dataAvailable = false; 
        Serial.println("period time: " + String(requestActualPeriodTime()));
    }
    else {
        //requestReflection();
        //delay(1000);
        if (millis() - startTime > 300000){
            Serial.println("requesting total values");
            startTime = millis();
            totals tvs = requestTotals();
            Serial.println("Total values: totalRevs = " + String(tvs.totalRevolutions) + ", totalPeriodTime = " + String(tvs.totalPeriodTime));
        }
    }
}
