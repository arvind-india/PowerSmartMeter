#include <Wire.h>
#include <EEPROM.h>

const int irOutPin = 12;
const int ledOutPin = 13;
const int analogInPin = A0;
const int intOutPin = A3;
const int revs_per_kWh = 150;

bool storeThresholds = false;
bool prepareMeasureMode = true;
int m = 0;
uint8_t buffer[32];
uint8_t r_buffer[32];
uint8_t cmd = 0xFF;
uint32_t start = 0;
uint32_t iPeriod = 0;
float counter = 0.0;
float power = 0.0;

enum irStates {
    unknown,
    red,
    blank
};
enum irStates irState;

enum runningModes {
  stopped,
  freerunMode,
  measureMode
};
enum runningModes runningMode = stopped;

int lowerThreshold = 0;
int upperThreshold = 0;

int measureReflection() {
  digitalWrite(irOutPin, LOW);
  delay(10);
  float irValueOff = analogRead(analogInPin);
  
  digitalWrite(irOutPin, HIGH);
  delay(10);
  float irValueOn = analogRead(analogInPin);

  return (int)irValueOn - (int)irValueOff;
}

enum irStates getIrState() {
  static enum irStates lastState;

  if (lowerThreshold == upperThreshold) {
    lastState = unknown;
    return unknown;
  }

  int sample = measureReflection();
  m = sample;
  if (sample > upperThreshold) {
    digitalWrite(ledOutPin, LOW);
    lastState = blank;
    return blank;
  }
  else if (sample < lowerThreshold) {
    digitalWrite(ledOutPin, HIGH);
    lastState = red;
    return red;
  }
  else { 
    return lastState;
  }  
}

void requestEvent(void) {
  if (cmd == 1) {
    // cmd 1 requests the actual reflection value (2 Bytes)
    if (runningMode != stopped) {
      // measurements are enabled, give back last refelection value
      buffer[0] = m >> 8;
      buffer[1] = m & 0x00FF;
    }
    else {
      // measurement is stopped, give back error code 0xFFFF
      buffer[0] = 0xFF;
      buffer[1] = 0xFF;
    }
    Wire.write(buffer, 2);  
  }
  else if (cmd == 4) {
    // transmit iPeriod to master
    buffer[0] = (uint8_t)(iPeriod >> 8 >> 8 >> 8);
    buffer[1] = (uint8_t)(iPeriod >> 8 >> 8);
    buffer[2] = (uint8_t)(iPeriod >> 8);
    buffer[3] = (uint8_t)(iPeriod);
    Wire.write(buffer, 4); 
  }
}

void receiveEvent(int anzahl)
{
  int index = 0;
  if (Wire.available()) {
    cmd = Wire.read();
    if (cmd == 1) {
      // cmd == 1 = request actual reflection value
      // only set command code and wait for the data request
      return;
    }
    if (cmd == 2) {
      // cmd == 2 = set thresholds. Wait for further 4 Bytes with the 2 values for lower and upper threshold
      while (Wire.available()) {
        r_buffer[index++] = Wire.read();
      }
      if (index >= 4)
        // set flag 'store thresholds'. values will be stored in main loop
        storeThresholds = true;
      return; 
    }
    if (cmd == 4) {
      // cmd == 4 = request actual period time
      // only set command code and wait for the data request
      return;
    }
    if (cmd == 0x80) {
      runningMode = stopped;
    }
    if (cmd == 0x81) {
      runningMode = freerunMode;
    }
    if (cmd == 0x82) {
      prepareMeasureMode = true;
      runningMode = measureMode;
    }
  }
  return;
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  
  pinMode(irOutPin, OUTPUT);
  pinMode(ledOutPin, OUTPUT);
  pinMode(intOutPin, OUTPUT);
  digitalWrite(irOutPin, LOW);
  digitalWrite(ledOutPin, LOW);
  digitalWrite(intOutPin, HIGH);
  
  Wire.begin(42);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  long eepromAddr = 0;
  uint8_t buf[4];
  uint8_t i = 0;
  while (i < 4) {
    buf[i++] = EEPROM.read(eepromAddr++);
  }
  int lowThres = (buf[0]<<8) + buf[1];
  int uppThres = (buf[2]<<8) + buf[3];
  if (lowThres != lowerThreshold) lowerThreshold = lowThres;
  if (uppThres != upperThreshold) upperThreshold = uppThres;
  Serial.println("From eeprom: lowerThreshold: " + String(lowerThreshold) + ", upperThreshold: " + String(upperThreshold));      
}

void loop() {
  if (storeThresholds) {
    storeThresholds = false;
    int lowThres = (r_buffer[0]<<8) + r_buffer[1];
    int uppThres = (r_buffer[2]<<8) + r_buffer[3];
    Serial.println("lowThres: " + String(lowThres) + ", uppThres: " + String(uppThres));   
    uint8_t i = 0;
    long eepromAddr = 0;
    while(i < 4) {
      EEPROM.write(eepromAddr++, r_buffer[i++]);
    }
    Serial.println("threshold values written to eeprom");
  }

  if (runningMode == freerunMode) {
    m = measureReflection();
    //Serial.println(String(cmd) + ":" + String(m));
  }
  
  else if (runningMode == measureMode) {
    if (prepareMeasureMode) {
      irState = unknown;
      while ((irState = getIrState()) == unknown)
        ;
      if (irState == blank) {
        Serial.println("Setup(), while state.blank");
        while((irState = getIrState()) == blank)
          ;
      }
    
      Serial.println("Setup(), while state.red");
      while((irState = getIrState()) == red)
        ;
      Serial.println("Setup(): red --> blank");
      start = millis();
      prepareMeasureMode = false;
    }
    while ((irState = getIrState()) == red)
      ;
    uint32_t T = millis() - start;
    iPeriod = T;
    Serial.println("red --> blank");
    
    // send T to database...
    Serial.println("T: " + String(T));

    if (T > 100) {
        //float period = T / 1000.0;
        //counter += 1.0 / revs_per_kWh; 
        //power = 3600.0 / (revs_per_kWh * period);
        // generate interrupt for master
        digitalWrite(intOutPin, LOW);
        Serial.print("\\");
        delay(1);
        Serial.print("/");
        digitalWrite(intOutPin, HIGH);
        //Serial.println("Writing to db: counter = " + String(counter) + ", power = " + String(power));
    }
    else {
        Serial.println("ignoring first sample after startup");
    }
    
    start = millis();
    while ((irState = getIrState()) == blank) 
      ;
    Serial.println("blank --> red");
  }
  
  else {
    delay(10);
  }
}
