#include <Wire.h>
#include <EEPROM.h>

const int irOutPin =    12;
const int ledOutPin =   13;
const int analogInPin = A0;
const int intOutPin =   A3;
//        SDA =         A4;
//        SCL =         A5;

bool      storeThresholds = false;
bool      prepareMeasureMode = true;
uint8_t   buffer[32];
uint8_t   r_buffer[32];
uint8_t   cmd = 0xFF;
uint16_t  lowerThreshold = 0;
uint16_t  upperThreshold = 0;
uint16_t  m = 0;
uint32_t  start = 0;
uint32_t  iPeriod = 0;
uint32_t  countsSinceLastQuery = 0;
uint32_t  periodTimeSinceLastQuery = 0;
float     counter = 0.0;
float     power = 0.0;

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
enum runningModes runningMode = measureMode;

uint16_t measureReflection() {
  digitalWrite(irOutPin, LOW);
  delay(10);
  float irValueOff = analogRead(analogInPin);
  
  digitalWrite(irOutPin, HIGH);
  delay(10);
  float irValueOn = analogRead(analogInPin);

  return (uint16_t)irValueOn - (uint16_t)irValueOff;
}

enum irStates getIrState() {
  static enum irStates lastState;

  if (lowerThreshold == upperThreshold) {
    lastState = unknown;
    return unknown;
  }

  uint16_t sample = measureReflection();
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
  if (cmd == 0x01) {
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
  else if (cmd == 0x04) {
    // transmit iPeriod to master
    buffer[0] = (uint8_t)(iPeriod >> 24);
    buffer[1] = (uint8_t)(iPeriod >> 16);
    buffer[2] = (uint8_t)(iPeriod >> 8);
    buffer[3] = (uint8_t)(iPeriod);
    Wire.write(buffer, 4); 
  }
  else if (cmd == 0x08) {
    buffer[0] = (uint8_t)(countsSinceLastQuery >> 24);
    buffer[1] = (uint8_t)(countsSinceLastQuery >> 16);
    buffer[2] = (uint8_t)(countsSinceLastQuery >> 8);
    buffer[3] = (uint8_t)(countsSinceLastQuery);
    buffer[4] = (uint8_t)(periodTimeSinceLastQuery >> 24);
    buffer[5] = (uint8_t)(periodTimeSinceLastQuery >> 16);
    buffer[6] = (uint8_t)(periodTimeSinceLastQuery >> 8);
    buffer[7] = (uint8_t)(periodTimeSinceLastQuery);
    countsSinceLastQuery = 0;
    periodTimeSinceLastQuery = 0;
    Wire.write(buffer, 8);
  }
  else if (cmd == 0x10) {
    // cmd 16 requests the actual running mode (1 Byte)
    buffer[0] = (uint8_t)runningMode;
    Wire.write(buffer, 1);  
  }
}

void receiveEvent(int anzahl)
{
  uint8_t index = 0;
  if (Wire.available()) {
    cmd = Wire.read();
    if (cmd == 0x01) {
      // cmd == 1 = request actual reflection value
      // only set command code and wait for the data request
      return;
    }
    if (cmd == 0x02) {
      // cmd == 2 = set thresholds. Wait for further 4 Bytes with the 2 values for lower and upper threshold
      while (Wire.available()) {
        r_buffer[index++] = Wire.read();
      }
      if (index >= 4)
        // set flag 'store thresholds'. values will be stored in main loop
        storeThresholds = true;
      return; 
    }
    if (cmd == 0x04) {
      // cmd == 4 = request actual period time
      // only set command code and wait for the data request
      return;
    }
    if (cmd == 0x08) {
      // cmd == 8 = request totals since last request
      // only set command code and wait for the data request
      return;
    }
    if (cmd == 0x10) {
      // cmd == 8 = request actual running mode
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
  uint16_t lowThres = (buf[0]<< 8) + buf[1];
  uint16_t uppThres = (buf[2]<< 8) + buf[3];
  if (lowThres != lowerThreshold) lowerThreshold = lowThres;
  if (uppThres != upperThreshold) upperThreshold = uppThres;
  Serial.println("From eeprom: lowerThreshold: " + String(lowerThreshold) + ", upperThreshold: " + String(upperThreshold));      
}

void loop() {
  if (storeThresholds) {
    storeThresholds = false;
    uint16_t lowThres = (r_buffer[0]<< 8) + r_buffer[1];
    uint16_t uppThres = (r_buffer[2]<< 8) + r_buffer[3];
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
    countsSinceLastQuery++;
    periodTimeSinceLastQuery += iPeriod;
    
    Serial.println("red --> blank");
    
    // send T to database...
    Serial.println("T: " + String(T));

    if (T > 100) {
        // generate interrupt for master
        digitalWrite(intOutPin, LOW);
        Serial.print("\\");
        delay(1);
        Serial.print("/");
        digitalWrite(intOutPin, HIGH);
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
