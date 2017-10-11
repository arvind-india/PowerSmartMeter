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
uint16_t  reflectionToTransmit = 0;
uint32_t  iPeriodToTransmit = 0;
uint32_t  countsSinceLastQueryToTransmit = 0;
uint32_t  periodTimeSinceLastQueryToTransmit = 0;

// define the states of the wheel of the electricity meter
// the wheel has a single red mark on it's circumference so we have the two states 'blank wheel' and 'red mark'
// as well as the state 'unknown' if we just started and do not have detected the postion of the wheel yet
enum irStates {
    unknown,
    red,
    blank
};
enum irStates irState = unknown;

// define the running modes for the measurements
// stopped = self explanatory
// freerun mode = reflection is permanently measured and the reflection measurement value
//      can be requested by the master by sending command 0x01
// measure mode = the device observes the wheel and measures the period time for each revolution of the wheel
//      therefore thresholds for the reflection have to be defined. If the measured reflection value
//      lies above the upper threshold the state 'blank' is considered. If the measured reflection value
//      lies below the lower threshold the state 'red' is considered.
enum runningModes {
    stopped,
    freerunMode,
    measureMode
};
enum runningModes runningMode = measureMode;
//enum runningModes runningMode = freerunMode;
enum runningModes runningModeToTransmit = runningMode;

// measure the reflection of the wheel. To minimize influence of the ambience light we take two measurements
// the first measurement is done with the ir emitter switched off, the second one is done with the ir emitter
// switched on. The reflection measurement vale is then the difference between the two measurements.
uint16_t measureReflection() {
    digitalWrite(irOutPin, LOW);
    delay(10);
    float irValueOff = analogRead(analogInPin);

    digitalWrite(irOutPin, HIGH);
    delay(10);
    float irValueOn = analogRead(analogInPin);

    return (uint16_t)irValueOn - (uint16_t)irValueOff;
}

// measure the state of the wheel. If the thresholds are not well defined the measurement is impossible
// and therefore the state 'unknown' will be returned
// if thresholds are defined the measured reflection value is compared against them and the state is
// returned accordingly
enum irStates getIrState() {
    static enum irStates lastState = unknown;

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

// this is the requestEvent callback for the i2c communication with the master. It's called when the
// master issues a Wire.requestFrom()
// according to the command that has been received the requested information is prepared to the
// buffer and then the proper number of bytes is written to the Wire
void requestEvent(void) {
    if (cmd == 0x01) {
        // cmd 1 requests the actual reflection value (2 Bytes)
        if (runningMode != stopped) {
            // measurements are enabled, give back last reflection value
            buffer[0] = (uint8_t)(reflectionToTransmit >> 8);
            buffer[1] = (uint8_t)reflectionToTransmit;
        }
        else {
            // measurement is stopped, give back error code 0xFFFF
            buffer[0] = 0xFF;
            buffer[1] = 0xFF;
        }
        Wire.write(buffer, 2);
    }

    else if (cmd == 0x04) {
        Serial.print("!");
        // transmit iPeriod to master
        buffer[0] = (uint8_t)(iPeriodToTransmit >> 24);
        buffer[1] = (uint8_t)(iPeriodToTransmit >> 16);
        buffer[2] = (uint8_t)(iPeriodToTransmit >> 8);
        buffer[3] = (uint8_t)(iPeriodToTransmit);
        Wire.write(buffer, 4);
        Serial.print("*");
    }

    else if (cmd == 0x08) {
        buffer[0] = (uint8_t)(countsSinceLastQueryToTransmit >> 24);
        buffer[1] = (uint8_t)(countsSinceLastQueryToTransmit >> 16);
        buffer[2] = (uint8_t)(countsSinceLastQueryToTransmit >> 8);
        buffer[3] = (uint8_t)(countsSinceLastQueryToTransmit);
        buffer[4] = (uint8_t)(periodTimeSinceLastQueryToTransmit >> 24);
        buffer[5] = (uint8_t)(periodTimeSinceLastQueryToTransmit >> 16);
        buffer[6] = (uint8_t)(periodTimeSinceLastQueryToTransmit >> 8);
        buffer[7] = (uint8_t)(periodTimeSinceLastQueryToTransmit);
        Wire.write(buffer, 8);
    }

    else if (cmd == 0x10) {
        // cmd 16 requests the actual running mode (1 Byte)
        buffer[0] = (uint8_t)runningModeToTransmit;
        Wire.write(buffer, 1);
    }

    else {
        // answer unknown command with an easy recognizable bit pattern as error code
        buffer[0] = 0xAA;
        buffer[1] = 0x55;
        buffer[2] = 0xAA;
        buffer[3] = 0x55;
        Wire.write(buffer, 4);
    }
}


void receiveEvent(int anzahl)
{
    uint8_t index = 0;
    if (Wire.available()) {
        cmd = Wire.read();

        if (cmd == 0x01) {
            // cmd == 1 = request actual reflection value
            reflectionToTransmit = m;
        }

        else if (cmd == 0x02) {
            // cmd == 2 = set thresholds. Wait for further 4 Bytes with the 2 values for lower and upper threshold
            while (Wire.available()) {
                r_buffer[index++] = Wire.read();
            }
            if (index >= 4) {
                // set flag 'store thresholds'. values will be stored in main loop
                storeThresholds = true;
            }
        }

        else if (cmd == 0x04) {
            // cmd == 4 = request actual period time
            iPeriodToTransmit = iPeriod;
        }

        else if (cmd == 0x08) {
            // cmd == 8 = request totals since last request
            countsSinceLastQueryToTransmit = countsSinceLastQuery;
            periodTimeSinceLastQueryToTransmit = periodTimeSinceLastQuery;
            countsSinceLastQuery = 0;
            periodTimeSinceLastQuery = 0;
        }

        else if (cmd == 0x10) {
            // cmd == 8 = request actual running mode
            runningModeToTransmit = runningMode;
        }

        else if (cmd == 0x80) {
            runningMode = stopped;
        }

        else if (cmd == 0x81) {
            runningMode = freerunMode;
        }

        else if (cmd == 0x82) {
            if (lowerThreshold != upperThreshold) {
                prepareMeasureMode = true;
                runningMode = measureMode;
            }
            else {
                // can not switch to measure mode without well defined thresholds
                // switch to freerunMode instead..
                runningMode = freerunMode;
            }
        }

        else {
            cmd = 0xFF;
        }
    }
    return;
}

void setup() {
    Serial.begin(9600);
    Serial.println();
    analogReference(INTERNAL);

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
    Serial.println(
            "From eeprom: lower threshold: " + String(lowerThreshold) +
            ", upper threshold: " + String(upperThreshold)
    );
}

void loop() {
    if (storeThresholds) {
        storeThresholds = false;
        uint16_t lowThres = (r_buffer[0]<< 8) + r_buffer[1];
        uint16_t uppThres = (r_buffer[2]<< 8) + r_buffer[3];
        Serial.println(
                "lowThres: " + String(lowThres) +
                ", uppThres: " + String(uppThres)
        );
        uint8_t i = 0;
        long eepromAddr = 0;
        while(i < 4) {
            EEPROM.write(eepromAddr++, r_buffer[i++]);
        }
        Serial.println("threshold values written to eeprom");
    }

    if (runningMode == freerunMode) {
        m = measureReflection();
        if (lowerThreshold == upperThreshold) {
            // if no thresholds are defined help with the alignment of the sensor.
            // therefore blink the led. The better the alignement the slower the led will blink
            digitalWrite(ledOutPin, HIGH);
            delay(m / 2);
            digitalWrite(ledOutPin, LOW);
            delay(m / 2);
        }
        //Serial.println(String(cmd) + ":" + String(m));
    }

    else if (runningMode == measureMode) {
        if (prepareMeasureMode) {
            // switched from any other mode to measure mode?
            // if so, let the wheel pass until we find the falling edge as a defined starting point
            irState = unknown;
            while ((irState = getIrState()) == unknown)
                ;

            if (irState == blank) {
                Serial.println("Setup(), while state.blank");
                while((irState = getIrState()) == blank)
                    ;
            }

            Serial.println("Setup(), while state.red");
            // wait for the falling edge and start measurement of period time
            while((irState = getIrState()) == red)
                ;
            Serial.println("Setup(): red --> blank");
            start = millis();
            prepareMeasureMode = false;
        }
        // wait for raising edge and finish measurement of period time
        while ((irState = getIrState()) == red)
            ;
        // calculate period time T
        uint32_t T = millis() - start;
        iPeriod = T;
        // calculate totals
        countsSinceLastQuery++;
        periodTimeSinceLastQuery += iPeriod;

        Serial.println("red --> blank");

        // send T to database...
        Serial.println("T: " + String(T));

        if (T > 100) {
            // if measured period time is long enough, generate interrupt for master...
            // (if we started in state red we would otherwise get an absurd high power consumption)
            digitalWrite(intOutPin, LOW);
            Serial.print("\\");
            delay(1);
            Serial.print("/");
            digitalWrite(intOutPin, HIGH);
        }
        else {
            Serial.println("ignoring first sample after startup");
        }

        // ...and start the measurement of period time again
        start = millis();
        while ((irState = getIrState()) == blank)
            ;
        Serial.println("blank --> red");
    }

    else {
        delay(10);
    }
}
