#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define DEBUG

#ifdef DEBUG
#define Sprint(a) (Serial.print(a))
#define Sprintln(a) (Serial.println(a))
#else
#define Sprint(a)
#define Sprintln(a)
#endif

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
uint16_t  lowerThresholdToStore = 0;
uint16_t  upperThresholdToStore = 0;
uint16_t  measured_reflection = 0;
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
    measureMode,
    alignementMode
};
enum runningModes runningMode = stopped;
//enum runningModes runningMode = measureMode;
//enum runningModes runningMode = freerunMode;
enum runningModes runningModeToTransmit = runningMode;

uint16_t measureReflection(void);
enum irStates getIrState(void);
void requestEvent(void);
void receiveEvent(int);

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
    measured_reflection = sample;
    if (sample > upperThreshold) {
        digitalWrite(ledOutPin, HIGH);
        lastState = blank;
        return blank;
    }
    else if (sample < lowerThreshold) {
        digitalWrite(ledOutPin, LOW);
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
        Sprint("!");
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
        Sprint("*");
    }

    else if (cmd == 0x04) {
        // transmit iPeriod to master
        Sprint("!");
        if (runningMode == measureMode) {
            buffer[0] = (uint8_t)(iPeriodToTransmit >> 24);
            buffer[1] = (uint8_t)(iPeriodToTransmit >> 16);
            buffer[2] = (uint8_t)(iPeriodToTransmit >> 8);
            buffer[3] = (uint8_t)(iPeriodToTransmit);
        }
        else {
            for (uint8_t i = 0; i < 4; i++) {
                buffer[i] = 0;
            }
        }
        Wire.write(buffer, 4);
        Sprint("*");
    }

    else if (cmd == 0x08) {
        // transmit totals to master
        Sprint("!");
        if (runningMode == measureMode) {
            buffer[0] = (uint8_t)(countsSinceLastQueryToTransmit >> 24);
            buffer[1] = (uint8_t)(countsSinceLastQueryToTransmit >> 16);
            buffer[2] = (uint8_t)(countsSinceLastQueryToTransmit >> 8);
            buffer[3] = (uint8_t)(countsSinceLastQueryToTransmit);
            buffer[4] = (uint8_t)(periodTimeSinceLastQueryToTransmit >> 24);
            buffer[5] = (uint8_t)(periodTimeSinceLastQueryToTransmit >> 16);
            buffer[6] = (uint8_t)(periodTimeSinceLastQueryToTransmit >> 8);
            buffer[7] = (uint8_t)(periodTimeSinceLastQueryToTransmit);
        }
        else {
            for (uint8_t i = 0; i < 8; i++) {
                buffer[i] = 0;
            }
        }
        Wire.write(buffer, 8);
        Sprint("*");
    }

    else if (cmd == 0x10) {
        // cmd 16 requests the actual running mode (1 Byte)
        Sprint("!");
        buffer[0] = (uint8_t)runningModeToTransmit;
        Wire.write(buffer, 1);
        Sprint("*");
    }

    else {
        // answer unknown command with an easy recognizable bit pattern as error code
        Sprint("!");
        buffer[0] = 0xAA;
        buffer[1] = 0x55;
        buffer[2] = 0xAA;
        buffer[3] = 0x55;
        Wire.write(buffer, 4);
        Sprint("*");
    }
}

void receiveEvent(int anzahl)
{
    uint8_t index = 0;
    if (Wire.available()) {
        cmd = Wire.read();

        //Sprintln("cmd = " + String(cmd));
        if (cmd == 0x01) {
            // cmd == 1 = request actual reflection value
            reflectionToTransmit = measured_reflection;
        }

        else if (cmd == 0x02) {
            // cmd == 2 = set thresholds. Wait for further 4 Bytes with the 2 values for lower and upper threshold
            while (Wire.available()) {
                r_buffer[index++] = Wire.read();
            }
            if (index >= 4) {
                // set flag 'store thresholds'. values will be stored in main loop
                lowerThresholdToStore = (r_buffer[0]<< 8) + r_buffer[1];
                upperThresholdToStore = (r_buffer[2]<< 8) + r_buffer[3];
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
            // cmd == 0x80 = stop measurements
            runningMode = stopped;
        }

        else if (cmd == 0x81) {
            // cmd == 0x81 = switch to runningmode freerunMode
            runningMode = freerunMode;
        }

        else if (cmd == 0x82) {
            // cmd == 0x82 = switch to runningmode measureMode
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

        else if (cmd == 0x83) {
            // cmd == 0x83 = switch to runningmode alignementMode
            runningMode = alignementMode;
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
    // switch off the internal pullup resistors
    digitalWrite(SDA, 0);
    digitalWrite(SCL, 0);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    long eepromAddr = 0;
    uint8_t buf[4];
    uint8_t i = 0;
    while (i < 4) {
        buf[i++] = EEPROM.read(eepromAddr++);
    }
    lowerThreshold = (uint16_t)((buf[0]<< 8) + buf[1]);
    upperThreshold = (uint16_t)((buf[2]<< 8) + buf[3]);
    Sprintln(
            "From eeprom: lower threshold: " + String(lowerThreshold) +
            ", upper threshold: " + String(upperThreshold)
    );
    if (lowerThreshold == upperThreshold) {
        runningMode = alignementMode;
    }
    else {
        prepareMeasureMode = true;
        runningMode = measureMode;
    }
}

void loop() {
    if (storeThresholds) {
        storeThresholds = false;
        Sprintln(
                "lowThres: " + String(lowerThresholdToStore) +
                ", uppThres: " + String(upperThresholdToStore)
        );

        long eepromAddr = 0;
        uint8_t buf[4];
        uint8_t i = 0;
        buf[0] = lowerThresholdToStore >> 8;
        buf[1] = lowerThresholdToStore;
        buf[2] = upperThresholdToStore >> 8;
        buf[3] = upperThresholdToStore;
        while(i < 4) {
            EEPROM.write(eepromAddr++, buf[i++]);
        }
        Sprintln("threshold values written to eeprom");
    }

    if (runningMode == alignementMode) {
        uint16_t m_refl = measureReflection();
        digitalWrite(ledOutPin, HIGH);
        delay(m_refl / 2);
        digitalWrite(ledOutPin, LOW);
        delay(m_refl / 2);
    }

    if (runningMode == freerunMode) {
        measured_reflection = measureReflection();
        //Serial.println(String(cmd) + ":" + String(measured_reflection));
    }

    else if (runningMode == measureMode) {
        if (prepareMeasureMode) {
            // switched from any other mode to measure mode?
            // if so, let the wheel pass until we find the falling edge as a defined starting point
            irState = unknown;
            while ((irState = getIrState()) == unknown)
                ;

            if (irState == blank) {
                Sprintln("Setup(), while state.blank");
                while((irState = getIrState()) == blank)
                    ;
            }

            Sprintln("Setup(), while state.red");
            // wait for the falling edge and start measurement of period time
            while((irState = getIrState()) == red)
                ;
            Sprintln("Setup(): red --> blank");
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

        Sprintln("red --> blank");

        // send T to database...
        Sprintln("T: " + String(T));

        if (T > 100) {
            // if measured period time is long enough, generate interrupt for master...
            // (if we started in state red we would otherwise get an absurd high power consumption)
            digitalWrite(intOutPin, LOW);
            Sprint("\\");
            delay(1);
            Sprint("/");
            digitalWrite(intOutPin, HIGH);
        }
        else {
            Sprintln("ignoring first sample after startup");
        }

        // ...and start the measurement of period time again
        start = millis();
        while ((irState = getIrState()) == blank)
            ;
        Sprintln("blank --> red");
    }

    else {
        delay(10);
    }
}
