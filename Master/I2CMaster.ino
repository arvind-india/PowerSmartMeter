#define BRZO

#ifndef BRZO
#include <Wire.h>
#else
#include <brzo_i2c.h>
#include "SH1106Brzo.h"
#endif
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <Fs.h>

SH1106Brzo  display(0x3c, D3, D4);

#define INT_PIN D2
#define BTN_PIN D7
#define RES_PIN D6

uint8_t SDA_PIN = D3;   // (0)
uint8_t SCL_PIN = D4;   // (2)
uint8_t SLV_ADR = 42;

uint8_t zeroCounter = 0;

uint16_t mw[1025];
uint16_t lowerTrigger = 0;
uint16_t upperTrigger = 0;

uint8_t buffer[32];
uint32_t startTime = 0;
volatile bool dataAvailable = false;
bool shouldSaveConfig = false;
const uint8_t revs_per_kWh = 150;
float counter = 0.0;
float power = 0.0;
char ccuip[16] = "192.168.178.54";
char counterConstant[6];
char ccuSysvarActPower[32];
char ccuSysvarPowerCounter[32];

struct totals
{
   uint32_t totalPeriodTime;
   uint32_t totalRevolutions;
};

void saveConfigCallback () {
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

bool loadConfig() { 
    File configFile = SPIFFS.open("/config.json", "r");
    if (!configFile) {
        Serial.println("ERROR: failed to open config file");
        return false;
    }

    size_t size = configFile.size();
    if (size > 1024) {
        Serial.println("ERROR: config file size is too large");
        return false;
    }

    // Allocate a buffer to store contents of the file.
    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& json = jsonBuffer.parseObject(buf.get());

    if (!json.success()) {
        Serial.println("ERROR: failed to parse config file");
        return false;
    }

    strcpy(counterConstant, json["counterConstant"]);
    strcpy(ccuip, json["ccuip"]);
    strcpy(ccuSysvarActPower, json["ccuSysvarActPower"]);
    strcpy(ccuSysvarPowerCounter, json["ccuSysvarPowerCounter"]);

    Serial.println("loaded configuration data: ");
    Serial.print("counterConstant: "); Serial.println(counterConstant);
    Serial.print("ccuip: "); Serial.println(ccuip);
    Serial.print("ccuSysvarActPower: "); Serial.println(ccuSysvarActPower);
    Serial.print("ccuSysvarPowerCounter: "); Serial.println(ccuSysvarPowerCounter);
    return true;
}

int setStateCcuSysVar(String sysvar, String value) {
    int rc = -1;
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;
        String url = "http://" + String(ccuip) + ":8181/cuxd.exe?ret=dom.GetObject(%22" + sysvar + "%22).State(" + value + ")";
        Serial.println("URL = " + url);
        http.begin(url);
        int httpCode = http.GET();
        rc = httpCode;
        Serial.println("httpcode = " + String(httpCode));
        if (httpCode > 0) {
            //String payload = http.getString();
        }
        if (httpCode != 200) {
            Serial.println("HTTP fail " + String(httpCode));
        }
        http.end();
    } else Serial.println("wifi not connected!");
    return rc;
}

#ifndef BRZO
void setThresholds(uint16_t lowThres, uint16_t uppThres) {
    Wire.beginTransmission(42);
    Wire.write(2);
    Wire.write(lowThres >> 8);
    Wire.write(lowThres & 0xFF);
    Wire.write(uppThres >> 8);
    Wire.write(uppThres & 0xFF);
    boolean allesgut = Wire.endTransmission();
}
#else
void setThresholds(uint16_t lowThres, uint16_t uppThres) {
    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[4];
    buf[0] = 0x02;
    brzo_i2c_write(buf, 1, false);

    buf[0] = (lowThres >> 8);
    buf[1] = (lowThres & 0xFF);
    buf[2] = (uppThres >> 8);
    buf[3] = (uppThres & 0xFF);
    brzo_i2c_write(buf, 4, false);

    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("setThresholds: " + String(res));
}
#endif

#ifndef BRZO
void setModeFreeRunningMode() {
    Wire.beginTransmission(42);
    Wire.write(0x81);
    boolean allesgut = Wire.endTransmission();
}
#else
void setModeFreeRunningMode() {
    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[4];
    buf[0] = 0x81;
    brzo_i2c_write(buf, 1, false);
    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("setModeFreeRunningMode: " + String(res));
}
#endif

#ifndef BRZO
void setModeMeasureMode() {
    Wire.beginTransmission(42);
    Wire.write(0x82);
    boolean allesgut = Wire.endTransmission();
}
#else
void setModeMeasureMode() {
    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[4];
    buf[0] = 0x82;
    brzo_i2c_write(buf, 1, false);
    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("setModeMeasureMode: " + String(res));
}
#endif

#ifndef BRZO
void setModeAlignementMode() {
    Wire.beginTransmission(42);
    Wire.write(0x83);
    boolean allesgut = Wire.endTransmission();
}
#else
void setModeAlignementMode() {
    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[4];
    buf[0] = 0x83;
    brzo_i2c_write(buf, 1, false);
    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("setModeAlignementMode: " + String(res));
}
#endif

#ifndef BRZO
totals requestTotals() {
    Wire.beginTransmission(42);
    Wire.write(8);
    boolean allesgut = Wire.endTransmission();
    delay(1);
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
#else
totals requestTotals() {
    totals totalValues;
    totalValues.totalRevolutions = 0;
    totalValues.totalPeriodTime = 0;
        
    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[8];
    buf[0] = 0x08;
    brzo_i2c_write(buf, 1, false);
    brzo_i2c_read(buffer, 8, false);
    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("requestTotals: " + String(res));
    if (res == 0) {
        totalValues.totalRevolutions = ((buffer[0] << 24) | (buffer[1] << 16)  | (buffer[2] << 8) | buffer[3]);
        totalValues.totalPeriodTime = ((buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7]);
    }
    return totalValues;
}
#endif

#ifndef BRZO
uint32_t requestActualPeriodTime() {
    Serial.print('#');
    Wire.beginTransmission(42);
    Wire.write(4);
    boolean allesgut = Wire.endTransmission();
    Serial.print('*');
    delay(1);
    Wire.requestFrom(42, 4);
    int index = 0;
    while (Wire.available()) {
        uint8_t b = Wire.read();
        //Serial.print("rb=" + String(b) + " ");
        buffer[index++] = b;   
    }
    uint32_t m = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
    return m;
}
#else
uint32_t requestActualPeriodTime() {
    uint32_t m = 0;

    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[4];
    buf[0] = 0x04;
    brzo_i2c_write(buf, 1, false);
    brzo_i2c_read(buf, 4, false);
    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("requestActualPeriodTime: " + String(res));
    if (res == 0) {
        m = ((buf[0] << 24) | (buf[1] << 16)  | (buf[2] << 8) | buf[3]);
    }
    return m;
}
#endif

#ifndef BRZO
uint16_t requestReflection(){
    Wire.beginTransmission(42);
    Wire.write(1);
    boolean allesgut = Wire.endTransmission();
    delay(1);
    Wire.requestFrom(42, 2);
    int index = 0;
    while (Wire.available()) {
        uint8_t rb = Wire.read();
        //Serial.print("rb=" + String(rb) + " ");
        buffer[index++] = rb;//Wire.read();   
    }
    allesgut = Wire.endTransmission();
    uint16_t m = (buffer[0] << 8) + buffer[1]; 
    return m;
}
#else
uint16_t requestReflection(){
    uint16_t m = 0;

    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[4];
    buf[0] = 0x01;
    brzo_i2c_write(buf, 1, true);
    brzo_i2c_read(buffer, 2, false);
    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("requestReflection: " + String(res));
    if (res == 0) {
        m = ((buffer[0] << 8) | buffer[1]);
    }
    return m;
}
#endif

#ifndef BRZO
uint8_t requestActualRunningMode() {
    Wire.beginTransmission(42);
    Wire.write(16);
    boolean allesgut = Wire.endTransmission();
    delay(1);
    Wire.requestFrom(42, 1);
    int index = 0;
    while (Wire.available()) {
        buffer[index++] = Wire.read();   
    }
    uint8_t m = buffer[0]; 
    return m;
}
#else
uint8_t requestActualRunningMode() {
    uint8_t m = 0;

    brzo_i2c_start_transaction(SLV_ADR, 100);
    uint8_t buf[4];
    buf[0] = 0x10;
    brzo_i2c_write(buf, 1, false);
    brzo_i2c_read(buf, 1, false);
    uint8_t res = brzo_i2c_end_transaction();
    Serial.println("requestActualRunningMode: " + String(res));
    if (res == 0) {
        m = buf[0];
    }
    return m;
}
#endif

void handleInterrupt() {
    dataAvailable = true;
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println();
    pinMode(INT_PIN, INPUT_PULLUP);
    pinMode(BTN_PIN, INPUT_PULLUP);
    pinMode(RES_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);

#ifndef BRZO
    Wire.begin(0, 2);
    
    // switch off the internal pullup resistors
    //digitalWrite(SDA, 0);
    //digitalWrite(SCL, 0);
#else
    // Setup i2c with clock stretching timeout of 2000 usec
    brzo_i2c_setup(SDA_PIN, SCL_PIN, 50000);
#endif

    requestActualRunningMode();
    display.init();
    display.flipScreenVertically();

    Serial.println("starting WiFi");
    WiFiManager wifiManager;
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    WiFiManagerParameter custom_counterConstant("counterConstant", "Z&auml;hlerkonstante Stromz&auml;hler", counterConstant, 6);
    WiFiManagerParameter custom_ccuip("ccu", "ip adresse der CCU", ccuip, 16);
    WiFiManagerParameter custom_ccuSysvarActPower("ccuSysvarActPower", "Sys.-Var. f&uuml;r akt. Leistung", ccuSysvarActPower, 32);
    WiFiManagerParameter custom_ccuSysvarPowerCounter("ccuSysvarPowerCounter", "Sys.-Var. f&uuml;r Z&auml;hlerstand", ccuSysvarPowerCounter, 32);
    wifiManager.addParameter(&custom_counterConstant);
    wifiManager.addParameter(&custom_ccuip);
    wifiManager.addParameter(&custom_ccuSysvarActPower);
    wifiManager.addParameter(&custom_ccuSysvarPowerCounter);

    if (digitalRead(BTN_PIN) == LOW) {
        Serial.println("deleting WiFi settings, starting with configuration");
        wifiManager.resetSettings();
    }
    
    wifiManager.autoConnect();
    if (shouldSaveConfig) {
        SPIFFS.begin();
        Serial.println("Config changed, saving config...");
        StaticJsonBuffer<200> jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["counterConstant"] = custom_counterConstant.getValue();
        json["ccuip"] = custom_ccuip.getValue();
        json["ccuSysvarActPower"] = custom_ccuSysvarActPower.getValue();
        json["ccuSysvarPowerCounter"] = custom_ccuSysvarPowerCounter.getValue();
    
        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            Serial.println("ERROR: failed to open config file for writing");
        }
        else {
            Serial.println("actual configuration to save:");
            json.printTo(Serial);
            Serial.println("");
            json.printTo(configFile);
            configFile.close();
            Serial.println("config file successfuly written");
        }
        SPIFFS.end();
        shouldSaveConfig = false;
    }

    Serial.println("going further...");
    Serial.println("mounting filesystem...");
    if (!SPIFFS.begin()) {
        Serial.println("Error mounting filesystem. This is a problem");
    }
    if (!loadConfig()) {
        Serial.println("Error loading config. This is a problem");
    }
    delay(500);
    Serial.println("actual runningMode = " + String(requestActualRunningMode()));
    startTime = millis();
}

void loop() {
    if (digitalRead(RES_PIN) == LOW && requestActualRunningMode() != 3) {
        Serial.println("Reset triggers to: (0 , 0)");      
        setThresholds(0, 0); 
        Serial.println("thresholds reset, switching to alignement mode");
        setModeAlignementMode();
        delay(1000);
    }
    
    else if (digitalRead(RES_PIN) == LOW && requestActualRunningMode() == 3) {
        uint16_t maxLowerThreshold = 0;
        uint16_t minUpperThreshold = 0;
        uint8_t successCounter = 0;

        Serial.println("trying to evaluate the IR reflection thresholds");
        for (uint16_t i=0; i < 1024; i++)
            mw[i] = 0;
        Serial.println("Changing to free running mode");
        setModeFreeRunningMode();
        delay(500);
        
        Serial.println("free running mode active..");
        while (successCounter <= 5) {
           
            
            uint32_t t1 = millis();
            while (millis() - t1 < 60000) {
                uint16_t r = requestReflection();
                //Serial.print("r = " + String(r));
                delay(25);
                mw[r]++;
            }
        
            uint16_t count = 0;
            uint16_t locMax = 0;
            for (uint16_t i=1; i<1025; i++) {
                Serial.println(String(i) + ":" + String(mw[i]) + " ");
                int16_t steigung = mw[i] - mw[i-1];
                //Serial.print("S: " + String(steigung) + " ");
                    
                if (steigung < -50) {
                    locMax = (i-1);
                    //Serial.println("Lokales Maximum gefunden bei: " + String(locMax));
                    count++;
                    if (count == 1)
                        maxLowerThreshold = locMax;
                    else {
                        if ( (locMax - maxLowerThreshold) > 500) {
                            minUpperThreshold = locMax;
                            break;
                        }
                        else {
                            maxLowerThreshold = locMax;    
                        }
                    }
                }
            }
            Serial.println("maxLowerThreshold = " + String(maxLowerThreshold) + ", minUpperThreshold = " + String(minUpperThreshold));
            if (maxLowerThreshold >= minUpperThreshold) {
                Serial.println("Could not evaluate thresholds, starting over..");
                //for (uint16_t i=0; i < 1025; i++)
                //    mw[i] = 0;
            }
            else {
                successCounter++;
                break;
            }
        }
        uint16_t width = minUpperThreshold - maxLowerThreshold;
        float gap = 0.25 * (float)width;
        lowerTrigger = maxLowerThreshold + (uint16_t)gap;
        upperTrigger = minUpperThreshold - (uint16_t)gap;
        Serial.println("Set triggers to: (" + String(lowerTrigger) + "," + String(upperTrigger) + ")");      
        setThresholds(lowerTrigger, upperTrigger); 
        Serial.println("thresholds set, switching to measure mode");
        setModeMeasureMode();
        Serial.println("measure mode active.."); 
        dataAvailable = false;
        startTime = millis();
    }
    
    if (dataAvailable) {
        //noInterrupts();
        Serial.println("Received interrupt that data is available");
        uint32_t pt = requestActualPeriodTime();
        Serial.println("period time: " + String(pt) + "ms");
        //delay(10);
        dataAvailable = false;
        //interrupts();

        //display.clear();
        //display.setTextAlignment(TEXT_ALIGN_LEFT);
        //display.setFont(ArialMT_Plain_16);
        //display.drawString(0, 6, "Tp: " + String(pt) + "ms");
        //display.display();
    }
    
    else {
        //requestReflection();
        //delay(1000);
        //if (millis() - startTime > 300000){
        if (millis() - startTime > 30000){
            Serial.println("requesting total values");
            startTime = millis();  
            totals tvs = requestTotals();
            Serial.println("Total values: totalRevs = " + String(tvs.totalRevolutions) + ", totalPeriodTime = " + String(tvs.totalPeriodTime)+ "ms");
            if (tvs.totalRevolutions || zeroCounter >= 12) {
                float period;
                zeroCounter = 0;
                if (tvs.totalRevolutions) {
                    period = (float)tvs.totalPeriodTime / 1000.0;
                    counter += (float)tvs.totalRevolutions / revs_per_kWh; 
                    power = (float)tvs.totalRevolutions * 3600.0 / (revs_per_kWh * period);
                }
                else {
                    Serial.println("writing zeros to database once an hour");
                    power = 0.0;
                }
                Serial.println("actual counter: " + String(counter) + "kWh, actual power: " + String(power)+ "kW");

                display.clear();
                display.setTextAlignment(TEXT_ALIGN_LEFT);
                display.setFont(ArialMT_Plain_10);
                display.drawString(4, 6, "aktueller Zählerstand:");
                display.drawString(32, 20, String(counter) + " kWh");
                display.drawString(4, 34, "aktuelle Leistung:");
                display.drawString(32, 48, String(power) + " kW");
                
                display.display();
         
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.println("WiFi connected, sending data");
                    int rc = setStateCcuSysVar(ccuSysvarActPower, String(power));
                    rc = setStateCcuSysVar(ccuSysvarPowerCounter, String(counter));
                }
                else {
                    Serial.println("WiFi not connected, trying to connect...");
                    String ssid = WiFi.SSID();
                    String pwd = WiFi.psk();
                    WiFi.begin(ssid.c_str(), pwd.c_str());
                }
            }
            else {
                zeroCounter++;
            }
        }
    }
}
