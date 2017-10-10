#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <Fs.h>

#define INT_PIN D2
#define BTN_PIN D7

uint16_t mw[1000];
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

void setThresholds(uint16_t lowThres, uint16_t uppThres) {
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
    Serial.print('#');
    Wire.beginTransmission(42);
    Wire.write(4);
    boolean allesgut = Wire.endTransmission();
    Serial.print('*');
    delay(1);
    Wire.requestFrom(42, 4);
    int index = 0;
    //delay(10);
    while (!Wire.available())
        ;
    while (Wire.available()) {
        uint8_t b = Wire.read();
        Serial.print("rb=" + String(b) + " ");
        buffer[index++] = b;   
    }
    uint32_t m = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
    return m;
}

uint16_t requestReflection(){
    Wire.beginTransmission(42);
    Wire.write(1);
    boolean allesgut = Wire.endTransmission();
    Wire.requestFrom(42, 2);
    int index = 0;
    while (Wire.available()) {
        uint8_t rb = Wire.read();
        //Serial.print("rb=" + String(rb) + " ");
        buffer[index++] = rb;//Wire.read();   
    }
    //Serial.println();
    uint16_t m = (buffer[0] << 8) + buffer[1]; 
    //Serial.println(m);
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
    Serial.println();
    pinMode(INT_PIN, INPUT_PULLUP);
    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), handleInterrupt, FALLING);
    Wire.begin(0, 2);


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
    //setThresholds(222, 777);
    //delay(500);
    //setModeFreeRunningMode();
    //setModeMeasureMode();
    Serial.println("actual runningMode = " + String(requestActualRunningMode()));
    startTime = millis();
}

void loop() {
    if (digitalRead(BTN_PIN) == LOW) {
        uint16_t maxLowerThreshold = 0;
        uint16_t minUpperThreshold = 0;
        uint8_t successCounter = 0;

        Serial.println("trying to evaluate the IR reflection thresholds");
        Serial.println("Changing to free running mode");
        setModeFreeRunningMode();
        delay(500);
        
        Serial.println("free running mode active..");
        while (successCounter <= 5) {
            for (uint16_t i=0; i < 1000; i++)
                mw[i] = 0;
            
            uint32_t t1 = millis();
            while (millis() - t1 < 60000) {
                uint16_t r = requestReflection();
                delay(1);
                mw[r]++;
            }
        
            uint16_t count = 0;
            uint16_t locMax = 0;
            for (uint16_t i=1; i<1000; i++) {
                Serial.println(String(i) + ":" + String(mw[i]) + " ");
                int16_t steigung = mw[i] - mw[i-1];
                //Serial.print("S: " + String(steigung) + " ");
                    
                if (steigung < -150) {
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
                for (uint16_t i=0; i < 1000; i++)
                    mw[i] = 0;
            }
            else {
                successCounter++;
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
        noInterrupts();
        Serial.println("Received interrupt that data is available");
        uint32_t pt = requestActualPeriodTime();
        Serial.println("period time: " + String(pt) + "ms");
        delay(10);
        dataAvailable = false;
        interrupts();
    }
    
    else {
        //requestReflection();
        //delay(1000);
        if (millis() - startTime > 300000){
            Serial.println("requesting total values");
            startTime = millis();  
            totals tvs = requestTotals();
            Serial.println("Total values: totalRevs = " + String(tvs.totalRevolutions) + ", totalPeriodTime = " + String(tvs.totalPeriodTime)+ "ms");
            if (tvs.totalRevolutions) {
                float period = (float)tvs.totalPeriodTime / 1000.0;
                counter += (float)tvs.totalRevolutions / revs_per_kWh; 
                power = (float)tvs.totalRevolutions * 3600.0 / (revs_per_kWh * period);
                Serial.println("actual counter: " + String(counter) + "kWh, actual power: " + String(power)+ "kW");
         
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
        }
    }
}
