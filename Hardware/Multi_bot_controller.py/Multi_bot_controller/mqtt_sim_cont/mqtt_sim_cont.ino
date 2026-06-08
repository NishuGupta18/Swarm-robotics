#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// --- UNIQUE CONFIG FOR EACH BOT ---
const int BOT_ID = 0; 
const char* ssid = "NISHU123";
const char* password = "123456789";
const char* mqtt_server = "10.216.105.1"; 
const int mqtt_port= 1883;

// --- Pins for bot 0---
const int PIN_M1 = 12, PIN_M2 = 13, PIN_M3 = 26; 
const int PIN_MOSFET = 23;    
const int PIN_LOWER =33;
const int PIN_UPPER = 32;


//pins for bot2//
//const int PIN_M1 = 12, PIN_M2 = 13, PIN_M3 = 26; 
//const int PIN_MOSFET = 22;    
//const int PIN_LOWER = 25;
//const int PIN_UPPER = 33;

//pins for bot4//
//const int PIN_M1 = 12, PIN_M2 = 13, PIN_M3 = 26; 
//const int PIN_MOSFET = 23;    
//const int PIN_LOWER = 33;
//const int PIN_UPPER = 32;



// --- Global Objects ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Servo m1, m2, m3, lower_servo, upper_servo;

// Declare strings globally, but initialize them in setup()
String cmd_topic;
String client_id;

void setup_wifi() {
    Serial.print(F("Connecting to WiFi: "));
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("\nWiFi connection failed! Restarting..."));
        ESP.restart();
    }
    Serial.println(F("\nWiFi Connected."));
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);
    
    if (error) return;

    // 1. ARM CONTROL - New ArduinoJson 7 Syntax
    if (!doc["base"].isNull())  lower_servo.write(doc["base"]);
    if (!doc["elbow"].isNull()) upper_servo.write(doc["elbow"]);

    // 2. MAGNET CONTROL
    if (!doc["mag"].isNull()) {
        int magValue = doc["mag"];
        digitalWrite(PIN_MOSFET, (magValue == 1) ? HIGH : LOW);
    }

    // 3. MOTION CONTROL
    float s1 = doc["s1"] | 0.0;
    float s2 = doc["s2"] | 0.0;
    float s3 = doc["s3"] | 0.0;
    
    m1.write(constrain(map(s1 * 10, -100, 100, 0, 180), 0, 180));
    m2.write(constrain(map(s2 * 10, -100, 100, 0, 180), 0, 180));
    m3.write(constrain(map(s3 * 10, -100, 100, 0, 180), 0, 180));
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection as " + client_id + "...");
        if (mqttClient.connect(client_id.c_str())) {
            Serial.println(F("connected"));
            mqttClient.subscribe(cmd_topic.c_str());
        } else {
            Serial.print(F("failed, rc="));
            Serial.print(mqttClient.state());
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize Strings here to avoid global constructor errors
    cmd_topic = "bot_" + String(BOT_ID) + "/cmd_vel";
    client_id = "ESP32_Bot_" + String(BOT_ID);

    pinMode(PIN_MOSFET, OUTPUT);
    digitalWrite(PIN_MOSFET, LOW);

    setup_wifi();
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    lower_servo.attach(PIN_LOWER, 500, 2500);
    upper_servo.attach(PIN_UPPER, 500, 2500);
    m1.attach(PIN_M1); 
    m2.attach(PIN_M2); 
    m3.attach(PIN_M3);
}

void loop() {
    if (!mqttClient.connected()) reconnect();
    mqttClient.loop();
}
