#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// --- UNIQUE CONFIG FOR EACH BOT ---
const int BOT_ID = 0; // Change this to 2 or 4 for your other robots
const char* ssid = "NISHU123";
const char* password = "123456789";
const char* mqtt_server = "10.216.105.124"; 

// --- Pins ---
// const int PIN_M1 = 12, PIN_M2 = 13, PIN_M3 = 26; 
// const int PIN_MOSFET = 23;    
// const int PIN_LOWER = 33;
// const int PIN_UPPER = 32;

// pins for BOT 2//
const int PIN_M1 = 13, PIN_M2 = 12, PIN_M3 = 26; 
const int PIN_MOSFET = 23;    
const int PIN_LOWER = 33;
const int PIN_UPPER = 32;

 WiFiClient espClient;
PubSubClient client(espClient);
Servo m1, m2, m3, lower_servo, upper_servo;

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void callback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) return;

  // 1. ARM CONTROL (Follows Python state machine exactly)
  if (doc.containsKey("base")) {
    lower_servo.write(doc["base"]);
  }
  if (doc.containsKey("elbow")) {
    upper_servo.write(doc["elbow"]);
  }

  // 2. MAGNET CONTROL
  if (doc.containsKey("mag")) {
    int magValue = doc["mag"];
    digitalWrite(PIN_MOSFET, (magValue == 1) ? HIGH : LOW);
  }

  // 3. MOTION CONTROL (Standard 3-wheel Omni)
  float s1 = doc["s1"] | 0.0;
  float s2 = doc["s2"] | 0.0;
  float s3 = doc["s3"] | 0.0;
  
  m1.write(constrain(map(s1 * 10, -100, 100, 0, 180), 0, 180));
  m2.write(constrain(map(s2 * 10, -100, 100, 0, 180), 0, 180));
  m3.write(constrain(map(s3 * 10, -100, 100, 0, 180), 0, 180));
}

void reconnect() {
  while (!client.connected()) {
    // Generate Unique Client ID (e.g., ESP32_Bot_0)
    String clientId = "ESP32_Bot_" + String(BOT_ID);
    
    if (client.connect(clientId.c_str())) { 
      // Subscribe to bot-specific topic (e.g., bot_0/motor_speeds)
      String topic = "bot_" + String(BOT_ID) + "/cmd_vel";
      client.subscribe(topic.c_str());
      Serial.println("Connected to topic: " + topic);
    } else {
      
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  lower_servo.attach(PIN_LOWER, 500, 2500);
  upper_servo.attach(PIN_UPPER, 500, 2500);
  m1.attach(PIN_M1); m2.attach(PIN_M2); m3.attach(PIN_M3);
  
  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
}