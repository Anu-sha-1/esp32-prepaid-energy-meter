#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <SPIFFS.h>

// 🏷️ Pin Assignments
#define D7 17
#define D4 21
#define D5 19
#define D6 18
#define RS 23
#define E 22
#define ENERGY_METER_PULSE 35  // Energy meter pulse input
#define CT_OUTPUT 34            // ZMCT103 Current sensor (Analog)
#define BUZZER 15               // Buzzer
#define RELAY 16                // Relay control
#define OPTOCOUPLER 26          // Theft detection

// 📟 LCD Configuration (Non-I2C)
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

// 📶 Wi-Fi Credentials
const char* ssid = "B1B2";
const char* password = "incorrect";

// 📡 ThingsBoard MQTT Details
const char* mqtt_server = "demo.thingsboard.io";
const int mqtt_port = 1883;
const char* access_token = "628ocwox5utvuvm0cfzs";
const char* telemetry_topic = "v1/devices/me/telemetry";
const char* rpc_topic = "v1/devices/me/rpc/request/+";

// 🛜 WiFi & MQTT Clients
WiFiClient espClient;
PubSubClient client(espClient);

// ⚡ Energy Meter Variables
volatile int pulseCount = 0;
float energyConsumed = 0.0;
const float pulsesPerKWh = 3200.0;

// 🔋 Energy Balance Variables
int units_remaining = 7;
bool flag_low_balance = false;
bool flag_no_balance = false;
bool flag_theft = false;

// ⏱ Timing Variables
unsigned long lastMQTTSend = 0;
unsigned long lastLCDUpdate = 0;

// 📌 Function Prototypes
void callback(char* topic, byte* payload, unsigned int length);
void sendTelemetry();
void checkLowBalance();
void checkNoBalance();
void detectTheft();
void calculateEnergy();
void updateLCD();
void sendMQTTAlert(String message);
void reconnectMQTT();

// 🏷️ Interrupt for Energy Meter Pulse Counting
void IRAM_ATTR countPulse() {
    pulseCount++;
}

// 📡 Setup Wi-Fi
void setupWiFi() {
    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
     Serial.println("\n✅ Wi-Fi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

// 📩 MQTT Callback for Recharge & RPC Commands
void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("Received MQTT Message on ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);

    // 🔹 Handle Recharge Requests via RPC
    if (String(topic).startsWith("v1/devices/me/rpc/request")) {
        int recharge_units = message.toInt();
        if (recharge_units > 0) {
            units_remaining += recharge_units;
            flag_low_balance = false;
            flag_no_balance = false;

            if (units_remaining > 0) {
                digitalWrite(RELAY, HIGH);  // Restore power
                sendMQTTAlert("Power reconnected after recharge.");
            }
            sendMQTTAlert("Balance recharged: " + String(recharge_units) + " units.");
            updateLCD();
        }
    }
}

// 📤 Send MQTT Telemetry Data to ThingsBoard
void sendTelemetry() {
    if (millis() - lastMQTTSend > 5000) {
        lastMQTTSend = millis();
        String telemetryPayload = "{\"units_remaining\": " + String(units_remaining) + 
                                  ", \"energy\": " + String(energyConsumed, 2) + 
                                  ", \"theft_detected\": " + String(flag_theft ? "true" : "false") + "}";

        client.publish(telemetry_topic, telemetryPayload.c_str());
        Serial.println("Sent to ThingsBoard: " + telemetryPayload);
    }
}

// 📤 Send MQTT Alert to ThingsBoard
void sendMQTTAlert(String alert_message) {
    if (client.connected()) {
        String alertPayload = "{\"alert\": \"" + alert_message + "\"}";
        client.publish(telemetry_topic, alertPayload.c_str());
        Serial.println("Alert Sent: " + alertPayload);
    }
}

// 🔄 Reconnect to MQTT
void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        if (client.connect("ESP32EnergyMeter", access_token, "")) {
            Serial.println("Connected!");
            client.subscribe(rpc_topic);
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(". Retrying in 5s...");
            delay(5000);
        }
    }
}

// 🔍 Get Current Value from ZMCT103
float getCurrent() {
    int raw = analogRead(CT_OUTPUT);
    float voltage = raw * (3.3 / 4095.0);
    float current = (voltage - 1.65) / 0.066;
    return abs(current);
}

// 🚨 Detect Low Balance
void checkLowBalance() {
    if (units_remaining <= 5 && !flag_low_balance) {
        flag_low_balance = true;
        sendMQTTAlert("Low balance alert: Recharge soon.");
        digitalWrite(BUZZER, HIGH);
        delay(500);
        digitalWrite(BUZZER, LOW);
    }
}

// 🚦 Handle Zero Balance
void checkNoBalance() {
    if (units_remaining <= 0 && !flag_no_balance) {
        flag_no_balance = true;
        digitalWrite(RELAY, LOW);
        sendMQTTAlert("Power disconnected due to zero balance.");
    }
}

// ⚠️ Detect Theft
void detectTheft() {
    float current_input = getCurrent();
    bool bypass_detected = digitalRead(OPTOCOUPLER) == HIGH;

    if (bypass_detected && current_input > 0.5 && !flag_theft) {
        flag_theft = true;
        sendMQTTAlert("⚠️ Theft Alert: Unauthorized power usage detected.");
        digitalWrite(RELAY, LOW);
        digitalWrite(BUZZER, HIGH);
        delay(1000);
        digitalWrite(BUZZER, LOW);
    }

    if (flag_theft && !bypass_detected) {
        flag_theft = false;
        digitalWrite(RELAY, HIGH);
        sendMQTTAlert("Theft cleared. Power restored.");
    }
}

// ⚡ Calculate Energy Consumption
void calculateEnergy() {
    energyConsumed = pulseCount / pulsesPerKWh;
    units_remaining = 7 - energyConsumed;

    if (units_remaining < 0) {
        units_remaining = 0;
        checkNoBalance();
    }
}

// 🖥️ Update LCD Display Efficiently
void updateLCD() {
    if (millis() - lastLCDUpdate > 1000) {
        lastLCDUpdate = millis();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Units: ");
        lcd.print(units_remaining);
        lcd.setCursor(0, 1);
        lcd.print(units_remaining <= 0 ? "Power Off" : "Energy: " + String(energyConsumed, 2) + " kWh");
    }
}

// 🔧 Setup Function
void setup() {
    Serial.begin(115200);
    EEPROM.begin(512);
    SPIFFS.begin(true);
    setupWiFi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    pinMode(BUZZER, OUTPUT);
    pinMode(RELAY, OUTPUT);
    //pinMode(OPTOCOUPLER, INPUT_PULLUP);  // Enable internal pull-up for optocoupler input
    attachInterrupt(digitalPinToInterrupt(ENERGY_METER_PULSE), countPulse, FALLING);

    lcd.begin(16, 2);
    lcd.print("Initializing...");
    digitalWrite(RELAY, HIGH);
}

// 🔄 Main Loop
void loop() {
    if (!client.connected()) reconnectMQTT();
    client.loop();

    checkLowBalance();
    checkNoBalance();
    detectTheft();
    calculateEnergy();
    sendTelemetry();
    updateLCD();
}
