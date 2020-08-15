#include <WiFi.h>
#include <DHT.h>
#include <WiFiClient.h>
#include <ArduinoOTA.h>                   // For OTA Programming
#include <ArduinoJson.h>                  // JSON
#include <PubSubClient.h>                 // MQTT
#include <EmonLib.h>
#include "secrets.h"

////////////////////////////////
// Time variables
////////////////////////////////
#define REPORTING_PERIOD                  30000
#define WATCHDOG_TIMEOUT_PERIOD           86400000

// WiFi
const char* ssid =                        CONFIG_WIFI_SSID;
const char* password =                    CONFIG_WIFI_PASSWORD;

// OTA
#define OTA_PASSWORD                      CONFIG_OTA_PASSWORD

// MQTT
#define MQTT_SERVER                       CONFIG_MQTT_SERVER
#define MQTT_SERVERPORT                   CONFIG_MQTT_SERVERPORT
#define MQTT_USERNAME                     CONFIG_MQTT_USER
#define MQTT_KEY                          CONFIG_MQTT_PASSWORD

//LED
#define LED_BUILTIN                       2

////////////////////////////////
// Variables
////////////////////////////////
char hostname[32];
const int numENERGIEMonitors =             1;

////////////////////////////////
// Complex Variables
////////////////////////////////
WiFiClient client;                        // TCP Client
PubSubClient mqttClient(client);          // MQTT

////////////////////////////////
// Util functions
////////////////////////////////
constexpr unsigned int str2int(const char* str, int h = 0) {
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

////////////////////////////////
// Structs
////////////////////////////////
struct ENERGIEMonitor {
  private:    
    int ENERGY_TRANSFORMER_PIN, WATER_FLOW_PIN, WATER_TEMPERATURE_PIN;

    double THERMISTOR_R25 = 50000.0;
    double THERMISTOR_B = 3950.0;

    uint32_t WaterFlowPulses, WaterFlowMillis, WaterTemperatureTemp;

    int DHT_PIN = 5;
    int DHT_TYPE = DHT22;

    EnergyMonitor emon;

    DHT dht = DHT(DHT_PIN, DHT_TYPE);
    
    unsigned long n;

    unsigned long LAST_COMMUNICATION = 0;

    char MQTT_TOPIC[96], buffer[512];
  public:
    String ThingId;
  
    double Temperature, Humidity, WaterTemperature, WaterFlow, EnergyConsumption;
    
    bool Active = true;
    
    ENERGIEMonitor(String _ThingId, int _ENERGY_TRANSFORMER_PIN, int _WATER_FLOW_PIN, int _WATER_TEMPERATURE_PIN, int _DHT_PIN, int _DHT_TYPE = DHT22) {
      ENERGY_TRANSFORMER_PIN = _ENERGY_TRANSFORMER_PIN;
      WATER_FLOW_PIN = _WATER_FLOW_PIN;
      WATER_TEMPERATURE_PIN = _WATER_TEMPERATURE_PIN;
      ThingId = _ThingId;
      DHT_PIN = _DHT_PIN;
      DHT_TYPE = _DHT_TYPE;
    }

    void begin(void (*f)()) {      
      pinMode(WATER_FLOW_PIN, INPUT_PULLUP);
      attachInterrupt(WATER_FLOW_PIN, f, RISING);
    
      pinMode(ENERGY_TRANSFORMER_PIN, INPUT_PULLDOWN);

      emon.current(ENERGY_TRANSFORMER_PIN, 30);

      dht.begin();

      WaterFlowMillis = millis();

      String commandTopicString = "telemetry/" + ThingId + "/energymonitor";
      commandTopicString.toCharArray(MQTT_TOPIC, 96);
    }

    void loop() {
      EnergyConsumption += emon.calcIrms(1480);
      WaterTemperatureTemp += analogRead(WATER_TEMPERATURE_PIN);
      n++;
      
      if (millis() - LAST_COMMUNICATION > REPORTING_PERIOD) {
        digitalWrite(LED_BUILTIN, HIGH);

        Temperature = dht.readTemperature();
        
        Humidity = dht.readHumidity();

        WaterTemperature = (double)WaterTemperatureTemp / (double)n;
        WaterTemperature = 10000.0 * (4095.0 / WaterTemperature - 1.0) / THERMISTOR_R25;        // (R/Ro)
        WaterTemperature = log(WaterTemperature);                                               // ln(R/Ro)
        WaterTemperature /= THERMISTOR_B;                                                       // 1/B * ln(R/Ro)
        WaterTemperature += 1.0 / (25.0 + 273.15);                                              // + (1/To)
        WaterTemperature = 1.0 / WaterTemperature;                                              // Invert
        WaterTemperature -= 273.15;                                                             // To Celcius

        WaterFlow = (double)WaterFlowPulses;
        WaterFlow /= (double)(millis() - WaterFlowMillis);
        WaterFlowPulses = 0;
        WaterFlowMillis = millis();
        WaterFlow *= 1000.00;
        WaterFlow /= 6.60;

        EnergyConsumption /= n;
        
        char *type[] = {"Temperature", "Humidity", "WaterTemperature", "Flow", "EnergyConsumption"};
        double value[] = {Temperature, Humidity, WaterTemperature, WaterFlow, EnergyConsumption};

        size_t capacity = JSON_OBJECT_SIZE(15);
        DynamicJsonDocument doc(capacity);

        for (int i = 0; i < 5; i++) {
          doc.clear();
          
          doc["type"] = type[i];
          doc["value"] = value[i];
          
          serializeJson(doc, buffer);

          mqttClient.publish(MQTT_TOPIC, buffer);
        }

        EnergyConsumption = 0;
        WaterTemperatureTemp = 0;
        n = 0;

        LAST_COMMUNICATION = millis();
        digitalWrite(LED_BUILTIN, LOW);
      }
    }

    void WaterFlowInterrupt() {
      WaterFlowPulses++;
    }
};
ENERGIEMonitor ENERGIEMonitors[numENERGIEMonitors] = {ENERGIEMonitor(CONFIG_THING_ID_RC, 34, 27, 32, 5)};
//ENERGIEMonitor ENERGIEMonitors[numENERGIEMonitors] = {ENERGIEMonitor(CONFIG_THING_ID_RC, 34, 27, 32, 5),
//                                                        ENERGIEMonitor(CONFIG_THING_ID_1A, 35, 26, 33, 6)};

////////////////////////////////
// Interrupts
////////////////////////////////
void IRAM_ATTR Interrupt_RC() {
  ENERGIEMonitors[0].WaterFlowInterrupt();
}
/*void IRAM_ATTR Interrupt_1A() {
  ENERGIEMonitors[1].WaterFlowInterrupt();
}*/

////////////////////////////////
// Setup
////////////////////////////////
void setup() {
  randomSeed(micros());

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  setupWiFi();
  setupOTA();
  setupMQTT();

  ENERGIEMonitors[0].begin(Interrupt_RC);
  //ENERGIEMonitors[1].begin(Interrupt_1A);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  connectWiFi();
  loopMQTT();
  loopOTA();
  
  for (int ENERGIEmonitor = 0; ENERGIEmonitor < numENERGIEMonitors; ENERGIEmonitor++) {
    ENERGIEMonitors[ENERGIEmonitor].loop();
  }
}

////////////////////////////////
// WiFi
////////////////////////////////
void setupWiFi() {
  uint32_t id = 0;
  for(int i=0; i<17; i=i+8) {
    id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  
  String hostnameString = "ENERGIEMonitor-" + String(id);
  hostnameString.toCharArray(hostname, 32);

  WiFi.mode(WIFI_STA);

  // Set your Static IP address
  IPAddress local_IP(192, 168, 1, 3);
  // Set your Gateway IP address
  IPAddress gateway(192, 168, 1, 254);
  
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(192, 168, 1, 254);
  IPAddress secondaryDNS(8, 8, 8, 8);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    ESP.restart();
  }

  WiFi.setHostname(hostname);

  connectWiFi();
}

bool connectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);

    const int kRetryCountWiFi = 20;
    int retryCountWiFi = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      if (retryCountWiFi++ > kRetryCountWiFi) {
        ESP.restart();
      }
    }
  }
  
  return WiFi.status() == WL_CONNECTED;
}

////////////////////////////////
// MQTT
////////////////////////////////
void setupMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_SERVERPORT);
  mqttClient.setCallback(MQTTOnMessage);

  mqttClient.setBufferSize(2048);
  
  loopMQTT();
}

void loopMQTT() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
}

void reconnectMQTT() {
  const int kRetryCountMQTT = 40;
  int retryCountMQTT = 0;
  // Loop until we're reconnected
  while (!mqttClient.connected()) {    
    // Attempt to connect
    if (mqttClient.connect(hostname, MQTT_USERNAME, MQTT_KEY)) {

      char commandTopicChar[64], debugTopicChar[64];
      String commandTopicString, debugTopicString;

      for (int ENERGIEmonitor = 0; ENERGIEmonitor < numENERGIEMonitors; ENERGIEmonitor++) {
        commandTopicString = "command/" + ENERGIEMonitors[ENERGIEmonitor].ThingId + "/+";
        commandTopicString.toCharArray(commandTopicChar, 64);
        debugTopicString = "debug/" + ENERGIEMonitors[ENERGIEmonitor].ThingId;
        debugTopicString.toCharArray(debugTopicChar, 64);
        
        mqttClient.subscribe(commandTopicChar);

        mqttClient.publish(debugTopicChar, "EnergyMonitor Connected!");
      }
    } else {
      // Wait 5 seconds before retrying
      unsigned long now = millis();
      while (millis() - now < 5000) {
        loopOTA();
        yield();
      }
    }
    if (retryCountMQTT++ > kRetryCountMQTT) {
      ESP.restart();
    }
  }
}

void MQTTOnMessage(char* topic, byte* payload, unsigned int length) {
  char commandTopicChar[64], debugTopicChar[64];
  String commandTopicString, debugTopicString;

  for (int ENERGIEmonitor = 0; ENERGIEmonitor < numENERGIEMonitors; ENERGIEmonitor++) {
    debugTopicString = "debug/" + ENERGIEMonitors[ENERGIEmonitor].ThingId;
    debugTopicString.toCharArray(debugTopicChar, 64);

    commandTopicString = "command/" + ENERGIEMonitors[ENERGIEmonitor].ThingId + "/restart";
    commandTopicString.toCharArray(commandTopicChar, 64);
    if (str2int(topic) == str2int(commandTopicChar)) {
      mqttClient.publish(debugTopicChar, "Restarting PowerMeterHAN!");
      ESP.restart();
    }

    commandTopicString = "command/" + ENERGIEMonitors[ENERGIEmonitor].ThingId + "/start";
    commandTopicString.toCharArray(commandTopicChar, 64);
    if (str2int(topic) == str2int(commandTopicChar)) {
      mqttClient.publish(debugTopicChar, "Starting PowerMeterHAN!");
      ENERGIEMonitors[ENERGIEmonitor].Active = true;
      return;
    }

    commandTopicString = "command/" + ENERGIEMonitors[ENERGIEmonitor].ThingId + "/stop";
    commandTopicString.toCharArray(commandTopicChar, 64);
    if (str2int(topic) == str2int(commandTopicChar)) {
      mqttClient.publish(debugTopicChar, "Stoping PowerMeterHAN!");
      ENERGIEMonitors[ENERGIEmonitor].Active = false;
      return;
    }
  }

  String topicString(topic);
  topicString += "/reply";
  char topicChar[64];
  topicString.toCharArray(topicChar, 64);
  mqttClient.publish(topicChar, "Invalid command received!");
}

////////////////////////////////
// OTA
////////////////////////////////
void setupOTA() {
  // Set OTA Hostname
  ArduinoOTA.setHostname(hostname);

  // Set OTA Password
  ArduinoOTA.setPassword((const char *)OTA_PASSWORD);

  // Init OTA
  ArduinoOTA.begin();

  // Loop OTA
  loopOTA();
}

void loopOTA() {
  ArduinoOTA.handle();
}
